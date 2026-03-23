# /data/openpilot/selfdrive/car/modules/LONG_module.py
"""
Human-tuned stock cruise syncing for XNOR.

This keeps the stable owner split and the no-lead curve behavior, but softens
the lead-release side so recovery starts sooner once a lead is clearly opening:

- lead-following still stays on the planner tail while a lead is constraining
- lead-hold persistence is shorter after the lead starts to clear
- opening leads with a healthy gap stop owning the target earlier
- no-lead curve control remains planner-first, with mapd only helping release

XNOR architecture adaptations retained:
- 5 Hz cruise stalk pacing
- stock cruise state gating
- startup freshness guard for the planner stream
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

from cereal import messaging
from openpilot.common.swaglog import cloudlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons

from openpilot.selfdrive.car.modules.ACC_module import ACCController, AccDecision


def _mono_ms() -> int:
  return time.monotonic_ns() // 1_000_000


@dataclass
class LongDecision:
  button: Optional[int]
  log: str = ""


class LongController:
  MIN_CRUISE_SPEED_MS = 17.1 * CV.MPH_TO_MS
  _LP_FRESH_NS = 1_500_000_000
  _PLANNER_DRAG_MARGIN_MS = 0.5
  _PLANNER_BELOW_EGO_MARGIN_MS = 0.3
  _STRONG_DECEL_ATARGET_MS2 = -0.5

  _CURVE_ENTRY_PERSIST_MS = 1000
  _CURVE_EXIT_PERSIST_MS = 600
  _CURVE_EXIT_RECOVERY_MS_PER_S = 1.0
  _CURVE_MIN_CRUISE_HOLD_MARGIN_MS = 5.0 * CV.MPH_TO_MS
  _CURVE_MAPD_MIN_HOLD_MARGIN_MS = 0.5 * CV.MPH_TO_MS
  _CURVE_HARD_ENTRY_EXTRA_MS = 3.0 * CV.MPH_TO_MS
  _CURVE_RELEASE_NEAR_TARGET_MARGIN_MS = 1.0 * CV.MPH_TO_MS
  _CURVE_HOLD_DROP_DEADBAND_MS = 2.0 * CV.MPH_TO_MS
  _MAPD_FRESH_NS = 1_500_000_000
  _CURVE_MAPD_RELEASE_PERSIST_MS = 400
  _LEAD_HOLD_PERSIST_MS = 650
  _LEAD_HOLD_RELEASE_MARGIN_MS = 0.45 * CV.MPH_TO_MS
  _LEAD_OPENING_VREL_MS = 0.25
  _LEAD_OPENING_GAP_MIN_M = 24.0
  _NO_LEAD_MAPD_CURRENT_GATE_MS = 2.0 * CV.MPH_TO_MS

  def __init__(self) -> None:
    self.acc = ACCController()
    self._sm = messaging.SubMaster(["longitudinalPlan", "radarState", "mapdOut"])

    self._lp_target_last_ms: Optional[float] = None
    self._lp_target_near_ms: Optional[float] = None
    self._lp_last_ns: int = 0
    self._lp_has_lead: bool = False
    self._lp_a_target: float = 0.0

    self._lead_present: bool = False
    self._lead_drel: float = 0.0
    self._lead_vrel: float = 0.0
    self._mapd_suggested_ms: Optional[float] = None
    self._mapd_map_curve_ms: Optional[float] = None
    self._mapd_vision_curve_ms: Optional[float] = None
    self._mapd_last_ns: int = 0

    self._last_info_log_ms: int = 0
    self._enabled_since_ms: int = 0
    self._last_active: bool = False
    self._last_lp_seen_ns: int = 0
    self._stable_plan_samples: int = 0

    self._curve_entry_candidate_since_ms: int = 0
    self._curve_exit_candidate_since_ms: int = 0
    self._curve_hold_active: bool = False
    self._curve_hold_target_ms: float = 0.0
    self._curve_hold_last_update_ms: int = 0
    self._curve_mapd_release_candidate_since_ms: int = 0
    self._lead_hold_until_ms: int = 0

  def _rate_log(self, msg: str) -> None:
    now = _mono_ms()
    if now - int(self._last_info_log_ms) < 1000:
      return
    self._last_info_log_ms = int(now)
    cloudlog.info(msg)

  @staticmethod
  def _extract_plan_speed_last(lp) -> Optional[float]:
    speeds = getattr(lp, "speeds", None)
    if not speeds:
      return None
    try:
      v_last = float(speeds[-1])
    except Exception:
      return None
    if not (math.isfinite(v_last) and v_last >= 0.0):
      return None
    return v_last

  @staticmethod
  def _extract_plan_speed_near_min(lp) -> Optional[float]:
    speeds = getattr(lp, "speeds", None)
    if not speeds:
      return None
    valid_speeds: list[float] = []
    try:
      count = max(1, int(math.ceil(len(speeds) * 0.6)))
      for v in speeds[:count]:
        vf = float(v)
        if math.isfinite(vf) and vf >= 0.0:
          valid_speeds.append(vf)
    except Exception:
      return None
    if not valid_speeds:
      return None
    return float(min(valid_speeds))

  @staticmethod
  def _curve_entry_threshold_ms(reference_ms: float) -> float:
    if float(reference_ms) >= (55.0 * CV.MPH_TO_MS):
      return 8.0 * CV.MPH_TO_MS
    return 2.0 * CV.MPH_TO_MS

  @staticmethod
  def _curve_exit_margin_ms(reference_ms: float) -> float:
    if float(reference_ms) >= (60.0 * CV.MPH_TO_MS):
      return 2.0 * CV.MPH_TO_MS
    if float(reference_ms) >= (40.0 * CV.MPH_TO_MS):
      return 1.5 * CV.MPH_TO_MS
    return 1.0 * CV.MPH_TO_MS

  @staticmethod
  def _reference_speed_ms(*, base_target_ms: Optional[float], current_set_ms: float, v_ego_ms: float) -> float:
    if base_target_ms is not None and float(base_target_ms) > 0.0:
      return float(base_target_ms)
    if float(current_set_ms) > 0.0:
      return float(current_set_ms)
    return float(max(0.0, v_ego_ms))

  def _resume_ceiling_ms(self, *, current_set_ms: float, v_ego_ms: float) -> float:
    retained_ceiling_ms = max(0.0, float(self.acc.acc_speed_kph) * CV.KPH_TO_MS)
    return max(float(current_set_ms), float(v_ego_ms), float(retained_ceiling_ms))

  def _mapd_curve_target_ms(self, *, now_ns: int) -> Optional[float]:
    if self._mapd_suggested_ms is None:
      return None
    if int(self._mapd_last_ns) <= 0:
      return None
    if (int(now_ns) - int(self._mapd_last_ns)) >= int(self._MAPD_FRESH_NS):
      return None
    suggested_ms = float(self._mapd_suggested_ms)
    if not (math.isfinite(suggested_ms) and suggested_ms > 0.1):
      return None
    return suggested_ms

  def _mapd_curve_floor_ms(self, *, now_ns: int) -> Optional[float]:
    if int(self._mapd_last_ns) <= 0:
      return None
    if (int(now_ns) - int(self._mapd_last_ns)) >= int(self._MAPD_FRESH_NS):
      return None

    candidates: list[float] = []
    for raw in (self._mapd_map_curve_ms, self._mapd_vision_curve_ms):
      if raw is None:
        continue
      val = float(raw)
      if math.isfinite(val) and val > 0.1:
        candidates.append(val)
    if not candidates:
      return None
    return float(max(candidates))

  def _should_hold_min_cruise_for_curve(self, *, now_ns: int, desired_ms: float, no_lead: bool) -> bool:
    if (not no_lead) or float(desired_ms) >= float(self.MIN_CRUISE_SPEED_MS):
      return False

    if float(desired_ms) >= (float(self.MIN_CRUISE_SPEED_MS) - float(self._CURVE_MIN_CRUISE_HOLD_MARGIN_MS)):
      return True

    curve_floor_ms = self._mapd_curve_floor_ms(now_ns=now_ns)
    if curve_floor_ms is None:
      return False

    return float(curve_floor_ms) >= (float(self.MIN_CRUISE_SPEED_MS) - float(self._CURVE_MAPD_MIN_HOLD_MARGIN_MS))

  def _maybe_release_curve_hold_from_mapd(self, *, now_ms: int, now_ns: int, reference_ms: float) -> bool:
    if not self._curve_hold_active:
      self._curve_mapd_release_candidate_since_ms = 0
      return False
    reference_ms = float(reference_ms)
    if reference_ms <= 0.1:
      self._curve_mapd_release_candidate_since_ms = 0
      return False

    mapd_target_ms = self._mapd_curve_target_ms(now_ns=now_ns)
    if mapd_target_ms is None:
      self._curve_mapd_release_candidate_since_ms = 0
      return False

    release_margin_ms = float(self._CURVE_RELEASE_NEAR_TARGET_MARGIN_MS)
    if float(mapd_target_ms) < (float(reference_ms) - float(release_margin_ms)):
      self._curve_mapd_release_candidate_since_ms = 0
      return False

    if int(self._curve_mapd_release_candidate_since_ms) == 0:
      self._curve_mapd_release_candidate_since_ms = int(now_ms)
      return False

    if (int(now_ms) - int(self._curve_mapd_release_candidate_since_ms)) >= int(self._CURVE_MAPD_RELEASE_PERSIST_MS):
      self._reset_curve_hold()
      return True
    return False

  def _reset_curve_hold(self) -> None:
    self._curve_entry_candidate_since_ms = 0
    self._curve_exit_candidate_since_ms = 0
    self._curve_hold_active = False
    self._curve_hold_target_ms = 0.0
    self._curve_hold_last_update_ms = 0
    self._curve_mapd_release_candidate_since_ms = 0

  def _reset_lead_hold(self) -> None:
    self._lead_hold_until_ms = 0

  def _lead_is_opening_clear(self, *, base_target_ms: float, v_ego_ms: float) -> bool:
    if (not self._lead_present) or float(self._lead_drel) <= 0.0:
      return False

    opening_gap_m = max(float(self._LEAD_OPENING_GAP_MIN_M), float(v_ego_ms) * 1.6)
    lead_speed_ms = max(0.0, float(v_ego_ms) + float(self._lead_vrel))
    return bool(
      float(self._lead_vrel) >= float(self._LEAD_OPENING_VREL_MS)
      and float(self._lead_drel) >= float(opening_gap_m)
      and float(lead_speed_ms) >= (float(base_target_ms) - float(self._LEAD_HOLD_RELEASE_MARGIN_MS))
    )

  def _lead_hold_active_now(self, *, now_ms: int, base_target_ms: float, planner_ms: float, v_ego_ms: float) -> bool:
    if int(self._lead_hold_until_ms) <= 0 or int(now_ms) > int(self._lead_hold_until_ms):
      self._lead_hold_until_ms = 0
      return False
    if float(planner_ms) <= 0.1:
      return False
    if self._lead_is_opening_clear(base_target_ms=float(base_target_ms), v_ego_ms=float(v_ego_ms)):
      self._lead_hold_until_ms = 0
      return False
    return float(planner_ms) < (float(base_target_ms) - float(self._LEAD_HOLD_RELEASE_MARGIN_MS))

  def _stabilize_no_lead_curve_target(self, *, now_ms: int, raw_target_ms: float, reference_ms: float) -> tuple[float, str]:
    raw_target_ms = float(raw_target_ms)
    reference_ms = float(reference_ms)
    if raw_target_ms <= 0.1 or reference_ms <= 0.1:
      self._reset_curve_hold()
      return raw_target_ms, "curve_invalid"

    delta_ms = max(0.0, reference_ms - raw_target_ms)
    entry_threshold_ms = float(self._curve_entry_threshold_ms(reference_ms))
    exit_margin_ms = float(self._curve_exit_margin_ms(reference_ms))
    hard_entry_threshold_ms = float(entry_threshold_ms + self._CURVE_HARD_ENTRY_EXTRA_MS)
    release_margin_ms = float(self._CURVE_RELEASE_NEAR_TARGET_MARGIN_MS)

    if raw_target_ms >= (reference_ms - release_margin_ms):
      self._reset_curve_hold()
      return reference_ms, "curve_clear"

    if not self._curve_hold_active:
      if delta_ms >= hard_entry_threshold_ms:
        self._curve_hold_active = True
        self._curve_hold_target_ms = raw_target_ms
        self._curve_hold_last_update_ms = int(now_ms)
        self._curve_entry_candidate_since_ms = 0
        self._curve_exit_candidate_since_ms = 0
        return float(self._curve_hold_target_ms), "curve_hold(hard_entry)"

      if delta_ms >= entry_threshold_ms:
        if int(self._curve_entry_candidate_since_ms) == 0:
          self._curve_entry_candidate_since_ms = int(now_ms)
        if (int(now_ms) - int(self._curve_entry_candidate_since_ms)) >= int(self._CURVE_ENTRY_PERSIST_MS):
          self._curve_hold_active = True
          self._curve_hold_target_ms = raw_target_ms
          self._curve_hold_last_update_ms = int(now_ms)
          self._curve_exit_candidate_since_ms = 0
          return float(self._curve_hold_target_ms), "curve_hold(entry)"
        return reference_ms, "curve_gate(wait)"
      self._curve_entry_candidate_since_ms = 0
      return reference_ms, "curve_gate(clear)"

    dt_s = max(0.0, (int(now_ms) - int(self._curve_hold_last_update_ms)) / 1000.0)
    self._curve_hold_last_update_ms = int(now_ms)
    self._curve_entry_candidate_since_ms = 0

    drop_deadband_ms = float(self._CURVE_HOLD_DROP_DEADBAND_MS)
    if raw_target_ms < (float(self._curve_hold_target_ms) - drop_deadband_ms):
      self._curve_hold_target_ms = float(raw_target_ms)
      self._curve_exit_candidate_since_ms = 0
    elif raw_target_ms > (float(self._curve_hold_target_ms) + exit_margin_ms):
      if int(self._curve_exit_candidate_since_ms) == 0:
        self._curve_exit_candidate_since_ms = int(now_ms)
      if (int(now_ms) - int(self._curve_exit_candidate_since_ms)) >= int(self._CURVE_EXIT_PERSIST_MS):
        recovery_ms = float(self._CURVE_EXIT_RECOVERY_MS_PER_S) * dt_s
        self._curve_hold_target_ms = min(float(raw_target_ms), float(self._curve_hold_target_ms) + recovery_ms)
    else:
      self._curve_exit_candidate_since_ms = 0

    desired_ms = min(float(self._curve_hold_target_ms), float(raw_target_ms))
    if (float(raw_target_ms) >= (float(desired_ms) - 0.05)) and (delta_ms < (0.5 * entry_threshold_ms)):
      self._reset_curve_hold()
      return float(raw_target_ms), "curve_release"

    return float(desired_ms), "curve_hold"

  def _poll_plan_and_lead(self, *, now_ns: int) -> None:
    try:
      self._sm.update(0)
    except Exception:
      return

    try:
      lp = self._sm["longitudinalPlan"]
      v_last = self._extract_plan_speed_last(lp)
      v_near = self._extract_plan_speed_near_min(lp)
      lp_mono_ns = int(self._sm.logMonoTime.get("longitudinalPlan", 0) or 0)
      if (v_last is not None) and (lp_mono_ns > 0):
        self._lp_target_last_ms = float(v_last)
        self._lp_target_near_ms = float(v_near if v_near is not None else v_last)
        self._lp_last_ns = int(lp_mono_ns)
        self._lp_has_lead = bool(getattr(lp, "hasLead", False))
        self._lp_a_target = float(getattr(lp, "aTarget", 0.0) or 0.0)
    except Exception:
      pass

    try:
      self._lead_present = False
      self._lead_drel = 0.0
      self._lead_vrel = 0.0
      if bool(self._sm.valid.get("radarState", False)):
        rs = self._sm["radarState"]
        lead_one = getattr(rs, "leadOne", None)
        if lead_one is not None and bool(getattr(lead_one, "status", False)):
          d_rel = float(getattr(lead_one, "dRel", 0.0) or 0.0)
          v_rel = float(getattr(lead_one, "vRel", 0.0) or 0.0)
          if d_rel > 0.0:
            self._lead_present = True
            self._lead_drel = d_rel
            self._lead_vrel = v_rel
    except Exception:
      pass

    try:
      if bool(self._sm.valid.get("mapdOut", False)):
        mo = self._sm["mapdOut"]
        suggested_ms = float(getattr(mo, "suggestedSpeed", 0.0) or 0.0)
        map_curve_ms = float(getattr(mo, "mapCurveSpeed", 0.0) or 0.0)
        vision_curve_ms = float(getattr(mo, "visionCurveSpeed", 0.0) or 0.0)
        mono_ns = int(self._sm.logMonoTime.get("mapdOut", 0) or 0)
        if mono_ns > 0:
          if math.isfinite(suggested_ms) and suggested_ms > 0.1:
            self._mapd_suggested_ms = suggested_ms
          if math.isfinite(map_curve_ms) and map_curve_ms > 0.1:
            self._mapd_map_curve_ms = map_curve_ms
          if math.isfinite(vision_curve_ms) and vision_curve_ms > 0.1:
            self._mapd_vision_curve_ms = vision_curve_ms
          self._mapd_last_ns = mono_ns
    except Exception:
      pass

  def _resolve_speed_limit_target_ms(self, CS, *, speed_units: str) -> tuple[Optional[float], bool, str]:
    """
    Unity-style speed-limit ownership for ACC.

    LONG/ACC own whether a positive CarState target is available. If the optional
    _tinkla wrapper exists, a false config disables it; otherwise a positive
    CarState target is sufficient.
    """
    use_speed_limit = True
    tinkla = getattr(CS, "_tinkla", None)
    if tinkla is not None:
      try:
        use_speed_limit = bool(getattr(tinkla, "adjust_acc_with_speed_limit"))
      except Exception:
        use_speed_limit = True
    if not use_speed_limit:
      return None, False, "config_disabled"

    speed_limit_target_ms = 0.0
    try:
      speed_limit_target_ms = float(CS._calc_speed_limit_target_ms(speed_units))
    except Exception:
      speed_limit_target_ms = 0.0

    if speed_limit_target_ms > 0.0:
      return float(speed_limit_target_ms), True, "carstate_speed_limit_target"
    return None, False, "none"

  def _lead_is_constraining(self, *, base_target_ms: float, v_ego_ms: float) -> bool:
    if (not self._lead_present) or float(self._lead_drel) <= 0.0:
      return False

    lead_speed_ms = max(0.0, float(v_ego_ms) + float(self._lead_vrel))
    lead_slower_than_base = lead_speed_ms < (float(base_target_ms) - 0.35)
    closing = float(self._lead_vrel) < -0.3
    near_gap_limit_m = min(80.0, max(25.0, float(v_ego_ms) * 2.2))
    near_lead = float(self._lead_drel) < float(near_gap_limit_m)
    return bool(lead_slower_than_base and (closing or near_lead))

  def _planner_drag_reasons(self, *, base_target_ms: float, planner_ms: float, v_ego_ms: float) -> list[str]:
    if float(planner_ms) <= 0.1:
      return []

    materially_below_base = float(planner_ms) < (float(base_target_ms) - float(self._PLANNER_DRAG_MARGIN_MS))
    materially_below_ego = float(planner_ms) < (float(v_ego_ms) - float(self._PLANNER_BELOW_EGO_MARGIN_MS))
    materially_below_clear = materially_below_base and materially_below_ego
    lead_constraining = self._lead_is_constraining(
      base_target_ms=float(base_target_ms),
      v_ego_ms=float(v_ego_ms),
    )
    lp_lead_constraining = bool(self._lp_has_lead) and (
      lead_constraining
      or materially_below_clear
      or (float(self._lp_a_target) <= float(self._STRONG_DECEL_ATARGET_MS2))
    )

    reasons: list[str] = []
    if lead_constraining:
      reasons.append("lead")
    if lp_lead_constraining:
      reasons.append("lp_hasLead")
    if float(self._lp_a_target) <= float(self._STRONG_DECEL_ATARGET_MS2):
      reasons.append("aTarget")
    if materially_below_clear:
      reasons.append("planner_low")
    return reasons

  def update(self, CS, *, enabled: bool, frame: int, now_ms: Optional[int] = None) -> LongDecision:
    now = _mono_ms() if now_ms is None else int(now_ms)
    now_ns = int(now) * 1_000_000

    if (int(frame) % 20) != 0:
      return LongDecision(None, "gated: 5Hz(frame)")

    controller_enabled = bool(enabled) and bool(getattr(CS, "enable_adaptive_cruise", False) or getattr(CS, "enableACC", False))
    if not controller_enabled:
      self._last_active = False
      self._enabled_since_ms = 0
      self._stable_plan_samples = 0
      self._last_lp_seen_ns = 0
      self._reset_curve_hold()
      self._reset_lead_hold()
      return LongDecision(None, "gated: not enabled/adaptive")

    stock_state = str(getattr(CS, "stock_cruise_state", "") or "")
    if stock_state not in ("ENABLED", "OVERRIDE", "STANDSTILL", "STANDBY"):
      self._last_active = False
      self._reset_curve_hold()
      self._reset_lead_hold()
      return LongDecision(None, f"gated: stock_state={stock_state or 'UNKNOWN'}")

    if not self._last_active:
      self._enabled_since_ms = int(now)
      self._stable_plan_samples = 0
      self._last_lp_seen_ns = 0
      self._reset_lead_hold()
    self._last_active = True

    cs_out = getattr(CS, "out", None)
    v_ego_ms = float(getattr(cs_out, "vEgo", 0.0) or 0.0)
    current_set_ms = float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0)
    speed_units = str(getattr(CS, "speed_units", "MPH") or "MPH")
    cruise_buttons = int(getattr(CS, "cruise_buttons", int(CruiseButtons.IDLE)) or 0)

    self._poll_plan_and_lead(now_ns=now_ns)
    lp_fresh = (
      (self._lp_target_last_ms is not None)
      and (int(self._lp_last_ns) > 0)
      and ((now_ns - int(self._lp_last_ns)) < int(self._LP_FRESH_NS))
    )

    if lp_fresh and int(self._lp_last_ns) != int(self._last_lp_seen_ns):
      self._stable_plan_samples = min(int(self._stable_plan_samples) + 1, 1000)
      self._last_lp_seen_ns = int(self._lp_last_ns)
    elif not lp_fresh:
      self._stable_plan_samples = 0
      self._last_lp_seen_ns = 0
      self._lp_has_lead = False
      self._lp_a_target = 0.0
      self._lp_target_last_ms = None
      self._lp_target_near_ms = None
      self._reset_curve_hold()
      self._reset_lead_hold()

    planner_last_ms = float(self._lp_target_last_ms) if (lp_fresh and self._lp_target_last_ms is not None) else float(current_set_ms)
    planner_near_ms = float(self._lp_target_near_ms) if (lp_fresh and self._lp_target_near_ms is not None) else float(planner_last_ms)
    desired_ms = float(planner_last_ms)
    src = "lp_last" if lp_fresh else "hold"

    startup_warmup = bool(self._enabled_since_ms and ((int(now) - int(self._enabled_since_ms)) < 2500))
    startup_invalid_clear = (
      startup_warmup
      and (not self._lead_present)
      and (
        (not lp_fresh)
        or (int(self._stable_plan_samples) < 2)
        or (float(planner_last_ms) <= 0.1)
        or (
          float(v_ego_ms) > float(self.MIN_CRUISE_SPEED_MS)
          and float(planner_last_ms) < max(float(self.MIN_CRUISE_SPEED_MS) * 0.90, float(current_set_ms) - (4.0 * CV.KPH_TO_MS))
        )
      )
    )

    speed_limit_target_ms, set_speed_limit_active, ceiling_src = self._resolve_speed_limit_target_ms(CS, speed_units=speed_units)

    if set_speed_limit_active and speed_limit_target_ms is not None:
      base_target_ms = float(speed_limit_target_ms)
      desired_ms = float(base_target_ms)
      src = f"speed_limit_target[{ceiling_src}]"

      if lp_fresh and self._lp_target_last_ms is not None:
        drag_reasons = self._planner_drag_reasons(
          base_target_ms=float(base_target_ms),
          planner_ms=float(planner_last_ms),
          v_ego_ms=float(v_ego_ms),
        )
        lead_owned = ("lead" in drag_reasons) or ("lp_hasLead" in drag_reasons)
        if lead_owned:
          desired_ms = min(float(base_target_ms), float(planner_last_ms))
          src = f"{src}+planner[{'+'.join(drag_reasons)}]"
          self._lead_hold_until_ms = int(now) + int(self._LEAD_HOLD_PERSIST_MS)
          self._reset_curve_hold()
        elif self._lead_hold_active_now(
          now_ms=int(now),
          base_target_ms=float(base_target_ms),
          planner_ms=float(planner_last_ms),
          v_ego_ms=float(v_ego_ms),
        ):
          desired_ms = min(float(base_target_ms), float(planner_last_ms))
          src = f"{src}+planner[lead_hold]"
          self._reset_curve_hold()
        elif (not self._lead_present) and (not self._lp_has_lead):
          self._reset_lead_hold()
          reference_ms = self._reference_speed_ms(
            base_target_ms=float(base_target_ms),
            current_set_ms=float(current_set_ms),
            v_ego_ms=float(v_ego_ms),
          )
          mapd_target_ms = self._mapd_curve_target_ms(now_ns=now_ns)
          gate_ms = float(self._NO_LEAD_MAPD_CURRENT_GATE_MS)
          if (
            mapd_target_ms is not None
            and float(mapd_target_ms) < (float(reference_ms) - float(self._CURVE_RELEASE_NEAR_TARGET_MARGIN_MS))
            and float(mapd_target_ms) < (float(v_ego_ms) - gate_ms)
          ):
            curve_target_ms, curve_state = self._stabilize_no_lead_curve_target(
              now_ms=int(now),
              raw_target_ms=float(mapd_target_ms),
              reference_ms=float(reference_ms),
            )
            if float(curve_target_ms) < float(base_target_ms):
              desired_ms = min(float(base_target_ms), float(curve_target_ms))
              src = f"{src}+{curve_state}[mapd]"
          else:
            self._reset_curve_hold()
        else:
          self._reset_curve_hold()
          self._reset_lead_hold()
      elif self._lead_present and (self._lead_drel < 80.0) and (self._lead_vrel < -0.5):
        lead_speed_ms = max(0.0, float(v_ego_ms) + float(self._lead_vrel))
        desired_ms = min(float(base_target_ms), max(float(self.MIN_CRUISE_SPEED_MS), float(lead_speed_ms)))
        src = f"{src}+stale_lead"
      else:
        self._reset_curve_hold()
    else:
      resume_ceiling_ms = self._resume_ceiling_ms(current_set_ms=float(current_set_ms), v_ego_ms=float(v_ego_ms))
      desired_ms = float(resume_ceiling_ms)
      src = "hold+ceiling"

      if lp_fresh:
        drag_reasons = self._planner_drag_reasons(
          base_target_ms=float(resume_ceiling_ms),
          planner_ms=float(planner_last_ms),
          v_ego_ms=float(v_ego_ms),
        )
        lead_owned = ("lead" in drag_reasons) or ("lp_hasLead" in drag_reasons)
        if lead_owned:
          desired_ms = float(planner_last_ms)
          src = "lp_last"
          self._lead_hold_until_ms = int(now) + int(self._LEAD_HOLD_PERSIST_MS)
          self._reset_curve_hold()
        elif self._lead_hold_active_now(
          now_ms=int(now),
          base_target_ms=float(resume_ceiling_ms),
          planner_ms=float(planner_last_ms),
          v_ego_ms=float(v_ego_ms),
        ):
          desired_ms = float(planner_last_ms)
          src = "lp_last[lead_hold]"
          self._reset_curve_hold()
        elif (not self._lead_present) and (not self._lp_has_lead):
          self._reset_lead_hold()
          curve_target_ms, curve_state = self._stabilize_no_lead_curve_target(
            now_ms=int(now),
            raw_target_ms=float(planner_near_ms),
            reference_ms=float(resume_ceiling_ms),
          )
          if self._maybe_release_curve_hold_from_mapd(
            now_ms=int(now),
            now_ns=now_ns,
            reference_ms=float(resume_ceiling_ms),
          ):
            curve_target_ms = float(resume_ceiling_ms)
            curve_state = "curve_clear(mapd)"
          if float(curve_target_ms) < float(self.MIN_CRUISE_SPEED_MS):
            hold_floor_ms = float(self.MIN_CRUISE_SPEED_MS) - float(self._CURVE_MIN_CRUISE_HOLD_MARGIN_MS)
            if float(curve_target_ms) >= float(hold_floor_ms):
              curve_target_ms = float(self.MIN_CRUISE_SPEED_MS)
              curve_state = f"{curve_state}+min_hold"
          desired_ms = min(float(resume_ceiling_ms), float(curve_target_ms))
          src = f"lp_near[{curve_state}]"
        else:
          self._reset_curve_hold()
          self._reset_lead_hold()
          desired_ms = float(resume_ceiling_ms)
          src = "hold+ceiling+lead_present"
      else:
        self._reset_curve_hold()
        if startup_invalid_clear:
          desired_ms = float(current_set_ms)
          src = "hold+startup_hold"
        else:
          desired_ms = float(resume_ceiling_ms)
          src = "hold+ceiling"

      if (not lp_fresh) and self._lead_present and (self._lead_drel < 80.0) and (self._lead_vrel < -0.5):
        lead_speed_ms = max(0.0, float(v_ego_ms) + float(self._lead_vrel))
        desired_ms = min(float(desired_ms), max(float(self.MIN_CRUISE_SPEED_MS), float(lead_speed_ms)))
        src = f"{src}+stale_lead"

    no_lead_curve_context = (not self._lead_present) and (not self._lp_has_lead)
    if self._should_hold_min_cruise_for_curve(
      now_ns=now_ns,
      desired_ms=float(desired_ms),
      no_lead=bool(no_lead_curve_context),
    ):
      desired_ms = max(float(desired_ms), float(self.MIN_CRUISE_SPEED_MS))
      if "min_hold" not in src:
        src = f"{src}+min_hold"

    stock_cruise_enabled = stock_state in ("ENABLED", "OVERRIDE", "STANDSTILL")
    brake_pressed = bool(getattr(cs_out, "brakePressed", False))

    decision: AccDecision = self.acc.update(
      now_ms=now,
      enabled=True,
      stock_cruise_enabled=stock_cruise_enabled,
      stock_cruise_state=stock_state,
      speed_units=speed_units,
      v_ego_ms=v_ego_ms,
      current_set_speed_ms=current_set_ms,
      desired_speed_ms=float(desired_ms),
      cruise_buttons=cruise_buttons,
      brake_pressed=brake_pressed,
      speed_limit_target_ms=speed_limit_target_ms,
      set_speed_limit_active=bool(set_speed_limit_active),
    )

    if decision.button is None or int(decision.button) == int(CruiseButtons.IDLE):
      return LongDecision(None, f"{decision.reason} src={src}")

    kph_to_u = CV.KPH_TO_MPH if speed_units == "MPH" else 1.0
    msg = (
      f"[XNOR_CRUISE_SYNC] src={src} uom={speed_units} "
      f"tgt={decision.target_kph * kph_to_u:.1f} cur={decision.current_kph * kph_to_u:.1f} "
      f"est={decision.est_kph * kph_to_u:.1f} btn={int(decision.button)} reason={decision.reason}"
    )
    self._rate_log(msg)
    return LongDecision(int(decision.button), msg)
