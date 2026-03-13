# /data/openpilot/selfdrive/car/modules/LONG_module.py
"""
Unity-parity stock cruise syncing for XNOR.

Patch118 keeps the Patch116 speed-limit ownership split and restores
lead-resume behaviour:
- clear-road desired speed comes from the live CarState speed-limit target
- planner tail can still drag that target down for genuine slowdowns
- mere lead presence no longer latches the planner as the owner once the lead
  stops materially constraining speed
- ACC keeps ownership of the adaptive cruise ceiling internally

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
  _PLANNER_DRAG_MARGIN_MS = 0.6
  _PLANNER_BELOW_EGO_MARGIN_MS = 0.4
  _STRONG_DECEL_ATARGET_MS2 = -0.5

  def __init__(self) -> None:
    self.acc = ACCController()
    self._sm = messaging.SubMaster(["longitudinalPlan", "radarState"])

    self._lp_target_ms: Optional[float] = None
    self._lp_last_ns: int = 0
    self._lp_has_lead: bool = False
    self._lp_a_target: float = 0.0

    self._lead_present: bool = False
    self._lead_drel: float = 0.0
    self._lead_vrel: float = 0.0

    self._last_info_log_ms: int = 0
    self._enabled_since_ms: int = 0
    self._last_active: bool = False
    self._last_lp_seen_ns: int = 0
    self._stable_plan_samples: int = 0

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

  def _poll_plan_and_lead(self, *, now_ns: int) -> None:
    try:
      self._sm.update(0)
    except Exception:
      return

    try:
      lp = self._sm["longitudinalPlan"]
      v_last = self._extract_plan_speed_last(lp)
      lp_mono_ns = int(self._sm.logMonoTime.get("longitudinalPlan", 0) or 0)
      if (v_last is not None) and (lp_mono_ns > 0):
        self._lp_target_ms = float(v_last)
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
      return LongDecision(None, "gated: not enabled/adaptive")

    stock_state = str(getattr(CS, "stock_cruise_state", "") or "")
    if stock_state not in ("ENABLED", "OVERRIDE", "STANDSTILL", "STANDBY"):
      self._last_active = False
      return LongDecision(None, f"gated: stock_state={stock_state or 'UNKNOWN'}")

    if not self._last_active:
      self._enabled_since_ms = int(now)
      self._stable_plan_samples = 0
      self._last_lp_seen_ns = 0
    self._last_active = True

    cs_out = getattr(CS, "out", None)
    v_ego_ms = float(getattr(cs_out, "vEgo", 0.0) or 0.0)
    current_set_ms = float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0)
    speed_units = str(getattr(CS, "speed_units", "MPH") or "MPH")
    cruise_buttons = int(getattr(CS, "cruise_buttons", int(CruiseButtons.IDLE)) or 0)

    self._poll_plan_and_lead(now_ns=now_ns)
    lp_fresh = (
      (self._lp_target_ms is not None)
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

    planner_ms = float(self._lp_target_ms) if (lp_fresh and self._lp_target_ms is not None) else float(current_set_ms)
    desired_ms = float(planner_ms)
    src = "lp_last" if lp_fresh else "hold"

    startup_warmup = bool(self._enabled_since_ms and ((int(now) - int(self._enabled_since_ms)) < 2500))
    startup_invalid_clear = (
      startup_warmup
      and (not self._lead_present)
      and (
        (not lp_fresh)
        or (int(self._stable_plan_samples) < 2)
        or (float(planner_ms) <= 0.1)
        or (
          float(v_ego_ms) > float(self.MIN_CRUISE_SPEED_MS)
          and float(planner_ms) < max(float(self.MIN_CRUISE_SPEED_MS) * 0.90, float(current_set_ms) - (4.0 * CV.KPH_TO_MS))
        )
      )
    )

    speed_limit_target_ms, set_speed_limit_active, ceiling_src = self._resolve_speed_limit_target_ms(CS, speed_units=speed_units)

    if set_speed_limit_active and speed_limit_target_ms is not None:
      base_target_ms = float(speed_limit_target_ms)
      desired_ms = float(base_target_ms)
      src = f"speed_limit_target[{ceiling_src}]"

      if lp_fresh and self._lp_target_ms is not None:
        drag_reasons = self._planner_drag_reasons(
          base_target_ms=float(base_target_ms),
          planner_ms=float(self._lp_target_ms),
          v_ego_ms=float(v_ego_ms),
        )
        if drag_reasons:
          desired_ms = min(float(base_target_ms), float(self._lp_target_ms))
          src = f"{src}+planner[{'+'.join(drag_reasons)}]"
      elif self._lead_present and (self._lead_drel < 80.0) and (self._lead_vrel < -0.5):
        lead_speed_ms = max(0.0, float(v_ego_ms) + float(self._lead_vrel))
        desired_ms = min(float(base_target_ms), max(float(self.MIN_CRUISE_SPEED_MS), float(lead_speed_ms)))
        src = f"{src}+stale_lead"
    else:
      if startup_invalid_clear:
        desired_ms = float(current_set_ms)
        src = f"{src}+startup_hold"

      if (not lp_fresh) and self._lead_present and (self._lead_drel < 80.0) and (self._lead_vrel < -0.5):
        lead_speed_ms = max(0.0, float(v_ego_ms) + float(self._lead_vrel))
        desired_ms = min(float(desired_ms), max(float(self.MIN_CRUISE_SPEED_MS), float(lead_speed_ms)))
        src = f"{src}+stale_lead"

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
