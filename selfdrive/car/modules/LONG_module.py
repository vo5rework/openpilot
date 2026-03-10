# /data/openpilot/selfdrive/car/modules/LONG_module.py
"""
Unity-parity stock-cruise syncing for XNOR.

What this patch restores
- LONG follows the planner tail directly again, like Unity.
- The adaptive max-cruise ceiling comes from carstate's separate owner
  (`acc_speed_max_ms` / `carState.vCruise`), not the current stock set speed.
- Turn and lead behavior stay in planner/MPC; LONG does not reinterpret them.

Why this is the right XNOR adaptation
- XNOR already publishes a Unity-style adaptive max cruise ceiling in carstate.
  Recent patches were not using it, so once stock cruise stepped down behind a
  lead there was often no remembered ceiling to climb back to.
- Restoring planner-tail semantics and using the adaptive ceiling lets lead
  pull-away recovery and vision turns behave closer to Unity without adding
  more downstream heuristics.
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

  def __init__(self) -> None:
    self.acc = ACCController()
    self._sm = messaging.SubMaster(["longitudinalPlan", "radarState"])

    self._lp_target_ms: Optional[float] = None
    self._lp_last_ns: int = 0

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

      # Unity consumed the planner tail as soon as the message existed rather than
      # waiting for the valid bit. Keep the freshness guard, but do not discard
      # a fresh finite plan during startup.
      if (v_last is not None) and (lp_mono_ns > 0):
        self._lp_target_ms = float(v_last)
        self._lp_last_ns = int(lp_mono_ns)
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

  def _resolve_accel_ceiling_ms(self, CS, *, speed_units: str) -> tuple[Optional[float], str]:
    # Unity keeps a separate adaptive max cruise owner internally. XNOR carstate
    # already mirrors that as acc_speed_max_ms / carState.vCruise; use it here.
    adaptive_ceiling_ms = 0.0
    try:
      adaptive_ceiling_ms = float(getattr(CS, "acc_speed_max_ms", 0.0) or 0.0)
    except Exception:
      adaptive_ceiling_ms = 0.0

    cs_out = getattr(CS, "out", None)
    try:
      out_v_cruise_kph = float(getattr(cs_out, "vCruise", 0.0) or 0.0)
      adaptive_ceiling_ms = max(adaptive_ceiling_ms, out_v_cruise_kph * CV.KPH_TO_MS)
    except Exception:
      pass

    tinkla = getattr(CS, "_tinkla", None)
    use_speed_limit = bool(tinkla and getattr(tinkla, "adjust_acc_with_speed_limit", False))
    sl_target_ms = 0.0
    if use_speed_limit:
      try:
        sl_target_ms = float(CS._calc_speed_limit_target_ms(speed_units))
      except Exception:
        sl_target_ms = 0.0

    # Unity parity: when speed-limit matching is active, the adaptive max
    # cruise ceiling becomes the current speed-limit target directly.
    if sl_target_ms > 0.1:
      return float(sl_target_ms), "speed_limit_target"
    if adaptive_ceiling_ms > 0.1:
      return float(adaptive_ceiling_ms), "adaptive_max"
    return None, "none"

  def update(self, CS, *, enabled: bool, frame: int, now_ms: Optional[int] = None) -> LongDecision:
    now = _mono_ms() if now_ms is None else int(now_ms)
    now_ns = int(now) * 1_000_000

    if (int(frame) % 20) != 0:
      return LongDecision(None, "gated: 5Hz(frame)")

    adaptive_enabled = bool(
      getattr(CS, "enable_adaptive_cruise", False)
      or getattr(CS, "enableACC", False)
    )
    controller_enabled = bool(enabled) and adaptive_enabled
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

    planner_ms = float(self._lp_target_ms) if (lp_fresh and self._lp_target_ms is not None) else float(current_set_ms)
    desired_ms = float(planner_ms)
    src = "lp_last" if lp_fresh else "hold"

    startup_warmup = bool(self._enabled_since_ms and ((int(now) - int(self._enabled_since_ms)) < 1800))
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
    if startup_invalid_clear:
      desired_ms = float(current_set_ms)
      src = f"{src}+startup_hold"

    max_accel_target_ms, ceiling_src = self._resolve_accel_ceiling_ms(CS, speed_units=speed_units)
    if max_accel_target_ms is not None:
      desired_ms = min(float(desired_ms), float(max_accel_target_ms))
      src = f"{src}+cap[{ceiling_src}]"

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
      max_accel_target_ms=max_accel_target_ms,
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
