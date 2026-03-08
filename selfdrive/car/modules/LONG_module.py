# /data/openpilot/selfdrive/car/modules/LONG_module.py
"""
Unity-style stock cruise set-speed syncing for XNOR.

Behavior
- Planner output remains the desired speed target on both clear road and with a
  lead present.
- The live speed-limit target (+ offset) acts only as an upper bound on increases,
  so clear-road re-acceleration still works while planner-driven slowdowns (such as
  vision turns) are preserved.
- Live follow-distance and controller smoothing from patch51 remain unchanged.

Why this patch exists
- Patch51 fixed smoothness and live follow-distance, but later LONG changes removed
  Unity's "speed-limit as base desired speed" behavior.
- In recent logs, clear-road `lp_last` sat slightly below the current stock set
  speed, so ACC never saw a real up-command.
- This restores the proven upward-accel path from the earlier working patch while
  keeping the newer stability improvements.
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
      if bool(self._sm.valid.get("longitudinalPlan", False)):
        lp = self._sm["longitudinalPlan"]
        v_last = self._extract_plan_speed_last(lp)
        if v_last is not None:
          self._lp_target_ms = float(v_last)
          self._lp_last_ns = int(self._sm.logMonoTime.get("longitudinalPlan", now_ns))
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
    """Return only the live speed-limit accel ceiling, if one exists.

    Unity's ACC keeps its own persistent max cruise owner internally. XNOR patch52
    was clamping the planner target against a separate owner field in LONG before ACC
    could act, which prevented clear-road and lead-pull-away acceleration.

    LONG should pass the planner target through unchanged and only provide a speed
    limit ceiling so ACC can cap *increases* without forcing a target decrease.
    """
    tinkla = getattr(CS, "_tinkla", None)
    use_speed_limit = bool(tinkla and getattr(tinkla, "adjust_acc_with_speed_limit", False))
    if not use_speed_limit:
      return None, "none"

    try:
      speed_limit_target_ms = float(CS._calc_speed_limit_target_ms(speed_units))
    except Exception:
      speed_limit_target_ms = 0.0

    if speed_limit_target_ms > 0.0:
      return float(speed_limit_target_ms), "speed_limit_target"
    return None, "none"

  def update(self, CS, *, enabled: bool, frame: int, now_ms: Optional[int] = None) -> LongDecision:
    now = _mono_ms() if now_ms is None else int(now_ms)
    now_ns = int(now) * 1_000_000

    if (int(frame) % 20) != 0:
      return LongDecision(None, "gated: 5Hz(frame)")

    if not bool(enabled) or not bool(getattr(CS, "enable_adaptive_cruise", False)):
      return LongDecision(None, "gated: not enabled/adaptive")

    stock_state = str(getattr(CS, "stock_cruise_state", "") or "")
    if stock_state not in ("ENABLED", "OVERRIDE", "STANDSTILL", "STANDBY"):
      return LongDecision(None, f"gated: stock_state={stock_state or 'UNKNOWN'}")

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

    planner_ms = float(self._lp_target_ms) if (lp_fresh and self._lp_target_ms is not None) else float(current_set_ms)
    desired_ms = float(planner_ms)
    src = "lp_last" if lp_fresh else "hold"

    max_accel_target_ms, ceiling_src = self._resolve_accel_ceiling_ms(CS, speed_units=speed_units)
    if max_accel_target_ms is not None:
      desired_ms = min(float(desired_ms), float(max_accel_target_ms))
      src = f"{src}+cap[{ceiling_src}]"

    if (not lp_fresh) and self._lead_present and (self._lead_drel < 80.0) and (self._lead_vrel < -0.5):
      lead_speed_ms = max(0.0, float(v_ego_ms) + float(self._lead_vrel))
      desired_ms = min(float(desired_ms), max(float(self.MIN_CRUISE_SPEED_MS), float(lead_speed_ms)))
      src = f"{src}+stale_lead"

    if self._lead_present and (max_accel_target_ms is not None) and (self._lead_vrel > 0.1):
      t_follow = 1.45
      try:
        follow_distance = int(getattr(cs_out, "followDistanceS", 255))
        if follow_distance != 255:
          t_follow = 0.7 + float(follow_distance) * 0.2
      except Exception:
        pass

      headway_m = max(4.5, float(v_ego_ms) * float(t_follow) + 2.5)
      extra_gap_m = float(self._lead_drel) - float(headway_m)
      if extra_gap_m > 1.5:
        catch_up_bias_ms = min(1.2, max(0.0, 0.12 * float(extra_gap_m)))
        lead_open_target_ms = min(float(max_accel_target_ms), float(v_ego_ms) + max(float(self._lead_vrel), 0.0) + float(catch_up_bias_ms))
        if lead_open_target_ms > float(desired_ms):
          desired_ms = float(lead_open_target_ms)
          src = f"{src}+lead_open"

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
