# /data/openpilot/selfdrive/car/modules/LONG_module.py
"""
Unity-parity stock-cruise syncing for XNOR.

What this patch fixes
- Uses the planner directly for lead following, instead of fighting back toward
  the speed limit while a lead is present.
- Uses the planner only as a clear-road slowdown cap for vision turns.
- Removes the non-Unity lead-open speed-limit push that was causing "hunting"
  behind a lead.
- Uses a more responsive but still spike-resistant near-term curve sample so
  real turns slow down, while slight-bend noise is still filtered.

Why this is the right adaptation
- The attached logs show real turns already produce a lower planner target, but
  patch89 could still drop cruise or ignore the slowdown because LONG/ACC were
  mixing that target with speed-limit ownership too aggressively.
- Unity's older path effectively followed the planner target. On XNOR, the clean
  split is:
    * lead present  -> follow the planner directly
    * no lead       -> use planner only to cap cruise for turn slowdown
"""

from __future__ import annotations

import math
import statistics
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
  _PLAN_FOLLOW_WINDOW = 3
  _PLAN_SLOWDOWN_WINDOW = 6
  _PLAN_SLOWDOWN_MARGIN_MS = 0.75
  _PLAN_SLOWDOWN_EGO_MARGIN_MS = 0.50
  _PLAN_SLOWDOWN_STRONG_EGO_MARGIN_MS = 1.00
  _PLAN_SLOWDOWN_A_MARGIN_MS2 = -0.15

  def __init__(self) -> None:
    self.acc = ACCController()
    self._sm = messaging.SubMaster(["longitudinalPlan", "radarState"])

    self._lp_follow_ms: Optional[float] = None
    self._lp_slowdown_ms: Optional[float] = None
    self._lp_has_lead: bool = False
    self._lp_a_target: float = 0.0
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

  @classmethod
  def _extract_plan_follow_speed(cls, lp) -> Optional[float]:
    speeds = getattr(lp, "speeds", None)
    if not speeds:
      return None

    window = []
    try:
      for s in speeds[:min(int(cls._PLAN_FOLLOW_WINDOW), len(speeds))]:
        v = float(s)
        if math.isfinite(v) and v >= 0.0:
          window.append(v)
    except Exception:
      return None

    if len(window) == 0:
      return None
    return float(statistics.median(window))

  @classmethod
  def _extract_plan_slowdown_speed(cls, lp) -> Optional[float]:
    speeds = getattr(lp, "speeds", None)
    if not speeds:
      return None

    window = []
    try:
      for s in speeds[:min(int(cls._PLAN_SLOWDOWN_WINDOW), len(speeds))]:
        v = float(s)
        if math.isfinite(v) and v >= 0.0:
          window.append(v)
    except Exception:
      return None

    if len(window) == 0:
      return None
    if len(window) == 1:
      return float(window[0])

    sorted_window = sorted(window)
    return float(sorted_window[1])

  def _poll_plan_and_lead(self, *, now_ns: int) -> None:
    try:
      self._sm.update(0)
    except Exception:
      return

    try:
      if bool(self._sm.valid.get("longitudinalPlan", False)):
        lp = self._sm["longitudinalPlan"]
        v_follow = self._extract_plan_follow_speed(lp)
        v_slow = self._extract_plan_slowdown_speed(lp)
        if (v_follow is not None) or (v_slow is not None):
          self._lp_follow_ms = float(v_follow) if v_follow is not None else None
          self._lp_slowdown_ms = float(v_slow) if v_slow is not None else None
          self._lp_has_lead = bool(getattr(lp, "hasLead", False))
          self._lp_a_target = float(getattr(lp, "aTarget", 0.0) or 0.0)
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
    """Return the live speed-limit target, if one exists."""
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

    controller_enabled = bool(enabled) and bool(getattr(CS, "enable_adaptive_cruise", False))
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
      (self._lp_slowdown_ms is not None)
      and (int(self._lp_last_ns) > 0)
      and ((now_ns - int(self._lp_last_ns)) < int(self._LP_FRESH_NS))
    )

    if lp_fresh and int(self._lp_last_ns) != int(self._last_lp_seen_ns):
      self._stable_plan_samples = min(int(self._stable_plan_samples) + 1, 1000)
      self._last_lp_seen_ns = int(self._lp_last_ns)
    elif not lp_fresh:
      self._stable_plan_samples = 0
      self._last_lp_seen_ns = 0

    max_accel_target_ms, ceiling_src = self._resolve_accel_ceiling_ms(CS, speed_units=speed_units)
    base_desired_ms = float(max_accel_target_ms) if max_accel_target_ms is not None else float(current_set_ms)
    desired_ms = float(base_desired_ms)
    src = f"base[{ceiling_src}]" if max_accel_target_ms is not None else "base[set_speed]"

    planner_follow_ms = float(self._lp_follow_ms) if (lp_fresh and self._lp_follow_ms is not None) else None
    planner_slow_ms = float(self._lp_slowdown_ms) if (lp_fresh and self._lp_slowdown_ms is not None) else None
    planner_has_lead = bool(lp_fresh and (self._lp_has_lead or self._lead_present))
    planner_a_target = float(self._lp_a_target) if lp_fresh else 0.0

    startup_warmup = bool(self._enabled_since_ms and ((int(now) - int(self._enabled_since_ms)) < 1800))

    if planner_has_lead and planner_follow_ms is not None and planner_follow_ms > 0.1:
      desired_ms = min(float(base_desired_ms), float(planner_follow_ms))
      desired_ms = max(0.0, float(desired_ms))
      src = f"{src}+lp_lead"
    else:
      allow_plan_slowdown = bool(
        planner_slow_ms is not None
        and (
          not startup_warmup
          or (
            int(self._stable_plan_samples) >= 2
            and float(planner_slow_ms) > 0.1
          )
        )
      )

      if allow_plan_slowdown and planner_slow_ms is not None:
        strong_plan_slowdown = bool(
          float(planner_a_target) <= float(self._PLAN_SLOWDOWN_A_MARGIN_MS2)
          or float(planner_slow_ms) < (float(v_ego_ms) - float(self._PLAN_SLOWDOWN_STRONG_EGO_MARGIN_MS))
        )
        should_apply_plan_slowdown = bool(
          float(planner_slow_ms) < (float(base_desired_ms) - float(self._PLAN_SLOWDOWN_MARGIN_MS))
          and (
            strong_plan_slowdown
            or float(planner_slow_ms) < (float(v_ego_ms) - float(self._PLAN_SLOWDOWN_EGO_MARGIN_MS))
          )
        )
        if should_apply_plan_slowdown:
          desired_ms = max(float(self.MIN_CRUISE_SPEED_MS), float(planner_slow_ms))
          src = f"{src}+plan_slow"

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
