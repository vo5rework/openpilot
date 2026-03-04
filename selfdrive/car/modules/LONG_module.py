# /data/openpilot/selfdrive/car/modules/LONG_module.py
"""
Unity-style stock cruise set-speed syncing (XNOR).

This module decides which Tesla cruise stalk button to emulate (if any) to move
stock cruise set speed toward a desired target speed.

Design goals
- Regression-safe: only uses speed-limit target when longitudinalPlan is missing.
- Lead-safe: when longitudinalPlan is available, clamp desired speed to the plan's
  near-horizon minimum (anticipates braking for a slowing lead).
- Smooth: evaluates at 5 Hz and ramps desired target to avoid oscillation.

Inputs (via CarState)
- CS._calc_speed_limit_target_ms(speed_units): speed-limit (+offset) target
- CS.enable_adaptive_cruise: double-tap gating
- CS.stock_cruise_state / CS.stock_cruise_set_speed_ms / CS.cruise_buttons
- CS.out.vEgo / CS.out.brakePressed / CS.out.gasPressed

Outputs
- LongDecision(button): CruiseButtons value to emulate once (pulse).
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
  # Unity constant: Tesla cruise only functions above ~17.1 mph
  MIN_CRUISE_SPEED_MS = 17.1 * CV.MPH_TO_MS

  def __init__(self) -> None:
    self.acc = ACCController()

    self._smooth_target_ms: Optional[float] = None
    self._last_gate_log_ms = 0

    # Plan/lead polling (conflated). Use SubMaster to avoid missing messages at 5 Hz.
    self._sm = messaging.SubMaster(["longitudinalPlan", "radarState"])
    self._lp_target_ms: Optional[float] = None
    self._lp_last_ns: int = 0
    self._lead_present: bool = False
    self._lead_vrel: float = 0.0
    self._lead_drel: float = 0.0

  def _gate_log(self, reason: str) -> None:
    now = _mono_ms()
    if now - int(self._last_gate_log_ms) < 1000:
      return
  def _poll_plan_and_lead(self, *, now_ns: int) -> None:
    """Cache near-horizon min speed from longitudinalPlan and basic lead state from radarState."""
    try:
      self._sm.update(0)
    except Exception:
      return

    try:
      if bool(self._sm.valid.get("longitudinalPlan", False)):
        lp = self._sm["longitudinalPlan"]
        speeds = getattr(lp, "speeds", None)
        if speeds:
          n = min(12, len(speeds))
          v = float(min(float(x) for x in speeds[:n]))
          if math.isfinite(v) and v >= 0.0:
            self._lp_target_ms = v
            self._lp_last_ns = int(self._sm.logMonoTime.get("longitudinalPlan", now_ns))
    except Exception:
      pass

    try:
      self._lead_present = False
      self._lead_vrel = 0.0
      self._lead_drel = 0.0
      if bool(self._sm.valid.get("radarState", False)):
        rs = self._sm["radarState"]
        lead = getattr(rs, "leadOne", None)
        if lead is not None and bool(getattr(lead, "status", False)):
          self._lead_present = True
          self._lead_vrel = float(getattr(lead, "vRel", 0.0) or 0.0)
          self._lead_drel = float(getattr(lead, "dRel", 0.0) or 0.0)
    except Exception:
      pass
      return

  @staticmethod
  def _uom_step_ms(speed_units: str) -> float:
    return float(CV.MPH_TO_MS if speed_units == "MPH" else CV.KPH_TO_MS)

  def update(self, CS, *, enabled: bool, frame: int, now_ms: Optional[int] = None) -> LongDecision:
    now = _mono_ms() if now_ms is None else int(now_ms)
    now_ns = int(now) * 1_000_000

    # 5 Hz eval (Unity: frame % 20 on a 100 Hz loop).
    if (int(frame) % 20) != 0:
      return LongDecision(None, "gated: 5Hz(frame)")

    tinkla = getattr(CS, "_tinkla", None)
    if not bool(tinkla and getattr(tinkla, "adjust_acc_with_speed_limit", False)):
      self._gate_log("adjust_acc_with_speed_limit false")
      return LongDecision(None, "gated: adjust_acc_with_speed_limit false")

    # Unity parity: speed matching only when adaptive is enabled (double-pull).
    if not bool(getattr(CS, "enable_adaptive_cruise", False)):
      return LongDecision(None, "gated: adaptive disabled")

    speed_units = str(getattr(CS, "speed_units", "MPH") or "MPH")

    # Speed limit (+offset) from CarState helper.
    try:
      speed_limit_target_ms = float(CS._calc_speed_limit_target_ms(speed_units))
    except Exception:
      speed_limit_target_ms = 0.0
    if speed_limit_target_ms <= 0.0:
      self._gate_log("no speed limit")
      return LongDecision(None, "gated: no speed limit")

    # Stock cruise state gates (Unity-like).
    stock_state = str(getattr(CS, "stock_cruise_state", "") or "")
    stock_active = stock_state in ("ENABLED", "OVERRIDE")
    if not stock_active:
      return LongDecision(None, f"gated: stock_state={stock_state or 'UNKNOWN'}")

    cs_out = getattr(CS, "out", None)
    v_ego_ms = float(getattr(cs_out, "vEgo", 0.0) or 0.0)
    current_set_ms = float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0)

    try:
      cruise_buttons = int(getattr(CS, "cruise_buttons", int(CruiseButtons.IDLE)) or 0)
    except Exception:
      cruise_buttons = int(CruiseButtons.IDLE)

    # Poll longitudinalPlan and clamp desired target to it (lead-safe).
    self._poll_plan_and_lead(now_ns=now_ns)
    lp_fresh = (self._lp_target_ms is not None) and (int(self._lp_last_ns) > 0) and ((now_ns - int(self._lp_last_ns)) < 1_500_000_000)

    desired_ms = float(speed_limit_target_ms)
    src = "sl"
    if lp_fresh:
      desired_ms = float(min(desired_ms, float(self._lp_target_ms or desired_ms)))
      src = "sl+lp"

    if (not lp_fresh) and bool(getattr(self, "_lead_present", False)):
      # Safety: if we can't see longitudinalPlan but radar reports a lead, don't increase set speed.
      desired_ms = float(min(desired_ms, current_set_ms))
      src = "sl+lead_hold"

    # Unity parity: if the planner ceiling is limited by stock set speed, we still need to
    # speed back up with the lead as the gap opens (and slow down if we close) while a lead exists.
    # This logic is intentionally conservative and only applies when radar reports a valid lead.
    if bool(getattr(self, "_lead_present", False)) and (v_ego_ms > 0.1):
      try:
        fd = int(getattr(cs_out, "followDistanceS", 255) or 255)
      except Exception:
        fd = 255
      if not (0 <= fd <= 6):
        fd = 3  # mid default when unknown
      t_follow = 0.7 + (0.2 * float(fd))
      desired_gap_m = max(5.0, float(v_ego_ms) * t_follow)

      lead_speed_ms = max(0.0, float(v_ego_ms) + float(getattr(self, "_lead_vrel", 0.0) or 0.0))
      lead_drel = float(getattr(self, "_lead_drel", 0.0) or 0.0)
      lead_vrel = float(getattr(self, "_lead_vrel", 0.0) or 0.0)

      # Only allow speeding up with a lead if the planner is not asking us to slow down already.
      planner_ok_to_accel = (not lp_fresh) or (float(self._lp_target_ms or 0.0) >= (float(v_ego_ms) - 0.25))

      # Lead pulling away and gap comfortably above target: raise toward lead speed (never above limit).
      if planner_ok_to_accel and (lead_vrel > 0.30) and (lead_drel > (desired_gap_m + 6.0)):
        pull_target_ms = float(min(float(speed_limit_target_ms), float(lead_speed_ms)))
        if pull_target_ms > desired_ms:
          desired_ms = pull_target_ms
          src = f"{src}+lead_pull"

      # Lead slowing and gap below target: lower toward lead speed (never below min cruise).
      if (lead_vrel < -0.30) and (lead_drel < (desired_gap_m + 2.0)):
        brake_target_ms = float(max(float(self.MIN_CRUISE_SPEED_MS), float(lead_speed_ms)))
        if brake_target_ms < desired_ms:
          desired_ms = brake_target_ms
          src = f"{src}+lead_brake"
    # When longitudinalPlan is fresh, bypass smoothing for responsive lead handling.
    if lp_fresh:
      self._smooth_target_ms = float(desired_ms)
    else:
      # Smooth ramp for speed-limit target when longitudinalPlan is stale.
      if self._smooth_target_ms is None or not math.isfinite(self._smooth_target_ms):
        base = current_set_ms if current_set_ms > 0.1 else max(v_ego_ms, float(self.MIN_CRUISE_SPEED_MS))
        self._smooth_target_ms = float(min(speed_limit_target_ms, max(base, float(self.MIN_CRUISE_SPEED_MS))))

      dt_s = 0.2  # 5 Hz
      step_ms = self._uom_step_ms(speed_units)
      # Faster convergence than before; ACCController already rate-limits pulses.
      up_msps = 5.0 * step_ms
      down_msps = 6.0 * step_ms

      if desired_ms > float(self._smooth_target_ms):
        self._smooth_target_ms = float(min(desired_ms, float(self._smooth_target_ms) + up_msps * dt_s))
      else:
        # For decreases, converge quickly to avoid closing on a slowing lead when plan is unavailable.
        self._smooth_target_ms = float(max(desired_ms, float(self._smooth_target_ms) - down_msps * dt_s))

    desired_ms = float(self._smooth_target_ms if self._smooth_target_ms is not None else desired_ms)
    # Run ACC decision logic.
    try:
      decision: AccDecision = self.acc.update(
        now_ms=now,
        enabled=bool(enabled),
        stock_cruise_enabled=True,
        stock_cruise_state=stock_state,
        speed_units=speed_units,
        v_ego_ms=v_ego_ms,
        current_set_speed_ms=current_set_ms,
        desired_speed_ms=desired_ms,
        cruise_buttons=cruise_buttons,
      )
    except TypeError:
      # Back-compat with older ACCController signature.
      decision = self.acc.update(
        now_ms=now,
        enabled=bool(enabled),
        stock_cruise_enabled=True,
        speed_units=speed_units,
        v_ego_ms=v_ego_ms,
        current_set_speed_ms=current_set_ms,
        desired_speed_ms=desired_ms,
        cruise_buttons=cruise_buttons,
      )

    if decision.button is None or int(decision.button) == int(CruiseButtons.IDLE):
      return LongDecision(None, f"{decision.reason} src={src}")

    kph_to_u = CV.KPH_TO_MPH if speed_units == "MPH" else 1.0
    msg = (
      f"[XNOR_CRUISE_SYNC] src={src} uom={speed_units} "
      f"tgt={decision.target_kph*kph_to_u:.1f} "
      f"cur={decision.current_kph*kph_to_u:.1f} "
      f"est={decision.est_kph*kph_to_u:.1f} "
      f"btn={int(decision.button)} reason={decision.reason}"
    )
    cloudlog.info(msg)
    return LongDecision(int(decision.button), msg)