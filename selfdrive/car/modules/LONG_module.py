# /data/openpilot/openpilot/selfdrive/car/modules/LONG_module.py
"""Unity-style speed-limit cruise syncing (XNOR).

Responsibilities:
  - decide which Tesla cruise stalk press to emulate (if any) to sync stock cruise set speed
    toward a desired target.
  - provide Unity-like smoothness by evaluating at 5 Hz (frame % 20) and using a smooth ramp
    target from longitudinalPlan when available.

Key Unity behaviors implemented:
  - Only adjust when DI_cruiseState == ENABLED (not OVERRIDE/PRE_CANCEL/etc).
  - Auto-(re)engage cruise in STANDBY using SET(current) when speed-limit matching is active and safe
    (Unity's _should_autoengage_cc outcome: engage at current speed, not last memorized).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

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

  def __init__(self) -> None:
    self.acc = ACCController()

    self._stock_active_prev = False
    self._last_eval_frame = -100000
    self._last_gate_log_ms = 0

    self._last_brake_ms = 0
    self._smooth_target_ms: Optional[float] = None
    self._engage_override_until_ms = 0

  @staticmethod
  def _uom_half_step_ms(speed_units: str) -> float:
    half_kph = (1.0 * CV.MPH_TO_KPH) if speed_units == "MPH" else 1.0
    return float(half_kph) * CV.KPH_TO_MS
    try:
      msg = messaging.recv_one_or_none(self._long_plan_sock)
    except Exception:
      return
    if msg is None:
      return
    try:
      lp = msg.longitudinalPlan
      speeds = getattr(lp, "speeds", None)
      if speeds and len(speeds) > 0:
        v = float(speeds[-1])
        if math.isfinite(v) and v >= 0.0:
          self._long_plan_v_target_ms = v
    except Exception:
      return

  def _gate_log(self, reason: str) -> None:
    now = _mono_ms()
    if now - int(self._last_gate_log_ms) < 1000:
      return
    self._last_gate_log_ms = int(now)
    cloudlog.info(f"[XNOR_CRUISE_GATE] {reason}")

  def update(self, CS, *, enabled: bool, frame: int, now_ms: Optional[int] = None) -> LongDecision:
    now = _mono_ms() if now_ms is None else int(now_ms)

    # Track brake timing for safe auto-engage.
    try:
      if bool(getattr(getattr(CS, "out", None), "brakePressed", False)):
        self._last_brake_ms = int(now)
    except Exception:
      pass

    # Unity: extend pause while held.
    try:
      cruise_buttons = int(getattr(CS, "cruise_buttons", int(CruiseButtons.IDLE)) or 0)
      self.acc.note_human_buttons(cruise_buttons, now_ms=now)
    except Exception:
      cruise_buttons = int(CruiseButtons.IDLE)

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

    desired_ms = float(speed_limit_target_ms)

    stock_state = str(getattr(CS, "stock_cruise_state", "") or "")
    stock_enabled = (stock_state == "ENABLED")
    stock_active = (stock_state in ("ENABLED", "OVERRIDE"))
    stock_standby = (stock_state == "STANDBY")

    v_ego_ms = float(getattr(getattr(CS, "out", None), "vEgo", 0.0) or 0.0)
    current_set_ms = float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0)
    src = str(getattr(CS, "_cruise_set_src", "none") or "none")

    # Auto-(re)engage in STANDBY (Unity: RES_ACCEL) when safe.
    if enabled and stock_standby and (v_ego_ms >= self.MIN_CRUISE_SPEED_MS):
      if (now - int(self._last_brake_ms)) > 2000:
        if self.acc._no_human_action_for(now_ms=now, milliseconds=1000) and self.acc._no_automated_action_for(now_ms=now, milliseconds=400):
          cloudlog.info("[XNOR_CRUISE_SYNC] autoengage: STANDBY -> SET(current) (Unity)")
          self.acc.automated_action_time_ms = int(now)
          return LongDecision(int(CruiseButtons.DECEL_SET), "autoengage_set_current")
      return LongDecision(None, "gated: standby")

    if not stock_active:
      return LongDecision(None, f"gated: stock_state={stock_state}")

    one_u_ms = float(CV.MPH_TO_MS if speed_units == "MPH" else CV.KPH_TO_MS)

    # ENABLED edge: correct set-speed to current vEgo (both directions).
    if stock_active and (not bool(self._stock_active_prev)):
      self._engage_override_until_ms = int(now) + 1500
      try:
        self.acc.human_action_time_ms = min(int(getattr(self.acc, "human_action_time_ms", 0)), int(now) - 3001)
      except Exception:
        pass

      if (v_ego_ms >= self.MIN_CRUISE_SPEED_MS) and (abs(current_set_ms - v_ego_ms) > (0.6 * one_u_ms)):
        if self.acc._no_automated_action_for(now_ms=now, milliseconds=400):
          btn = int(CruiseButtons.DECEL_SET)
          cloudlog.info(f"[XNOR_CRUISE_SYNC] engage: SET(current) btn={btn} to converge to vEgo")
          self.acc.automated_action_time_ms = int(now)
          self._stock_active_prev = bool(stock_enabled)
          return LongDecision(btn, "engage_set_current")

    self._stock_active_prev = bool(stock_enabled)

    # Fallback for 1.5s after enable: bias desired toward vEgo (or speed limit, whichever is higher).
    if int(now) < int(self._engage_override_until_ms):
      if (v_ego_ms >= self.MIN_CRUISE_SPEED_MS) and (abs(current_set_ms - v_ego_ms) > (0.6 * one_u_ms)):
        desired_ms = float(max(speed_limit_target_ms, v_ego_ms))
        src = f"{src}|engage_to_vEgo"
      else:
        self._engage_override_until_ms = 0

    # Smooth ramp of desired target.
    if self._smooth_target_ms is None:
      base = current_set_ms if current_set_ms > 0.1 else max(v_ego_ms, float(self.MIN_CRUISE_SPEED_MS))
      self._smooth_target_ms = float(base)

    dt_s = 0.2  # 5Hz
    up_msps = 2.0 * (CV.MPH_TO_MS if speed_units == "MPH" else CV.KPH_TO_MS)
    down_msps = 3.0 * (CV.MPH_TO_MS if speed_units == "MPH" else CV.KPH_TO_MS)

    if desired_ms > float(self._smooth_target_ms):
      self._smooth_target_ms = float(min(desired_ms, float(self._smooth_target_ms) + up_msps * dt_s))
    else:
      self._smooth_target_ms = float(max(desired_ms, float(self._smooth_target_ms) - down_msps * dt_s))

    desired_ms = float(self._smooth_target_ms)

    try:
      decision: AccDecision = self.acc.update(
        now_ms=now,
        enabled=bool(enabled),
        stock_cruise_enabled=True,
        speed_units=speed_units,
        v_ego_ms=v_ego_ms,
        current_set_speed_ms=current_set_ms,
        desired_speed_ms=desired_ms,
        cruise_buttons=cruise_buttons,
      )
    except TypeError:
      decision = self.acc.update(
        now_ms=now,
        enabled=bool(enabled),
        speed_units=speed_units,
        v_ego_ms=v_ego_ms,
        current_set_speed_ms=current_set_ms,
        desired_speed_ms=desired_ms,
        cruise_buttons=cruise_buttons,
      )

    if decision.button is None or int(decision.button) == int(CruiseButtons.IDLE):
      return LongDecision(None, decision.reason)

    kph_to_u = CV.KPH_TO_MPH if speed_units == "MPH" else 1.0
    msg = (
      f"[XNOR_CRUISE_SYNC] uom={speed_units} "
      f"tgt={decision.target_kph*kph_to_u:.1f} "
      f"cur={decision.current_kph*kph_to_u:.1f} "
      f"est={decision.est_kph*kph_to_u:.1f} "
      f"btn={int(decision.button)} reason={decision.reason} src={src}"
    )
    cloudlog.info(msg)
    return LongDecision(int(decision.button), msg)
