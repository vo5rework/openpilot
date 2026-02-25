"""
/data/openpilot/selfdrive/car/modules/LONG_module.py

Unity-style LONG module wrapper (XNOR-friendly).

Only responsibility: decide which Tesla cruise stalk press to emulate (if any)
to sync stock cruise set speed to a desired target.

This controller:
  - runs decisions at 5 Hz (Unity: frame % 20) for smoother stepping
  - uses longitudinalPlan.v_target (speeds[-1]) when available to ramp gently
  - caps the target by map speed limit (+ offset) via CarState helper
  - on cruise enable edge, temporarily overrides the human-pause so we can
    immediately pull set speed down to current speed (avoids "resume at last speed")
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import cereal.messaging as messaging

from openpilot.common.swaglog import cloudlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons

from openpilot.selfdrive.car.modules.ACC_module import ACCController, AccDecision


def _now_ms() -> int:
  return time.monotonic_ns() // 1_000_000


@dataclass
class LongDecision:
  button: Optional[int]
  log: str = ""


class LongController:
  """Unity-style cruise speed-limit syncing."""

  MIN_CRUISE_SPEED_MS = 17.1 * CV.MPH_TO_MS

  def __init__(self) -> None:
    self.acc = ACCController()

    self._last_eval_ms = 0
    self._stock_enabled_prev = False
    self._engage_override_until_ms = 0

    self._long_plan_sock = None
    self._long_plan_v_target_ms = 0.0
    try:
      self._long_plan_sock = messaging.sub_sock("longitudinalPlan", conflate=True)
    except Exception:
      self._long_plan_sock = None

  @staticmethod
  def _uom_half_step_ms(speed_units: str) -> float:
    half_kph = (1.0 * CV.MPH_TO_KPH) if speed_units == "MPH" else 1.0
    return float(half_kph) * CV.KPH_TO_MS

  def _poll_long_plan(self) -> None:
    if self._long_plan_sock is None:
      return
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

  def update(self, CS, *, enabled: bool, now_ms: Optional[int] = None) -> LongDecision:
    now = _now_ms() if now_ms is None else int(now_ms)

    # Always track human stalk actions (Unity: extend pause while held).
    try:
      cruise_buttons = int(getattr(CS, "cruise_buttons", int(CruiseButtons.IDLE)) or 0)
      self.acc.note_human_buttons(cruise_buttons, now_ms=now)
    except Exception:
      cruise_buttons = int(CruiseButtons.IDLE)

    # Run control at 5 Hz (Unity: frame % 20).
    if (now - int(self._last_eval_ms)) < 200:
      return LongDecision(None, "gated: 5Hz")
    self._last_eval_ms = int(now)

    tinkla = getattr(CS, "_tinkla", None)
    if not bool(tinkla and getattr(tinkla, "adjust_acc_with_speed_limit", False)):
      return LongDecision(None, "gated: adjust_acc_with_speed_limit false")

    speed_units = str(getattr(CS, "speed_units", "MPH") or "MPH")

    # Map speed limit (+offset) from CarState helper.
    try:
      speed_limit_target_ms = float(CS._calc_speed_limit_target_ms(speed_units))
    except Exception:
      speed_limit_target_ms = 0.0
    if speed_limit_target_ms <= 0.0:
      return LongDecision(None, "gated: no speed limit")

    # Use longitudinalPlan.v_target (smooth ramp), capped by speed limit.
    self._poll_long_plan()
    desired_ms = float(speed_limit_target_ms)
    if float(self._long_plan_v_target_ms) > 0.1:
      desired_ms = float(min(desired_ms, float(self._long_plan_v_target_ms)))

    # Unity parity: only adjust when Tesla reports cruise ENABLED.
    stock_state = str(getattr(CS, "stock_cruise_state", "") or "")
    stock_enabled = (stock_state == "ENABLED")

    # Detect enable edge: immediately prevent "resume at last set speed".
    if stock_enabled and (not bool(self._stock_enabled_prev)):
      self._engage_override_until_ms = int(now) + 1500
      # Bypass the 3s human pause for this short correction window.
      try:
        self.acc.human_action_time_ms = min(int(getattr(self.acc, "human_action_time_ms", 0)), int(now) - 3001)
      except Exception:
        pass
      # Allow immediate evaluation next tick.
      self._last_eval_ms = int(now) - 200
    self._stock_enabled_prev = bool(stock_enabled)

    # Cruise set-speed readback (m/s)
    current_set_ms = float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0)
    src = str(getattr(CS, "_cruise_set_src", "none") or "none")
    if stock_enabled and src.startswith("unknown"):
      return LongDecision(None, "gated: cruise set src unknown")

    v_ego_ms = float(getattr(getattr(CS, "out", None), "vEgo", 0.0) or 0.0)

    # Engage override: pull set speed down to current speed (capped by limit).
    if stock_enabled and int(now) < int(self._engage_override_until_ms):
      half_step_ms = self._uom_half_step_ms(speed_units)
      if current_set_ms <= max(0.0, v_ego_ms + 0.6 * half_step_ms):
        self._engage_override_until_ms = 0
      else:
        desired_ms = float(min(speed_limit_target_ms, max(v_ego_ms, float(self.MIN_CRUISE_SPEED_MS))))
        src = f"{src}|engage_to_vEgo"

    decision: AccDecision = self.acc.update(
      now_ms=now,
      enabled=bool(enabled),
      stock_cruise_enabled=bool(stock_enabled),
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
