"""
/data/openpilot/selfdrive/car/modules/ACC_module.py

Unity-parity cruise stalk decision logic (button selection + pacing).

This module decides WHICH CruiseButtons value to emulate to move stock cruise set speed
toward a desired target speed. It does NOT send CAN.

XNOR notes:
- Vehicle-side CRC/counter/bus handling stays in CarController (TeslaCAN.create_action_request + CS.msg_stw_actn_req).
- This module matches Unity's ACCController._calc_button behavior and timing gates:
  - throttle automation for 3s after any non-IDLE/NON-MAIN human cruise button
  - throttle automation for 400ms after any automated press
  - only adjust when cruise_state == "ENABLED"
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons


def _now_ms() -> int:
  return time.monotonic_ns() // 1_000_000


def get_cc_units_kph(speed_units: str) -> tuple[float, float]:
  """Unity parity: 1/5 MPH for imperial, 1/5 KPH for metric, returned in kph."""
  if speed_units == "MPH":
    return 1.0 * CV.MPH_TO_KPH, 5.0 * CV.MPH_TO_KPH
  return 1.0, 5.0


def should_be_throttled(btn: int) -> bool:
  """Unity parity: throttle any button other than MAIN/IDLE."""
  b = int(btn or 0)
  return b not in (int(CruiseButtons.MAIN), int(CruiseButtons.IDLE))


@dataclass
class AccDecision:
  button: Optional[int]
  reason: str
  target_kph: float = 0.0
  current_kph: float = 0.0


class ACCController:
  # Unity constant: Tesla cruise only functions above ~17.1 mph
  MIN_CRUISE_SPEED_MS = 17.1 * CV.MPH_TO_MS

  def __init__(self) -> None:
    self.human_cruise_action_time_ms = 0
    self.automated_cruise_action_time_ms = 0

  def note_human_buttons(self, cruise_buttons: int, *, now_ms: Optional[int] = None) -> None:
    """Unity parity: update timestamp whenever a throttled button is present."""
    now = _now_ms() if now_ms is None else int(now_ms)
    if should_be_throttled(cruise_buttons):
      self.human_cruise_action_time_ms = now

  def _no_human_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return int(now_ms) > int(self.human_cruise_action_time_ms) + int(milliseconds)

  def _no_automated_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return int(now_ms) > int(self.automated_cruise_action_time_ms) + int(milliseconds)

  def update(
    self,
    *,
    now_ms: int,
    enabled: bool,
    cruise_state: str,
    speed_units: str,
    v_ego_ms: float,
    v_cruise_actual_kph: float,
    acc_speed_kph: float,
    desired_speed_ms: float,
    cruise_buttons: int,
  ) -> AccDecision:
    """
    Unity parity (_calc_button):
      - only adjust when cruise_state == ENABLED
      - blocks for 3s after human action (any non MAIN/IDLE button)
      - blocks for 400ms after automated action
      - CANCEL when target below min cruise OR when decel is very large
    """
    self.note_human_buttons(cruise_buttons, now_ms=now_ms)

    if not enabled:
      return AccDecision(None, "gated: not enabled")

    if str(cruise_state) != "ENABLED":
      return AccDecision(None, f"gated: cruise_state={cruise_state}")

    if not self._no_human_action_for(now_ms=now_ms, milliseconds=3000):
      return AccDecision(None, "gated: recent human action")

    if not self._no_automated_action_for(now_ms=now_ms, milliseconds=400):
      return AccDecision(None, "gated: cooldown")

    if desired_speed_ms <= 0.1 or v_cruise_actual_kph <= 0.1:
      return AccDecision(None, "gated: missing target/current", desired_speed_ms * CV.MS_TO_KPH, v_cruise_actual_kph)

    half_press_kph, full_press_kph = get_cc_units_kph(speed_units)

    target_kph = float(desired_speed_ms) * CV.MS_TO_KPH
    current_kph = float(v_cruise_actual_kph)

    speed_offset_kph = target_kph - current_kph
    btn: Optional[int] = None

    if desired_speed_ms < self.MIN_CRUISE_SPEED_MS:
      btn = int(CruiseButtons.CANCEL)
    elif speed_offset_kph < (-2.0 * full_press_kph) and current_kph > 0.0:
      btn = int(CruiseButtons.CANCEL)
    elif speed_offset_kph < (-0.6 * full_press_kph) and current_kph > 0.0:
      btn = int(CruiseButtons.DECEL_2ND)
    elif speed_offset_kph < (-0.9 * half_press_kph) and current_kph > 0.0:
      btn = int(CruiseButtons.DECEL_SET)
    elif float(v_ego_ms) > float(self.MIN_CRUISE_SPEED_MS):
      available_speed_kph = float(acc_speed_kph) - current_kph
      if speed_offset_kph >= full_press_kph and full_press_kph < available_speed_kph:
        btn = int(CruiseButtons.RES_ACCEL_2ND)
      elif speed_offset_kph >= half_press_kph and half_press_kph < available_speed_kph:
        btn = int(CruiseButtons.RES_ACCEL)

    if btn is None:
      return AccDecision(None, "no-op", target_kph, current_kph)

    self.automated_cruise_action_time_ms = int(now_ms)
    return AccDecision(btn, "press", target_kph, current_kph)
