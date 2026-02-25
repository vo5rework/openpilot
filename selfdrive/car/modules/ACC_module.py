# /data/openpilot/selfdrive/car/modules/ACC_module.py
"""Unity-parity cruise stalk selection (XNOR).

Decides which Tesla cruise stalk button to emulate to move *stock* cruise set speed
toward a desired target speed.

Unity parity points:
  - only acts when stock cruise is ENABLED (gated by caller)
  - blocks for 3s after any human stalk interaction (extended while held)
  - blocks for 400ms after any automated press
  - uses cruise set-speed readback directly (no estimator once DBC is correct)
  - uses Tesla step sizes (1/5 and 1/1 in MPH/KPH units)
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons


def _now_ms() -> int:
  return time.monotonic_ns() // 1_000_000


def _cc_units_kph(speed_units: str) -> tuple[float, float]:
  if speed_units == "MPH":
    return 1.0 * CV.MPH_TO_KPH, 5.0 * CV.MPH_TO_KPH
  return 1.0, 5.0


@dataclass
class AccDecision:
  button: Optional[int]
  reason: str
  target_kph: float = 0.0
  current_kph: float = 0.0
  est_kph: float = 0.0


class ACCController:
  # Unity constant: Tesla cruise only functions above ~17.1 mph
  MIN_CRUISE_SPEED_MS = 17.1 * CV.MPH_TO_MS

  def __init__(self) -> None:
    self.human_action_time_ms = 0
    self.automated_action_time_ms = 0
    self.prev_cruise_buttons = int(CruiseButtons.IDLE)

  def note_human_buttons(self, cruise_buttons: int, *, now_ms: Optional[int] = None) -> None:
    """Unity: extend the 3s pause continuously while a human holds a stalk button."""
    now = _now_ms() if now_ms is None else int(now_ms)
    btn = int(cruise_buttons or 0)
    if btn not in (int(CruiseButtons.MAIN), int(CruiseButtons.IDLE)):
      self.human_action_time_ms = now
    self.prev_cruise_buttons = btn

  def _no_human_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return now_ms > int(self.human_action_time_ms) + int(milliseconds)

  def _no_automated_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return now_ms > int(self.automated_action_time_ms) + int(milliseconds)

  def update(
    self,
    *,
    now_ms: int,
    enabled: bool,
    stock_cruise_enabled: bool,
    speed_units: str,
    v_ego_ms: float,
    current_set_speed_ms: float,
    desired_speed_ms: float,
    cruise_buttons: int,
  ) -> AccDecision:
    self.note_human_buttons(cruise_buttons, now_ms=now_ms)

    if not enabled:
      return AccDecision(None, "gated: not enabled")
    if not stock_cruise_enabled:
      return AccDecision(None, "gated: stock cruise not enabled")
    if not self._no_human_action_for(now_ms=now_ms, milliseconds=3000):
      return AccDecision(None, "gated: recent human action")
    if not self._no_automated_action_for(now_ms=now_ms, milliseconds=400):
      return AccDecision(None, "gated: cooldown")
    if desired_speed_ms <= 0.1 or current_set_speed_ms <= 0.1:
      return AccDecision(None, "gated: missing target/current")

    half_kph, full_kph = _cc_units_kph(speed_units)

    target_kph = float(desired_speed_ms) * CV.MS_TO_KPH
    current_kph = float(current_set_speed_ms) * CV.MS_TO_KPH
    speed_offset_kph = target_kph - current_kph

    # Unity: do not try to adjust if below min cruise; CANCEL if target below min cruise
    if desired_speed_ms < self.MIN_CRUISE_SPEED_MS:
      self.automated_action_time_ms = now_ms
      return AccDecision(int(CruiseButtons.CANCEL), "cancel: target below min cruise", target_kph, current_kph, current_kph)

    # Unity CANCEL guard for large decel requests
    if speed_offset_kph < (-2.0 * full_kph) and current_kph > 0.0:
      self.automated_action_time_ms = now_ms
      return AccDecision(int(CruiseButtons.CANCEL), "cancel: large decel", target_kph, current_kph, current_kph)

    btn: Optional[int] = None

    # Reduce speed significantly
    if speed_offset_kph < (-0.6 * full_kph) and current_kph > 0.0:
      btn = int(CruiseButtons.DECEL_2ND)
    # Reduce slightly
    elif speed_offset_kph < (-0.9 * half_kph) and current_kph > 0.0:
      btn = int(CruiseButtons.DECEL_SET)
    # Increase speed only if car is above min cruise
    elif float(v_ego_ms) > float(self.MIN_CRUISE_SPEED_MS):
      available_kph = target_kph - current_kph
      if speed_offset_kph >= full_kph and full_kph < available_kph:
        btn = int(CruiseButtons.RES_ACCEL_2ND)
      elif speed_offset_kph >= half_kph and half_kph < available_kph:
        btn = int(CruiseButtons.RES_ACCEL)

    if btn is None:
      return AccDecision(None, "no-op", target_kph, current_kph, current_kph)

    # Unity: if trying to slow below min cruise speed, just cancel cruise (prevents SCCM crash).
    if CruiseButtons.is_decel(int(btn)) and (current_kph - 1.0) < (self.MIN_CRUISE_SPEED_MS * CV.MS_TO_KPH):
      btn = int(CruiseButtons.CANCEL)

    self.automated_action_time_ms = now_ms
    return AccDecision(int(btn), "press", target_kph, current_kph, current_kph)
