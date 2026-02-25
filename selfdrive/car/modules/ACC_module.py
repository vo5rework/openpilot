"""
/data/openpilot/selfdrive/car/modules/ACC_module.py

Unity-parity cruise stalk button selection + pacing (XNOR-friendly).

This module decides *which* Tesla cruise stalk button to emulate to move the stock
cruise SET speed toward a desired target speed (typically map/sign speed limit).

It does NOT send CAN. It outputs a CruiseButtons value (or None) plus debug fields.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons


def _now_ms() -> int:
  return time.monotonic_ns() // 1_000_000


def _get_cc_units_kph(speed_units: str) -> tuple[float, float]:
  # Unity: imperial cars adjust cruise in 1/5 mph; metric in 1/5 kph.
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
  # Unity constant: Tesla cruise only functions above ~17.1 mph.
  MIN_CRUISE_SPEED_MS = 17.1 * CV.MPH_TO_MS

  def __init__(self) -> None:
    self.human_action_time_ms = 0
    self.automated_action_time_ms = 0
    self.prev_cruise_buttons = int(CruiseButtons.IDLE)

  def note_human_buttons(self, cruise_buttons: int, *, now_ms: Optional[int] = None) -> None:
    """Unity parity: extend the 3s pause continuously while a human holds a stalk button."""
    now = _now_ms() if now_ms is None else int(now_ms)
    btn = int(cruise_buttons or 0)

    # Unity: throttle automation on any button other than MAIN/IDLE; update while held.
    if btn not in (int(CruiseButtons.MAIN), int(CruiseButtons.IDLE)):
      self.human_action_time_ms = now

    self.prev_cruise_buttons = btn

  def _no_human_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return int(now_ms) > int(self.human_action_time_ms) + int(milliseconds)

  def _no_automated_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return int(now_ms) > int(self.automated_action_time_ms) + int(milliseconds)

  def update(
    self,
    *,
    now_ms: int,
    enabled: bool,
    cruise_state: str,
    speed_units: str,
    v_ego_ms: float,
    current_set_speed_ms: float,
    desired_speed_ms: float,
    cruise_buttons: int,
  ) -> AccDecision:
    """Return the CruiseButtons int to press, or None.

    Unity parity:
      - Adjust only when cruise_state == "ENABLED"
      - Block for 3s after human action (while-held extends)
      - Block for 400ms after automated action
      - CANCEL on large negative deltas or below min cruise speed
      - SCCM crash guard: if trying to decel at min cruise speed, CANCEL instead
    """
    self.note_human_buttons(cruise_buttons, now_ms=now_ms)

    if not enabled:
      return AccDecision(None, "gated: not enabled")

    if str(cruise_state or "") != "ENABLED":
      return AccDecision(None, f"gated: cruise_state={cruise_state}")

    if not self._no_human_action_for(now_ms=now_ms, milliseconds=3000):
      return AccDecision(None, "gated: recent human action")

    if not self._no_automated_action_for(now_ms=now_ms, milliseconds=400):
      return AccDecision(None, "gated: cooldown")

    if desired_speed_ms <= 0.1 or current_set_speed_ms <= 0.1:
      return AccDecision(None, "gated: missing target/current")

    half_press_kph, full_press_kph = _get_cc_units_kph(str(speed_units or "MPH"))

    target_kph = float(desired_speed_ms) * CV.MS_TO_KPH
    current_kph = float(current_set_speed_ms) * CV.MS_TO_KPH
    speed_offset_kph = target_kph - current_kph

    btn: Optional[int] = None

    # Unity: cancel if target below min cruise speed.
    if float(desired_speed_ms) < float(self.MIN_CRUISE_SPEED_MS):
      btn = int(CruiseButtons.CANCEL)

    # Unity: cancel for large decel deltas.
    if speed_offset_kph < (-2.0 * full_press_kph) and current_kph > 0.0:
      btn = int(CruiseButtons.CANCEL)
    # Reduce speed significantly.
    elif speed_offset_kph < (-0.6 * full_press_kph) and current_kph > 0.0:
      btn = int(CruiseButtons.DECEL_2ND)
    # Reduce speed slightly.
    elif speed_offset_kph < (-0.9 * half_press_kph) and current_kph > 0.0:
      btn = int(CruiseButtons.DECEL_SET)
    # Increase speed if possible.
    elif float(v_ego_ms) > float(self.MIN_CRUISE_SPEED_MS):
      available_speed_kph = target_kph - current_kph
      if speed_offset_kph >= full_press_kph and full_press_kph < available_speed_kph:
        btn = int(CruiseButtons.RES_ACCEL_2ND)
      elif speed_offset_kph >= half_press_kph and half_press_kph < available_speed_kph:
        btn = int(CruiseButtons.RES_ACCEL)

    if btn is None:
      return AccDecision(None, "no-op", target_kph, current_kph, current_kph)

    # Unity SCCM crash guard: if trying to slow below min cruise speed, cancel instead.
    if CruiseButtons.is_decel(int(btn)) and (current_kph - 1.0) < (self.MIN_CRUISE_SPEED_MS * CV.MS_TO_KPH):
      btn = int(CruiseButtons.CANCEL)

    self.automated_action_time_ms = int(now_ms)
    return AccDecision(int(btn), "press", target_kph, current_kph, current_kph)
