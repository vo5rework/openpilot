# /data/openpilot/selfdrive/car/modules/ACC_module.py
"""
Unity-parity Tesla cruise stalk button selection for XNOR.

This restores the key Unity separation that was missing in the live XNOR path:
- LONG provides the planner target as the desired speed.
- ACC owns the adaptive cruise ceiling internally (`acc_speed_kph`).
- Speed-limit changes raise that ceiling, but automated slowdowns do not lower it.

The result should be closer to Unity's behavior:
- step down toward the planner target when needed
- keep a separate ceiling available for later re-acceleration
- avoid the repeated `src=lp_last` -> lower set speed -> no recovery loop seen in the swaglog
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from cereal import messaging
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons


def _now_ms() -> int:
  return time.monotonic_ns() // 1_000_000


def _cc_units_kph(speed_units: str) -> tuple[float, float]:
  if speed_units == "MPH":
    return 1.0 * CV.MPH_TO_KPH, 5.0 * CV.MPH_TO_KPH
  return 1.0, 5.0


@dataclass
class LeadInfo:
  status: bool = False
  d_rel: float = 0.0
  v_rel: float = 0.0


@dataclass
class AccDecision:
  button: Optional[int]
  reason: str
  target_kph: float = 0.0
  current_kph: float = 0.0
  est_kph: float = 0.0


class ACCController:
  MIN_CRUISE_SPEED_MS = 17.1 * CV.MPH_TO_MS

  _HUMAN_COOLDOWN_MS = 3000
  _AUTO_COOLDOWN_MS = 400
  _AUTO_COOLDOWN_ACCEL_MS = 250
  _FAST_DECEL_RESUME_HOLDOFF_MS = 2000
  _LEAD_FRESH_MS = 700
  _AUTOENGAGE_SPEED_WINDOW_MS = 0.8
  _AUTO_ECHO_IGNORE_MS = 750

  def __init__(self) -> None:
    self.human_action_time_ms = 0
    self.automated_action_time_ms = 0
    self.prev_cruise_buttons = int(CruiseButtons.IDLE)

    self._last_auto_button = int(CruiseButtons.IDLE)
    self._last_auto_button_time_ms = 0

    self.fast_decel_time_ms = 0
    self.lead_last_seen_time_ms = 0
    self.acc_speed_kph = 0.0
    self.speed_limit_kph = 0.0
    self.prev_speed_limit_kph = 0.0

    self._radar_sm = messaging.SubMaster(["radarState"])

  @staticmethod
  def _button_direction(button: int) -> int:
    if CruiseButtons.is_accel(button):
      return 1
    if CruiseButtons.is_decel(button) or int(button) == int(CruiseButtons.CANCEL):
      return -1
    return 0

  def _update_max_acc_speed_from_button(self, *, button: int, speed_units: str) -> None:
    half_press_kph, full_press_kph = _cc_units_kph(speed_units)
    speed_change_map = {
      int(CruiseButtons.RES_ACCEL): float(half_press_kph),
      int(CruiseButtons.RES_ACCEL_2ND): float(full_press_kph),
      int(CruiseButtons.DECEL_SET): -1.0 * float(half_press_kph),
      int(CruiseButtons.DECEL_2ND): -1.0 * float(full_press_kph),
    }
    self.acc_speed_kph += float(speed_change_map.get(int(button), 0.0))
    self.acc_speed_kph = min(float(self.acc_speed_kph), 170.0)
    self.acc_speed_kph = max(float(self.acc_speed_kph), 0.0)

  def note_human_buttons(self, cruise_buttons: int, *, speed_units: str, now_ms: Optional[int] = None) -> None:
    now = _now_ms() if now_ms is None else int(now_ms)
    btn = int(cruise_buttons or 0)

    if btn == int(self.prev_cruise_buttons):
      return

    # Ignore recent echoes of our own virtual pulse so ACC does not throttle itself
    # as "human input" on the readback edge.
    recent_auto_echo = (
      btn not in (int(CruiseButtons.IDLE), int(CruiseButtons.MAIN))
      and btn == int(self._last_auto_button)
      and (now - int(self._last_auto_button_time_ms)) <= int(self._AUTO_ECHO_IGNORE_MS)
    )

    if btn not in (int(CruiseButtons.IDLE), int(CruiseButtons.MAIN)) and not recent_auto_echo:
      self.human_action_time_ms = now
      self._update_max_acc_speed_from_button(button=btn, speed_units=speed_units)

    self.prev_cruise_buttons = btn

  def _no_human_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return int(now_ms) >= int(self.human_action_time_ms) + int(milliseconds)

  def _no_automated_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return int(now_ms) >= int(self.automated_action_time_ms) + int(milliseconds)

  def _poll_lead(self, *, now_ms: int) -> LeadInfo:
    try:
      self._radar_sm.update(0)
    except Exception:
      return LeadInfo()

    try:
      if not bool(self._radar_sm.valid.get("radarState", False)):
        return LeadInfo()

      rs = self._radar_sm["radarState"]
      lead = getattr(rs, "leadOne", None)
      if lead is None or not bool(getattr(lead, "status", False)):
        return LeadInfo()

      d_rel = float(getattr(lead, "dRel", 0.0) or 0.0)
      v_rel = float(getattr(lead, "vRel", 0.0) or 0.0)
      if d_rel <= 0.0:
        return LeadInfo()

      self.lead_last_seen_time_ms = int(now_ms)
      return LeadInfo(True, d_rel, v_rel)
    except Exception:
      return LeadInfo()

  @staticmethod
  def _seconds_to_collision(*, lead: LeadInfo) -> float:
    if not lead.status or lead.d_rel <= 0.0 or lead.v_rel >= 0.0:
      return 1e6
    return float(lead.d_rel) / max(1e-3, -float(lead.v_rel))

  def _fast_decel_required(self, *, v_ego_ms: float, lead: LeadInfo) -> bool:
    if not lead.status or lead.d_rel <= 0.0:
      return False

    collision_imminent = self._seconds_to_collision(lead=lead) < 4.0
    lead_absolute_speed_ms = float(v_ego_ms) + float(lead.v_rel)
    lead_too_slow = lead_absolute_speed_ms < float(self.MIN_CRUISE_SPEED_MS)
    return bool(collision_imminent or lead_too_slow)

  def _should_autoengage_cc(
    self,
    *,
    now_ms: int,
    v_ego_ms: float,
    brake_pressed: bool,
    lead: LeadInfo,
  ) -> bool:
    if float(v_ego_ms) < float(self.MIN_CRUISE_SPEED_MS):
      return False
    if bool(brake_pressed):
      return False
    if int(now_ms) <= int(self.fast_decel_time_ms) + int(self._FAST_DECEL_RESUME_HOLDOFF_MS):
      return False

    recent_lead = lead.status or ((int(now_ms) - int(self.lead_last_seen_time_ms)) < int(self._LEAD_FRESH_MS))
    if recent_lead and lead.status:
      if self._fast_decel_required(v_ego_ms=v_ego_ms, lead=lead):
        return False
      materially_closing = (lead.d_rel > 0.0) and (lead.v_rel < -1.2)
      if materially_closing:
        return False

    return True

  def _record_button(self, *, now_ms: int, button: int) -> None:
    self.automated_action_time_ms = int(now_ms)
    self._last_auto_button = int(button)
    self._last_auto_button_time_ms = int(now_ms)
    if int(button) == int(CruiseButtons.CANCEL):
      self.fast_decel_time_ms = int(now_ms)

  def update(
    self,
    *,
    now_ms: int,
    enabled: bool,
    stock_cruise_enabled: bool,
    stock_cruise_state: str,
    speed_units: str,
    v_ego_ms: float,
    current_set_speed_ms: float,
    desired_speed_ms: float,
    cruise_buttons: int,
    brake_pressed: bool = False,
    speed_limit_target_ms: Optional[float] = None,
    set_speed_limit_active: bool = False,
  ) -> AccDecision:
    self.note_human_buttons(cruise_buttons, speed_units=speed_units, now_ms=now_ms)
    lead = self._poll_lead(now_ms=now_ms)

    self.prev_speed_limit_kph = float(self.speed_limit_kph)
    if bool(set_speed_limit_active) and speed_limit_target_ms is not None and float(speed_limit_target_ms) > 0.0:
      self.speed_limit_kph = float(speed_limit_target_ms) * CV.MS_TO_KPH
      if int(self.prev_speed_limit_kph) != int(self.speed_limit_kph):
        self.acc_speed_kph = float(self.speed_limit_kph)
    else:
      self.speed_limit_kph = 0.0

    if not enabled:
      return AccDecision(None, "gated: not enabled")

    current_kph = float(current_set_speed_ms) * CV.MS_TO_KPH
    if self.acc_speed_kph <= 0.0:
      self.acc_speed_kph = max(float(current_kph), float(v_ego_ms) * CV.MS_TO_KPH, float(self.speed_limit_kph))
    else:
      self.acc_speed_kph = max(float(self.acc_speed_kph), float(current_kph))

    stock_state = str(stock_cruise_state or "").upper()
    half_kph, full_kph = _cc_units_kph(speed_units)

    if stock_state == "STANDBY":
      if (
        float(desired_speed_ms) >= float(v_ego_ms) - float(self._AUTOENGAGE_SPEED_WINDOW_MS)
        and self._no_human_action_for(now_ms=now_ms, milliseconds=self._HUMAN_COOLDOWN_MS)
        and self._no_automated_action_for(now_ms=now_ms, milliseconds=self._AUTO_COOLDOWN_MS)
        and self._should_autoengage_cc(now_ms=now_ms, v_ego_ms=v_ego_ms, brake_pressed=brake_pressed, lead=lead)
      ):
        self._record_button(now_ms=now_ms, button=int(CruiseButtons.RES_ACCEL))
        return AccDecision(int(CruiseButtons.RES_ACCEL), "autoengage: RES", current_kph, current_kph, current_kph + float(half_kph))
      return AccDecision(None, "standby: no autoengage", current_kph, current_kph, current_kph)

    if stock_state not in ("ENABLED", "OVERRIDE", "STANDSTILL"):
      return AccDecision(None, f"gated: stock_state={stock_state or 'UNKNOWN'}", current_kph, current_kph, current_kph)

    if not stock_cruise_enabled:
      return AccDecision(None, "gated: stock cruise not enabled", current_kph, current_kph, current_kph)

    if not self._no_human_action_for(now_ms=now_ms, milliseconds=self._HUMAN_COOLDOWN_MS):
      return AccDecision(None, "gated: recent human action", current_kph, current_kph, current_kph)

    if not self._no_automated_action_for(now_ms=now_ms, milliseconds=self._AUTO_COOLDOWN_MS):
      return AccDecision(None, "gated: cooldown", current_kph, current_kph, current_kph)

    if float(desired_speed_ms) <= 0.1 or float(current_set_speed_ms) <= 0.1:
      return AccDecision(None, "gated: missing target/current", current_kph, current_kph, current_kph)

    target_kph = float(desired_speed_ms) * CV.MS_TO_KPH
    speed_offset_kph = float(target_kph) - float(current_kph)
    available_speed_kph = max(0.0, float(self.acc_speed_kph) - float(current_kph))

    button: Optional[int] = None

    # Keep emergency/below-min handling narrow. Unity primarily steps the set speed
    # down; it does not use XNOR's earlier broad engaged-control cancel rules.
    fast_decel_required = self._fast_decel_required(v_ego_ms=v_ego_ms, lead=lead)
    if float(desired_speed_ms) < float(self.MIN_CRUISE_SPEED_MS):
      button = int(CruiseButtons.CANCEL)
    elif fast_decel_required and self._seconds_to_collision(lead=lead) < 2.5 and float(current_kph) > 0.0:
      button = int(CruiseButtons.CANCEL)
    elif speed_offset_kph < (-2.0 * float(full_kph)) and current_kph > 0.0:
      button = int(CruiseButtons.CANCEL)
    elif speed_offset_kph < (-0.6 * float(full_kph)) and current_kph > 0.0:
      button = int(CruiseButtons.DECEL_2ND)
    elif speed_offset_kph < (-0.9 * float(half_kph)) and current_kph > 0.0:
      button = int(CruiseButtons.DECEL_SET)
    elif float(v_ego_ms) > float(self.MIN_CRUISE_SPEED_MS):
      if speed_offset_kph >= float(full_kph) and float(full_kph) < float(available_speed_kph):
        button = int(CruiseButtons.RES_ACCEL_2ND)
      elif speed_offset_kph >= float(half_kph) and float(half_kph) < float(available_speed_kph):
        button = int(CruiseButtons.RES_ACCEL)

    if button is None:
      return AccDecision(None, "no-op", target_kph, current_kph, current_kph)

    if CruiseButtons.is_decel(button):
      if (float(current_kph) - float(full_kph)) < (float(self.MIN_CRUISE_SPEED_MS) * CV.MS_TO_KPH):
        button = int(CruiseButtons.CANCEL)

    self._record_button(now_ms=now_ms, button=int(button))

    est_kph = float(current_kph)
    if int(button) == int(CruiseButtons.RES_ACCEL_2ND):
      est_kph += float(full_kph)
    elif int(button) == int(CruiseButtons.RES_ACCEL):
      est_kph += float(half_kph)
    elif int(button) == int(CruiseButtons.DECEL_2ND):
      est_kph = max(0.0, est_kph - float(full_kph))
    elif int(button) == int(CruiseButtons.DECEL_SET):
      est_kph = max(0.0, est_kph - float(half_kph))
    else:
      est_kph = 0.0

    reason = "cancel" if int(button) == int(CruiseButtons.CANCEL) else "press"
    return AccDecision(int(button), reason, target_kph, current_kph, est_kph)
