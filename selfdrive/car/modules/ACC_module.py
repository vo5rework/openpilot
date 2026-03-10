# /data/openpilot/selfdrive/car/modules/ACC_module.py
"""
Unity-parity Tesla cruise stalk button selection for XNOR.

What this patch restores
- ACC decisions come from the planner/LONG desired speed, like Unity.
- Lead handling stays primarily in MPC/planner; ACC is not a second lead controller.
- Stock-cruise CANCEL is reserved for true emergency / below-min cases.
- Automated set-speed changes keep lightweight XNOR readback guards only.

Why this is the right XNOR adaptation
- Unity's older ACC logic was simple: compare desired target vs current stock set
  and step the Tesla stalk toward it.
- Later XNOR patches added extra lead-specific accel/decel heuristics in ACC,
  which created 1-2 mph nibbling and made re-accel depend on ACC thresholds
  instead of the planner target.
- Keeping the Unity decision thresholds while retaining modest readback damping
  gives smoother follow behavior in XNOR's stock-cruise-sync architecture.
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


def _button_should_be_throttled(button: int) -> bool:
  return int(button) not in (int(CruiseButtons.MAIN), int(CruiseButtons.IDLE))


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
  _AUTO_COOLDOWN_ACCEL_MS = 200
  _READBACK_WAIT_MS = 350
  _REVERSAL_DAMP_MS = 250
  _LEAD_REVERSAL_DAMP_MS = 250
  _REVERSAL_PERSIST_MS = 150
  _FAST_DECEL_RESUME_HOLDOFF_MS = 2000
  _LEAD_FRESH_MS = 700
  _AUTOENGAGE_SPEED_WINDOW_MS = 0.8

  def __init__(self) -> None:
    self.human_action_time_ms = 0
    self.automated_action_time_ms = 0
    self.prev_cruise_buttons = int(CruiseButtons.IDLE)

    self._last_auto_button = int(CruiseButtons.IDLE)
    self._last_auto_direction = 0
    self._last_auto_readback_kph = 0.0
    self._awaiting_readback = False
    self._direction_change_time_ms = 0
    self._pending_reversal_direction = 0
    self._pending_reversal_since_ms = 0

    self.fast_decel_time_ms = 0
    self.lead_last_seen_time_ms = 0
    self._radar_sm = messaging.SubMaster(["radarState"])

  @staticmethod
  def _button_direction(button: int) -> int:
    if CruiseButtons.is_accel(button):
      return 1
    if CruiseButtons.is_decel(button) or int(button) == int(CruiseButtons.CANCEL):
      return -1
    return 0

  def note_human_buttons(self, cruise_buttons: int, *, now_ms: Optional[int] = None) -> None:
    now = _now_ms() if now_ms is None else int(now_ms)
    current_button = int(cruise_buttons)
    button_changed = current_button != int(self.prev_cruise_buttons)

    automated_echo = bool(
      button_changed
      and _button_should_be_throttled(current_button)
      and current_button == int(self._last_auto_button)
      and (int(now) - int(self.automated_action_time_ms)) < 1200
    )

    if button_changed and _button_should_be_throttled(current_button) and not automated_echo:
      self.human_action_time_ms = int(now)
      self._awaiting_readback = False
      self._pending_reversal_direction = 0
      self._pending_reversal_since_ms = 0
      self._last_auto_direction = 0

    if button_changed and current_button == int(CruiseButtons.IDLE):
      # Readback release for an automated pulse should not count as a human action,
      # but it should let the controller issue the next step immediately.
      self._awaiting_readback = False

    self.prev_cruise_buttons = current_button

  def _no_human_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return (int(now_ms) - int(self.human_action_time_ms)) >= int(milliseconds)

  def _no_automated_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return (int(now_ms) - int(self.automated_action_time_ms)) >= int(milliseconds)

  def _refresh_readback_state(self, *, now_ms: int, readback_kph: float, half_kph: float) -> None:
    if not self._awaiting_readback:
      return

    moved_enough = abs(float(readback_kph) - float(self._last_auto_readback_kph)) >= (0.45 * float(half_kph))
    timed_out = (int(now_ms) - int(self.automated_action_time_ms)) > 1200
    if moved_enough or timed_out:
      self._awaiting_readback = False

  def _poll_lead(self, *, now_ms: int) -> LeadInfo:
    try:
      self._radar_sm.update(0)
    except Exception:
      pass

    lead = LeadInfo()
    try:
      if bool(self._radar_sm.valid.get("radarState", False)):
        rs = self._radar_sm["radarState"]
        lead_one = getattr(rs, "leadOne", None)
        if lead_one is not None and bool(getattr(lead_one, "status", False)):
          d_rel = float(getattr(lead_one, "dRel", 0.0) or 0.0)
          v_rel = float(getattr(lead_one, "vRel", 0.0) or 0.0)
          if d_rel > 0.0:
            lead = LeadInfo(True, d_rel, v_rel)
            self.lead_last_seen_time_ms = int(now_ms)
    except Exception:
      pass
    return lead

  @staticmethod
  def _seconds_to_collision(*, lead: LeadInfo) -> float:
    if (not lead.status) or (lead.d_rel <= 0.0) or (lead.v_rel >= -0.01):
      return 1e6
    return float(lead.d_rel) / max(0.01, -float(lead.v_rel))

  def _fast_decel_required(self, *, v_ego_ms: float, lead: LeadInfo) -> bool:
    if not lead.status or lead.d_rel <= 0.0:
      return False

    collision_imminent = self._seconds_to_collision(lead=lead) < 4.0
    lead_absolute_speed_ms = float(v_ego_ms) + float(lead.v_rel)
    lead_too_slow = lead_absolute_speed_ms < float(self.MIN_CRUISE_SPEED_MS)

    return bool(collision_imminent or lead_too_slow)

  def _cancel_required(self, *, v_ego_ms: float, current_set_speed_ms: float, desired_speed_ms: float, lead: LeadInfo) -> bool:
    if not lead.status or lead.d_rel <= 0.0:
      return False

    ttc_s = self._seconds_to_collision(lead=lead)
    desired_below_min = float(desired_speed_ms) < float(self.MIN_CRUISE_SPEED_MS)
    current_near_min = float(current_set_speed_ms) <= (float(self.MIN_CRUISE_SPEED_MS) + (0.55 * CV.KPH_TO_MS))
    materially_closing = float(lead.v_rel) < -1.5

    return bool(
      (ttc_s < 2.5)
      or (desired_below_min and current_near_min and materially_closing and ttc_s < 5.0)
    )

  def _should_autoengage_cc(self, *, now_ms: int, v_ego_ms: float, brake_pressed: bool, lead: LeadInfo) -> bool:
    if float(v_ego_ms) <= float(self.MIN_CRUISE_SPEED_MS):
      return False
    if bool(brake_pressed):
      return False
    if (int(now_ms) - int(self.fast_decel_time_ms)) < int(self._FAST_DECEL_RESUME_HOLDOFF_MS):
      return False
    if lead.status and self._fast_decel_required(v_ego_ms=v_ego_ms, lead=lead):
      return False
    return True

  def _clear_pending_reversal(self) -> None:
    self._pending_reversal_direction = 0
    self._pending_reversal_since_ms = 0

  def _allow_lead_reversal(self, *, now_ms: int, direction: int, speed_offset_kph: float, half_kph: float, lead: LeadInfo) -> bool:
    if direction == 0 or (not lead.status):
      self._clear_pending_reversal()
      return True

    threshold = 0.55 * float(half_kph)
    if abs(float(speed_offset_kph)) >= threshold:
      self._clear_pending_reversal()
      return True

    if self._pending_reversal_direction == direction:
      return (int(now_ms) - int(self._pending_reversal_since_ms)) >= int(self._REVERSAL_PERSIST_MS)

    self._pending_reversal_direction = int(direction)
    self._pending_reversal_since_ms = int(now_ms)
    return False

  def _record_button(self, *, now_ms: int, button: int, readback_kph: float, half_kph: float) -> None:
    self.automated_action_time_ms = int(now_ms)
    self._last_auto_button = int(button)
    self._last_auto_direction = self._button_direction(int(button))
    self._last_auto_readback_kph = float(readback_kph)
    self._awaiting_readback = int(button) != int(CruiseButtons.CANCEL)

    if self._last_auto_direction != 0:
      self._direction_change_time_ms = int(now_ms)

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
    max_accel_target_ms: Optional[float] = None,
  ) -> AccDecision:
    self.note_human_buttons(cruise_buttons, now_ms=now_ms)
    lead = self._poll_lead(now_ms=now_ms)

    if not enabled:
      return AccDecision(None, "gated: not enabled")

    stock_state = str(stock_cruise_state or "").upper()

    if stock_state == "STANDBY":
      if (
        float(desired_speed_ms) >= float(v_ego_ms) - float(self._AUTOENGAGE_SPEED_WINDOW_MS)
        and self._no_human_action_for(now_ms=now_ms, milliseconds=self._HUMAN_COOLDOWN_MS)
        and self._no_automated_action_for(now_ms=now_ms, milliseconds=self._AUTO_COOLDOWN_MS)
        and self._should_autoengage_cc(now_ms=now_ms, v_ego_ms=v_ego_ms, brake_pressed=brake_pressed, lead=lead)
      ):
        half_kph, _ = _cc_units_kph(speed_units)
        readback_kph = float(current_set_speed_ms) * CV.MS_TO_KPH
        self._record_button(now_ms=now_ms, button=int(CruiseButtons.RES_ACCEL), readback_kph=readback_kph, half_kph=half_kph)
        self._awaiting_readback = False
        return AccDecision(int(CruiseButtons.RES_ACCEL), "autoengage: RES")
      return AccDecision(None, "standby: no autoengage")

    if stock_state not in ("ENABLED", "OVERRIDE", "STANDSTILL"):
      return AccDecision(None, f"gated: stock_state={stock_state or 'UNKNOWN'}")

    if not stock_cruise_enabled:
      return AccDecision(None, "gated: stock cruise not enabled")

    if not self._no_human_action_for(now_ms=now_ms, milliseconds=self._HUMAN_COOLDOWN_MS):
      return AccDecision(None, "gated: recent human action")

    if float(desired_speed_ms) <= 0.1 or float(current_set_speed_ms) <= 0.1:
      return AccDecision(None, "gated: missing target/current")

    half_kph, full_kph = _cc_units_kph(speed_units)
    fast_decel_required = self._fast_decel_required(v_ego_ms=v_ego_ms, lead=lead)
    desired_target_ms = float(desired_speed_ms)

    if (not fast_decel_required) and desired_target_ms < float(self.MIN_CRUISE_SPEED_MS):
      desired_target_ms = float(self.MIN_CRUISE_SPEED_MS)

    target_kph = float(desired_target_ms) * CV.MS_TO_KPH
    readback_kph = float(current_set_speed_ms) * CV.MS_TO_KPH
    current_kph = float(readback_kph)

    max_target_kph = float(target_kph)
    if max_accel_target_ms is not None and float(max_accel_target_ms) > 0.1:
      max_target_kph = float(max_accel_target_ms) * CV.MS_TO_KPH
    max_target_kph = max(max_target_kph, float(current_kph))

    self._refresh_readback_state(now_ms=now_ms, readback_kph=readback_kph, half_kph=half_kph)

    speed_offset_kph = float(target_kph) - float(current_kph)
    available_speed_kph = float(max_target_kph) - float(current_kph)

    cancel_required = self._cancel_required(
      v_ego_ms=v_ego_ms,
      current_set_speed_ms=current_set_speed_ms,
      desired_speed_ms=desired_target_ms,
      lead=lead,
    )

    button: Optional[int] = None
    if cancel_required and (current_kph > 0.0):
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
      self._clear_pending_reversal()
      return AccDecision(None, "no-op", target_kph, current_kph, current_kph)

    cooldown_ms = int(self._AUTO_COOLDOWN_ACCEL_MS) if CruiseButtons.is_accel(button) else int(self._AUTO_COOLDOWN_MS)
    if not self._no_automated_action_for(now_ms=now_ms, milliseconds=cooldown_ms):
      return AccDecision(None, "gated: cooldown", target_kph, current_kph, current_kph)

    if CruiseButtons.is_decel(button):
      min_after_full = float(current_kph) - float(full_kph)
      min_after_half = float(current_kph) - float(half_kph)
      min_target_kph = float(self.MIN_CRUISE_SPEED_MS) * CV.MS_TO_KPH
      if min_after_full < min_target_kph:
        if min_after_half >= min_target_kph:
          button = int(CruiseButtons.DECEL_SET)
        elif cancel_required:
          button = int(CruiseButtons.CANCEL)
        else:
          self._clear_pending_reversal()
          return AccDecision(None, "no-op:min cruise clamp", target_kph, current_kph, current_kph)

    direction = self._button_direction(int(button))
    if int(button) != int(CruiseButtons.CANCEL):
      # Unity parity for speed recovery: do not add extra accel-side readback or
      # reversal damping beyond the normal automated-action cooldown.
      if CruiseButtons.is_decel(button):
        if (
          self._awaiting_readback
          and direction == self._last_auto_direction
          and (int(now_ms) - int(self.automated_action_time_ms)) < int(self._READBACK_WAIT_MS)
          and abs(float(speed_offset_kph)) < float(full_kph)
        ):
          return AccDecision(None, "gated: waiting readback", target_kph, current_kph, current_kph)

        reversal_damp_ms = int(self._LEAD_REVERSAL_DAMP_MS if lead.status else self._REVERSAL_DAMP_MS)
        if (
          direction != 0
          and self._last_auto_direction != 0
          and direction != self._last_auto_direction
          and (int(now_ms) - int(self._direction_change_time_ms)) < reversal_damp_ms
          and abs(float(speed_offset_kph)) < (0.75 * float(half_kph))
        ):
          return AccDecision(None, "gated: reversal damp", target_kph, current_kph, current_kph)

        if not self._allow_lead_reversal(
          now_ms=now_ms,
          direction=direction,
          speed_offset_kph=speed_offset_kph,
          half_kph=half_kph,
          lead=lead,
        ):
          return AccDecision(None, "gated: reversal persist", target_kph, current_kph, current_kph)

    self._record_button(now_ms=now_ms, button=int(button), readback_kph=readback_kph, half_kph=half_kph)

    est_kph = float(readback_kph)
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

    reason = "cancel: imminent" if int(button) == int(CruiseButtons.CANCEL) else "press"
    return AccDecision(int(button), reason, target_kph, current_kph, est_kph)
