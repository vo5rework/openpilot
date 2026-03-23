# /data/openpilot/selfdrive/car/modules/ACC_module.py
"""
Human-tuned Tesla cruise stalk button selection for XNOR.

This keeps the Unity-style owner split:
- LONG provides the desired speed target.
- ACC owns the adaptive cruise ceiling internally.
- Speed-limit changes may raise that ceiling, but automated slowdowns do not lower it.

The acceleration side is tuned to feel more natural:
- automated recovery uses only 1-step RES presses
- cadence speeds up for large clear-road gaps
- cadence stays slower just after a lead clears so recovery does not feel jerky
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
  _AUTO_COOLDOWN_ACCEL_BASE_MS = 700
  _AUTO_COOLDOWN_ACCEL_FAST_MS = 550
  _AUTO_COOLDOWN_ACCEL_SLOW_MS = 950
  _ACCEL_AFTER_LEAD_CLEAR_SETTLE_MS = 700
  _FAST_DECEL_RESUME_HOLDOFF_MS = 2000
  _LEAD_FRESH_MS = 700
  _AUTOENGAGE_SPEED_WINDOW_MS = 0.8
  _AUTO_ECHO_IGNORE_MS = 750

  def __init__(self) -> None:
    self.human_action_time_ms = 0
    self.automated_action_time_ms = 0
    self.prev_cruise_buttons = int(CruiseButtons.IDLE)
    self._last_human_button = int(CruiseButtons.IDLE)
    self._last_human_button_time_ms = 0

    self._last_auto_button = int(CruiseButtons.IDLE)
    self._last_auto_button_time_ms = 0

    self.fast_decel_time_ms = 0
    self.lead_last_seen_time_ms = 0
    self._last_lead_status = False
    self._lead_cleared_time_ms = 0
    self.acc_speed_kph = 0.0
    self.speed_limit_kph = 0.0
    self.prev_speed_limit_kph = 0.0
    self._prev_enabled = False
    self._manual_lower_hold_active = False
    self._manual_hold_restore_ceiling_kph = 0.0
    self._manual_hold_restore_requested = False

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
      self._last_human_button = int(btn)
      self._last_human_button_time_ms = int(now)
      if CruiseButtons.is_decel(btn):
        if not self._manual_lower_hold_active:
          self._manual_hold_restore_ceiling_kph = max(float(self._manual_hold_restore_ceiling_kph), float(self.acc_speed_kph), float(self.speed_limit_kph))
        self._manual_lower_hold_active = True
        self._manual_hold_restore_requested = False
      elif CruiseButtons.is_accel(btn):
        if self._manual_lower_hold_active or float(self._manual_hold_restore_ceiling_kph) > 0.0:
          self._manual_hold_restore_requested = True
        self._manual_lower_hold_active = False
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


  def _note_lead_transition(self, *, lead: LeadInfo, now_ms: int) -> None:
    lead_present = bool(lead.status)

    if bool(self._last_lead_status) and not lead_present:
      self._lead_cleared_time_ms = int(now_ms)
    elif lead_present:
      # Treat a clearly opening, now-healthy lead as effectively cleared for the
      # resume cadence logic so recovery can begin smoothly without feeling sticky.
      opening_gap = float(lead.v_rel) > 0.5
      healthy_gap = float(lead.d_rel) > 35.0
      if opening_gap and healthy_gap:
        self._lead_cleared_time_ms = int(now_ms)

    self._last_lead_status = lead_present

  def _accel_cooldown_ms(
    self,
    *,
    now_ms: int,
    speed_offset_kph: float,
    available_speed_kph: float,
    lead: LeadInfo,
  ) -> int:
    if lead.status:
      return int(self._AUTO_COOLDOWN_ACCEL_SLOW_MS)

    if (int(now_ms) - int(self._lead_cleared_time_ms)) <= int(self._ACCEL_AFTER_LEAD_CLEAR_SETTLE_MS):
      return int(self._AUTO_COOLDOWN_ACCEL_SLOW_MS)

    if float(speed_offset_kph) >= 8.0 and float(available_speed_kph) >= 4.0:
      return int(self._AUTO_COOLDOWN_ACCEL_FAST_MS)

    return int(self._AUTO_COOLDOWN_ACCEL_BASE_MS)

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

  def _consume_recent_manual_raise_clear(
    self,
    *,
    now_ms: int,
    speed_limit_kph: float,
    current_kph: float,
    speed_units: str,
  ) -> bool:
    btn = int(self._last_human_button)
    if btn not in (int(CruiseButtons.RES_ACCEL), int(CruiseButtons.RES_ACCEL_2ND)):
      return False
    if (int(now_ms) - int(self._last_human_button_time_ms)) > int(self._AUTO_ECHO_IGNORE_MS):
      return False

    half_kph, _ = _cc_units_kph(speed_units)
    limit_kph = float(speed_limit_kph)
    if limit_kph <= 0.0:
      return False
    if limit_kph <= (float(self.acc_speed_kph) + max(0.5 * float(half_kph), 0.2)):
      return False
    if limit_kph <= (float(current_kph) + max(0.5 * float(half_kph), 0.2)):
      return False

    self.acc_speed_kph = max(float(limit_kph), float(self._manual_hold_restore_ceiling_kph))
    self._manual_lower_hold_active = False
    self._manual_hold_restore_ceiling_kph = 0.0
    self._manual_hold_restore_requested = False
    self._last_human_button = int(CruiseButtons.IDLE)
    self._last_human_button_time_ms = 0
    return True


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
    self._note_lead_transition(lead=lead, now_ms=now_ms)

    self.prev_speed_limit_kph = float(self.speed_limit_kph)
    if bool(set_speed_limit_active) and speed_limit_target_ms is not None and float(speed_limit_target_ms) > 0.0:
      self.speed_limit_kph = float(speed_limit_target_ms) * CV.MS_TO_KPH
      # A real posted-limit change should reseed the ACC ceiling and clear any
      # temporary manual lower hold from the previous limit context.
      if int(self.prev_speed_limit_kph) != int(self.speed_limit_kph):
        self.acc_speed_kph = float(self.speed_limit_kph)
        self._manual_lower_hold_active = False
        self._manual_hold_restore_ceiling_kph = 0.0
        self._manual_hold_restore_requested = False
    else:
      self.speed_limit_kph = 0.0

    current_kph = float(current_set_speed_ms) * CV.MS_TO_KPH
    manual_raise_cleared_to_limit = self._consume_recent_manual_raise_clear(
      now_ms=now_ms,
      speed_limit_kph=float(self.speed_limit_kph),
      current_kph=float(current_kph),
      speed_units=speed_units,
    )
    enabled_edge = bool(enabled) and not bool(self._prev_enabled)
    disabled_edge = (not bool(enabled)) and bool(self._prev_enabled)
    self._prev_enabled = bool(enabled)

    if disabled_edge:
      if self._manual_lower_hold_active or float(self._manual_hold_restore_ceiling_kph) > 0.0:
        self._manual_hold_restore_requested = True
      self._manual_lower_hold_active = False

    if not enabled:
      return AccDecision(None, "gated: not enabled")

    target_kph_seed = max(float(current_kph), float(desired_speed_ms) * CV.MS_TO_KPH)
    if enabled_edge or self.acc_speed_kph <= 0.0:
      # Re-engage should restore a sane ceiling from the live set speed / ego speed.
      self.acc_speed_kph = max(
        float(current_kph),
        float(v_ego_ms) * CV.MS_TO_KPH,
        float(self.speed_limit_kph),
        float(target_kph_seed),
        float(self._manual_hold_restore_ceiling_kph),
      )
      self._manual_lower_hold_active = False
      self._manual_hold_restore_ceiling_kph = 0.0
      self._manual_hold_restore_requested = False
    elif self._manual_hold_restore_requested:
      restore_kph = max(
        float(current_kph),
        float(v_ego_ms) * CV.MS_TO_KPH,
        float(self.speed_limit_kph),
        float(target_kph_seed),
        float(self._manual_hold_restore_ceiling_kph),
      )
      self.acc_speed_kph = max(float(self.acc_speed_kph), float(restore_kph))
      self._manual_lower_hold_active = False
      self._manual_hold_restore_ceiling_kph = 0.0
      self._manual_hold_restore_requested = False
    elif self._manual_lower_hold_active:
      # Preserve a manually lowered ceiling until the driver explicitly raises it
      # again or a new posted speed-limit context supersedes it.
      self.acc_speed_kph = max(float(self.acc_speed_kph), 0.0)
    else:
      # Normal behavior: keep the internal ACC ceiling aligned with the active
      # clear-road ceiling so engage and post-curve recovery do not get stuck at
      # the initial set speed. Only raise the ceiling here; slowdowns still come
      # from the desired target path, not by lowering acc_speed_kph.
      self.acc_speed_kph = max(float(self.acc_speed_kph), float(current_kph), float(self.speed_limit_kph), float(target_kph_seed))
      self._manual_hold_restore_ceiling_kph = 0.0

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
    min_cruise_kph = float(self.MIN_CRUISE_SPEED_MS) * CV.MS_TO_KPH

    if float(desired_speed_ms) < float(self.MIN_CRUISE_SPEED_MS):
      button = int(CruiseButtons.CANCEL)
    elif fast_decel_required and self._seconds_to_collision(lead=lead) < 2.5 and float(current_kph) > 0.0:
      button = int(CruiseButtons.CANCEL)
    elif speed_offset_kph < (-2.0 * float(full_kph)) and current_kph > 0.0:
      button = int(CruiseButtons.DECEL_2ND)
    elif speed_offset_kph < (-0.6 * float(full_kph)) and current_kph > 0.0:
      button = int(CruiseButtons.DECEL_2ND)
    elif speed_offset_kph < (-0.9 * float(half_kph)) and current_kph > 0.0:
      button = int(CruiseButtons.DECEL_SET)
    elif float(v_ego_ms) > float(self.MIN_CRUISE_SPEED_MS):
      accel_cooldown_ms = self._accel_cooldown_ms(
        now_ms=now_ms,
        speed_offset_kph=float(speed_offset_kph),
        available_speed_kph=float(available_speed_kph),
        lead=lead,
      )
      accel_ready = self._no_automated_action_for(now_ms=now_ms, milliseconds=accel_cooldown_ms)
      accel_threshold_kph = max(0.75 * float(half_kph), 0.8)
      available_threshold_kph = max(float(half_kph) - 0.05, 0.5)
      if accel_ready and speed_offset_kph >= float(accel_threshold_kph) and available_speed_kph >= float(available_threshold_kph):
        button = int(CruiseButtons.RES_ACCEL)

    if button is None:
      return AccDecision(None, "no-op", target_kph, current_kph, current_kph)

    if CruiseButtons.is_decel(button):
      if int(button) == int(CruiseButtons.DECEL_2ND) and (float(current_kph) - float(full_kph)) < float(min_cruise_kph):
        if (float(current_kph) - float(half_kph)) >= float(min_cruise_kph):
          button = int(CruiseButtons.DECEL_SET)
        else:
          button = None
      elif int(button) == int(CruiseButtons.DECEL_SET) and (float(current_kph) - float(half_kph)) < float(min_cruise_kph):
        button = None

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
