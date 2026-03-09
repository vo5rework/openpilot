# /data/openpilot/selfdrive/car/modules/ACC_module.py
"""
Unity-parity Tesla cruise stalk button selection for XNOR.

What this patch fixes
- Brings engaged ACC button selection closer to Unity again while keeping the
  useful XNOR readback smoothing.
- Removes the non-Unity "fast decel lead => immediate CANCEL" behavior that was
  still causing nuisance dropouts instead of stepped decel.
- Relaxes lead-follow accel and reversal damping so the car can recover speed
  more naturally as a lead opens instead of lingering below the target.

Why this is the right adaptation
- The latest logs show lead-following is mostly a threshold problem now, not a
  planner-horizon problem: the set speed steps down, then hesitates to step back
  up.
- Unity's older engaged ACC logic was simpler and less conservative. XNOR still
  benefits from readback and reversal damping because stock-cruise stalk
  commands are discrete, but the damping needs to stay light enough that it does
  not suppress normal re-accel or turn large slowdowns into a full CANCEL.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from typing import Optional

from cereal import messaging
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons


def _now_ms() -> int:
  return time.monotonic_ns() // 1_000_000


def _cc_units_kph(speed_units: str) -> tuple[float, float]:
  """Return Tesla half/full stalk step sizes in kph."""
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
  _AUTO_COOLDOWN_ACCEL_MS = 200
  _READBACK_WAIT_MS = 500
  _REVERSAL_DAMP_MS = 400
  _LEAD_REVERSAL_DAMP_MS = 450
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
    btn = int(cruise_buttons or 0)
    if btn not in (int(CruiseButtons.IDLE), int(CruiseButtons.MAIN)):
      self.human_action_time_ms = now
    self.prev_cruise_buttons = btn

  def _no_human_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return now_ms > int(self.human_action_time_ms) + int(milliseconds)

  def _no_automated_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return now_ms > int(self.automated_action_time_ms) + int(milliseconds)

  def _refresh_readback_state(self, *, now_ms: int, readback_kph: float, half_kph: float) -> None:
    if not self._awaiting_readback:
      return

    age_ms = int(now_ms) - int(self.automated_action_time_ms)
    moved_kph = float(readback_kph) - float(self._last_auto_readback_kph)
    confirmed = False

    if self._last_auto_direction > 0:
      confirmed = moved_kph >= (0.45 * float(half_kph))
    elif self._last_auto_direction < 0:
      confirmed = moved_kph <= (-0.45 * float(half_kph))

    if confirmed or (age_ms >= int(self._READBACK_WAIT_MS)):
      self._awaiting_readback = False

  def _poll_lead(self, *, now_ms: int) -> LeadInfo:
    lead = LeadInfo()
    try:
      self._radar_sm.update(0)
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
    if (not lead.status) or (lead.d_rel <= 0.0):
      return float(sys.maxsize)
    if lead.v_rel >= 0.0:
      return float(sys.maxsize)
    return abs(float(lead.d_rel) / float(lead.v_rel))

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
      materially_closing = (lead.d_rel > 0.0) and (lead.v_rel < -1.2)
      if materially_closing:
        return False
      if self._fast_decel_required(v_ego_ms=v_ego_ms, lead=lead):
        return False

    return True

  def _clear_pending_reversal(self) -> None:
    self._pending_reversal_direction = 0
    self._pending_reversal_since_ms = 0

  def _allow_lead_reversal(
    self,
    *,
    now_ms: int,
    direction: int,
    speed_offset_kph: float,
    half_kph: float,
    lead: LeadInfo,
  ) -> bool:
    if (not lead.status) or direction == 0 or self._last_auto_direction == 0 or direction == self._last_auto_direction:
      self._clear_pending_reversal()
      return True

    if abs(float(speed_offset_kph)) >= (1.35 * float(half_kph)):
      self._clear_pending_reversal()
      return True

    if int(direction) != int(self._pending_reversal_direction):
      self._pending_reversal_direction = int(direction)
      self._pending_reversal_since_ms = int(now_ms)
      return False

    return (int(now_ms) - int(self._pending_reversal_since_ms)) >= int(self._REVERSAL_PERSIST_MS)


  def _record_button(self, *, now_ms: int, button: int, readback_kph: float, half_kph: float) -> None:
    direction = self._button_direction(int(button))
    if direction != self._last_auto_direction:
      self._direction_change_time_ms = int(now_ms)

    self._last_auto_button = int(button)
    self._last_auto_direction = int(direction)
    self._clear_pending_reversal()
    self._last_auto_readback_kph = float(readback_kph)
    self._awaiting_readback = int(button) not in (int(CruiseButtons.CANCEL), int(CruiseButtons.RES_ACCEL))
    self.automated_action_time_ms = int(now_ms)

    if int(button) == int(CruiseButtons.CANCEL):
      self.fast_decel_time_ms = int(now_ms)
      self._awaiting_readback = False

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
    """Return one CruiseButtons press to move stock cruise toward the desired target."""
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
    min_target_kph = float(self.MIN_CRUISE_SPEED_MS) * CV.MS_TO_KPH
    desired_target_ms = float(desired_speed_ms)

    # Keep stock cruise alive for clear-road curve slowdowns. Below-min targets
    # are clamped to Tesla's minimum cruise speed unless a true fast-decel lead
    # event requires a CANCEL.
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

    opening_or_clear = (not lead.status) or (lead.v_rel > 0.0) or (lead.d_rel > 40.0)
    mild_lead_follow = bool(lead.status and abs(float(lead.v_rel)) < 0.6 and 12.0 < float(lead.d_rel) < 50.0)
    steady_lead_follow = bool(lead.status and abs(float(lead.v_rel)) < 0.35 and 15.0 < float(lead.d_rel) < 35.0)
    strong_lead_opening = bool(lead.status and (float(lead.v_rel) > 0.8 or float(lead.d_rel) > 38.0))

    accel_half_kph = float(half_kph) * (0.70 if opening_or_clear else 1.0)
    accel_full_kph = float(full_kph) * (0.80 if opening_or_clear else 1.0)

    # Unity was willing to re-accelerate as soon as the lead opened and the
    # planner target came back up. Keep XNOR's smoothing, but do not raise the
    # accel thresholds in steady follow the way earlier patches did.
    if mild_lead_follow and not strong_lead_opening:
      accel_half_kph = min(accel_half_kph, 0.85 * float(half_kph))
    if steady_lead_follow and not strong_lead_opening:
      accel_half_kph = min(accel_half_kph, 0.75 * float(half_kph))
      accel_full_kph = min(accel_full_kph, 0.90 * float(full_kph))

    decel_half_kph = 0.9 * float(half_kph)
    if mild_lead_follow and not fast_decel_required:
      decel_half_kph = min(decel_half_kph, 0.90 * float(half_kph))
    if steady_lead_follow and not fast_decel_required:
      decel_half_kph = min(decel_half_kph, 0.80 * float(half_kph))

    allow_accel_full_step = (
      (not lead.status)
      or strong_lead_opening
      or (speed_offset_kph >= (2.0 * float(full_kph)) and not steady_lead_follow)
    )
    allow_decel_full_step = (
      (not lead.status)
      or (lead.status and ((float(lead.v_rel) < -3.2) or speed_offset_kph < (-1.35 * float(full_kph))))
    )

    cancel_required = self._cancel_required(
      v_ego_ms=v_ego_ms,
      current_set_speed_ms=current_set_speed_ms,
      desired_speed_ms=desired_target_ms,
      lead=lead,
    )

    button: Optional[int] = None
    if cancel_required and (current_kph > 0.0):
      button = int(CruiseButtons.CANCEL)
    elif allow_decel_full_step and speed_offset_kph < (-0.6 * float(full_kph)) and current_kph > 0.0:
      button = int(CruiseButtons.DECEL_2ND)
    elif speed_offset_kph < (-0.9 * float(decel_half_kph)) and current_kph > 0.0:
      button = int(CruiseButtons.DECEL_SET)
    elif float(v_ego_ms) > float(self.MIN_CRUISE_SPEED_MS):
      if allow_accel_full_step and speed_offset_kph >= float(accel_full_kph) and float(full_kph) < float(available_speed_kph):
        button = int(CruiseButtons.RES_ACCEL_2ND)
      elif speed_offset_kph >= float(accel_half_kph) and float(half_kph) < float(available_speed_kph):
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
      if (
        self._awaiting_readback
        and direction == self._last_auto_direction
        and (int(now_ms) - int(self.automated_action_time_ms)) < int(self._READBACK_WAIT_MS)
        and (
          (direction < 0 and abs(float(speed_offset_kph)) < float(full_kph))
          or (direction > 0 and abs(float(speed_offset_kph)) < (0.55 * float(full_kph)))
        )
      ):
        return AccDecision(None, "gated: waiting readback", target_kph, current_kph, current_kph)

      steady_lead_follow = bool(lead.status and abs(float(lead.v_rel)) < 0.35 and 15.0 < float(lead.d_rel) < 35.0)
      reversal_damp_ms = int(self._LEAD_REVERSAL_DAMP_MS if lead.status else self._REVERSAL_DAMP_MS)
      if (
        direction != 0
        and self._last_auto_direction != 0
        and direction != self._last_auto_direction
        and (int(now_ms) - int(self._direction_change_time_ms)) < reversal_damp_ms
        and abs(float(speed_offset_kph)) < ((0.55 * float(half_kph)) if steady_lead_follow else (0.50 * float(half_kph) if lead.status else 0.60 * float(half_kph)))
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

    if int(button) == int(CruiseButtons.CANCEL):
      reason = "cancel: fast_decel"
    else:
      reason = "press"

    return AccDecision(int(button), reason, target_kph, current_kph, est_kph)
