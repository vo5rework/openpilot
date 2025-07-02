# /data/openpilot/selfdrive/car/modules/ACC_module.py
"""
Unity-style Tesla stock-cruise button selection for XNOR.

What changed for this patch
- Re-centered the button logic on the attached pre-mapd Unity ACC behavior.
- Removed the XNOR-specific readback/reversal latch logic that was causing
  oscillation and nuisance cancel/re-engage behavior.
- Kept only the two cancel paths that are still safety-relevant on XNOR:
  1) target below Tesla's minimum stock-cruise speed
  2) true fast-decel / imminent-lead case

Why this exists
- The attached Unity ACC is intentionally simple: desired speed comes from the
  planner/LONG path, and stock cruise is nudged with basic up/down stalk steps.
- Later XNOR refactors added additional latches, direction damping, and
  large-negative-offset cancel behavior. Those protections are useful in theory,
  but in practice they created the "latch roulette" the recent logs show.
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
  _FAST_DECEL_RESUME_HOLDOFF_MS = 2000
  _LEAD_FRESH_MS = 700

  def __init__(self) -> None:
    self.human_action_time_ms = 0
    self.automated_action_time_ms = 0
    self.fast_decel_time_ms = 0
    self.lead_last_seen_time_ms = 0
    self.prev_cruise_buttons = int(CruiseButtons.IDLE)
    self._radar_sm = messaging.SubMaster(["radarState"])

  def note_human_buttons(self, cruise_buttons: int, *, now_ms: Optional[int] = None) -> None:
    now = _now_ms() if now_ms is None else int(now_ms)
    btn = int(cruise_buttons or 0)
    if btn not in (int(CruiseButtons.IDLE), int(CruiseButtons.MAIN)):
      self.human_action_time_ms = now
    self.prev_cruise_buttons = btn

  def _no_human_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return int(now_ms) > int(self.human_action_time_ms) + int(milliseconds)

  def _no_automated_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return int(now_ms) > int(self.automated_action_time_ms) + int(milliseconds)

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

    recent_lead = bool(lead.status or ((int(now_ms) - int(self.lead_last_seen_time_ms)) < int(self._LEAD_FRESH_MS)))
    if recent_lead and lead.status:
      materially_closing = (lead.d_rel > 0.0) and (lead.v_rel < 0.0)
      if materially_closing or self._fast_decel_required(v_ego_ms=v_ego_ms, lead=lead):
        return False

    return True

  def _record_button(self, *, now_ms: int, button: int) -> None:
    self.automated_action_time_ms = int(now_ms)
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
        float(desired_speed_ms) >= float(v_ego_ms)
        and self._no_human_action_for(now_ms=now_ms, milliseconds=self._HUMAN_COOLDOWN_MS)
        and self._no_automated_action_for(now_ms=now_ms, milliseconds=self._AUTO_COOLDOWN_MS)
        and self._should_autoengage_cc(now_ms=now_ms, v_ego_ms=v_ego_ms, brake_pressed=brake_pressed, lead=lead)
      ):
        half_kph, _ = _cc_units_kph(speed_units)
        readback_kph = float(current_set_speed_ms) * CV.MS_TO_KPH
        self._record_button(now_ms=now_ms, button=int(CruiseButtons.RES_ACCEL))
        return AccDecision(int(CruiseButtons.RES_ACCEL), "autoengage: RES", readback_kph, readback_kph, readback_kph + half_kph)
      return AccDecision(None, "standby: no autoengage")

    if stock_state not in ("ENABLED", "OVERRIDE", "STANDSTILL"):
      return AccDecision(None, f"gated: stock_state={stock_state or 'UNKNOWN'}")

    if not stock_cruise_enabled:
      return AccDecision(None, "gated: stock cruise not enabled")

    if not self._no_human_action_for(now_ms=now_ms, milliseconds=self._HUMAN_COOLDOWN_MS):
      return AccDecision(None, "gated: recent human action")

    if not self._no_automated_action_for(now_ms=now_ms, milliseconds=self._AUTO_COOLDOWN_MS):
      return AccDecision(None, "gated: cooldown")

    if float(desired_speed_ms) <= 0.1 or float(current_set_speed_ms) <= 0.1:
      return AccDecision(None, "gated: missing target/current")

    half_kph, full_kph = _cc_units_kph(speed_units)

    target_kph = float(desired_speed_ms) * CV.MS_TO_KPH
    current_kph = float(current_set_speed_ms) * CV.MS_TO_KPH

    if max_accel_target_ms is not None and float(max_accel_target_ms) > 0.1:
      max_target_kph = max(float(current_kph), float(max_accel_target_ms) * CV.MS_TO_KPH)
    else:
      max_target_kph = float(current_kph)

    speed_offset_kph = float(target_kph) - float(current_kph)
    available_speed_kph = float(max_target_kph) - float(current_kph)

    button: Optional[int] = None
    if float(desired_speed_ms) < float(self.MIN_CRUISE_SPEED_MS):
      button = int(CruiseButtons.CANCEL)
    elif self._fast_decel_required(v_ego_ms=v_ego_ms, lead=lead) and current_kph > 0.0:
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

    if CruiseButtons.is_decel(button) and (current_kph - float(full_kph)) < (self.MIN_CRUISE_SPEED_MS * CV.MS_TO_KPH):
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

    if int(button) == int(CruiseButtons.CANCEL):
      reason = "cancel: fast_decel" if self._fast_decel_required(v_ego_ms=v_ego_ms, lead=lead) else "cancel: below_min_speed"
    else:
      reason = "press"

    return AccDecision(int(button), reason, target_kph, current_kph, est_kph)
