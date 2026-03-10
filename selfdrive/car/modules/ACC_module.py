# /data/openpilot/selfdrive/car/modules/ACC_module.py
"""
Unity-parity Tesla cruise stalk button selection for XNOR.

What this patch restores from Unity
- ACC owns a separate adaptive max-speed ceiling (`acc_speed_kph`).
- Automated slowdowns do not permanently lower that ceiling.
- Human stalk changes and speed-limit changes re-seed the ceiling.
- Button selection compares the planner target against the actual Tesla stock
  set speed, while accel headroom comes from the separate ceiling.

Why this is the right next step
- Recent patches kept moving the ceiling between carstate, planner, and LONG.
  Unity kept that ownership inside ACC itself.
- If the stock set speed is stepped down behind a lead, Unity can still climb
  back up later because `acc_speed_kph` stays high. Without that, XNOR can
  "slow only" forever.

XNOR adaptations retained
- Uses real stock-cruise readback (`current_set_speed_ms`) for the current set.
- Ignores recent echoes of its own virtual stalk pulses when tracking human input.
- Keeps the narrowed cancel path for genuinely imminent / below-min situations.
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
  _AUTO_COOLDOWN_ACCEL_MS = 200
  _AUTOENGAGE_SPEED_WINDOW_MS = 0.8
  _FAST_DECEL_RESUME_HOLDOFF_MS = 2000
  _AUTO_ECHO_IGNORE_MS = 1200

  def __init__(self) -> None:
    self.human_action_time_ms = 0
    self.automated_action_time_ms = 0
    self.prev_cruise_buttons = int(CruiseButtons.IDLE)

    self._last_auto_button = int(CruiseButtons.IDLE)
    self._last_auto_button_time_ms = 0

    self.acc_speed_kph = 0.0
    self.speed_limit_kph = 0.0
    self.prev_speed_limit_kph = 0.0

    self.fast_decel_time_ms = 0
    self.lead_last_seen_time_ms = 0
    self._radar_sm = messaging.SubMaster(["radarState"])

  def _seed_ceiling_from_current(self, *, current_set_speed_ms: float) -> None:
    current_kph = float(current_set_speed_ms) * CV.MS_TO_KPH
    if current_kph > 0.0:
      self.acc_speed_kph = max(float(self.acc_speed_kph), float(current_kph))

  def _sync_acc_ceiling(
    self,
    *,
    current_set_speed_ms: float,
    max_accel_target_ms: Optional[float],
  ) -> None:
    current_kph = float(current_set_speed_ms) * CV.MS_TO_KPH
    self.prev_speed_limit_kph = float(self.speed_limit_kph)
    self.speed_limit_kph = float(max_accel_target_ms) * CV.MS_TO_KPH if (max_accel_target_ms is not None and float(max_accel_target_ms) > 0.1) else 0.0

    if self.acc_speed_kph <= 0.0:
      self.acc_speed_kph = max(current_kph, self.speed_limit_kph)

    if self.speed_limit_kph > 0.0:
      if int(self.prev_speed_limit_kph) != int(self.speed_limit_kph):
        self.acc_speed_kph = float(self.speed_limit_kph)
      else:
        self.acc_speed_kph = max(float(self.acc_speed_kph), float(self.speed_limit_kph), float(current_kph))
    else:
      self.acc_speed_kph = max(float(self.acc_speed_kph), float(current_kph))

  def _update_max_acc_speed_from_human(
    self,
    *,
    button: int,
    current_set_speed_ms: float,
    speed_units: str,
  ) -> None:
    self._seed_ceiling_from_current(current_set_speed_ms=current_set_speed_ms)
    half_kph, full_kph = _cc_units_kph(speed_units)
    speed_change_map = {
      int(CruiseButtons.RES_ACCEL): float(half_kph),
      int(CruiseButtons.RES_ACCEL_2ND): float(full_kph),
      int(CruiseButtons.DECEL_SET): -float(half_kph),
      int(CruiseButtons.DECEL_2ND): -float(full_kph),
    }
    self.acc_speed_kph = max(0.0, float(self.acc_speed_kph) + float(speed_change_map.get(int(button), 0.0)))

  def note_human_buttons(
    self,
    cruise_buttons: int,
    *,
    now_ms: Optional[int] = None,
    current_set_speed_ms: float = 0.0,
    speed_units: str = "MPH",
  ) -> None:
    now = _now_ms() if now_ms is None else int(now_ms)
    btn = int(cruise_buttons or 0)
    prev_btn = int(self.prev_cruise_buttons)

    edge = (btn != prev_btn)
    throttled = btn not in (int(CruiseButtons.IDLE), int(CruiseButtons.MAIN))
    auto_echo = (
      edge
      and btn == int(self._last_auto_button)
      and (int(now) - int(self._last_auto_button_time_ms)) <= int(self._AUTO_ECHO_IGNORE_MS)
    )

    if edge and throttled and not auto_echo:
      self.human_action_time_ms = int(now)
      self._update_max_acc_speed_from_human(
        button=btn,
        current_set_speed_ms=current_set_speed_ms,
        speed_units=speed_units,
      )

    self.prev_cruise_buttons = int(btn)

  def _no_human_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return int(now_ms) > int(self.human_action_time_ms) + int(milliseconds)

  def _no_automated_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return int(now_ms) > int(self.automated_action_time_ms) + int(milliseconds)

  def _poll_lead(self, *, now_ms: int) -> LeadInfo:
    try:
      self._radar_sm.update(0)
    except Exception:
      return LeadInfo()

    if not bool(self._radar_sm.valid.get("radarState", False)):
      return LeadInfo()

    try:
      rs = self._radar_sm["radarState"]
      lead = getattr(rs, "leadOne", None)
      if lead is not None and bool(getattr(lead, "status", False)):
        d_rel = float(getattr(lead, "dRel", 0.0) or 0.0)
        v_rel = float(getattr(lead, "vRel", 0.0) or 0.0)
        if d_rel > 0.0:
          self.lead_last_seen_time_ms = int(now_ms)
          return LeadInfo(True, d_rel, v_rel)
    except Exception:
      pass

    return LeadInfo()

  @staticmethod
  def _seconds_to_collision(*, lead: LeadInfo) -> float:
    if (not lead.status) or lead.d_rel <= 0.0:
      return sys.maxsize
    if lead.v_rel >= 0.0:
      return sys.maxsize
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

  def _should_autoengage_cc(self, *, now_ms: int, v_ego_ms: float, brake_pressed: bool, lead: LeadInfo) -> bool:
    cruise_ready = (
      float(v_ego_ms) >= float(self.MIN_CRUISE_SPEED_MS)
      and int(now_ms) > int(self.fast_decel_time_ms) + int(self._FAST_DECEL_RESUME_HOLDOFF_MS)
    )
    if not cruise_ready or brake_pressed:
      return False

    slow_lead = bool(
      lead.status
      and lead.d_rel > 0.0
      and (
        lead.v_rel < 0.0
        or self._fast_decel_required(v_ego_ms=v_ego_ms, lead=lead)
      )
    )
    return not slow_lead

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
    max_accel_target_ms: Optional[float] = None,
  ) -> AccDecision:
    self.note_human_buttons(
      cruise_buttons,
      now_ms=now_ms,
      current_set_speed_ms=current_set_speed_ms,
      speed_units=speed_units,
    )
    self._sync_acc_ceiling(
      current_set_speed_ms=current_set_speed_ms,
      max_accel_target_ms=max_accel_target_ms,
    )
    lead = self._poll_lead(now_ms=now_ms)

    if not enabled:
      return AccDecision(None, "gated: not enabled")

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

    desired_target_ms = float(desired_speed_ms)
    fast_decel_required = self._fast_decel_required(v_ego_ms=v_ego_ms, lead=lead)
    cancel_required = self._cancel_required(
      v_ego_ms=v_ego_ms,
      current_set_speed_ms=current_set_speed_ms,
      desired_speed_ms=desired_target_ms,
      lead=lead,
    )

    if (not fast_decel_required) and desired_target_ms < float(self.MIN_CRUISE_SPEED_MS):
      desired_target_ms = float(self.MIN_CRUISE_SPEED_MS)

    target_kph = float(desired_target_ms) * CV.MS_TO_KPH
    current_kph = float(current_set_speed_ms) * CV.MS_TO_KPH
    available_speed_kph = max(0.0, float(self.acc_speed_kph) - float(current_kph))
    speed_offset_kph = float(target_kph) - float(current_kph)

    button: Optional[int] = None
    if cancel_required and current_kph > 0.0:
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

    cooldown_ms = int(self._AUTO_COOLDOWN_ACCEL_MS) if CruiseButtons.is_accel(button) else int(self._AUTO_COOLDOWN_MS)
    if not self._no_automated_action_for(now_ms=now_ms, milliseconds=cooldown_ms):
      return AccDecision(None, "gated: cooldown", target_kph, current_kph, current_kph)

    if CruiseButtons.is_decel(button):
      min_target_kph = float(self.MIN_CRUISE_SPEED_MS) * CV.MS_TO_KPH
      min_after_full = float(current_kph) - float(full_kph)
      min_after_half = float(current_kph) - float(half_kph)
      if min_after_full < min_target_kph:
        if min_after_half >= min_target_kph:
          button = int(CruiseButtons.DECEL_SET)
        elif cancel_required:
          button = int(CruiseButtons.CANCEL)
        else:
          return AccDecision(None, "no-op:min cruise clamp", target_kph, current_kph, current_kph)

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

    reason = "cancel: fast_decel" if int(button) == int(CruiseButtons.CANCEL) else "press"
    return AccDecision(int(button), reason, target_kph, current_kph, est_kph)
