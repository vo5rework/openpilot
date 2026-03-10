# /data/openpilot/selfdrive/car/modules/ACC_module.py
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
  _AUTO_COOLDOWN_ACCEL_MS = 200
  _AUTO_ECHO_IGNORE_MS = 350
  _AUTOENGAGE_SPEED_WINDOW_MS = 0.8

  def __init__(self) -> None:
    self.human_action_time_ms = 0
    self.automated_action_time_ms = 0
    self.prev_cruise_buttons = int(CruiseButtons.IDLE)
    self._last_auto_button = int(CruiseButtons.IDLE)
    self._last_auto_time_ms = 0

    self.acc_speed_kph = 0.0
    self.speed_limit_kph = 0.0
    self._adaptive_prev = False
    self._radar_sm = messaging.SubMaster(["radarState"])

  def _is_virtual_echo(self, button: int, *, now_ms: int) -> bool:
    return (
      int(button) == int(self._last_auto_button)
      and (0 <= (int(now_ms) - int(self._last_auto_time_ms)) <= int(self._AUTO_ECHO_IGNORE_MS))
    )

  @staticmethod
  def _should_be_throttled(button: int) -> bool:
    return int(button) not in (int(CruiseButtons.IDLE), int(CruiseButtons.MAIN))

  def _poll_lead(self, *, now_ms: int) -> LeadInfo:
    try:
      self._radar_sm.update(0)
    except Exception:
      return LeadInfo()

    try:
      if not bool(self._radar_sm.valid.get("radarState", False)):
        return LeadInfo()
      lead = self._radar_sm["radarState"].leadOne
      if lead is None or not bool(getattr(lead, "status", False)):
        return LeadInfo()
      d_rel = float(getattr(lead, "dRel", 0.0) or 0.0)
      v_rel = float(getattr(lead, "vRel", 0.0) or 0.0)
      if d_rel <= 0.0:
        return LeadInfo()
      return LeadInfo(True, d_rel, v_rel)
    except Exception:
      return LeadInfo()

  def _no_human_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return int(now_ms) > int(self.human_action_time_ms) + int(milliseconds)

  def _no_automated_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return int(now_ms) > int(self.automated_action_time_ms) + int(milliseconds)

  def _update_internal_ceiling(
    self,
    *,
    adaptive_enabled: bool,
    now_ms: int,
    current_set_kph: float,
    v_ego_kph: float,
    speed_limit_kph: float,
    cruise_buttons: int,
  ) -> None:
    btn = int(cruise_buttons or 0)
    edge = btn != int(self.prev_cruise_buttons)
    if adaptive_enabled and (not self._adaptive_prev):
      seed_kph = float(speed_limit_kph) if float(speed_limit_kph) > 0.1 else float(current_set_kph)
      self.acc_speed_kph = max(float(v_ego_kph), float(seed_kph), float(self.acc_speed_kph))
    elif adaptive_enabled and float(speed_limit_kph) > 0.1 and abs(float(speed_limit_kph) - float(self.speed_limit_kph)) >= 0.05:
      self.acc_speed_kph = max(float(v_ego_kph), float(speed_limit_kph))
    elif not adaptive_enabled:
      self.acc_speed_kph = 0.0

    if adaptive_enabled and edge and self._should_be_throttled(btn) and (not self._is_virtual_echo(btn, now_ms=now_ms)):
      self.human_action_time_ms = int(now_ms)
      self.acc_speed_kph = max(float(v_ego_kph), float(current_set_kph))
      if btn == int(CruiseButtons.CANCEL):
        self.acc_speed_kph = 0.0

    self.speed_limit_kph = float(speed_limit_kph)
    self._adaptive_prev = bool(adaptive_enabled)
    self.prev_cruise_buttons = int(btn)

  def _should_autoengage_cc(self, *, v_ego_ms: float, brake_pressed: bool, lead: LeadInfo) -> bool:
    if bool(brake_pressed) or float(v_ego_ms) <= float(self.MIN_CRUISE_SPEED_MS):
      return False
    if not lead.status:
      return True
    return float(lead.v_rel) > -2.0 or float(lead.d_rel) > 8.0

  def update(
    self,
    *,
    now_ms: int,
    enabled: bool,
    adaptive_enabled: bool,
    stock_cruise_state: str,
    speed_units: str,
    v_ego_ms: float,
    current_set_speed_ms: float,
    desired_speed_ms: float,
    cruise_buttons: int,
    speed_limit_target_ms: Optional[float] = None,
    brake_pressed: bool = False,
  ) -> AccDecision:
    lead = self._poll_lead(now_ms=now_ms)

    current_kph = float(current_set_speed_ms) * CV.MS_TO_KPH
    v_ego_kph = float(v_ego_ms) * CV.MS_TO_KPH
    speed_limit_kph = float(speed_limit_target_ms or 0.0) * CV.MS_TO_KPH

    self._update_internal_ceiling(
      adaptive_enabled=bool(adaptive_enabled),
      now_ms=int(now_ms),
      current_set_kph=float(current_kph),
      v_ego_kph=float(v_ego_kph),
      speed_limit_kph=float(speed_limit_kph),
      cruise_buttons=int(cruise_buttons),
    )

    if not enabled:
      return AccDecision(None, "gated: not enabled")

    stock_state = str(stock_cruise_state or "").upper()

    if stock_state == "STANDBY":
      if (
        float(desired_speed_ms) >= float(v_ego_ms) - float(self._AUTOENGAGE_SPEED_WINDOW_MS)
        and self._no_human_action_for(now_ms=now_ms, milliseconds=self._HUMAN_COOLDOWN_MS)
        and self._no_automated_action_for(now_ms=now_ms, milliseconds=self._AUTO_COOLDOWN_MS)
        and self._should_autoengage_cc(v_ego_ms=v_ego_ms, brake_pressed=brake_pressed, lead=lead)
      ):
        half_kph, _ = _cc_units_kph(speed_units)
        self.automated_action_time_ms = int(now_ms)
        self._last_auto_time_ms = int(now_ms)
        self._last_auto_button = int(CruiseButtons.RES_ACCEL)
        return AccDecision(int(CruiseButtons.RES_ACCEL), "autoengage: RES", current_kph + half_kph, current_kph, current_kph + half_kph)
      return AccDecision(None, "standby: no autoengage")

    if stock_state not in ("ENABLED", "OVERRIDE", "STANDSTILL"):
      return AccDecision(None, f"gated: stock_state={stock_state or 'UNKNOWN'}")

    if not adaptive_enabled:
      return AccDecision(None, "gated: adaptive disabled")

    if not self._no_human_action_for(now_ms=now_ms, milliseconds=self._HUMAN_COOLDOWN_MS):
      return AccDecision(None, "gated: recent human action")

    if float(desired_speed_ms) <= 0.1 or float(current_set_speed_ms) <= 0.1:
      return AccDecision(None, "gated: missing target/current")

    cooldown_ms = int(self._AUTO_COOLDOWN_ACCEL_MS if float(desired_speed_ms) > float(current_set_speed_ms) else self._AUTO_COOLDOWN_MS)
    if not self._no_automated_action_for(now_ms=now_ms, milliseconds=cooldown_ms):
      return AccDecision(None, "gated: cooldown")

    half_press_kph, full_press_kph = _cc_units_kph(speed_units)
    target_kph = float(desired_speed_ms) * CV.MS_TO_KPH
    speed_offset_kph = float(target_kph) - float(current_kph)

    ceiling_kph = float(self.acc_speed_kph)
    if float(speed_limit_kph) > 0.1:
      ceiling_kph = max(float(ceiling_kph), float(speed_limit_kph))
    ceiling_kph = max(float(ceiling_kph), float(current_kph))
    available_speed_kph = float(ceiling_kph) - float(current_kph)

    button: Optional[int] = None
    if float(desired_speed_ms) < float(self.MIN_CRUISE_SPEED_MS):
      button = int(CruiseButtons.CANCEL)
    elif speed_offset_kph < (-2.0 * float(full_press_kph)) and current_kph > 0.0:
      button = int(CruiseButtons.CANCEL)
    elif speed_offset_kph < (-0.6 * float(full_press_kph)) and current_kph > 0.0:
      button = int(CruiseButtons.DECEL_2ND)
    elif speed_offset_kph < (-0.9 * float(half_press_kph)) and current_kph > 0.0:
      button = int(CruiseButtons.DECEL_SET)
    elif float(v_ego_ms) > float(self.MIN_CRUISE_SPEED_MS):
      if speed_offset_kph >= float(full_press_kph) and float(full_press_kph) < float(available_speed_kph):
        button = int(CruiseButtons.RES_ACCEL_2ND)
      elif speed_offset_kph >= float(half_press_kph) and float(half_press_kph) < float(available_speed_kph):
        button = int(CruiseButtons.RES_ACCEL)

    if button is None:
      return AccDecision(None, "no-op", target_kph, current_kph, current_kph)

    if CruiseButtons.is_decel(button) and (float(current_kph) - float(full_press_kph)) < (float(self.MIN_CRUISE_SPEED_MS) * CV.MS_TO_KPH):
      button = int(CruiseButtons.CANCEL)

    self.automated_action_time_ms = int(now_ms)
    self._last_auto_time_ms = int(now_ms)
    self._last_auto_button = int(button)

    est_kph = float(current_kph)
    if int(button) == int(CruiseButtons.RES_ACCEL_2ND):
      est_kph += float(full_press_kph)
    elif int(button) == int(CruiseButtons.RES_ACCEL):
      est_kph += float(half_press_kph)
    elif int(button) == int(CruiseButtons.DECEL_2ND):
      est_kph = max(0.0, est_kph - float(full_press_kph))
    elif int(button) == int(CruiseButtons.DECEL_SET):
      est_kph = max(0.0, est_kph - float(half_press_kph))
    else:
      est_kph = 0.0

    reason = "cancel" if int(button) == int(CruiseButtons.CANCEL) else "press"
    return AccDecision(int(button), reason, target_kph, current_kph, est_kph)
