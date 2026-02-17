# /data/openpilot/opendbc/car/tesla/carcontroller.py
"""Tesla CarController (xnor C3)

This file adds two Unity-parity behaviors while keeping steering logic minimal/stable:

1) ALC blinker hold:
   - When ALC is starting/engaged (or during the internal tap latch window),
     keep Tesla blinker ON by overriding TurnIndLvr_Stat via STW_ACTN_RQ.
   - Cancel the blinker when ALC reports done.

2) ACC speed-limit sync:
   - When enabled and Tesla cruise is engaged, match Tesla cruise set speed to
     Tesla map speed limit by emulating cruise stalk up/down via STW_ACTN_RQ.

Both features use TeslaCAN.create_action_request(), which computes the correct
counter + CRC8 (Unity parity).
"""

from __future__ import annotations

import time
import numpy as np

from openpilot.common.params import Params

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

from opendbc.car.tesla.teslacan import TeslaCAN
try:
  from opendbc.car.tesla.teslacan_legacy import TeslaCANLegacy as TeslaCANLegacy
except ImportError:
  from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven as TeslaCANLegacy

from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS


# Unity button values (SpdCtrlLvr_Stat)
BTN_IDLE = 0
BTN_CANCEL = 1
BTN_MAIN = 2
BTN_UP2 = 4
BTN_DOWN2 = 8
BTN_UP1 = 16
BTN_DOWN1 = 32

# TurnIndLvr_Stat (Tesla)
TURN_NONE = 3
TURN_LEFT = 1
TURN_RIGHT = 2


class _CachedParams:
  def __init__(self) -> None:
    self._p = Params()
    self._last_read_frame = -100000
    self.autopilot_disabled = False
    self.pedal_enabled = False
    self.adjust_acc_with_speed_limit = False
    self.speed_limit_offset_uom = 0.0
    self.speed_limit_use_relative = False

  def refresh(self, frame: int) -> None:
    if (frame - self._last_read_frame) < 50:
      return
    self._last_read_frame = int(frame)
    self.autopilot_disabled = bool(self._p.get_bool("TinklaAutopilotDisabled"))
    self.pedal_enabled = bool(self._p.get_bool("TinklaPedalEnabled") or self._p.get_bool("PedalEnabled"))
    self.adjust_acc_with_speed_limit = bool(self._p.get_bool("TinklaAdjustAccWithSpeedLimit"))
    try:
      self.speed_limit_offset_uom = float(self._p.get("TinklaSpeedLimitOffset", encoding="utf-8") or "0")
    except Exception:
      self.speed_limit_offset_uom = 0.0
    self.speed_limit_use_relative = bool(self._p.get_bool("TinklaSpeedLimitUseRelative"))


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, VM=None):
    super().__init__(dbc_names, CP)

    self.CP = CP
    self.frame = 0
    self._params = _CachedParams()

    self._op659_prev_btn = 0
    self.apply_angle_last = 0.0

    # For STW_ACTN_RQ injection we always use TeslaCAN (CRC + counter)
    # even on legacy cars (steering still uses TeslaCANLegacy).
    if CP.carFingerprint in LEGACY_CARS:
      self.packers = {
        CANBUS.party: CANPacker(dbc_names[Bus.party]),
        CANBUS.powertrain: CANPacker(dbc_names[Bus.pt]),
      }
      self.tesla_can = TeslaCANLegacy(self.packers)
      self._stw_can = TeslaCAN(self.packers[CANBUS.party])
    else:
      self.packer = CANPacker(dbc_names[Bus.party])
      self.tesla_can = TeslaCAN(self.packer)
      self._stw_can = self.tesla_can

    # pacing / debouncing
    self._last_human_stalk_frame = -100000
    self._last_speed_sync_frame = -100000
    self._last_turn_hold_frame = -100000
    self._turn_cancel_until_frame = -1

  def _emit_internal_0x659(self, CS, can_sends):
    stalk_btn = int(getattr(CS, "cruise_buttons", 0) or 0)
    prev_btn = int(self._op659_prev_btn)

    main_edge = (stalk_btn == BTN_MAIN) and (prev_btn != BTN_MAIN)
    cancel_edge = (stalk_btn == BTN_CANCEL) and (prev_btn != BTN_CANCEL)

    self._op659_prev_btn = stalk_btn

    if (self.frame % 10 == 0) or main_edge or cancel_edge:
      for bus in (CANBUS.party, CANBUS.party + 4):
        can_sends.append(self._stw_can.create_fake_das_msg(
          pedalEnabled=self._params.pedal_enabled,
          autopilot_disabled=self._params.autopilot_disabled,
          bus=bus,
          stalk_main=main_edge,
          stalk_cancel=cancel_edge,
        ))

  def _stw_buses(self) -> list[int]:
    buses = [CANBUS.party]
    # Unity parity: AP1/AP2 cars also require the message on the autopilot bus.
    if self.CP.carFingerprint in LEGACY_CARS:
      buses.append(CANBUS.autopilot_party)
    return buses

  def _send_stw_actn(self, can_sends, CS, cruise_btn: int, turn: int | None) -> None:
    msg = getattr(CS, "msg_stw_actn_req", None)
    if msg is None:
      return

    values = dict(msg)
    if turn is not None:
      values["TurnIndLvr_Stat"] = int(turn)

    for bus in self._stw_buses():
      # insert first to win race vs real stalk frames
      can_sends.insert(0, self._stw_can.create_action_request(bus, values, int(cruise_btn)))

  def _speed_limit_offset_ms(self, CS, speed_limit_ms: float) -> float:
    if not self._params.adjust_acc_with_speed_limit:
      return 0.0

    offset_uom = float(self._params.speed_limit_offset_uom or 0.0)
    if offset_uom == 0.0:
      return 0.0

    if self._params.speed_limit_use_relative:
      return speed_limit_ms * offset_uom / 100.0

    # Unity parity: offset is in car speed units
    su = str(getattr(CS, "speed_units", "MPH"))
    if su == "KPH":
      return offset_uom * (1000.0 / 3600.0)
    return offset_uom * (1609.344 / 3600.0)

  def _choose_speed_sync_button(self, CS, target_ms: float, current_ms: float) -> int | None:
    # if no valid target/current
    if target_ms <= 0.1 or current_ms <= 0.1:
      return None

    diff_ms = float(target_ms - current_ms)
    su = str(getattr(CS, "speed_units", "MPH"))
    diff_uom = diff_ms * (3.6 if su == "KPH" else 2.2369362920544)

    # deadband
    if abs(diff_uom) < 0.6:
      return None

    # emulate Unity's half/ full press thresholds
    half_press = 1.0  # ~1 unit
    full_press = 5.0  # ~5 units
    if diff_uom >= full_press:
      return BTN_UP2
    if diff_uom >= half_press:
      return BTN_UP1
    if diff_uom <= -full_press:
      return BTN_DOWN2
    if diff_uom <= -half_press:
      return BTN_DOWN1
    return None

  def _compute_turn_hold(self, CS) -> int | None:
    # Do not override when driver is holding the stalk (full signal)
    if int(getattr(CS, "turnSignalStalkState", 0) or 0) != 0:
      return None

    # If we recently requested a cancel, keep forcing neutral for a short time
    if self._turn_cancel_until_frame >= self.frame:
      return TURN_NONE

    # Prefer explicit ALC direction when available
    alca_active = bool(getattr(CS, "alca_pre_engage", False) or getattr(CS, "alca_engaged", False))
    if alca_active:
      d = int(getattr(CS, "alca_direction", 0) or 0)
      if d == 1:
        return TURN_LEFT
      if d == 2:
        return TURN_RIGHT

    # During the tap latch window, keep the requested direction
    latch_until = int(getattr(CS, "_alc_tap_latch_until", 0) or 0)
    if latch_until > self.frame:
      d = int(getattr(CS, "_alc_tap_latch_dir", 0) or 0)
      if d == 1:
        return TURN_LEFT
      if d == 2:
        return TURN_RIGHT

    # If lane change just finished, request cancel once
    if bool(getattr(CS, "alca_done", False)):
      self._turn_cancel_until_frame = int(self.frame + 50)  # ~0.5s @ 100Hz
      return TURN_NONE

    return None

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    self._params.refresh(self.frame)
    self._emit_internal_0x659(CS, can_sends)

    autopilot_disabled = bool(self._params.autopilot_disabled)
    human_control = bool(getattr(CS, "human_control", False))

    lat_active = (
      bool(CC.latActive) and
      autopilot_disabled and
      (not CS.out.cruiseState.standstill) and
      (not human_control)
    )

    # Steering (50Hz)
    if self.frame % 2 == 0:
      if human_control:
        self.apply_angle_last = float(CS.out.steeringAngleDeg)
      else:
        self.apply_angle_last = float(apply_std_steer_angle_limits(
          float(actuators.steeringAngleDeg),
          float(self.apply_angle_last),
          float(getattr(CS.out, "vEgoRaw", CS.out.vEgo)),
          float(CS.out.steeringAngleDeg),
          lat_active,
          CarControllerParams.ANGLE_LIMITS,
        ))

      if self.CP.carFingerprint in LEGACY_CARS:
        counter = (self.frame // 2) % 16
        can_sends.append(self.tesla_can.create_steering_control(counter, self.apply_angle_last, lat_active))
      else:
        can_sends.append(self.tesla_can.create_steering_control(self.apply_angle_last, lat_active))

    # EPS allow (legacy)
    if (self.CP.carFingerprint in LEGACY_CARS) and (self.frame % 10 == 0):
      counter = (self.frame // 10) % 16
      can_sends.append(self.tesla_can.create_steering_allowed(counter))

    # --- Unity parity: STW_ACTN_RQ injection (only when OP is controlling steering)
    if autopilot_disabled and bool(getattr(CC, "enabled", False) or CC.latActive):
      # If user touched the cruise stalk, back off briefly.
      if int(getattr(CS, "cruise_buttons", 0) or 0) != 0:
        self._last_human_stalk_frame = int(self.frame)

      allow_inject = (self.frame - self._last_human_stalk_frame) > 100  # ~1s

      turn_hold = self._compute_turn_hold(CS)

      # Speed-limit sync at 5Hz (Unity does frame % 20)
      cruise_btn = None
      if allow_inject and self._params.adjust_acc_with_speed_limit:
        if (self.frame - self._last_speed_sync_frame) >= 20:
          if bool(getattr(CS, "stock_cruise_enabled", False)):
            speed_limit_ms = float(getattr(CS, "speed_limit_ms", 0.0) or 0.0)
            offset_ms = float(self._speed_limit_offset_ms(CS, speed_limit_ms))
            target_ms = speed_limit_ms + offset_ms
            current_ms = float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0)
            cruise_btn = self._choose_speed_sync_button(CS, target_ms, current_ms)

      if cruise_btn is not None:
        self._last_speed_sync_frame = int(self.frame)
        self._send_stw_actn(can_sends, CS, cruise_btn, turn_hold)
      else:
        # Turn hold at 10Hz while active
        if turn_hold is not None and (self.frame - self._last_turn_hold_frame) >= 10:
          self._last_turn_hold_frame = int(self.frame)
          self._send_stw_actn(can_sends, CS, BTN_IDLE, turn_hold)

    # Longitudinal (optional)
    if self.CP.openpilotLongitudinalControl and (self.frame % 4 == 0):
      state = 13 if CC.cruiseControl.cancel else 4
      accel = float(np.clip(float(actuators.accel), CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      counter = (self.frame // 4) % 8
      long_active = bool(CC.longActive) and (not autopilot_disabled)

      can_sends.append(
        self.tesla_can.create_longitudinal_command(
          state, accel, counter, float(CS.out.vEgo), long_active
        )
      )

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = float(self.apply_angle_last)

    self.frame += 1
    return new_actuators, can_sends
