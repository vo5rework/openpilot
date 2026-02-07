# /data/openpilot/opendbc/car/tesla/carcontroller.py
import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

from opendbc.car.tesla.teslacan import TeslaCAN
try:
  # We provide TeslaCANLegacy alias in teslacan_legacy.py to avoid the confusing "Raven" name.
  from opendbc.car.tesla.teslacan_legacy import TeslaCANLegacy as TeslaCANLegacy
except ImportError:
  from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven as TeslaCANLegacy

from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR
from opendbc.car.common.conversions import Conversions as CV
from openpilot.common.params import Params

try:
  from opendbc.car.tesla.teslacan import create_fake_das_msg as create_fake_das
except ImportError:
  from opendbc.car.tesla.teslacan import create_fake_das_message as create_fake_das


class CarController(CarControllerBase):
  """
  Tesla legacy (HW2) controller.

  Unity parity:
    - publish internal contract frame (0x659) to BOTH pandas (bus 0 and bus 4)
    - only byte5 bits are used: bit7 autopilot_disabled, bit5 pedal_enabled, bit1 main_edge, bit0 cancel_edge
    - do NOT publish 0x659 from multiple code paths (prevents bus storms / EPS faults)
  """

  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)

    self.CP = CP
    self.frame = 0

    self.prev_cruise_buttons = 0
    self.params = Params()
    self._cached_autopilot_disabled = False
    self._cached_pedal_enabled = False
    self._params_last_read_frame = -100000

    self._op659_prev_btn = 0
    self.apply_angle_last = 0.0

    if CP.carFingerprint in LEGACY_CARS:
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.packers = {
        CANBUS.party: CANPacker(dbc_names[Bus.party]),
        CANBUS.powertrain: CANPacker(dbc_names[Bus.pt]),
      }
      self.tesla_can = TeslaCANLegacy(self.packers)
    else:
      self.packer = CANPacker(dbc_names[Bus.party])
      self.tesla_can = TeslaCAN(self.packer)

  def _refresh_cached_params(self) -> None:
    if (self.frame - self._params_last_read_frame) >= 50:
      self._params_last_read_frame = self.frame
      self._cached_autopilot_disabled = bool(self.params.get_bool("TinklaAutopilotDisabled"))
      self._cached_pedal_enabled = bool(self.params.get_bool("TinklaPedalEnabled") or self.params.get_bool("PedalEnabled"))

  def _emit_internal_0x659(self, CS, can_sends) -> None:
    stalk_btn = int(getattr(CS, "cruise_buttons", 0) or 0)
    prev_btn = int(self._op659_prev_btn)

    main_edge = (stalk_btn == 2) and (prev_btn != 2)
    cancel_edge = (stalk_btn == 1) and (prev_btn != 1)
    self._op659_prev_btn = stalk_btn

    if (self.frame % 10 == 0) or main_edge or cancel_edge:
      for bus in (CANBUS.party, CANBUS.party + 4):
        can_sends.append(create_fake_das(
          self._cached_pedal_enabled,
          self._cached_autopilot_disabled,
          bus=bus,
          stalk_main=main_edge,
          stalk_cancel=cancel_edge,
        ))

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    self._refresh_cached_params()
    self._emit_internal_0x659(CS, can_sends)

    autopilot_disabled = self._cached_autopilot_disabled

    # Lateral can only be active when AP is disabled (Unity parity)
    lat_active = bool(CC.latActive) and autopilot_disabled and (not CS.out.cruiseState.standstill)

    # Steering at 50Hz (every 2 frames at 100Hz control loop)
    if self.frame % 2 == 0:
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

    # EPS allow at 10Hz on legacy (0x27D)
    if (self.CP.carFingerprint in LEGACY_CARS) and (self.frame % 10 == 0):
      counter = (self.frame // 10) % 16
      can_sends.append(self.tesla_can.create_steering_allowed(counter))

    # Longitudinal (optional). On HW2 legacy this is usually on the powertrain panda (0x2BF).
    if self.CP.openpilotLongitudinalControl and (self.frame % 4 == 0):
      state = 13 if CC.cruiseControl.cancel else 4
      accel = float(np.clip(float(actuators.accel), CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      counter = (self.frame // 4) % 8
      long_active = bool(CC.longActive) and (not autopilot_disabled)

      if self.CP.carFingerprint in LEGACY_CARS:
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, counter, float(CS.out.vEgo), long_active))
      else:
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, counter, float(CS.out.vEgo), long_active))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = float(self.apply_angle_last)

    self.frame += 1
    return new_actuators, can_sends
