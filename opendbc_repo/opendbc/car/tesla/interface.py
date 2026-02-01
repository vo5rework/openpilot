from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, CAR, TeslaLegacyParams, LEGACY_CARS
from opendbc.car.tesla.radar_interface import RadarInterface
from cereal import messaging

try:
  from selfdrive.car.modules.ALC_module import ALCController
  from selfdrive.car.modules.BLNK_module import BLNKController
  from selfdrive.car.modules.HSO_module import HSOController
  from selfdrive.car.modules.CFG_module import load_bool_param, load_float_param
except ImportError:  # pragma: no cover
  from openpilot.selfdrive.car.modules.ALC_module import ALCController
  from openpilot.selfdrive.car.modules.BLNK_module import BLNKController
  from openpilot.selfdrive.car.modules.HSO_module import HSOController
  from openpilot.selfdrive.car.modules.CFG_module import load_bool_param, load_float_param



class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface


def __init__(self, CP, CarController, CarState):
  super().__init__(CP, CarController, CarState)

  # Unity parity sockets/modules
  self.CS.laP = messaging.sub_sock('lateralPlan')
  self.CS.human_control = False

  # Params (Unity-style)
  self.CS._tinkla_enable_alc = load_bool_param("TinklaEnableALC", True)
  self.CS._tinkla_alc_delay = load_float_param("TinklaAlcDelay", 0.75)

  self.CS.alca_controller = ALCController()
  self.CS.blinker_controller = BLNKController()
  self.CS.HSO = HSOController()

def pre_apply(self, c: structs.CarControl, now_nanos: int | None = None) -> None:
  # Update per-frame Unity parity state machines before apply()
  self.CS.lat_plan = messaging.recv_one_or_none(self.CS.laP)

  try:
    self.CS._tinkla_enable_alc = load_bool_param("TinklaEnableALC", True)
    self.CS._tinkla_alc_delay = load_float_param("TinklaAlcDelay", 0.75)
    self.CS.alca_controller.autoStartAlcaDelay = float(self.CS._tinkla_alc_delay)
  except Exception:
    pass

  try:
    self.CS.human_control = bool(self.CS.HSO.update_stat(self.CS, bool(c.latActive), c.actuators, self.frame))
  except Exception:
    self.CS.human_control = False

  try:
    self.CS.blinker_controller.update_state(self.CS, self.frame)
    self.CS.tap_direction = int(getattr(self.CS.blinker_controller, "tap_direction", 0))
  except Exception:
    self.CS.tap_direction = 0

  if bool(getattr(self.CS, "_tinkla_enable_alc", True)):
    try:
      self.CS.alca_controller.autoStartAlcaDelay = float(getattr(self.CS, "_tinkla_alc_delay", 0.75))
      self.CS.alca_controller.update(bool(c.latActive), self.CS, self.frame, getattr(self.CS, "lat_plan", None))
    except Exception:
      pass

def post_update(self, c: structs.CarControl, ret: structs.CarState) -> None:
  # Tap-only blinkers and ALC autostart spoof (Unity parity)
  enable_alc = bool(getattr(self.CS, "_tinkla_enable_alc", True))

  try:
    stalk_released = int(getattr(self.CS, "turnSignalStalkState", 0)) == 0
    tap_dir = int(getattr(self.CS, "tap_direction", 0))
    left_lamp = bool(getattr(self.CS, "leftBlinkerLamp", False))
    right_lamp = bool(getattr(self.CS, "rightBlinkerLamp", False))
    ret.leftBlinker = left_lamp and stalk_released and (tap_dir == 1)
    ret.rightBlinker = right_lamp and stalk_released and (tap_dir == 2)
  except Exception:
    pass

  if enable_alc and bool(getattr(self.CS, "alca_need_engagement", False)):
    try:
      ret.steeringPressed = True
      direction = int(getattr(self.CS, "alca_direction", 0))
      ret.steeringTorque = 0.1 if direction == 1 else (-0.1 if direction == 2 else ret.steeringTorque)
    except Exception:
      pass

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    if candidate in LEGACY_CARS:
      return CarInterface._get_params_sx(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs)

    ret.brand = "tesla"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = False

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

      ret.vEgoStopping = 0.1
      ret.vEgoStarting = 0.1
      ret.stoppingDecelRate = 0.3

    # ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X) # dashcam only, pending find invalidLkasSetting signal

    return ret

  @staticmethod
  def _get_params_sx(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "tesla"

    if not any(0x201 in f for f in fingerprint.values()):
      ret.flags |= TeslaLegacyParams.NO_SDM1.value

    if candidate in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, ):
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW1)),
      ]
    elif candidate in (CAR.TESLA_MODEL_S_HW2,):
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW2)),
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW2 | TeslaSafetyFlags.FLAG_EXTERNAL_PANDA)),
      ]
    elif candidate in (CAR.TESLA_MODEL_S_HW3,):
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW3)),
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW3 | TeslaSafetyFlags.FLAG_EXTERNAL_PANDA)),
      ]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = False

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = candidate in (CAR.TESLA_MODEL_S_HW2, )

    ret.alphaLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = True
    ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

    ret.vEgoStopping = 0.1
    ret.vEgoStarting = 0.1
    ret.stoppingDecelRate = 0.3

    # ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X) # dashcam only, pending find invalidLkasSetting signal

    return ret