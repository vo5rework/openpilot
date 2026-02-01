"""Tesla CarInterface (xnor-c3) with legacy-only Unity parity latch.

Copy this file to BOTH:
  - opendbc/car/tesla/interface.py
  - opendbc_repo/opendbc/car/tesla/interface.py

Legacy-only additions:
  - Capture latest STW_ACTN_RQ into CS.msg_stw_actn_req (enables legacy virtual stalk emulation).
  - Unity-style cruiseEnabled latch (allows engagement under Tesla's low-speed cruise restriction).
"""

import copy

from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.radar_interface import RadarInterface
from opendbc.car.tesla.values import CAR, LEGACY_CARS, TeslaLegacyParams, TeslaSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  def __init__(self, CP: structs.CarParams):
    super().__init__(CP)
    self._unity_cruise_enabled = False

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    if candidate in LEGACY_CARS:
      return CarInterface._get_params_sx(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs)

    ret.brand = "tesla"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

      ret.vEgoStopping = 0.1
      ret.vEgoStarting = 0.1
      ret.stoppingDecelRate = 0.3

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

    # Unity C3 parity for legacy S/X: reduce EPS faults from low-speed/standstill steering
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

    return ret

  def update(self, can_packets):
    ret = super().update(can_packets)

    if self.CP.carFingerprint in LEGACY_CARS:
      self._legacy_capture_action_request()
      self._legacy_apply_unity_cruise_latch(ret)

    return ret

  def _legacy_capture_action_request(self) -> None:
    """Keep a copy of the latest STW_ACTN_RQ for legacy virtual stalk emulation."""
    try:
      cp_chassis = self.can_parsers[Bus.chassis]
      msg = cp_chassis.vl.get("STW_ACTN_RQ", None)
      if isinstance(msg, dict) and msg:
        self.CS.msg_stw_actn_req = copy.copy(msg)
    except Exception:
      pass

    # Expose speed units for conversions (best-effort).
    try:
      cp_chassis = self.can_parsers[Bus.chassis]
      raw = int(cp_chassis.vl["DI_state"]["DI_speedUnits"])
      units = self.CS.can_defines["DI_state"]["DI_speedUnits"].get(raw, None)
      if units in ("MPH", "KPH"):
        self.CS.speed_units = units
    except Exception:
      pass

  def _legacy_apply_unity_cruise_latch(self, ret: structs.CarState) -> None:
    """Unity parity: allow engagement even when Tesla won't enable cruise under ~18mph."""
    try:
      if getattr(self.CS, "autopark", False):
        self._unity_cruise_enabled = False

      cp_chassis = self.can_parsers[Bus.chassis]
      stw = None
      spd = None

      try:
        raw_stw = int(cp_chassis.vl["STW_ACTN_RQ"]["StW_Lvr_Stat"])
        stw = self.CS.can_defines["STW_ACTN_RQ"]["StW_Lvr_Stat"].get(raw_stw, None)
      except Exception:
        stw = None

      try:
        raw_spd = int(cp_chassis.vl["STW_ACTN_RQ"]["SpdCtrlLvr_Stat"])
        spd = self.CS.can_defines["STW_ACTN_RQ"]["SpdCtrlLvr_Stat"].get(raw_spd, None)
      except Exception:
        spd = None

      # Driver cancel / brake cancels latch
      if stw == "STW_BACK" or bool(getattr(ret, "brakePressed", False)):
        self._unity_cruise_enabled = False

      # Any forward/up/down or speed lever action enables latch (MAIN/RES/SET/speed +/-)
      if stw in ("STW_FWD", "STW_UP", "STW_DOWN") or spd in ("FWD", "RWD", "UP_1ST", "DN_1ST", "UP_2ND", "DN_2ND"):
        self._unity_cruise_enabled = True

      # If Tesla cruise is actually enabled, keep our latch enabled
      if bool(getattr(ret.cruiseState, "enabled", False)):
        self._unity_cruise_enabled = True

      ret.cruiseState.available = True
      ret.cruiseState.enabled = bool(self._unity_cruise_enabled)
    except Exception:
      pass
