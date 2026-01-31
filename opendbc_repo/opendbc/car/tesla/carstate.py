import copy
import time
from dataclasses import dataclass

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.values import (
  DBC,
  CANBUS,
  GEAR_MAP,
  STEER_THRESHOLD,
  CAR,
  TeslaLegacyParams,
  LEGACY_CARS,
  CruiseButtons,
)

from openpilot.common.params import Params

ButtonType = structs.CarState.ButtonEvent.Type


@dataclass(slots=True)
class _TinklaState:
  autopilot_disabled: bool = False
  adjust_acc_with_speed_limit: bool = True
  speed_limit_use_relative: bool = False
  speed_limit_offset_uom: float = 0.0
  enable_alc: bool = False



class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.party])

    if self.CP.carFingerprint in LEGACY_CARS:
      if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
        CANBUS.chassis = 1
        CANBUS.radar = 5
      elif self.CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, ):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.can_define_party = CANDefine(DBC[CP.carFingerprint][Bus.party])
      self.can_define_pt = CANDefine(DBC[CP.carFingerprint][Bus.pt])
      self.can_define_chassis = CANDefine(DBC[CP.carFingerprint][Bus.chassis])
      self.can_defines = {
        **self.can_define_party.dv,
        **self.can_define_pt.dv,
        **self.can_define_chassis.dv,
      }
      self.shifter_values = self.can_defines["DI_torque2"]["DI_gear"]
    else:
      self.shifter_values = self.can_define.dv["DI_systemStatus"]["DI_gear"]

    self.autopark = False
    self.autopark_prev = False
    self.cruise_enabled_prev = False

    self.hands_on_level = 0
    self.das_control = None

    # --- Unity parity hooks / controller dependencies ---
    self.params = Params()
    self._tinkla = _TinklaState()
    self._tinkla_last_refresh_ts = 0.0

    # Needed by Tesla CarController for legacy virtual stalk messages
    self.msg_stw_actn_req = None

    # Speed limit matching support (uom = mph/kph depending on speed_units)
    self.speed_units = "MPH"
    self.speed_limit_uom = 0.0
    self.user_speed_limit_offset_uom = 0.0

    # Autopilot-disabled "lat-only cruise" latch (Unity-style)
    self.cruiseEnabled = False
    self._cruise_btn_prev = CruiseButtons.IDLE

    self.tap_direction = 0


  def _refresh_tinkla(self) -> None:
    now = time.monotonic()
    if (now - self._tinkla_last_refresh_ts) < 1.0:
      return
    self._tinkla_last_refresh_ts = now

    p = self.params
    self._tinkla.autopilot_disabled = bool(p.get_bool("TinklaAutopilotDisabled"))
    self._tinkla.adjust_acc_with_speed_limit = bool(p.get_bool("TinklaAdjustAccWithSpeedLimit"))
    self._tinkla.speed_limit_use_relative = bool(p.get_bool("TinklaSpeedLimitUseRelative"))
    self._tinkla.enable_alc = False


    try:
      raw = p.get("TinklaSpeedLimitOffset")
      self._tinkla.speed_limit_offset_uom = float(raw.decode("utf-8")) if raw is not None else 0.0
    except Exception:
      self._tinkla.speed_limit_offset_uom = 0.0



  def _calc_speed_limit_target_ms(self, speed_units: str) -> float:
    if self.speed_limit_uom <= 0.0:
      return 0.0

    offset_uom = self.user_speed_limit_offset_uom if self._tinkla.speed_limit_use_relative else self._tinkla.speed_limit_offset_uom
    target_uom = float(self.speed_limit_uom) + float(offset_uom)

    if speed_units == "KPH":
      return target_uom * CV.KPH_TO_MS
    return target_uom * CV.MPH_TO_MS

  def update_autopark_state(self, autopark_state: str, cruise_enabled: bool):
    autopark_now = autopark_state in ("ACTIVE", "COMPLETE", "SELFPARK_STARTED")
    if autopark_now and not self.autopark_prev and not self.cruise_enabled_prev:
      self.autopark = True
    if not autopark_now:
      self.autopark = False
    self.autopark_prev = autopark_now
    self.cruise_enabled_prev = cruise_enabled

  def update(self, can_parsers) -> structs.CarState:
    self._refresh_tinkla()
    if self.CP.carFingerprint in LEGACY_CARS:
      return self.update_legacy(can_parsers)

    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp_party.vl["DI_speed"]["DI_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Gas pedal
    ret.gasPressed = cp_party.vl["DI_torque1"]["DI_pedalPos"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_party.vl["BrakeMessage"]["driverBrakeStatus"] == 2

    # Steering wheel
    ret.steeringAngleDeg = -cp_party.vl["EPAS_sysStatus"]["EPAS_internalSAS"]
    ret.steeringRateDeg = -cp_party.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"]
    ret.steeringTorque = -cp_party.vl["EPAS_sysStatus"]["EPAS_torsionBarTorque"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    # Cruise + speed units
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp_party.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(int(cp_party.vl["DI_state"]["DI_speedUnits"]), None)
    self.speed_units = speed_units or "MPH"

    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    ret.cruiseState.enabled = cruise_enabled
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False

    # Unity-style autopilot-disabled latch (safe no-op if STW missing)
    if self._tinkla.autopilot_disabled:
      try:
        cruise_btn = int(cp_party.vl["STW_ACTN_RQ"]["SpdCtrlLvr_Stat"])
        if cruise_btn != CruiseButtons.IDLE and self._cruise_btn_prev == CruiseButtons.IDLE:
          self.cruiseEnabled = cruise_btn != CruiseButtons.CANCEL
        self._cruise_btn_prev = cruise_btn
      except Exception:
        pass
      if ret.brakePressed:
        self.cruiseEnabled = False
      ret.cruiseState.available = True
      ret.cruiseState.enabled = bool(self.cruiseEnabled)

    if speed_units == "KPH":
      ret.cruiseState.speed = max(cp_party.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
    elif speed_units == "MPH":
      ret.cruiseState.speed = max(cp_party.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_define.dv["DI_systemStatus"]["DI_gear"].get(int(cp_party.vl["DI_systemStatus"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Doors
    DOORS = ["DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR", "DOOR_STATE_FrontTrunk", "BOOT_STATE"]
    ret.doorOpen = any((self.can_define.dv["GTW_carState"][door].get(int(cp_party.vl["GTW_carState"][door]), "OPEN") == "OPEN") for door in DOORS)

    ret.leftBlinker = cp_party.vl["GTW_carState"]["BC_indicatorLStatus"] == 1
    ret.rightBlinker = cp_party.vl["GTW_carState"]["BC_indicatorRStatus"] == 1


    # Messages needed by carcontroller (modern)
    try:
      self.das_control = copy.copy(cp_party.vl["DAS_control"])
    except Exception:
      self.das_control = None
    try:
      self.msg_stw_actn_req = copy.copy(cp_party.vl["STW_ACTN_RQ"])
    except Exception:
      self.msg_stw_actn_req = None

    # Speed limit sources (controller uses _calc_speed_limit_target_ms)
    try:
      ui_sl = float(cp_party.vl["UI_gpsVehicleSpeed"]["UI_mppSpeedLimit"]) * 5.0
    except Exception:
      ui_sl = 0.0
    try:
      das_sl = float(cp_ap_party.vl["AutopilotStatus"]["DAS_fusedSpeedLimit"]) * 5.0
    except Exception:
      das_sl = 0.0
    sl_candidates = [v for v in (ui_sl, das_sl) if v > 0.0]
    self.speed_limit_uom = float(min(sl_candidates)) if sl_candidates else 0.0
    try:
      self.user_speed_limit_offset_uom = float(cp_party.vl["UI_gpsVehicleSpeed"]["UI_userSpeedOffset"])
    except Exception:
      self.user_speed_limit_offset_uom = 0.0

    return ret

  def update_legacy(self, can_parsers) -> structs.CarState:
    self._refresh_tinkla()

    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    cp_pt = can_parsers[Bus.pt]
    cp_ap_pt = can_parsers[Bus.ap_pt]
    cp_chassis = can_parsers[Bus.chassis]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp_chassis.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Gas pedal
    ret.gasPressed = cp_pt.vl["DI_torque1"]["DI_pedalPos"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_chassis.vl["BrakeMessage"]["driverBrakeStatus"] == 2

    # Steering wheel
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
      epas_status = cp_party.vl["EPAS_sysStatus"]
    else:
      epas_status = cp_chassis.vl["EPAS_sysStatus"]
    self.hands_on_level = epas_status["EPAS_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status["EPAS_internalSAS"]
    ret.steeringRateDeg = -cp_chassis.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"]
    ret.steeringTorque = -epas_status["EPAS_torsionBarTorque"]

    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    eac_status = self.can_defines["EPAS_sysStatus"]["EPAS_eacStatus"].get(int(epas_status["EPAS_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    ret.steerFaultTemporary = eac_status == "EAC_INHIBITED"

    eac_error_code = self.can_defines["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(int(epas_status["EPAS_eacErrorCode"]), None)
    ret.steeringDisengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                         eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_defines["DI_torque2"]["DI_gear"].get(int(cp_chassis.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Cruise state + units
    cruise_state = self.can_defines["DI_state"]["DI_cruiseState"].get(int(cp_chassis.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_defines["DI_state"]["DI_speedUnits"].get(int(cp_chassis.vl["DI_state"]["DI_speedUnits"]), None)
    self.speed_units = speed_units or "MPH"

    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")

    ret.cruiseState.enabled = cruise_enabled
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False
    ret.standstill = cruise_state == "STANDSTILL"
    ret.accFaulted = cruise_state == "FAULT"

    if speed_units == "KPH":
      ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
    elif speed_units == "MPH":
      ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)

    # Unity-style autopilot-disabled latch: allow "enabled" without Tesla cruise
    cruise_btn = int(cp_chassis.vl["STW_ACTN_RQ"]["SpdCtrlLvr_Stat"])
    if cruise_btn != CruiseButtons.IDLE and self._cruise_btn_prev == CruiseButtons.IDLE:
      if cruise_btn == CruiseButtons.CANCEL:
        self.cruiseEnabled = False
      else:
        self.cruiseEnabled = True
    self._cruise_btn_prev = cruise_btn

    if ret.brakePressed or ret.gearShifter != structs.CarState.GearShifter.drive:
      self.cruiseEnabled = False

    if self._tinkla.autopilot_disabled:
      ret.cruiseState.available = True
      ret.cruiseState.enabled = bool(self.cruiseEnabled)

    # Doors
    DOORS = ["DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR", "DOOR_STATE_FrontTrunk", "BOOT_STATE"]
    ret.doorOpen = any((self.can_defines["GTW_carState"][door].get(int(cp_chassis.vl["GTW_carState"][door]), "OPEN") == "OPEN") for door in DOORS)

    ret.leftBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorLStatus"] == 1
    ret.rightBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorRStatus"] == 1


    # Seatbelt
    if self.CP.flags & TeslaLegacyParams.NO_SDM1:
      ret.seatbeltUnlatched = cp_chassis.vl["RCM_status"]["RCM_buckleDriverStatus"] != 1
    else:
      ret.seatbeltUnlatched = cp_chassis.vl["SDM1"]["SDM_bcklDrivStatus"] != 1

    # AEB
    ret.stockAeb = cp_ap_pt.vl["DAS_control"]["DAS_aebEvent"] == 1

    # LKAS
    ret.stockLkas = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == 2

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap_pt.vl["DAS_control"])
    self.msg_stw_actn_req = copy.copy(cp_chassis.vl["STW_ACTN_RQ"])

    # Speed limit sources
    try:
      ui_sl = float(cp_chassis.vl["UI_gpsVehicleSpeed"]["UI_mppSpeedLimit"]) * 5.0
    except Exception:
      ui_sl = 0.0
    try:
      das_sl = float(cp_ap_party.vl["AutopilotStatus"]["DAS_fusedSpeedLimit"]) * 5.0
    except Exception:
      das_sl = 0.0
    sl_candidates = [v for v in (ui_sl, das_sl) if v > 0.0]
    self.speed_limit_uom = float(min(sl_candidates)) if sl_candidates else 0.0
    try:
      self.user_speed_limit_offset_uom = float(cp_chassis.vl["UI_gpsVehicleSpeed"]["UI_userSpeedOffset"])
    except Exception:
      self.user_speed_limit_offset_uom = 0.0

    return ret

  @staticmethod
  def get_can_parsers(CP):
    if CP.carFingerprint in LEGACY_CARS:
      return {
        Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.party),
        Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.autopilot_party),
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CANBUS.powertrain),
        Bus.ap_pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CANBUS.autopilot_powertrain),
        Bus.chassis: CANParser(DBC[CP.carFingerprint][Bus.chassis], [], CANBUS.chassis if CP.carFingerprint == CAR.TESLA_MODEL_S_HW3 else CANBUS.party),
      }

    return {
      Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.party),
      Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.autopilot_party)
    }
