import copy

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.values import DBC, CANBUS, GEAR_MAP, STEER_THRESHOLD, CAR, TeslaLegacyParams, LEGACY_CARS

from openpilot.common.params import Params

ButtonType = structs.CarState.ButtonEvent.Type

# SpdCtrlLvr_Stat values from Tesla DBCs (tesla_can.dbc)
_CRUISE_BTN_IDLE = 0
_CRUISE_BTN_CANCEL = 1   # FWD
_CRUISE_BTN_MAIN = 2     # RWD
_CRUISE_BTN_UP_2ND = 4
_CRUISE_BTN_DN_2ND = 8
_CRUISE_BTN_UP_1ST = 16
_CRUISE_BTN_DN_1ST = 32





class _BlinkerTapDetector:
  """Unity parity: turn signal stalk held <= 550ms counts as a 'tap'."""

  def __init__(self, tap_duration_frames: int = 55):
    self._tap_duration_frames = tap_duration_frames
    self.tap_direction = 0
    self._start_frame = 0
    self._end_frame = 0
    self._prev_stalk_state = 0

  def update(self, stalk_state: int, lamp_active: bool, frame: int) -> int:
    # opposite direction cancels
    if self.tap_direction > 0 and stalk_state > 0 and self.tap_direction != stalk_state:
      self._reset()

    if stalk_state > 0 and self._prev_stalk_state == 0:
      self._start_frame = frame
    elif stalk_state == 0 and self._prev_stalk_state > 0:
      if frame - self._start_frame <= self._tap_duration_frames:
        self.tap_direction = self._prev_stalk_state
        self._end_frame = frame
      else:
        self._reset()

    # stop maintaining once lamps stop flashing
    if self.tap_direction > 0 and stalk_state == 0 and not lamp_active:
      self._reset()

    self._prev_stalk_state = stalk_state
    return self.tap_direction

  def _reset(self) -> None:
    self.tap_direction = 0
    self._start_frame = 0
    self._end_frame = 0

class _TinklaParamCache:
  """Lightweight Params cache for carstate hotpath."""
  def __init__(self) -> None:
    self._params = Params()
    self._frame = 0

    self.autopilot_disabled = False
    self.adjust_acc_with_speed_limit = True
    self.speed_limit_use_relative = False
    self.speed_limit_offset = 0.0  # uom (mph/kph) or percent if relative

  def tick(self) -> None:
    self._frame += 1
    if (self._frame % 10) != 0:
      return

    self.autopilot_disabled = self._params.get_bool("TinklaAutopilotDisabled")
    self.adjust_acc_with_speed_limit = self._params.get_bool("TinklaAdjustAccWithSpeedLimit")
    self.speed_limit_use_relative = self._params.get_bool("TinklaSpeedLimitUseRelative")

    try:
      v = self._params.get("TinklaSpeedLimitOffset", return_default=True)
      self.speed_limit_offset = float(v) if v is not None else 0.0
    except Exception:
      self.speed_limit_offset = 0.0


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.ignore_stock_aeb = False

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
    self._frame = 0
    self._blinker_tap = _BlinkerTapDetector()
    self._turn_signal_stalk_state = 0

    self._tinkla = _TinklaParamCache()
    self._manual_cruise_enabled = False
    self._manual_cruise_speed_kph = 0.0
    self._prev_cruise_button = _CRUISE_BTN_IDLE

    self._speed_limit_ms = 0.0

  def update_autopark_state(self, autopark_state: str, cruise_enabled: bool):
    autopark_now = autopark_state in ("ACTIVE", "COMPLETE", "SELFPARK_STARTED")
    if autopark_now and not self.autopark_prev and not self.cruise_enabled_prev:
      self.autopark = True
    if not autopark_now:
      self.autopark = False
    self.autopark_prev = autopark_now
    self.cruise_enabled_prev = cruise_enabled

  @staticmethod
  def _safe_define_lookup(defines, msg, sig, raw, default=None):
    try:
      return defines[msg][sig].get(int(raw), default)
    except Exception:
      return default

  def _get_speed_units(self, cp, defines) -> str:
    try:
      return defines["DI_state"]["DI_speedUnits"].get(int(cp.vl["DI_state"]["DI_speedUnits"]), "MPH")
    except Exception:
      return "MPH"

  def _update_speed_limit(self, cp) -> None:
    """Reads Tesla map/sign speed limit signals when available (Unity parity)."""
    try:
      msu = int(cp.vl["UI_gpsVehicleSpeed"]["UI_mapSpeedLimitUnits"])
    except Exception:
      return

    map_uom_to_ms = CV.KPH_TO_MS if msu == 1 else CV.MPH_TO_MS
    map_ms_to_uom = CV.MS_TO_KPH if msu == 1 else CV.MS_TO_MPH

    try:
      speed_limit_type = int(cp.vl["UI_driverAssistMapData"]["UI_mapSpeedLimit"])
    except Exception:
      speed_limit_type = 0

    base_map_speed_limit_mps = 0.0
    try:
      rd_sign = int(cp.vl["UI_driverAssistRoadSign"]["UI_roadSign"])
      if rd_sign == 3:  # ROAD_SIGN_SPEED_LIMIT
        base_map_speed_limit_mps = float(cp.vl["UI_driverAssistRoadSign"]["UI_baseMapSpeedLimitMPS"])
        base_map_speed_limit_mps = int(base_map_speed_limit_mps * map_ms_to_uom + 0.99) / map_ms_to_uom
    except Exception:
      pass

    speed_limit_ms = 0.0
    try:
      if base_map_speed_limit_mps > 0.0 and (speed_limit_type != 0x1F or base_map_speed_limit_mps >= 5.56):
        speed_limit_ms = base_map_speed_limit_mps
      else:
        speed_limit_ms = float(cp.vl["UI_gpsVehicleSpeed"]["UI_mppSpeedLimit"]) * map_uom_to_ms
    except Exception:
      pass

    self._speed_limit_ms = float(max(speed_limit_ms, 0.0))

  def _calc_speed_limit_target_ms(self, speed_units: str) -> float:
    if self._speed_limit_ms <= 0.0:
      return 0.0

    if self._tinkla.speed_limit_use_relative:
      offset_ms = (self._tinkla.speed_limit_offset / 100.0) * self._speed_limit_ms
    else:
      offset_ms = (self._tinkla.speed_limit_offset * CV.KPH_TO_MS) if speed_units == "KPH" else (self._tinkla.speed_limit_offset * CV.MPH_TO_MS)

    return max(self._speed_limit_ms + offset_ms, 0.0)

  def _update_manual_cruise_from_stalk(self, cp, speed_units: str, v_ego_kph: float) -> None:
    try:
      btn = int(cp.vl["STW_ACTN_RQ"]["SpdCtrlLvr_Stat"])
    except Exception:
      return

    if btn == self._prev_cruise_button:
      return
    self._prev_cruise_button = btn

    if btn == _CRUISE_BTN_IDLE:
      return

    if btn == _CRUISE_BTN_MAIN:
      self._manual_cruise_enabled = True
      if self._manual_cruise_speed_kph <= 0.1:
        self._manual_cruise_speed_kph = max(v_ego_kph, 0.0)
      return

    if btn == _CRUISE_BTN_CANCEL:
      self._manual_cruise_enabled = False
      return

    step_uom = 5.0 if btn in (_CRUISE_BTN_UP_2ND, _CRUISE_BTN_DN_2ND) else 1.0
    step_kph = step_uom if speed_units == "KPH" else step_uom * CV.MPH_TO_KPH

    if btn in (_CRUISE_BTN_UP_1ST, _CRUISE_BTN_UP_2ND):
      self._manual_cruise_speed_kph += step_kph
    elif btn in (_CRUISE_BTN_DN_1ST, _CRUISE_BTN_DN_2ND):
      self._manual_cruise_speed_kph -= step_kph

    self._manual_cruise_speed_kph = max(self._manual_cruise_speed_kph, 0.0)

  def _apply_manual_cruise_state(self, ret: structs.CarState, speed_units: str) -> None:
    ret.cruiseState.available = True
    ret.cruiseState.enabled = self._manual_cruise_enabled and (not ret.doorOpen) and (not ret.seatbeltUnlatched) and (ret.gearShifter == structs.CarState.GearShifter.drive)

    if self._manual_cruise_speed_kph <= 0.1:
      self._manual_cruise_speed_kph = max(ret.vEgo * CV.MS_TO_KPH, 0.0)

    # Unity parity: auto-adjust & auto-lower when limit drops
    if self._tinkla.adjust_acc_with_speed_limit and ret.cruiseState.enabled:
      tgt = self._calc_speed_limit_target_ms(speed_units)
      if tgt > 0.0:
        self._manual_cruise_speed_kph = tgt * CV.MS_TO_KPH

    ret.cruiseState.speed = max(self._manual_cruise_speed_kph * CV.KPH_TO_MS, 1e-3)
    ret.cruiseState.standstill = ret.standstill

  def update(self, can_parsers) -> structs.CarState:
    self._tinkla.tick()
    if self.CP.carFingerprint in LEGACY_CARS:
      return self.update_legacy(can_parsers)

    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    ret = structs.CarState()

    self._frame += 1

    # Vehicle speed
    ret.vEgoRaw = cp_party.vl["DI_speed"]["DI_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = False

    # Gas pedal
    ret.gasPressed = cp_party.vl["DI_systemStatus"]["DI_accelPedalPos"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_party.vl["ESP_status"]["ESP_driverBrakeApply"] == 2

    # Steering wheel
    epas_status = cp_party.vl["EPAS3S_sysStatus"]
    self.hands_on_level = epas_status["EPAS3S_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status["EPAS3S_internalSAS"]
    ret.steeringRateDeg = -cp_ap_party.vl["SCCM_steeringAngleSensor"]["SCCM_steeringAngleSpeed"]
    ret.steeringTorque = -epas_status["EPAS3S_torsionBarTorque"]

    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    eac_status = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacStatus"].get(int(epas_status["EPAS3S_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    ret.steerFaultTemporary = eac_status == "EAC_INHIBITED"

    eac_error_code = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacErrorCode"].get(int(epas_status["EPAS3S_eacErrorCode"]), None)
    ret.steeringDisengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    # Doors / blinkers / seatbelt / gear (needed for manual cruise gating)
    ret.gearShifter = GEAR_MAP[self.can_define.dv["DI_systemStatus"]["DI_gear"].get(int(cp_party.vl["DI_systemStatus"]["DI_gear"]), "DI_GEAR_INVALID")]
    ret.doorOpen = cp_party.vl["UI_warning"]["anyDoorOpen"] == 1
    # Blinkers (Unity ALCA parity)
    lamp_left = cp_chassis.vl["GTW_carState"]["BC_indicatorLStatus"] == 1
    lamp_right = cp_chassis.vl["GTW_carState"]["BC_indicatorRStatus"] == 1

    # TurnIndLvr_Stat: 0 idle, 1 left, 2 right, 3 SNA
    try:
      stalk_raw = int(cp_chassis.vl["STW_ACTN_RQ"]["TurnIndLvr_Stat"])
    except Exception:
      stalk_raw = 0

    stalk_state = 0 if stalk_raw == 3 else stalk_raw

    self._turn_signal_stalk_state = stalk_state
    self._blinker_tap.update(stalk_state, lamp_left or lamp_right, self._frame)

    ret.leftBlinker = lamp_left and (stalk_state == 0) and (self._blinker_tap.tap_direction == 1)
    ret.rightBlinker = lamp_right and (stalk_state == 0) and (self._blinker_tap.tap_direction == 2)


    if self.CP.flags & TeslaLegacyParams.NO_SDM1:
      ret.seatbeltUnlatched = cp_chassis.vl["RCM_status"]["RCM_buckleDriverStatus"] != 1
    else:
      ret.seatbeltUnlatched = cp_chassis.vl["SDM1"]["SDM_bcklDrivStatus"] != 1

    # Speed limit signals (optional)
    self._update_speed_limit(cp_chassis)

    # Cruise state
    cruise_state = self.can_defines["DI_state"]["DI_cruiseState"].get(int(cp_chassis.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self._get_speed_units(cp_chassis, self.can_defines)
    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")

    if self._tinkla.autopilot_disabled:
      self._update_manual_cruise_from_stalk(cp_chassis, speed_units, ret.vEgo * CV.MS_TO_KPH)
      self._apply_manual_cruise_state(ret, speed_units)
    else:
      ret.cruiseState.enabled = cruise_enabled
      if speed_units == "KPH":
        ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
      else:
        ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)
      ret.cruiseState.available = (cruise_state == "STANDBY" or ret.cruiseState.enabled)
      ret.cruiseState.standstill = False

    ret.standstill = cruise_state == "STANDSTILL"
    ret.accFaulted = cruise_state == "FAULT"

    # AEB
    ret.stockAeb = cp_ap_pt.vl["DAS_control"]["DAS_aebEvent"] == 1

    # LKAS
    ret.stockLkas = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == 2

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap_pt.vl["DAS_control"])

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
      Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.autopilot_party),
    }
