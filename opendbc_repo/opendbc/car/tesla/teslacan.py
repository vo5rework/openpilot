from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CANBUS, CarControllerParams
def _crc8_j1850(data: bytes) -> int:
  """CRC-8/J1850 (poly 0x1D, init 0xFF, xorout 0xFF), MSB-first.

  Verified against 660 on-road STW_ACTN_RQ frames (addr 0x045) captured on HW2.
  """
  crc = 0xFF
  for b in data:
    crc ^= b
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) if (crc & 0x80) else (crc << 1)
      crc &= 0xFF
  return crc ^ 0xFF


def create_fake_das_msg(pedal_enabled: bool,
                        autopilot_disabled: bool,
                        bus: int,
                        stalk_main: bool = False,
                        stalk_cancel: bool = False):
  """Internal openpilot->panda carrier (0x659). Safety consumes + blocks it from the car.

  Byte5 bits:
    bit7: autopilot_disabled
    bit5: pedal_enabled
    bit1: stalk_main (edge)
    bit0: stalk_cancel (edge)
  """
  dat = bytearray(8)
  dat[5] = ((0x20 if pedal_enabled else 0) |
            (0x80 if autopilot_disabled else 0) |
            (0x02 if stalk_main else 0) |
            (0x01 if stalk_cancel else 0))
  return (0x659, bytes(dat), bus)


def create_fake_das_message(pedal_enabled: bool,
                            autopilot_disabled: bool,
                            *,
                            stalk_main: bool = False,
                            stalk_cancel: bool = False,
                            bus: int = 0):
  """Compatibility alias for forks expecting create_fake_das_message()."""
  return create_fake_das_msg(pedal_enabled, autopilot_disabled, bus,
                             stalk_main=stalk_main, stalk_cancel=stalk_cancel)

class TeslaCAN:
  def _create_fake_das(self, pedal_enabled: bool, autopilot_disabled: bool, bus: int,
                       stalk_main: bool = False, stalk_cancel: bool = False):
    """Compatibility wrapper expected by some Tesla controller forks."""
    return create_fake_das_msg(pedal_enabled, autopilot_disabled, bus,
                               stalk_main=stalk_main, stalk_cancel=stalk_cancel)

  def __init__(self, packer):
    self.packer = packer

  def create_steering_control(self, angle, enabled):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
    }

    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active, set_speed_kph: float | None = None):
    from opendbc.car.interfaces import V_CRUISE_MAX

    if set_speed_kph is not None:
      set_speed = float(max(0.0, min(float(set_speed_kph), V_CRUISE_MAX)))
    else:
      set_speed = max(v_ego * CV.MS_TO_KPH, 0.0)
      if active:
        # TODO: this causes jerking after gas override when above set speed
        set_speed = 0.0 if accel < 0 else V_CRUISE_MAX

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": counter,
    }
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)
  def create_steering_allowed(self, counter):
    values = {
      "APS_eacAllow": 1,
    }

    return self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)

  def create_stalk_request(self,
                           bus: int,
                           msg_stw_actn_req: dict | None,
                           *,
                           cruise_button: int | None = None,
                           turn_signal_stalk_state: int | None = None) -> tuple[int, int, bytes]:
    """Create STW_ACTN_RQ from the latest observed seed frame."""
    values = dict(msg_stw_actn_req or {})
    if cruise_button is not None:
      values["SpdCtrlLvr_Stat"] = int(cruise_button)
    if turn_signal_stalk_state is not None:
      values["TurnIndLvr_Stat"] = int(turn_signal_stalk_state)

    counter = (int(values.get("MC_STW_ACTN_RQ", 0)) + 1) % 16
    values["MC_STW_ACTN_RQ"] = counter
    values["CRC_STW_ACTN_RQ"] = 0

    msg = self.packer.make_can_msg("STW_ACTN_RQ", int(bus), values)
    dat = msg[1]
    values["CRC_STW_ACTN_RQ"] = _crc8_j1850(dat[:7])
    return self.packer.make_can_msg("STW_ACTN_RQ", int(bus), values)

  def create_action_request(self, bus: int, msg_stw_actn_req: dict, cruise_button: int) -> tuple[int, int, bytes]:
    """Create STW_ACTN_RQ to emulate cruise stalk up/down/cancel (Unity parity)."""
    return self.create_stalk_request(int(bus), msg_stw_actn_req, cruise_button=int(cruise_button))



  def create_body_controls_message(self, turn: int, hazard: int, bus: int, counter: int = 1):
    values = {
      "DAS_headlightRequest": 0,
      "DAS_hazardLightRequest": int(hazard),
      "DAS_wiperSpeed": 0,
      "DAS_turnIndicatorRequest": int(turn),
      "DAS_highLowBeamDecision": 3,
      "DAS_highLowBeamOffReason": 5,
      "DAS_turnIndicatorRequestReason": 1 if int(turn) > 0 else 0,
      "DAS_bodyControlsCounter": int(counter),
      "DAS_bodyControlsChecksum": 0,
    }
    return self.packer.make_can_msg("DAS_bodyControls", int(bus), values)


  def create_das_status(self,
                        das_op_status: int,
                        collision_warning: int,
                        lane_departure_warning: int,
                        hands_on_state: int,
                        alca_state: int,
                        blind_spot_left: bool,
                        blind_spot_right: bool,
                        speed_limit_uom: float,
                        das_csa_state: int,
                        fleet_speed_state: int,
                        bus: int,
                        counter: int):
    values = {
      "autopilotStatus": int(das_op_status),
      "DAS_blindSpotRearLeft": 1 if bool(blind_spot_left) else 0,
      "DAS_blindSpotRearRight": 1 if bool(blind_spot_right) else 0,
      "DAS_fusedSpeedLimit": float(max(speed_limit_uom, 0.0)),
      "DAS_suppressSpeedWarning": 0,
      "DAS_summonObstacle": 0,
      "DAS_summonClearedGate": 0,
      "DAS_visionOnlySpeedLimit": float(max(speed_limit_uom, 0.0)),
      "DAS_heaterState": 0,
      "DAS_forwardCollisionWarning": int(collision_warning),
      "DAS_autoparkReady": 0,
      "DAS_autoParked": 0,
      "DAS_autoparkWaitingForBrake": 0,
      "DAS_summonFwdLeashReached": 0,
      "DAS_summonRvsLeashReached": 0,
      "DAS_sideCollisionAvoid": 0,
      "DAS_sideCollisionWarning": 0,
      "DAS_sideCollisionInhibit": 0,
      "DAS_csaState": int(das_csa_state),
      "DAS_laneDepartureWarning": int(lane_departure_warning),
      "DAS_fleetSpeedState": int(fleet_speed_state),
      "DAS_autopilotHandsOnState": int(hands_on_state),
      "DAS_autoLaneChangeState": int(alca_state),
      "DAS_summonAvailable": 0,
      "DAS_statusCounter": int(counter) & 0xF,
      "DAS_statusChecksum": 0,
    }
    return self.packer.make_can_msg("AutopilotStatus", int(bus), values)

  def create_das_status2(self,
                         das_csa_state: int,
                         acc_speed_limit_mph: float,
                         fcw: int,
                         bus: int,
                         counter: int):
    fcw_sig = 0x0F if int(fcw) == 0 else 0x01
    values = {
      "DAS_accSpeedLimit": float(max(acc_speed_limit_mph, 0.0)),
      "DAS_pmmObstacleSeverity": 0,
      "DAS_pmmLoggingRequest": 0,
      "DAS_activationFailureStatus": 0,
      "DAS_pmmUltrasonicsFaultReason": 0,
      "DAS_pmmRadarFaultReason": 0,
      "DAS_pmmSysFaultReason": 0,
      "DAS_pmmCameraFaultReason": 0,
      "DAS_ACC_report": 1,
      "DAS_lssState": int(das_csa_state),
      "DAS_radarTelemetry": 1,
      "DAS_robState": 2,
      "DAS_driverInteractionLevel": 0,
      "DAS_ppOffsetDesiredRamp": 0.0,
      "DAS_longCollisionWarning": int(fcw_sig),
      "DAS_status2Counter": int(counter) & 0xF,
      "DAS_status2Checksum": 0,
    }
    return self.packer.make_can_msg("DAS_status2", int(bus), values)



  def create_das_warning_matrix0(self, can_errors: int, steering_override: int, not_in_drive: int, bus: int):
    dat = bytes((
      0,
      0,
      0,
      (int(steering_override) & 0x7F) | ((int(can_errors) & 0x01) << 7),
      0,
      ((int(not_in_drive) & 0x01) << 7),
      0,
      0,
    ))
    return (0x329, dat, int(bus))

  def create_das_warning_matrix1(self, bus: int):
    return (0x369, b"\x00\x00\x00\x00\x00\x00\x00\x00", int(bus))

  def create_das_warning_matrix3(self,
                                 gas_to_resume: int,
                                 acc_no_seatbelt: int,
                                 noisy_environment: int,
                                 ap_unavailable: int,
                                 lkas_unavailable: int,
                                 lc_temp_unavailable_speed: int,
                                 lc_temp_unavailable_road: int,
                                 lc_aborting: int,
                                 acc_camera_blind: int,
                                 rack_detected: int,
                                 driver_overriding: int,
                                 stop_sign_warning: int,
                                 stop_light_warning: int,
                                 bus: int):
    dat = bytes((
      ((int(gas_to_resume) & 0x01) << 1) |
      ((int(stop_sign_warning) & 0x01) << 3) |
      ((int(stop_light_warning) & 0x01) << 4),
      ((int(noisy_environment) & 0x01) << 1) |
      ((int(ap_unavailable) & 0x01) << 5) |
      ((int(lkas_unavailable) & 0x01) << 6) |
      ((int(rack_detected) & 0x01) << 7),
      ((int(acc_no_seatbelt) & 0x01) << 2) |
      ((int(driver_overriding) & 0x01) << 7),
      ((int(lc_temp_unavailable_speed) & 0x01) << 2) |
      ((int(lc_temp_unavailable_road) & 0x01) << 3) |
      ((int(lc_aborting) & 0x01) << 4) |
      ((int(acc_camera_blind) & 0x01) << 5),
      0,
      0,
      0,
      0,
    ))
    return (0x349, dat, int(bus))

  def create_fake_das_msg(self, pedalEnabled: bool, autopilot_disabled: bool, bus: int = CANBUS.party, *,


                          stalk_main: bool = False, stalk_cancel: bool = False):


    """Internal openpilot->panda msg (0x659). Safety consumes + blocks it from the car.


    Byte5 bits: bit7=autopilot_disabled, bit5=pedalEnabled, bit1=stalk_main(edge), bit0=stalk_cancel(edge)


    """


    dat = bytearray(8)


    dat[5] = ((0x20 if pedalEnabled else 0) |


              (0x80 if autopilot_disabled else 0) |


              (0x02 if stalk_main else 0) |


              (0x01 if stalk_cancel else 0))


    return (0x659, bytes(dat), bus)



def tesla_checksum(address: int, sig, d: bytearray) -> int:
  """Checksum used by opendbc dbc packer for Tesla frames."""
  checksum = (address & 0xFF) + ((address >> 8) & 0xFF)
  checksum_byte = sig.start_bit // 8
  for i in range(len(d)):
    if i != checksum_byte:
      checksum += d[i]
  return checksum & 0xFF

