import copy

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CANBUS, CarControllerParams


class TeslaCAN:
  def __init__(self, packer):
    self.packer = packer
  @staticmethod
  def _crc_stw_actn_req(data: bytes) -> int:
    """CRC-8 as implemented by Tesla SCCM for STW_ACTN_RQ."""
    crc = 0
    for byte in data:
      crc ^= byte
      for _ in range(8):
        if crc & 0x80:
          crc = ((crc << 1) ^ 0x11D) & 0xFF
        else:
          crc = (crc << 1) & 0xFF
    return crc ^ 0xFF

  def create_action_request(self, base_msg: dict, button: int, counter: int) -> tuple[int, int, bytes]:
    """Create a STW_ACTN_RQ cruise stalk virtual button request."""
    values = copy.copy(base_msg) if base_msg is not None else {}
    values["SpdCtrlLvr_Stat"] = int(button)
    values["MC_STW_ACTN_RQ"] = int(counter) & 0xF

    # Pack once to compute CRC over first 7 bytes, then repack with CRC_STW_ACTN_RQ.
    msg = self.packer.make_can_msg("STW_ACTN_RQ", CANBUS.party, values)
    crc = self._crc_stw_actn_req(msg[2][:7])
    values["CRC_STW_ACTN_RQ"] = crc
    return self.packer.make_can_msg("STW_ACTN_RQ", CANBUS.party, values)


  def create_steering_control(self, angle, enabled):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
    }

    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active):
    from opendbc.car.interfaces import V_CRUISE_MAX

    set_speed = max(v_ego * CV.MS_TO_KPH, 0)
    if active:
      # TODO: this causes jerking after gas override when above set speed
      set_speed = 0 if accel < 0 else V_CRUISE_MAX

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


def tesla_checksum(address: int, sig, d: bytearray) -> int:
  checksum = (address & 0xFF) + ((address >> 8) & 0xFF)
  checksum_byte = sig.start_bit // 8
  for i in range(len(d)):
    if i != checksum_byte:
      checksum += d[i]
  return checksum & 0xFF