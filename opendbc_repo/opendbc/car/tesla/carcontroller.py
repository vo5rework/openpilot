import time
import numpy as np
from cereal import messaging
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR, CruiseButtons, CruiseState
from opendbc.car.vehicle_model import VehicleModel


def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  # A Model 3 at 40 m/s using the Model Y limits sees a <0.3% difference in max angle (from curvature factor)
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")



class _VirtualStalkCruiseController:
  """Unity-style virtual stalk speed controller (refactor for 0.10.1)."""

  # Unity constant; used to avoid spamming SCCM below its stable range.
  MIN_CRUISE_SPEED_MS = 17.1

  HUMAN_OVERRIDE_WINDOW_MS = 3000
  AUTO_PRESS_INTERVAL_MS = 400

  def __init__(self) -> None:
    self._last_human_ms = 0.0
    self._last_auto_ms = 0.0
    self._speed_limit_target_ms = 0.0
    self._speed_limit_last_uom = -1
    self._action_counter = 0

  @staticmethod
  def _press_steps(speed_units: str) -> tuple[float, float]:
    # half press = 1 unit, full press = 5 units
    if speed_units == "KPH":
      return 1.0, 5.0
    return 1.0 * 1.60934, 5.0 * 1.60934

  def _speed_limit_target_kph(self, CS) -> float:
    try:
      tgt_ms = CS._calc_speed_limit_target_ms(CS.speed_units)
    except Exception:
      tgt_ms = 0.0
    self._speed_limit_target_ms = float(tgt_ms)
    return tgt_ms * 3.6

  def _plan_target_ms(self, sm) -> float | None:
    if sm is None or not sm.valid.get("longitudinalPlan", False):
      return None
    lp = sm["longitudinalPlan"]
    try:
      if len(lp.speeds) > 0:
        return float(lp.speeds[-1])
    except Exception:
      return None
    return None

  def _calc_button(self, CS, desired_speed_kph: float) -> int:
    cur_kph = float(getattr(CS, "v_cruise_actual_kph", 0.0))
    if cur_kph <= 0.0:
      cur_kph = float(CS.out.cruiseState.speed) * 3.6

    half_press_kph, full_press_kph = self._press_steps(getattr(CS, "speed_units", "MPH"))
    speed_offset = desired_speed_kph - cur_kph

    if abs(speed_offset) < half_press_kph:
      return CruiseButtons.IDLE
    if speed_offset > full_press_kph:
      return CruiseButtons.RES_ACCEL_2ND
    if speed_offset > half_press_kph:
      return CruiseButtons.RES_ACCEL
    if speed_offset < -full_press_kph:
      return CruiseButtons.DECEL_2ND
    if speed_offset < -half_press_kph:
      return CruiseButtons.DECEL_SET
    return CruiseButtons.IDLE

  def update(self, CS, sm, now_nanos: int) -> int:
    now_ms = now_nanos / 1e6

    # track human stalk activity
    try:
      if CS.cruise_buttons != CS.prev_cruise_buttons and CS.cruise_buttons != CruiseButtons.IDLE:
        self._last_human_ms = now_ms
    except Exception:
      pass

    if (now_ms - self._last_human_ms) < self.HUMAN_OVERRIDE_WINDOW_MS:
      return CruiseButtons.IDLE
    if (now_ms - self._last_auto_ms) < self.AUTO_PRESS_INTERVAL_MS:
      return CruiseButtons.IDLE

    if not CS.out.cruiseState.enabled:
      return CruiseButtons.IDLE

    # compute targets
    desired_kph = float(CS.out.cruiseState.speed) * 3.6

    plan_ms = self._plan_target_ms(sm)
    if plan_ms is not None and plan_ms > 0.0:
      desired_kph = max(desired_kph, plan_ms * 3.6)

    # speed limit matching: always clamp to limit target when active, and use it as the accel target.
    if getattr(CS, "_tinkla", None) is not None and CS._tinkla.adjust_acc_with_speed_limit:
      limit_kph = self._speed_limit_target_kph(CS)
      if limit_kph > 1.0:
        desired_kph = min(max(desired_kph, limit_kph), limit_kph)
        # If planner wants to slow down (lead), allow it to pull target below limit.
        if plan_ms is not None and plan_ms > 0.0:
          desired_kph = min(desired_kph, plan_ms * 3.6)

    desired_kph = max(desired_kph, 0.0)
    btn = self._calc_button(CS, desired_kph)
    if btn != CruiseButtons.IDLE:
      self._last_auto_ms = now_ms
    return btn

  def next_counter(self, base_msg: dict | None) -> int:
    if base_msg is not None and "MC_STW_ACTN_RQ" in base_msg:
      try:
        return (int(base_msg["MC_STW_ACTN_RQ"]) + 1) & 0xF
      except Exception:
        pass
    self._action_counter = (self._action_counter + 1) & 0xF
    return self._action_counter

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.hands_on_level_limit = 3
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(self.packer)

    self._sm = messaging.SubMaster(['longitudinalPlan'])
    self._stalk_ctrl = _VirtualStalkCruiseController()


    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

    if CP.carFingerprint in LEGACY_CARS:
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1,):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.packers = {CANBUS.party: CANPacker(dbc_names[Bus.party]), CANBUS.powertrain: CANPacker(dbc_names[Bus.pt])}
      self.tesla_can = TeslaCANRaven(self.packers)
      from opendbc.car.tesla.interface import CarInterface
      self.VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # Tesla EPS enforces disabling steering on heavy lateral override force.
    # When enabling in a tight curve, we wait until user reduces steering force to start steering.
    # Canceling is done on rising edge and is handled generically with CC.cruiseControl.cancel
    lat_active = CC.latActive and CS.hands_on_level < self.hands_on_level_limit

    if self.frame % 2 == 0:
      # Angular rate limit based on speed
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)
      if self.CP.carFingerprint in LEGACY_CARS:
        cntr = (self.frame // 2) % 16
        can_sends.append(self.tesla_can.create_steering_control(cntr, self.apply_angle_last, lat_active))
      else:
        can_sends.append(self.tesla_can.create_steering_control(self.apply_angle_last, lat_active))

    if self.frame % 10 == 0 and self.CP.carFingerprint not in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, ):
      cntr = (self.frame // 10) % 16
      can_sends.append(self.tesla_can.create_steering_allowed(cntr))


    # Virtual stalk requests (Unity-style ACC speed matching)
    if self.frame % 20 == 0:
      try:
        self._sm.update(0)
        btn = self._stalk_ctrl.update(CS, self._sm, now_nanos)
        if btn != CruiseButtons.IDLE and CS.msg_stw_actn_req is not None and hasattr(self.tesla_can, 'create_action_request'):
          cntr = self._stalk_ctrl.next_counter(CS.msg_stw_actn_req)
          can_sends.append(self.tesla_can.create_action_request(CS.msg_stw_actn_req, btn, cntr))
      except Exception:
        pass


    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        state = 13 if CC.cruiseControl.cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))

    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if CC.cruiseControl.cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends