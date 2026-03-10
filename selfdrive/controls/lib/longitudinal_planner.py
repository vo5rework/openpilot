#!/usr/bin/env python3
import inspect
import math
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.modeld.constants import ModelConstants

A_CRUISE_MIN = -1.2
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0.0, 10.0, 25.0, 40.0]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
MAX_VEL_ERR = 5.0


_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20.0, 40.0]


def get_max_accel(v_ego: float) -> float:
  return float(np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS))


def get_speed_error(model_msg, v_ego: float) -> float:
  try:
    temporal_pose = getattr(model_msg, "temporalPose", None)
    if temporal_pose is not None:
      trans = getattr(temporal_pose, "trans", [])
      if len(trans):
        vel_err = np.clip(float(trans[0]) - float(v_ego), -MAX_VEL_ERR, MAX_VEL_ERR)
        return float(vel_err)
  except Exception:
    pass
  try:
    if len(model_msg.velocity.x):
      vel_err = np.clip(float(model_msg.velocity.x[0]) - float(v_ego), -MAX_VEL_ERR, MAX_VEL_ERR)
      return float(vel_err)
  except Exception:
    pass
  return 0.0


def limit_accel_in_turns(v_ego: float, angle_steers: float, a_target, CP):
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.0))
  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    try:
      self.mpc = LongitudinalMpc(dt=dt)
    except TypeError:
      self.mpc = LongitudinalMpc()
    mpc_update_params = tuple(inspect.signature(self.mpc.update).parameters.keys())
    self._mpc_update_first_param = mpc_update_params[0] if len(mpc_update_params) > 0 else ""
    self._mpc_update_accepts_carstate = "carstate" in mpc_update_params
    self.mpc.mode = "acc"

    self.dt = dt
    self.fcw = False
    self.allow_throttle = True
    self.curve_detected = False
    self.v_model_error = 0.0

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory_full = np.zeros(ModelConstants.IDX_N)
    self.a_desired_trajectory_full = np.zeros(ModelConstants.IDX_N)
    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  @staticmethod
  def parse_model(model_msg, model_error: float, v_ego: float, enable_turn_slowdown: bool = True, turn_slowdown_factor: float = 1.0):
    if (
      len(model_msg.position.x) == ModelConstants.IDX_N
      and len(model_msg.velocity.x) == ModelConstants.IDX_N
      and len(model_msg.acceleration.x) == ModelConstants.IDX_N
    ):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x) - model_error * T_IDXS_MPC
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x) - model_error
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))

    curve_detected = False
    if enable_turn_slowdown and len(getattr(model_msg.orientationRate, "z", [])) == ModelConstants.IDX_N:
      max_lat_accel = np.interp(v_ego, [5.0, 10.0, 20.0], [1.5, 2.0, 3.0])
      curvatures = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.orientationRate.z) / np.clip(v, 0.3, 100.0)
      max_v = float(turn_slowdown_factor) * np.sqrt(max_lat_accel / (np.abs(curvatures) + 1e-3)) - 2.0
      v_clipped = np.minimum(max_v, v)
      curve_detected = bool(np.any(v_clipped < (v - 0.05)))
      v = v_clipped

    return x, v, a, j, curve_detected

  def update(self, sm):
    controls_state = sm["controlsState"]
    selfdrive_state = sm["selfdriveState"]

    experimental_mode = bool(getattr(controls_state, "experimentalMode", getattr(selfdrive_state, "experimentalMode", False)))
    enabled_state = bool(getattr(controls_state, "enabled", getattr(selfdrive_state, "enabled", False)))
    personality = getattr(selfdrive_state, "personality", 0)

    self.mpc.mode = "blended" if experimental_mode else "acc"

    v_ego = float(sm["carState"].vEgo)

    controls_v_cruise = float(getattr(controls_state, "vCruise", 0.0) or 0.0)
    carstate_v_cruise = float(getattr(sm["carState"], "vCruise", 0.0) or 0.0)
    v_cruise_kph = controls_v_cruise if 0.1 < controls_v_cruise < float(V_CRUISE_UNSET) else carstate_v_cruise
    v_cruise_kph = min(v_cruise_kph, float(V_CRUISE_MAX))
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    long_control_off = controls_state.longControlState == LongCtrlState.off
    force_slow_decel = bool(getattr(controls_state, "forceDecel", False))

    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not enabled_state
    prev_accel_constraint = not (reset_state or sm["carState"].standstill)

    if self.mpc.mode == "acc":
      accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
      accel_limits_turns = limit_accel_in_turns(v_ego, float(sm["carState"].steeringAngleDeg), accel_limits, self.CP)
    else:
      accel_limits = [ACCEL_MIN, ACCEL_MAX]
      accel_limits_turns = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = float(np.clip(sm["carState"].aEgo, accel_limits[0], accel_limits[1]))

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    self.v_model_error = get_speed_error(sm["modelV2"], v_ego)

    if force_slow_decel:
      v_cruise = 0.0

    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)

    self.mpc.set_weights(prev_accel_constraint, personality=personality)
    if hasattr(self.mpc, "set_accel_limits"):
      self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    x, v, a, j, self.curve_detected = self.parse_model(sm["modelV2"], self.v_model_error, v_ego, True, 1.0)

    if self._mpc_update_first_param == "carstate":
      self.mpc.update(sm["carState"], sm["radarState"], v_cruise, x, v, a, j, personality=personality)
    else:
      update_kwargs = {"personality": personality}
      if self._mpc_update_accepts_carstate:
        update_kwargs["carstate"] = sm["carState"]
      self.mpc.update(sm["radarState"], v_cruise, x, v, a, j, **update_kwargs)

    self.v_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.a_solution)
    self.v_desired_trajectory = self.v_desired_trajectory_full[:CONTROL_N]
    self.a_desired_trajectory = self.a_desired_trajectory_full[:CONTROL_N]
    self.j_desired_trajectory = np.interp(ModelConstants.T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    self.fcw = self.mpc.crash_cnt > 2 and not sm["carState"].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    a_prev = self.a_desired
    self.a_desired = float(np.interp(DT_MDL, ModelConstants.T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + DT_MDL * (self.a_desired + a_prev) / 2.0

    action_t = float(getattr(self.CP, "longitudinalActuatorDelay", 0.0) or 0.0) + DT_MDL
    self.output_a_target, self.output_should_stop = get_accel_from_plan(
      self.v_desired_trajectory,
      self.a_desired_trajectory,
      ModelConstants.T_IDXS[:CONTROL_N],
      action_t=action_t,
      vEgoStopping=self.CP.vEgoStopping,
    )
    for idx in range(2):
      accel_limits_turns[idx] = np.clip(accel_limits_turns[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = float(np.clip(self.output_a_target, accel_limits_turns[0], accel_limits_turns[1]))
    self.prev_accel_clip = list(accel_limits_turns)
    self.allow_throttle = True

  def publish(self, sm, pm):
    plan_send = messaging.new_message("longitudinalPlan")
    plan_send.valid = sm.all_checks(service_list=["carState", "controlsState"])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime["modelV2"]
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime["modelV2"]
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm["radarState"].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw
    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = True
    pm.send("longitudinalPlan", plan_send)
