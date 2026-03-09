#!/usr/bin/env python3
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
from openpilot.selfdrive.car.modules.CFG_module import load_bool_param, load_float_param

LON_MPC_STEP = 0.2
A_CRUISE_MIN = -1.2
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0.0, 10.0, 25.0, 40.0]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20.0, 40.0]
MAX_VEL_ERR = 5.0


def get_max_accel(v_ego: float) -> float:
  return float(np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS))


def get_speed_error(model_msg, v_ego: float) -> float:
  try:
    temporal_pose = getattr(model_msg, "temporalPose", None)
    if temporal_pose is not None:
      trans = getattr(temporal_pose, "trans", [])
      if len(trans):
        return float(np.clip(float(trans[0]) - float(v_ego), -MAX_VEL_ERR, MAX_VEL_ERR))
  except Exception:
    pass
  try:
    if len(model_msg.velocity.x):
      return float(np.clip(float(model_msg.velocity.x[0]) - float(v_ego), -MAX_VEL_ERR, MAX_VEL_ERR))
  except Exception:
    pass
  return 0.0


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.0))
  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.mpc.mode = "acc"
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

    self.v_model_error = 0.0
    self.personality = 0
    self.param_read_counter = 0
    self.enable_turn_slowdown = load_bool_param("TinklaTurnSlowdown", True)
    self.turn_slowdown_factor = load_float_param("TinklaTurnSlowdownFactor", 1.0)

  def read_param(self, controls_state=None, selfdrive_state=None):
    for src in (selfdrive_state, controls_state):
      if src is None:
        continue
      try:
        val = int(getattr(src, "personality"))
        self.personality = val
        return
      except Exception:
        pass

  @staticmethod
  def parse_model(model_msg, model_error: float, v_ego: float, enable_turn_slowdown: bool, turn_slowdown_factor: float):
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

    if enable_turn_slowdown and len(model_msg.orientationRate.z) == ModelConstants.IDX_N:
      max_lat_accel = np.interp(v_ego, [5.0, 10.0, 20.0], [1.5, 2.0, 3.0])
      curvatures = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.orientationRate.z) / np.clip(v, 0.3, 100.0)
      max_v = float(turn_slowdown_factor) * np.sqrt(max_lat_accel / (np.abs(curvatures) + 1e-3)) - 2.0
      v = np.minimum(max_v, v)

    return x, v, a, j

  def update(self, sm):
    controls_state = sm["controlsState"]
    selfdrive_state = sm["selfdriveState"]

    if self.param_read_counter % 50 == 0:
      self.read_param(controls_state=controls_state, selfdrive_state=selfdrive_state)
    self.param_read_counter += 1

    experimental_mode = bool(getattr(controls_state, "experimentalMode", getattr(selfdrive_state, "experimentalMode", False)))
    enabled_state = bool(getattr(controls_state, "enabled", getattr(selfdrive_state, "enabled", False)))
    self.mpc.mode = "blended" if experimental_mode else "acc"

    v_ego = float(sm["carState"].vEgo)
    carstate_v_cruise = float(getattr(sm["carState"], "vCruise", 0.0) or 0.0)
    controls_v_cruise = float(getattr(controls_state, "vCruise", 0.0) or 0.0)
    v_cruise_kph = min(max(carstate_v_cruise, controls_v_cruise), float(V_CRUISE_MAX))
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = (v_cruise_kph > 0.1) and (v_cruise_kph < float(V_CRUISE_UNSET))

    long_control_off = controls_state.longControlState == LongCtrlState.off
    force_slow_decel = bool(getattr(controls_state, "forceDecel", False))

    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not enabled_state
    if self.CP.brand != "tesla":
      reset_state = reset_state or not v_cruise_initialized

    prev_accel_constraint = not (reset_state or sm["carState"].standstill)

    if self.mpc.mode == "acc":
      accel_clip = [A_CRUISE_MIN, get_max_accel(v_ego)]
      steer_angle_without_offset = sm["carState"].steeringAngleDeg - sm["liveParameters"].angleOffsetDeg
      accel_clip_turns = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)
    else:
      accel_clip = [ACCEL_MIN, ACCEL_MAX]
      accel_clip_turns = list(accel_clip)

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = np.clip(sm["carState"].aEgo, accel_clip[0], accel_clip[1])

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    self.v_model_error = get_speed_error(sm["modelV2"], v_ego)

    if force_slow_decel:
      v_cruise = 0.0

    accel_clip_turns[0] = min(accel_clip_turns[0], self.a_desired + 0.05)
    accel_clip_turns[1] = max(accel_clip_turns[1], self.a_desired - 0.05)

    self.mpc.set_weights(prev_accel_constraint, personality=self.personality)
    self.mpc.set_accel_limits(accel_clip_turns[0], accel_clip_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    x, v, a, j = self.parse_model(
      sm["modelV2"],
      self.v_model_error,
      v_ego,
      self.enable_turn_slowdown,
      self.turn_slowdown_factor,
    )
    self.mpc.update(sm["radarState"], v_cruise, x, v, a, j, personality=self.personality, carstate=sm["carState"])

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    self.fcw = self.mpc.crash_cnt > 2 and not sm["carState"].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t = self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(
      self.v_desired_trajectory,
      self.a_desired_trajectory,
      CONTROL_N_T_IDX,
      action_t=action_t,
      vEgoStopping=self.CP.vEgoStopping,
    )
    self.output_a_target = float(np.clip(output_a_target_mpc, accel_clip_turns[0], accel_clip_turns[1]))
    self.output_should_stop = bool(output_should_stop_mpc)
    self.prev_accel_clip = accel_clip_turns

  def publish(self, sm, pm):
    plan_send = messaging.new_message("longitudinalPlan")
    plan_send.valid = sm.all_checks(service_list=["carState", "controlsState", "selfdriveState", "radarState"])

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
