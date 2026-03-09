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

LON_MPC_STEP = 0.2
A_CRUISE_MIN = -4.0
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0.0, 10.0, 25.0, 40.0]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5
MAX_VEL_ERR = 5.0

_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20.0, 40.0]

CURVE_PEAK_THRESHOLD = 0.0020
AREA_THRESHOLD = 0.012
VISION_CURVE_TARGET_LAT_A = 2.1
CURVE_SLOWDOWN_SPEED_MARGIN = 0.5


def get_max_accel(v_ego: float) -> float:
  return float(np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS))


def get_coast_accel(pitch: float) -> float:
  return float(np.sin(pitch) * -5.65 - 0.3)


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


def _rate_limited_filter(filt: FirstOrderFilter, new_x: float, rc_up: float, rc_down: float) -> float:
  if new_x < float(filt.x):
    filt.update_alpha(rc_down)
  else:
    filt.update_alpha(rc_up)
  return float(filt.update(new_x))


def limit_accel_in_turns(v_ego: float, angle_steers: float, a_target, CP):
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.0))
  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    mpc_update_params = tuple(inspect.signature(self.mpc.update).parameters.keys())
    self._mpc_update_first_param = mpc_update_params[0] if len(mpc_update_params) > 0 else ""
    self._mpc_update_accepts_carstate = "carstate" in mpc_update_params
    set_weights_params = tuple(inspect.signature(self.mpc.set_weights).parameters.keys())
    self._mpc_set_weights_accepts_custom = "weights" in set_weights_params
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

    self.v_turn_filter = FirstOrderFilter(init_v, 0.2, self.dt)
    self.curve_detected = False
    self.curve_slowdown_active = False
    self.v_model_error = 0.0

  @staticmethod
  def parse_model(model_msg, model_error: float, v_ego: float, v_turn_filter: FirstOrderFilter):
    x = np.zeros(len(T_IDXS_MPC))
    v = np.zeros(len(T_IDXS_MPC))
    a = np.zeros(len(T_IDXS_MPC))
    j = np.zeros(len(T_IDXS_MPC))

    if (
      len(model_msg.position.x) == ModelConstants.IDX_N
      and len(model_msg.velocity.x) == ModelConstants.IDX_N
      and len(model_msg.acceleration.x) == ModelConstants.IDX_N
    ):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x) - model_error * T_IDXS_MPC
      v_raw = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)

      v_corrected = np.maximum(v_raw, max(0.0, v_ego - 1.5)) - model_error

      if len(model_msg.orientationRate.z) == ModelConstants.IDX_N:
        raw_curv = np.abs(np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.orientationRate.z)) / np.clip(v_corrected, 0.3, 100.0)
        num_idx = len(raw_curv)
        if num_idx > 0:
          far_start = min(10, max(0, num_idx - 1))
          far_end = min(32, num_idx)
          far_curv_peak = float(np.max(raw_curv[far_start:far_end])) if far_end > far_start else 0.0
        else:
          far_curv_peak = 0.0

        anticipatory_slowdown = float(np.interp(far_curv_peak, [0.0008, 0.003], [1.0, 0.78]))
        curve_area = float(np.sum(raw_curv[:25]) * LON_MPC_STEP)
        max_curv_ahead = float(np.max(raw_curv[2:20])) if num_idx > 2 else 0.0

        if (curve_area > AREA_THRESHOLD) or (max_curv_ahead > CURVE_PEAK_THRESHOLD):
          lat_stress_factor = float((v_ego ** 2) * max_curv_ahead)
          torque_multiplier = float(np.interp(lat_stress_factor, [0.015, 0.05], [1.0, 0.65]))
          dynamic_multiplier = float(np.interp(max_curv_ahead, [0.0015, 0.008], [1.0, 0.70]))
          max_v_curve = dynamic_multiplier * torque_multiplier * anticipatory_slowdown * math.sqrt(max(VISION_CURVE_TARGET_LAT_A / (max_curv_ahead + 1e-4), 0.0))
          v_curve = _rate_limited_filter(v_turn_filter, max_v_curve, rc_up=0.30, rc_down=0.05)
          # Keep Unity curve detection, but only arm slowdown when the filtered curve target is below ego speed.
          curve_slowdown_active = bool(v_curve < (v_ego - CURVE_SLOWDOWN_SPEED_MARGIN))
          v = np.minimum(v_curve, v_corrected)
          return x, v, a, j, True, curve_slowdown_active

        v = v_corrected
        if anticipatory_slowdown < 1.0:
          v = np.minimum(v * anticipatory_slowdown, v)
        return x, v, a, j, bool(anticipatory_slowdown < 0.98), False

      v = v_corrected

    return x, v, a, j, False, False

  def update(self, sm):
    controls_state = sm["controlsState"]
    selfdrive_state = sm["selfdriveState"]

    experimental_mode = bool(getattr(controls_state, "experimentalMode", getattr(selfdrive_state, "experimentalMode", False)))
    enabled_state = bool(getattr(controls_state, "enabled", getattr(selfdrive_state, "enabled", False)))
    personality = getattr(selfdrive_state, "personality", 0)

    mode = "blended" if experimental_mode else "acc"
    self.mpc.mode = mode

    if len(sm["carControl"].orientationNED) == 3:
      accel_coast = get_coast_accel(sm["carControl"].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = float(sm["carState"].vEgo)

    carstate_v_cruise = float(getattr(sm["carState"], "vCruise", 0.0) or 0.0)
    v_cruise_kph = min(carstate_v_cruise, float(V_CRUISE_MAX))
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = (v_cruise_kph > 0.1) and (v_cruise_kph < float(V_CRUISE_UNSET))

    long_control_off = controls_state.longControlState == LongCtrlState.off
    force_slow_decel = bool(getattr(controls_state, "forceDecel", False))

    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not enabled_state
    if self.CP.brand != "tesla":
      reset_state = reset_state or not v_cruise_initialized
    prev_accel_constraint = not (reset_state or sm["carState"].standstill)

    if mode == "acc":
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
    x, v, a, j, self.curve_detected, self.curve_slowdown_active = self.parse_model(sm["modelV2"], self.v_model_error, v_ego, self.v_turn_filter)

    throttle_prob = 1.0
    try:
      if len(sm["modelV2"].meta.disengagePredictions.gasPressProbs) > 1:
        throttle_prob = float(sm["modelV2"].meta.disengagePredictions.gasPressProbs[1])
    except Exception:
      throttle_prob = 1.0
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip_turns[0])
      clipped_accel_coast_interp = np.interp(
        v_ego,
        [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED * 2.0],
        [accel_clip_turns[1], clipped_accel_coast],
      )
      accel_clip_turns[1] = min(accel_clip_turns[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0

    if self.curve_slowdown_active and self._mpc_set_weights_accepts_custom:
      self.mpc.set_weights(prev_accel_constraint, personality=personality, weights=[2.0, 5.0, 40.0])
    else:
      self.mpc.set_weights(prev_accel_constraint, personality=personality)

    if hasattr(self.mpc, "set_accel_limits"):
      if self.curve_slowdown_active:
        self.mpc.set_accel_limits(accel_clip_turns[0], min(self.a_desired - 0.3, -0.1))
      else:
        self.mpc.set_accel_limits(accel_clip_turns[0], accel_clip_turns[1])

    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    v_target = v if self.curve_slowdown_active else np.maximum(v, max(0.0, float(v_cruise) - 0.1))

    if self._mpc_update_first_param == "carstate":
      self.mpc.update(sm["carState"], sm["radarState"], v_cruise, x, v_target, a, j, personality=personality)
    else:
      update_kwargs = {"personality": personality}
      if self._mpc_update_accepts_carstate:
        update_kwargs["carstate"] = sm["carState"]
      self.mpc.update(sm["radarState"], v_cruise, x, v_target, a, j, **update_kwargs)

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
    output_a_target_e2e = sm["modelV2"].action.desiredAcceleration
    output_should_stop_e2e = sm["modelV2"].action.shouldStop

    if mode == "acc":
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc
    else:
      output_a_target = min(output_a_target_mpc, output_a_target_e2e)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc

    for idx in range(2):
      accel_clip_turns[idx] = np.clip(accel_clip_turns[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip_turns[0], accel_clip_turns[1])
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
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send("longitudinalPlan", plan_send)
