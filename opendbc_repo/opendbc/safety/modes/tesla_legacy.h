#pragma once

#include "opendbc/safety/safety_declarations.h"

// Tesla legacy (S/X) safety mode with Unity-parity AP-disabled engagement and dual-panda support.
//
// This version is hardened against "get out/in" cycles by:
//  - not requiring stalk (0x45) on *both* buses in RX checks (alt bus match instead)
//  - not requiring optional stock-detect frames in RX checks (prevents false ControlsMismatch)
//  - resetting steering rate state when controls are not allowed (prevents immediate "steering limits exceeded"
//    due to stale desired_angle_last across sessions)
//
// Contract:
//  - Real stalk (0x45, byte0[5:0]): 2=MAIN (pull), 1=CANCEL (push) -> pcm_cruise_check(true/false)
//  - Internal carrier (0x659, byte5):
//      bit7 autopilot_disabled
//      bit5 pedal_enabled
//      bit1 main_edge
//      bit0 cancel_edge
//    0x659 is consumed by safety and must never reach the car.

#define TESLA_FLAG_LONG_CONTROL        1U
#define TESLA_FLAG_EXTERNAL_PANDA      2U
#define TESLA_FLAG_HW1                 4U
#define TESLA_FLAG_HW2                 8U
#define TESLA_FLAG_HW3                 16U
#define TESLA_FLAG_OP_STALK_ENABLE     32U

#define TESLA_LEGACY_MAX_ANGLE_CAN     3600  // 360.0 deg in 0.1 deg CAN units

static bool tesla_legacy_external_panda = false;
static bool tesla_legacy_hw1 = false;
static bool tesla_legacy_hw2 = false;
static bool tesla_legacy_hw3 = false;
static bool tesla_legacy_op_stalk_enable = false;

static bool tesla_legacy_op_autopilot_disabled = false;
static bool tesla_legacy_op_pedal_enabled = false;
static bool tesla_legacy_controls_allowed_prev = false;

static uint8_t tesla_legacy_chassis_bus = 0U;
static uint16_t tesla_legacy_das_control_addr = 0x2BFU;
static uint16_t tesla_legacy_di_torque1_addr = 0x106U;

static inline bool tesla_legacy_has_ap_hardware(void) {
  return tesla_legacy_hw1 || tesla_legacy_hw2 || tesla_legacy_hw3;
}

static inline bool tesla_legacy_use_stalk_for_controls_allowed(void) {
  return tesla_legacy_op_stalk_enable || (!tesla_legacy_has_ap_hardware()) || tesla_legacy_op_autopilot_disabled;
}

static void tesla_legacy_rx_hook(const CANPacket_t *msg) {
  // Unity-parity: latch controlsAllowed from real stalk when AP is disabled (or forced by param)
  if ((msg->addr == 0x45U) && tesla_legacy_use_stalk_for_controls_allowed()) {
    const uint8_t lever = (uint8_t)(GET_BYTES(msg, 0U, 1U) & 0x3FU);
    if (lever == 2U) {
      pcm_cruise_check(true);
    } else if (lever == 1U) {
      pcm_cruise_check(false);
    }
  }

  // EPAS_sysStatus: steering angle (0.1 deg) and hands-on
  if ((!tesla_legacy_external_panda) && (msg->bus == 0U) && (msg->addr == 0x370U)) {
    const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192;
    update_sample(&angle_meas, angle_meas_new);

    const int hands_on_level = msg->data[4] >> 6;
    const int eac_status = msg->data[6] >> 5;
    const int eac_error_code = msg->data[2] >> 4;
    steering_disengage = (hands_on_level >= 3) || ((eac_status == 0) && (eac_error_code == 9));
  }

  // Vehicle speed (ESP_private1)
  if ((!tesla_legacy_external_panda) && (msg->bus == tesla_legacy_chassis_bus) && (msg->addr == 0x155U)) {
    const float speed = ((msg->data[6] | (msg->data[5] << 8)) * 0.01F) * KPH_TO_MS;
    UPDATE_VEHICLE_SPEED(speed);
  }

  // Gas pressed (DI_torque1) - PT panda (and HW1)
  if ((tesla_legacy_external_panda || tesla_legacy_hw1) && (msg->bus == 0U) && (msg->addr == tesla_legacy_di_torque1_addr)) {
    gas_pressed = msg->data[6] != 0U;
  }

  // Brake pressed
  if (((tesla_legacy_external_panda) && (msg->bus == 0U) && (msg->addr == 0x1F8U)) ||
      ((!tesla_legacy_external_panda) && (msg->bus == tesla_legacy_chassis_bus) && (msg->addr == 0x20AU))) {
    brake_pressed = (((msg->data[0] & 0x0CU) >> 2) != 1U);
  }

  // Cruise state (DI_state)
  if (((tesla_legacy_external_panda) && (msg->bus == 0U) && (msg->addr == 0x256U)) ||
      ((!tesla_legacy_external_panda) && (msg->bus == tesla_legacy_chassis_bus) && (msg->addr == 0x368U))) {

    const int cruise_state = (msg->data[1] >> 4) & 0x07U;
    const bool cruise_engaged = (cruise_state == 2) || (cruise_state == 3) || (cruise_state == 4) ||
                                (cruise_state == 6) || (cruise_state == 7);
    vehicle_moving = cruise_state != 3;

    if (!tesla_legacy_op_autopilot_disabled && !tesla_legacy_use_stalk_for_controls_allowed()) {
      pcm_cruise_check(cruise_engaged);
    }
  }
}

static bool tesla_legacy_tx_hook(const CANPacket_t *msg) {
  // Internal openpilot->panda carrier (blocked from car)
  if (msg->addr == 0x659U) {
    const uint8_t b5 = (uint8_t)GET_BYTES(msg, 5U, 1U);
    tesla_legacy_op_autopilot_disabled = (b5 & 0x80U) != 0U;
    tesla_legacy_op_pedal_enabled = (b5 & 0x20U) != 0U;

    const bool stalk_main_edge = (b5 & 0x02U) != 0U;
    const bool stalk_cancel_edge = (b5 & 0x01U) != 0U;

    // Harden against stale steering-state across "get out/in":
    // When controls are not allowed, keep desired_angle_last pinned to measured angle.
    if (!controls_allowed) {
      desired_angle_last = CLAMP(angle_meas.values[0], -TESLA_LEGACY_MAX_ANGLE_CAN, TESLA_LEGACY_MAX_ANGLE_CAN);
      rt_angle_msgs = 0U;
      ts_angle_check_last = microsecond_timer_get();
    }

    if (stalk_main_edge) {
      desired_angle_last = CLAMP(angle_meas.values[0], -TESLA_LEGACY_MAX_ANGLE_CAN, TESLA_LEGACY_MAX_ANGLE_CAN);
      rt_angle_msgs = 0U;
      ts_angle_check_last = microsecond_timer_get();
      pcm_cruise_check(true);
    } else if (stalk_cancel_edge) {
      pcm_cruise_check(false);
    }

    return false;
  }

  // Steering control (0x488) - internal panda only
  if ((!tesla_legacy_external_panda) && (msg->addr == 0x488U)) {
    const AngleSteeringLimits limits = {
      .max_angle = TESLA_LEGACY_MAX_ANGLE_CAN,
      .angle_deg_to_can = 10.0F,   // 0.1 deg units
      .frequency = 50U,
    };
    const AngleSteeringParams params = {
      .slip_factor = -0.0005666493436310427F,
      .steer_ratio = 15.0F,
      .wheelbase = 2.96F,
    };

    const int raw_angle_can = ((msg->data[0] & 0x7FU) << 8) | msg->data[1];
    const int desired_angle = raw_angle_can - 16384;  // 0.1 deg
    const int steer_control_type = msg->data[2] >> 6;
    const bool steer_control_enabled = steer_control_type == 1;

    // Wake/restart robustness: reset steering history on controlsAllowed rising edge.
    if (!controls_allowed) {
      tesla_legacy_controls_allowed_prev = false;
    }
    if (controls_allowed && !tesla_legacy_controls_allowed_prev) {
      desired_angle_last = desired_angle;
      rt_angle_msgs = 0U;
      ts_angle_check_last = microsecond_timer_get();
      tesla_legacy_controls_allowed_prev = true;
    }

    bool violation = steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, limits, params);

    // Only NONE (0) or ANGLE_CONTROL (1) are allowed
    if (!((steer_control_type == 0) || (steer_control_type == 1))) {
      violation = true;
    }

    return !violation;
  }

  // Longitudinal (0x2BF) - external panda only (HW2/HW3) (no extra checks here; handled in mode-specific code elsewhere)
  return true;
}

// Forwarding control: return true to block the message from being forwarded.
// We keep forwarding enabled and only block nothing here (the platform-level forwarding is handled elsewhere).
static bool tesla_legacy_fwd_hook(int bus_num, int addr) {
  (void)bus_num;
  (void)addr;
  return false;
}

static safety_config tesla_legacy_init(uint16_t param) {
  tesla_legacy_external_panda = GET_FLAG(param, TESLA_FLAG_EXTERNAL_PANDA);
  tesla_legacy_hw1 = GET_FLAG(param, TESLA_FLAG_HW1);
  tesla_legacy_hw2 = GET_FLAG(param, TESLA_FLAG_HW2);
  tesla_legacy_hw3 = GET_FLAG(param, TESLA_FLAG_HW3);
  tesla_legacy_op_stalk_enable = GET_FLAG(param, TESLA_FLAG_OP_STALK_ENABLE);

  tesla_legacy_op_autopilot_disabled = false;
  tesla_legacy_op_pedal_enabled = false;
  tesla_legacy_controls_allowed_prev = false;

  tesla_legacy_chassis_bus = 0U;
  tesla_legacy_di_torque1_addr = 0x106U;
  tesla_legacy_das_control_addr = 0x2BFU;

  // TX allowlists (panda-local buses)
  static const CanMsg TESLA_LEGACY_TX_MSGS[] = {
    {.addr = 0x659, .bus = 0U, .len = 8, .check_relay = false, .disable_static_blocking = true},
    {.addr = 0x488, .bus = 0U, .len = 4, .check_relay = true,  .disable_static_blocking = true},
    {.addr = 0x27D, .bus = 0U, .len = 3, .check_relay = true,  .disable_static_blocking = true},
  };

  static const CanMsg TESLA_LEGACY_PT_TX_MSGS[] = {
    {.addr = 0x659, .bus = 0U, .len = 8, .check_relay = false, .disable_static_blocking = true},
    {.addr = 0x2BF, .bus = 0U, .len = 8, .check_relay = true,  .disable_static_blocking = true},
  };

  // One RX check for stalk: allow bus0 OR bus2 (whichever exists on that panda).
  // IMPORTANT: do not require both, or you'll get ControlsMismatch after wake cycles.
  static RxCheck TESLA_LEGACY_HW2_RX_CHECKS[] = {
    {.msg = {{0x45,  0, 8, 10U,  .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true},
             {0x45,  2, 8, 10U,  .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true},
             {0}}},
    {.msg = {{0x370, 0, 8, 25U,  .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},
    {.msg = {{0x155, 0, 8, 50U,  .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},
    {.msg = {{0x20A, 0, 8, 50U,  .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},
    {.msg = {{0x368, 0, 8, 10U,  .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},
  };

  // PT panda RX checks: do NOT require stalk here; it latches via 0x659 edges.
  static RxCheck TESLA_LEGACY_PT_RX_CHECKS[] = {
    {.msg = {{0x106, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},
    {.msg = {{0x1F8, 0, 8, 50U,  .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},
    {.msg = {{0x256, 0, 8, 10U,  .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},
  };

  return tesla_legacy_external_panda
    ? BUILD_SAFETY_CFG(TESLA_LEGACY_PT_RX_CHECKS, TESLA_LEGACY_PT_TX_MSGS)
    : BUILD_SAFETY_CFG(TESLA_LEGACY_HW2_RX_CHECKS, TESLA_LEGACY_TX_MSGS);
}

const safety_hooks tesla_legacy_hooks = {
  .init = tesla_legacy_init,
  .rx = tesla_legacy_rx_hook,
  .tx = tesla_legacy_tx_hook,
  .fwd = tesla_legacy_fwd_hook,
};
