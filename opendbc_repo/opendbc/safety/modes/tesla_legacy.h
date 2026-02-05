#pragma once

#include "opendbc/safety/safety_declarations.h"

// Tesla legacy (Model S/X HW1/HW2/HW3) safety mode with Unity-parity dual-panda behavior.
//
// Key points:
//  - Uses safetyParam bitflags to select panda role (powertrain vs non-powertrain), like Unity.
//  - Consumes internal openpilot->panda carrier 0x659 (never forwarded to car) for:
//      * autopilot_disabled (bit7), pedal_enabled (bit5)
//      * stalk main/cancel edges (bit1/bit0) to latch controlsAllowed on pandas that don't see STW_ACTN_RQ.
//  - Uses real stalk message 0x45 (byte0[5:0]) to latch controlsAllowed when enabled.
//  - Provides proper RxChecks so rx_hook runs (XNOR safety requires RxChecks wiring).

// safetyParam bitflags (match opendbc/car/tesla/values.py)
#define TESLA_FLAG_LONG_CONTROL        1U
#define TESLA_FLAG_EXTERNAL_PANDA      2U   // "powertrain panda" in Unity naming
#define TESLA_FLAG_HW1                 4U
#define TESLA_FLAG_HW2                 8U
#define TESLA_FLAG_HW3                 16U
#define TESLA_FLAG_OP_STALK_ENABLE     32U

static bool tesla_legacy_external_panda = false;
static bool tesla_legacy_hw1 = false;
static bool tesla_legacy_hw2 = false;
static bool tesla_legacy_hw3 = false;
static bool tesla_legacy_op_stalk_enable = false;

static bool tesla_legacy_op_autopilot_disabled = false;  // openpilot->panda state (0x659 bit7)
static bool tesla_legacy_op_pedal_enabled = false;       // openpilot->panda state (0x659 bit5)

static bool tesla_legacy_stock_aeb = false;

// Only rising edges while controls are not allowed are considered for these systems.
// TODO: Only LKAS (non-emergency) is currently supported since we've only seen it.
static bool tesla_legacy_stock_lkas = false;
static bool tesla_legacy_stock_lkas_prev = false;

static uint8_t tesla_legacy_chassis_bus = 0U;
static uint16_t tesla_legacy_das_control_addr = 0x2BFU;
static uint16_t tesla_legacy_di_torque1_addr = 0x106U;

static inline bool tesla_legacy_has_ap_hardware(void) {
  return tesla_legacy_hw1 || tesla_legacy_hw2 || tesla_legacy_hw3;
}

static inline bool tesla_legacy_use_stalk_for_controls_allowed(void) {
  // Unity: latch controlsAllowed on the real stalk when Tesla AP is disabled.
  // XNOR: safetyParam can also force stalk usage for controlsAllowed.
  return tesla_legacy_op_stalk_enable || (!tesla_legacy_has_ap_hardware()) || tesla_legacy_op_autopilot_disabled;
}

static void tesla_legacy_rx_hook(const CANPacket_t *msg) {
  // Stalk (STW_ACTN_RQ): byte0[5:0], 2=MAIN (pull), 1=CANCEL (push)
  if ((msg->addr == 0x45U) && tesla_legacy_use_stalk_for_controls_allowed()) {
    const uint8_t lever = (uint8_t)(GET_BYTES(msg, 0U, 1U) & 0x3FU);
    if (lever == 2U) {
      pcm_cruise_check(true);
    } else if (lever == 1U) {
      pcm_cruise_check(false);
    }
  }

  // Steering angle + hands-on / faults (EPAS_sysStatus)
  if ((!tesla_legacy_external_panda) && (msg->bus == 0U) && (msg->addr == 0x370U)) {
    // Store in 0.1 deg to match command units
    const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192;
    update_sample(&angle_meas, angle_meas_new);

    const int hands_on_level = msg->data[4] >> 6;  // handsOnLevel
    const int eac_status = msg->data[6] >> 5;      // eacStatus
    const int eac_error_code = msg->data[2] >> 4;  // eacErrorCode

    // Disengage on normal user override, or if high angle-rate fault from rapid user override
    steering_disengage = (hands_on_level >= 3) || ((eac_status == 0) && (eac_error_code == 9));
  }

  // Vehicle speed (ESP_private1: ESP_vehicleSpeed)
  if ((!tesla_legacy_external_panda) && (msg->bus == tesla_legacy_chassis_bus) && (msg->addr == 0x155U)) {
    // Vehicle speed: (0.01 * val) * KPH_TO_MS
    const float speed = ((msg->data[6] | (msg->data[5] << 8)) * 0.01F) * KPH_TO_MS;
    UPDATE_VEHICLE_SPEED(speed);
  }

  // Gas pressed
  if ((tesla_legacy_external_panda || tesla_legacy_hw1) && (msg->bus == 0U) && (msg->addr == tesla_legacy_di_torque1_addr)) {
    gas_pressed = msg->data[6] != 0U;
  }

  // Brake pressed
  if (((tesla_legacy_external_panda) && (msg->bus == 0U) && (msg->addr == 0x1F8U)) ||
      ((!tesla_legacy_external_panda) && (msg->bus == tesla_legacy_chassis_bus) && (msg->addr == 0x20AU))) {
    brake_pressed = (((msg->data[0] & 0x0CU) >> 2) != 1U);
  }

  // Cruise state (DI_state): only used when not using stalk for controlsAllowed and Tesla AP isn't disabled.
  if (((tesla_legacy_external_panda) && (msg->bus == 0U) && (msg->addr == 0x256U)) ||
      ((!tesla_legacy_external_panda) && (msg->bus == tesla_legacy_chassis_bus) && (msg->addr == 0x368U))) {

    const int cruise_state = (msg->data[1] >> 4) & 0x07U;
    const bool cruise_engaged = (cruise_state == 2) ||  // ENABLED
                                (cruise_state == 3) ||  // STANDSTILL
                                (cruise_state == 4) ||  // OVERRIDE
                                (cruise_state == 6) ||  // PRE_FAULT
                                (cruise_state == 7);    // PRE_CANCEL
    vehicle_moving = cruise_state != 3;  // not STANDSTILL

    if (!tesla_legacy_op_autopilot_disabled && !tesla_legacy_use_stalk_for_controls_allowed()) {
      pcm_cruise_check(cruise_engaged);
    }
  }

  // Stock AEB/LKAS detection on AP bus (bus 2 on each panda)
  if (msg->bus == 2U) {
    // DAS_control: AEB active (stock system)
    if ((tesla_legacy_external_panda || tesla_legacy_hw1) && (msg->addr == tesla_legacy_das_control_addr)) {
      tesla_legacy_stock_aeb = (msg->data[2] & 0x03U) == 1U;  // AEB_ACTIVE
    }

    // DAS_steeringControl: LKAS active (stock system)
    if ((!tesla_legacy_external_panda) && (msg->addr == 0x488U)) {
      const int steering_control_type = msg->data[2] >> 6;
      const bool stock_lkas_now = steering_control_type == 2;  // LANE_KEEP_ASSIST

      if (stock_lkas_now && !tesla_legacy_stock_lkas_prev && !controls_allowed) {
        tesla_legacy_stock_lkas = true;
      }
      if (!stock_lkas_now) {
        tesla_legacy_stock_lkas = false;
      }
      tesla_legacy_stock_lkas_prev = stock_lkas_now;
    }
  }
}

static bool tesla_legacy_tx_hook(const CANPacket_t *msg) {
  // Internal openpilot->panda carrier (0x659): consumed by safety, never forwarded to car.
  // Byte5 bits:
  //   bit7: autopilot_disabled, bit5: pedalEnabled, bit1: stalk_main(edge), bit0: stalk_cancel(edge)
  if (msg->addr == 0x659U) {
    const uint8_t b5 = (uint8_t)GET_BYTES(msg, 5U, 1U);
    tesla_legacy_op_autopilot_disabled = (b5 & 0x80U) != 0U;
    tesla_legacy_op_pedal_enabled = (b5 & 0x20U) != 0U;

    const bool stalk_main_edge = (b5 & 0x02U) != 0U;
    const bool stalk_cancel_edge = (b5 & 0x01U) != 0U;

    if (stalk_main_edge) {
      pcm_cruise_check(true);
    } else if (stalk_cancel_edge) {
      pcm_cruise_check(false);
    }

    // block from car
    return false;
  }

  const AngleSteeringLimits TESLA_STEERING_LIMITS = {
    .max_angle = 3600,  // 360 deg, EPAS faults above this
    .angle_deg_to_can = 10.0F,
    .frequency = 50U,
  };

  const AngleSteeringParams TESLA_STEERING_PARAMS = {
    .slip_factor = -0.0005666493436310427F,
    .steer_ratio = 15.0F,
    .wheelbase = 2.96F,
  };

  const LongitudinalLimits TESLA_LONG_LIMITS = {
    .max_accel = 425,       // 2 m/s^2
    .min_accel = 288,       // -3.48 m/s^2
    .inactive_accel = 375,  // 0 m/s^2
  };

  bool violation = false;

  // DAS_steeringControl (0x488): angle control
  if ((!tesla_legacy_external_panda) && (msg->addr == 0x488U)) {
    const int raw_angle_can = ((msg->data[0] & 0x7FU) << 8) | msg->data[1];
    const int desired_angle = raw_angle_can - 16384;
    const int steer_control_type = msg->data[2] >> 6;
    const bool steer_control_enabled = steer_control_type == 1;  // ANGLE_CONTROL

    if (steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled,
                                  TESLA_STEERING_LIMITS, TESLA_STEERING_PARAMS)) {
      violation = true;
    }

    const bool valid_type = (steer_control_type == 0) || (steer_control_type == 1);  // NONE or ANGLE_CONTROL
    if (!valid_type) {
      violation = true;
    }

    if (tesla_legacy_stock_lkas) {
      violation = true;
    }
  }

  // DAS_control: longitudinal control
  if ((tesla_legacy_external_panda || tesla_legacy_hw1) && (msg->addr == tesla_legacy_das_control_addr)) {
    const int aeb_event = msg->data[2] & 0x03U;
    if (aeb_event != 0) {
      violation = true;
    }

    if (tesla_legacy_stock_aeb) {
      violation = true;
    }

    const int raw_accel_max = ((msg->data[6] & 0x1FU) << 4) | (msg->data[5] >> 4);
    const int raw_accel_min = ((msg->data[5] & 0x0FU) << 5) | (msg->data[4] >> 3);

    // Prevent both accel limits from being negative (reverse-after-standstill hazard)
    if ((raw_accel_max < TESLA_LONG_LIMITS.inactive_accel) && (raw_accel_min < TESLA_LONG_LIMITS.inactive_accel)) {
      violation = true;
    }

    violation |= longitudinal_accel_checks(raw_accel_max, TESLA_LONG_LIMITS);
    violation |= longitudinal_accel_checks(raw_accel_min, TESLA_LONG_LIMITS);
  }

  return !violation;
}

static bool tesla_legacy_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  // Only block on AP bus (bus 2). Allow everything else to forward.
  if (bus_num == 2) {
    // APS_eacMonitor: suppress stock "steering allowed" when we're controlling
    if ((!tesla_legacy_external_panda) && (!tesla_legacy_hw1) && (addr == 0x27DU)) {
      block_msg = true;
    }

    // DAS_steeringControl: suppress stock LKAS steering when not active
    if ((!tesla_legacy_external_panda) && (addr == 0x488U) && (!tesla_legacy_stock_lkas)) {
      block_msg = true;
    }

    // DAS_control: suppress stock AEB long when not active
    if ((tesla_legacy_external_panda || tesla_legacy_hw1) && (addr == tesla_legacy_das_control_addr) && (!tesla_legacy_stock_aeb)) {
      block_msg = true;
    }
  }

  return block_msg;
}

static safety_config tesla_legacy_init(uint16_t param) {
  tesla_legacy_external_panda = GET_FLAG(param, TESLA_FLAG_EXTERNAL_PANDA);
  tesla_legacy_hw1 = GET_FLAG(param, TESLA_FLAG_HW1);
  tesla_legacy_hw2 = GET_FLAG(param, TESLA_FLAG_HW2);
  tesla_legacy_hw3 = GET_FLAG(param, TESLA_FLAG_HW3);
  tesla_legacy_op_stalk_enable = GET_FLAG(param, TESLA_FLAG_OP_STALK_ENABLE);

  // reset dynamic state
  tesla_legacy_op_autopilot_disabled = false;
  tesla_legacy_op_pedal_enabled = false;
  tesla_legacy_stock_aeb = false;
  tesla_legacy_stock_lkas = false;
  tesla_legacy_stock_lkas_prev = false;

  tesla_legacy_chassis_bus = 0U;
  tesla_legacy_di_torque1_addr = 0x106U;
  tesla_legacy_das_control_addr = 0x2BFU;  // HW2/HW3 powertrain message id

  // TX allowlists (bus numbers are panda-local: 0/1/2)
  static const CanMsg TESLA_LEGACY_TX_MSGS[] = {
    {0x659, 0, 8, .check_relay = false, .disable_static_blocking = true},  // internal carrier (blocked in tx_hook)
    {0x488, 0, 4, .check_relay = true,  .disable_static_blocking = true},  // DAS_steeringControl
    {0x27D, 0, 3, .check_relay = true,  .disable_static_blocking = true},  // APS_eacMonitor
  };

  static const CanMsg TESLA_LEGACY_PT_TX_MSGS[] = {
    {0x659, 0, 8, .check_relay = false, .disable_static_blocking = true},  // internal carrier (blocked in tx_hook)
    {0x2BF, 0, 8, .check_relay = true,  .disable_static_blocking = true},  // DAS_control
  };

  // RX checks (bus numbers are panda-local)
  static RxCheck TESLA_LEGACY_HW2_RX_CHECKS[] = {
    // STW_ACTN_RQ (stalk): allow either bus 0 or bus 2 (first seen locks)
    {.msg = {
      {0x45,  0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true},
      {0x45,  2, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true},
      {0},
    }},
    {.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},  // EPAS_sysStatus
    {.msg = {{0x155, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},  // ESP_B (vehicle speed)
    {.msg = {{0x20A, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},  // BrakeMessage
    {.msg = {{0x368, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},  // DI_state
  };

  static RxCheck TESLA_LEGACY_PT_RX_CHECKS[] = {
    // Powertrain panda does not necessarily see the stalk on its bus 0.
    // Keep RX checks to always-present powertrain frames to avoid safetyRxChecksInvalid.
    {.msg = {{0x106, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},  // DI_torque1
    {.msg = {{0x1F8, 0, 8, 50U,  .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},  // BrakeMessage (PT)
    {.msg = {{0x256, 0, 8, 10U,  .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},  // DI_state (PT)
  };

  if (tesla_legacy_external_panda) {
    return BUILD_SAFETY_CFG(TESLA_LEGACY_PT_RX_CHECKS, TESLA_LEGACY_PT_TX_MSGS);
  }
  return BUILD_SAFETY_CFG(TESLA_LEGACY_HW2_RX_CHECKS, TESLA_LEGACY_TX_MSGS);
}

const safety_hooks tesla_legacy_hooks = {
  .init = tesla_legacy_init,
  .rx = tesla_legacy_rx_hook,
  .tx = tesla_legacy_tx_hook,
  .fwd = tesla_legacy_fwd_hook,
};
