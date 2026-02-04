#pragma once

// Tesla legacy safety mode (XNOR Route A)
// Minimal Unity-parity "internal carrier" contract without relying on repo-specific RxCheck helpers.
//
// Goals:
//  - Use stalk (0x45) edges to request controls_allowed (kept OUTSIDE any brake logic).
//  - Parse internal carrier (0x659) in tx_hook to learn AP-disabled / pedal-enabled and (optionally) edges.
//  - Block 0x659 from ever reaching the vehicle (carrier is internal-only).
//
// NOTE: This header is included by opendbc/safety/safety.h, which already includes
//       safety_declarations.h, helpers.h, lateral.h, longitudinal.h. Do NOT include
//       test-only headers here.

#define TESLA_LEGACY_ARRLEN(x) (sizeof(x) / sizeof((x)[0]))

static bool tesla_legacy_op_autopilot_disabled = false;
static bool tesla_legacy_op_pedal_enabled = false;

// Stalk state (0x45)
static uint8_t tesla_legacy_last_stalk = 0U;

// ---- Steering angle limits (AngleSteeringLimits uses lookup tables) ----
static const struct lookup_t tesla_legacy_angle_rate_up = {
  .x = {0.0F, 5.0F, 15.0F},
  .y = {10.0F, 1.6F, 0.3F},
};

static const struct lookup_t tesla_legacy_angle_rate_down = {
  .x = {0.0F, 5.0F, 15.0F},
  .y = {10.0F, 7.0F, 0.8F},
};

// desired_angle in Tesla legacy is commonly degrees * 100 (CAN units)
static const AngleSteeringLimits tesla_legacy_steering_limits = {
  .max_angle = 75000,                 // 750 deg * 100 (conservative)
  .angle_deg_to_can = 100.0F,
  .angle_rate_up_lookup = tesla_legacy_angle_rate_up,
  .angle_rate_down_lookup = tesla_legacy_angle_rate_down,
  .max_angle_error = 5000,            // 50 deg * 100
  .angle_error_min_speed = 1.0F,
  .frequency = 50U,
  .angle_is_curvature = false,
  .enforce_angle_error = true,
  .inactive_angle_is_zero = true,
};

// Vehicle model params for steer_angle_cmd_checks_vm (conservative defaults)
static const AngleSteeringParams tesla_legacy_steering_params = {
  .slip_factor = 0.001F,
  .steer_ratio = 15.0F,
  .wheelbase = 2.96F,
};

// ---- TX allowlist ----
// IMPORTANT: include all struct fields (check_relay, disable_static_blocking) to satisfy -Werror.
static const CanMsg tesla_legacy_tx_msgs[] = {
  // Steering request
  {0x488, 0, 8, false, false},
  {0x488, 2, 8, false, false},

  // Internal carrier (allowed to be sent to the panda firmware, but blocked from the car in tx_hook)
  {0x659, 0, 8, false, false},
  {0x659, 2, 8, false, false},
};

// ---- Hooks ----
static void tesla_legacy_rx_hook(const CANPacket_t *msg) {
  const int addr = GET_ADDR(msg);

  // STW_ACTN_RQ / stalk (0x45)
  if (addr == 0x45) {
    // observed: dat[0]=0x42 when lever=2; use low 6 bits as lever position
    const uint8_t lever = (uint8_t)(msg->data[0] & 0x3FU);

    const bool main_edge = (lever == 2U) && (tesla_legacy_last_stalk != 2U);
    const bool cancel_edge = (lever == 1U) && (tesla_legacy_last_stalk != 1U);

    tesla_legacy_last_stalk = lever;

    // Keep stalk gating OUTSIDE any brake/gas checks.
    // Only allow engage when AP is disabled (Unity parity).
    if (main_edge && tesla_legacy_op_autopilot_disabled) {
      controls_allowed = true;
    }
    if (cancel_edge) {
      controls_allowed = false;
    }
  }
}

static bool tesla_legacy_tx_hook(const CANPacket_t *msg) {
  const int addr = GET_ADDR(msg);

  // Internal carrier: parse then block from being transmitted to the vehicle
  if (addr == 0x659) {
    const uint8_t b5 = msg->data[5];

    // Contract (Tesla659Carrier):
    //  bit7: AP disabled
    //  bit6: pedal enabled
    //  bit0: main edge
    //  bit1: cancel edge
    tesla_legacy_op_autopilot_disabled = (b5 & 0x80U) != 0U;
    tesla_legacy_op_pedal_enabled = (b5 & 0x40U) != 0U;

    const bool main_edge = (b5 & 0x01U) != 0U;
    const bool cancel_edge = (b5 & 0x02U) != 0U;

    // Optional: also honor carrier edges (so both pandas behave identically)
    if (main_edge && tesla_legacy_op_autopilot_disabled) {
      controls_allowed = true;
    }
    if (cancel_edge) {
      controls_allowed = false;
    }

    // Never allow 0x659 to go to the car
    return false;
  }

  // Steering angle command checks
  if (addr == 0x488) {
    const int desired_angle = (int16_t)GET_BYTES(msg, 0U, 2U);
    const bool steer_control_enabled = (msg->data[2] & 0x01U) != 0U;

    if (steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled,
                                  tesla_legacy_steering_limits, tesla_legacy_steering_params)) {
      return false;
    }
  }

  return true;
}

static int tesla_legacy_fwd_hook(int bus_num, int addr) {
  (void)bus_num;
  (void)addr;
  return -1;
}

static safety_config tesla_legacy_init(uint16_t param) {
  (void)param;

  controls_allowed = false;
  tesla_legacy_op_autopilot_disabled = false;
  tesla_legacy_op_pedal_enabled = false;
  tesla_legacy_last_stalk = 0U;

  // Avoid repo-specific RxCheck wiring while iterating; no rx checks enforced here.
  safety_config cfg = {0};
  cfg.rx_checks = NULL;
  cfg.rx_checks_len = 0U;
  cfg.tx_msgs = tesla_legacy_tx_msgs;
  cfg.tx_msgs_len = (uint16_t)TESLA_LEGACY_ARRLEN(tesla_legacy_tx_msgs);
  return cfg;
}

const safety_hooks tesla_legacy_hooks = {
  .init = tesla_legacy_init,
  .rx = tesla_legacy_rx_hook,
  .tx = tesla_legacy_tx_hook,
  .fwd = tesla_legacy_fwd_hook,
};
