#pragma once
// /data/openpilot/opendbc_repo/opendbc/safety/modes/tesla_legacy.h
//
// Unity-parity Tesla "legacy" (HW2) safety for dual-panda harnessing.
//
// Key parity points:
//   - No relay checking (check_relay=false on all TX allowlist entries)
//   - Stalk gating identical to Unity: enable OP only when stock AP is disabled
//   - Forwarding parity:
//       * bus0 -> bus2 : block IC-integration frames (prevents duplicates/flooding)
//       * bus2 -> bus0 : optionally hide/neutralize post-disengage HUD faults for ~4s
//       * bus0 -> bus2 : when OP is allowed, force EPAS eacStatus from "inhibited" -> "available"
//   - Internal carrier (0x659) is consumed and blocked from the car; also carries OP engage/cancel edges.

#include "opendbc/safety/safety_declarations.h"
#include "opendbc/safety/helpers.h"

#define TESLA_LEGACY_PARAM_POWERTRAIN_PANDA   0x01U
#define TESLA_LEGACY_PARAM_HAS_AP_HARDWARE    0x02U
#define TESLA_LEGACY_PARAM_HAS_IC_INTEGRATION 0x08U
#define TESLA_LEGACY_PARAM_OP_STALK_ENABLE    0x20U

// Unity: TIME_TO_HIDE_ERRORS = 4 seconds
#define TESLA_LEGACY_TIME_TO_HIDE_ERRORS_US (4U * 1000000U)

static bool tesla_legacy_powertrain_panda = false;
static bool tesla_legacy_has_ap_hw = false;
static bool tesla_legacy_has_ic_integration = false;
static bool tesla_legacy_op_stalk_enable = false;

// Internal carrier bits (0x659, byte5)
static bool tesla_legacy_pedal_enabled = false;
static bool tesla_legacy_op_autopilot_disabled = false;
static bool tesla_legacy_op_stalk_main_edge = false;
static bool tesla_legacy_op_stalk_cancel_edge = false;

// Stock AP state (read on AP bus)
static bool tesla_legacy_autopilot_enabled = false;
static bool tesla_legacy_eac_enabled = false;
static bool tesla_legacy_autopark_enabled = false;

// Assist state (read from EPAS)
static bool tesla_legacy_hands_on = false;

static bool tesla_legacy_controls_allowed_prev = false;
static uint32_t tesla_legacy_time_op_disengaged = 0U;

static inline uint8_t tesla_legacy_compute_checksum(const CANPacket_t *msg) {
  // Unity tesla_compute_checksum: sum(addr bytes + payload excluding checksum byte)
  uint8_t checksum = (uint8_t)(msg->addr & 0xFFU) + (uint8_t)((msg->addr >> 8) & 0xFFU);
  const uint8_t len = GET_LEN(msg);
  for (uint8_t i = 0U; (i + 1U) < len; i++) {
    checksum = (uint8_t)(checksum + msg->data[i]);
  }
  return checksum;
}

static inline void tesla_legacy_set_last_byte_checksum(CANPacket_t *msg) {
  const uint8_t len = GET_LEN(msg);
  if (len > 0U) {
    msg->data[len - 1U] = tesla_legacy_compute_checksum(msg);
  }
}

static void tesla_legacy_apply_op_engage_edges(void) {
  // Edge-driven engage/cancel from internal carrier (0x659) for Unity parity.
  if (tesla_legacy_op_stalk_enable && tesla_legacy_op_autopilot_disabled) {
    if (tesla_legacy_op_stalk_main_edge) {
      pcm_cruise_check(true);
    }
    if (tesla_legacy_op_stalk_cancel_edge) {
      pcm_cruise_check(false);
    }
  }
  tesla_legacy_op_stalk_main_edge = false;
  tesla_legacy_op_stalk_cancel_edge = false;
}

static int tesla_legacy_rx_hook(CANPacket_t *to_push) {
  const int addr = to_push->addr;
  const int bus = to_push->bus;

  // ---- Stock AP state on AP bus (bus2) ----
  if (tesla_legacy_has_ap_hw && !tesla_legacy_powertrain_panda && (bus == 2)) {
    if (addr == 0x399) {
      tesla_legacy_autopilot_enabled = ((GET_BYTE(to_push, 0) & 0x0FU) > 2U);
      if (tesla_legacy_autopilot_enabled) {
        pcm_cruise_check(false);
      }
    }

    if (addr == 0x219) {
      tesla_legacy_eac_enabled = (((GET_BYTE(to_push, 0) & 0x0CU) >> 2) > 0U);
      tesla_legacy_autopark_enabled = ((GET_BYTE(to_push, 1) & 0x01U) != 0U);
      if (tesla_legacy_eac_enabled || tesla_legacy_autopark_enabled) {
        pcm_cruise_check(false);
      }
    }
  }

  // ---- Steering angle / driver torque (EPAS) ----
  // Uses DAS_steeringAngle-like content for angle_meas + hands_on.
  if (addr == 0x370) {
    const int16_t raw_angle = (int16_t)((GET_BYTE(to_push, 4) << 8) | GET_BYTE(to_push, 5));
    const int desired_angle = ((int)raw_angle) - 16384;

    update_sample(&angle_meas, desired_angle);

    const int hands_on_state = GET_BYTE(to_push, 3) & 0x3U;
    tesla_legacy_hands_on = (hands_on_state != 0);
  }

  // ---- Stalk (Unity: 0x45) ----
  if (addr == 0x45) {
    if ((!tesla_legacy_has_ap_hw) || (tesla_legacy_has_ap_hw && tesla_legacy_op_autopilot_disabled)) {
      const int ap_lever_position = GET_BYTE(to_push, 0) & 0x3FU;
      if (tesla_legacy_op_stalk_enable) {
        if (ap_lever_position == 2) {
          pcm_cruise_check(true);
        } else if (ap_lever_position == 1) {
          pcm_cruise_check(false);
        }
      }
    }
  }

  // ---- Internal engage/cancel edges (0x659) ----
  tesla_legacy_apply_op_engage_edges();

  // Track disengage time for HUD-fault hiding window
  if (tesla_legacy_controls_allowed_prev && !controls_allowed) {
    tesla_legacy_time_op_disengaged = microsecond_timer_get();
  }
  tesla_legacy_controls_allowed_prev = controls_allowed;

  return true;
}

static int tesla_legacy_tx_hook(CANPacket_t *to_send) {
  const int addr = to_send->addr;
  bool violation = false;

  // Internal carrier (0x659) from OP -> panda: consume + block from CAN.
  if (addr == 0x659) {
    const uint8_t b5 = GET_BYTE(to_send, 5);

    tesla_legacy_op_autopilot_disabled = ((b5 & 0x80U) != 0U);
    tesla_legacy_pedal_enabled = ((b5 & 0x20U) != 0U);
    tesla_legacy_op_stalk_main_edge = ((b5 & 0x02U) != 0U);
    tesla_legacy_op_stalk_cancel_edge = ((b5 & 0x01U) != 0U);

    // Note: edges are applied in rx_hook for deterministic ordering with controls_allowed tracking.
    return false;
  }

  // If stock AP is not disabled, do not allow OP actuation frames.
  if (tesla_legacy_has_ap_hw && !tesla_legacy_op_autopilot_disabled) {
    if ((addr == 0x488) || (addr == 0x27D)) {
      return false;
    }
  }

  // IC-integration frames should only be emitted when enabled by params.
  if (!tesla_legacy_has_ic_integration) {
    if ((addr == 0x389) || (addr == 0x399) || (addr == 0x239) || (addr == 0x309) ||
        (addr == 0x3A9) || (addr == 0x3B1) || (addr == 0x329) || (addr == 0x349) ||
        (addr == 0x369) || (addr == 0x3E9)) {
      return false;
    }
  }

  // Steering angle request (DAS_steeringControl, 0x488, 4 bytes on legacy)
  if (addr == 0x488) {
    const int16_t raw_angle = (int16_t)((GET_BYTE(to_send, 0) << 8) | GET_BYTE(to_send, 1));
    const int desired_angle = ((int)raw_angle) - 16384;

    const int steer_control_type = GET_BYTE(to_send, 2) & 0x3U;
    const bool steer_control_enabled = (steer_control_type != 0) && (steer_control_type != 3);

    // Unity parity: do not allow angle control when OP not allowed, unless driver is steering.
    if (!controls_allowed) {
      if (steer_control_enabled) {
        violation |= !tesla_legacy_hands_on;
      }
      if ((desired_angle < (angle_meas.min - 1)) || (desired_angle > (angle_meas.max + 1))) {
        violation |= !tesla_legacy_hands_on;
      }
    }

    if (controls_allowed) {
      violation |= steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, TESLA_STEERING_LIMITS, TESLA_STEERING_PARAMS);
    }

    desired_angle_last = desired_angle;
  }

  // Steering allow (APS_eacMonitor, 0x27D, 3 bytes on legacy)
  if (addr == 0x27D) {
    // Allow; no extra checks for legacy parity.
  }

  return !violation;
}

static bool tesla_legacy_fwd_hook(int bus_num, int addr) {
  (void)bus_num;
  // Never forward internal carrier (even if misrouted)
  if (addr == 0x659) {
    return true;
  }
  return false;
}

static bool tesla_legacy_fwd_msg_hook(int bus_num, CANPacket_t *to_fwd) {
  const int addr = to_fwd->addr;

  // No AP harnessing -> no forwarding needed
  if (!tesla_legacy_has_ap_hw) {
    return true;
  }

  // Unity: bus0 -> bus2 (car -> AP), block IC-integration stuff to prevent duplicates/flooding.
  if (bus_num == 0) {
    // Force EPAS eacStatus "inhibited" (2) -> "available" (1) when OP is in control and stock features are not active.
    if ((addr == 0x370) && controls_allowed && !(tesla_legacy_autopilot_enabled || tesla_legacy_eac_enabled || tesla_legacy_autopark_enabled)) {
      const uint8_t b6 = to_fwd->data[6];
      const uint8_t eac_status = (b6 & 0xE0U) >> 5;
      if (eac_status == 2U) {
        to_fwd->data[6] = (uint8_t)((b6 & 0x1FU) | (1U << 5));
        tesla_legacy_set_last_byte_checksum(to_fwd);
      }
    }

    if ((addr == 0x399) || (addr == 0x389) || (addr == 0x239) || (addr == 0x309) ||
        (addr == 0x3A9) || (addr == 0x329) || (addr == 0x369) || (addr == 0x349)) {
      return true;
    }

    // Also prevent steering-control echoes crossing buses.
    if ((addr == 0x488) || (addr == 0x27D)) {
      return true;
    }

    return false;
  }

  // Unity: bus2 -> bus0 (AP -> car), optionally hide post-disengage warnings for ~4s.
  if (bus_num == 2) {
    const uint32_t ts = microsecond_timer_get();
    const uint32_t dt = get_ts_elapsed(ts, tesla_legacy_time_op_disengaged);

    if (!controls_allowed && !tesla_legacy_autopilot_enabled && (dt <= TESLA_LEGACY_TIME_TO_HIDE_ERRORS_US)) {
      if (addr == 0x389) {
        const uint32_t w0 = (GET_BYTES_04(to_fwd) & 0xFFFF3FFFU);
        WORD_TO_BYTE_ARRAY(&to_fwd->data[0], w0);
        const uint32_t w1 = (GET_BYTES_48(to_fwd) & 0x00FFFFFFU);
        WORD_TO_BYTE_ARRAY(&to_fwd->data[4], w1);
        tesla_legacy_set_last_byte_checksum(to_fwd);
      } else if (addr == 0x399) {
        const uint32_t w0 = ((GET_BYTES_04(to_fwd) & 0xFFFFFFF0U) | 2U);
        WORD_TO_BYTE_ARRAY(&to_fwd->data[0], w0);
        const uint32_t w1 = (GET_BYTES_48(to_fwd) & 0x00FFFFFFU);
        WORD_TO_BYTE_ARRAY(&to_fwd->data[4], w1);
        tesla_legacy_set_last_byte_checksum(to_fwd);
      } else if ((addr == 0x329) || (addr == 0x349) || (addr == 0x369)) {
        for (uint8_t i = 0U; i < GET_LEN(to_fwd); i++) {
          to_fwd->data[i] = 0U;
        }
        tesla_legacy_set_last_byte_checksum(to_fwd);
      }
    }

    return false;
  }

  return false;
}

static const CanMsg TESLA_LEGACY_TX_MSGS_AP[] = {
  // Steering (legacy: 4 bytes)
  {0x488, 0, 4, .check_relay = false},
  // Steering allow monitor (legacy: 3 bytes)
  {0x27D, 0, 3, .check_relay = false},
  // Internal carrier (blocked in tx_hook)
  {0x659, 0, 8, .check_relay = false},

  // Optional IC/body frames (Unity parity; gated by tesla_legacy_has_ic_integration in tx_hook)
  {0x389, 0, 8, .check_relay = false},
  {0x399, 0, 8, .check_relay = false},
  {0x239, 0, 8, .check_relay = false},
  {0x309, 0, 8, .check_relay = false},
  {0x3A9, 0, 8, .check_relay = false},
  {0x3B1, 0, 8, .check_relay = false},
  {0x329, 0, 8, .check_relay = false},
  {0x349, 0, 8, .check_relay = false},
  {0x369, 0, 8, .check_relay = false},
  {0x3E9, 0, 8, .check_relay = false},
};

static const CanMsg TESLA_LEGACY_TX_MSGS_PT[] = {
  // Longitudinal (legacy)
  {0x2BF, 0, 8, .check_relay = false},
  // Internal carrier (blocked in tx_hook)
  {0x659, 0, 8, .check_relay = false},
};

static safety_config tesla_legacy_init(uint16_t param) {
  tesla_legacy_powertrain_panda = GET_FLAG(param, TESLA_LEGACY_PARAM_POWERTRAIN_PANDA);
  tesla_legacy_has_ap_hw = GET_FLAG(param, TESLA_LEGACY_PARAM_HAS_AP_HARDWARE);
  tesla_legacy_has_ic_integration = GET_FLAG(param, TESLA_LEGACY_PARAM_HAS_IC_INTEGRATION);
  tesla_legacy_op_stalk_enable = GET_FLAG(param, TESLA_LEGACY_PARAM_OP_STALK_ENABLE);

  tesla_legacy_pedal_enabled = false;
  tesla_legacy_op_autopilot_disabled = false;
  tesla_legacy_op_stalk_main_edge = false;
  tesla_legacy_op_stalk_cancel_edge = false;

  tesla_legacy_autopilot_enabled = false;
  tesla_legacy_eac_enabled = false;
  tesla_legacy_autopark_enabled = false;

  tesla_legacy_hands_on = false;
  tesla_legacy_controls_allowed_prev = false;
  tesla_legacy_time_op_disengaged = microsecond_timer_get();

  // Unity parity: allow forwarding only on the AP panda (powertrain panda blocks forwarding)
  safety_config cfg = tesla_legacy_powertrain_panda
    ? (safety_config){.tx_msgs = TESLA_LEGACY_TX_MSGS_PT, .tx_msgs_len = ARRAYSIZE(TESLA_LEGACY_TX_MSGS_PT), .disable_forwarding = true}
    : (safety_config){.tx_msgs = TESLA_LEGACY_TX_MSGS_AP, .tx_msgs_len = ARRAYSIZE(TESLA_LEGACY_TX_MSGS_AP), .disable_forwarding = false};

  return cfg;
}

static safety_hooks tesla_legacy_hooks = {
  .init = tesla_legacy_init,
  .rx = tesla_legacy_rx_hook,
  .tx = tesla_legacy_tx_hook,
  .fwd = tesla_legacy_fwd_hook,
  .fwd_msg = tesla_legacy_fwd_msg_hook,
};
