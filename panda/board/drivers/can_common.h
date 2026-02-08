#pragma once

#include "can_common_declarations.h"

uint8_t can_controller_enable_mask = 0xFF;

void can_set_enable_mask(uint8_t mask) {
  can_controller_enable_mask = mask;
}

static void can_init_all(void) {
  for (uint8_t i = 0; i < PANDA_CAN_CNT; i++) {
    const uint8_t bit = (1U << i);
    const bool enabled = (can_controller_enable_mask & bit) != 0U;

    if (enabled) {
      can_init(i);
    } else {
      can_deinit(i);
    }
  }
}
