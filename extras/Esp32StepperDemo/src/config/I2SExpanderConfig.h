#pragma once

#include <stdint.h>

#define I2S_NUM_CHIPS 4
#define I2S_NUM_PINS (I2S_NUM_CHIPS * 8)

struct I2SExpanderConfig {
  bool enabled;
  uint8_t data_pin;
  uint8_t bclk_pin;
  uint8_t ws_pin;
  uint32_t pin_states;

  void setDefaults() {
    enabled = false;
    data_pin = 32;
    bclk_pin = 33;
    ws_pin = 14;
    pin_states = 0;
  }
};
