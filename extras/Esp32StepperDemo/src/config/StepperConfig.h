#pragma once

#include <stdint.h>

enum StepperDriverType {
  STEPPER_DRIVER_RMT,
  STEPPER_DRIVER_I2S_DIRECT,
  STEPPER_DRIVER_I2S_MUX
};

enum PinSource {
  PIN_SOURCE_NONE = 0,
  PIN_SOURCE_GPIO,
  PIN_SOURCE_I2S,
  PIN_SOURCE_EXTERNAL
};

struct StepperPinConfig {
  PinSource source;
  uint8_t pin;
  bool active_low;
  uint16_t delay_us;

  void setDefaults() {
    source = PIN_SOURCE_NONE;
    pin = 255;
    active_low = false;
    delay_us = 0;
  }
};

struct StepperConfig {
  uint8_t id;
  char name[32];

  StepperDriverType driver;
  StepperPinConfig step_pin;
  StepperPinConfig dir_pin;
  bool dir_high_counts_up;

  StepperPinConfig enable_pin_low;
  StepperPinConfig enable_pin_high;

  uint32_t speed_us;
  uint32_t acceleration;
  uint32_t linear_accel_steps;
  uint32_t jump_start_steps;

  bool auto_enable;
  uint32_t delay_to_enable_us;
  uint16_t delay_to_disable_ms;

  int32_t current_position;
  bool is_enabled;

  void setDefaults() {
    id = 255;
    name[0] = '\0';
    driver = STEPPER_DRIVER_RMT;
    step_pin.setDefaults();
    dir_pin.setDefaults();
    dir_high_counts_up = true;
    enable_pin_low.setDefaults();
    enable_pin_high.setDefaults();
    speed_us = 50;
    acceleration = 10000;
    linear_accel_steps = 0;
    jump_start_steps = 0;
    auto_enable = false;
    delay_to_enable_us = 1000;
    delay_to_disable_ms = 100;
    current_position = 0;
    is_enabled = false;
  }
};
