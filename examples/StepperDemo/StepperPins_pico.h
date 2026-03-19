#ifndef STEPPER_PINS_PICO_H
#define STEPPER_PINS_PICO_H

#include "StepperConfig.h"

const uint8_t led_pin = LED_BUILTIN;

const struct stepper_config_s pico_config_0[] = {
    {
      step : 14,
      enable_low_active : 13,
      enable_high_active : PIN_UNDEFINED,
      direction : 15,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 500000,
      off_delay_ms : 5000
    },
    {
      step : 18,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : PIN_UNDEFINED,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 500000,
      off_delay_ms : 5000
    },
    {
      step : 19,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : PIN_UNDEFINED,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 500000,
      off_delay_ms : 5000
    },
    {step : PIN_UNDEFINED},
    {step : PIN_UNDEFINED},
    {step : PIN_UNDEFINED},
    {step : PIN_UNDEFINED},
    {step : PIN_UNDEFINED},
    {step : PIN_UNDEFINED},
    {step : PIN_UNDEFINED},
    {step : PIN_UNDEFINED}};
#define NUM_CONFIGS 1
const struct stepper_config_set_s stepper_configs[NUM_CONFIGS] = {
    {"Default", pico_config_0},
};

#endif
