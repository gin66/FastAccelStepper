#ifndef STEPPER_PINS_ESP32C6_H
#define STEPPER_PINS_ESP32C6_H

#include "StepperConfig.h"

const uint8_t led_pin = 8;

const struct stepper_config_s esp32_config_0[] = {
    {
      step : 6,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : 7,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 50,
      off_delay_ms : 1000,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
    {
      step : 15,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : 18,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 500,
      off_delay_ms : 1000,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
    {step : PIN_UNDEFINED}};
#define NUM_CONFIGS 1
const struct stepper_config_set_s stepper_configs[NUM_CONFIGS] = {
    {"C6-Test", esp32_config_0},
};

#endif
