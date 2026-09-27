#ifndef NAXES_TIMED_PINS_ESP32_H
#define NAXES_TIMED_PINS_ESP32_H

#include "StepperConfig.h"

// ESP32 / ESP32-C3 / ESP32-S2 / ESP32-S3: the RMT/LEDC driver is not limited
// to fixed pins, so the two axes are wired freely. These are placeholder
// GPIO assignments to be adapted to the actual board wiring.
const uint8_t NaxesTimed_led_pin = PIN_UNDEFINED;
const struct stepper_config_s NaxesTimed_config_0[] = {
    {
      step : 17,
      enable_low_active : 26,
      enable_high_active : PIN_UNDEFINED,
      direction : 18,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : false,
      on_delay_us : 0,
      off_delay_ms : 1000,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
    {
      step : 15,
      enable_low_active : 13,
      enable_high_active : PIN_UNDEFINED,
      direction : 19,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : false,
      on_delay_us : 0,
      off_delay_ms : 1000,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
    STEPPER_CONFIG_END};

#endif
