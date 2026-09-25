#ifndef NAXES_PINS_PICO_H
#define NAXES_PINS_PICO_H

#include "StepperConfig.h"

// Raspberry Pi Pico / Pico W / nanorp2040connect: GPIO is not fixed to a timer
// channel, so three axes are wired freely. Placeholder assignments to be
// adapted to the actual wiring.
const uint8_t naxes_led_pin = LED_BUILTIN;
const struct stepper_config_s naxes_config_0[] = {
    {
      step : 14,
      enable_low_active : 13,
      enable_high_active : PIN_UNDEFINED,
      direction : 15,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : false,
      on_delay_us : 0,
      off_delay_ms : 5000
    },
    {
      step : 18,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : 19,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : false,
      on_delay_us : 0,
      off_delay_ms : 5000
    },
    {
      step : 20,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : 21,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : false,
      on_delay_us : 0,
      off_delay_ms : 5000
    },
    STEPPER_CONFIG_END};

#endif
