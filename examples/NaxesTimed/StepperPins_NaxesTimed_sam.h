#ifndef NAXES_TIMED_PINS_SAM_H
#define NAXES_TIMED_PINS_SAM_H

#include "StepperConfig.h"

// Atmel SAM / SAMD (Due, Feather M4): TCC waveform outputs. Two axes wired
// to TCC outputs. Placeholder assignments to be adapted to the actual board.
const uint8_t NaxesTimed_led_pin = PIN_UNDEFINED;
const struct stepper_config_s NaxesTimed_config_0[] = {
    {
      step : 10,  // TCC0/WO
      enable_low_active : 21,
      enable_high_active : PIN_UNDEFINED,
      direction : 23,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : false,
      on_delay_us : 0,
      off_delay_ms : 1000
    },
    {
      step : 5,  // TCC1/WO
      enable_low_active : 22,
      enable_high_active : PIN_UNDEFINED,
      direction : 24,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : false,
      on_delay_us : 0,
      off_delay_ms : 1000
    },
    STEPPER_CONFIG_END};

#endif
