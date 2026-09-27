#ifndef NAXES_TIMED_PINS_AVR_H
#define NAXES_TIMED_PINS_AVR_H

#include "StepperConfig.h"

// Two axes on the AVR Timer 1 channels OC1A (digital 9) / OC1B (digital 10).
// The ATmega168/328/328p expose exactly these two step channels, and the
// larger AVRs are used with the same two so one config array fits all.
const uint8_t NaxesTimed_led_pin = PIN_UNDEFINED;
const struct stepper_config_s NaxesTimed_config_0[] = {
    {
      step : stepPinStepper1A,  // OC1A, digital 9
      enable_low_active : 6,    // PD6
      enable_high_active : PIN_UNDEFINED,
      direction : 5,  // PD5
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : false,
      on_delay_us : 0,
      off_delay_ms : 5000
    },
    {
      step : stepPinStepper1B,  // OC1B, digital 10
      enable_low_active : 8,    // PB0
      enable_high_active : PIN_UNDEFINED,
      direction : 7,  // PD7
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : false,
      on_delay_us : 0,
      off_delay_ms : 5000
    },
    STEPPER_CONFIG_END};

#endif
