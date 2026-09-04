#ifndef STEPPER_PINS_SAMD_H
#define STEPPER_PINS_SAMD_H

#include "StepperConfig.h"

// Hardware configuration for the Adafruit Feather M4 (SAMD51J19A).
// Step pins need a TCC waveform output in the variant pin table; the
// Feather M4 breaks out TCC0 (pins 0/1/10/11/12/13) and TCC1 (pins 5/6/9).
// Only the first two entries below connect; the remaining ones are placeholders
// for boards that expose more TCC outputs (stepperConnectToPin() returns NULL).
const uint8_t led_pin = PIN_LED_13;
const struct stepper_config_s stepper_config[MAX_STEPPER] = {
    {
      step : 10,  // TCC0/WO[0]
      enable_low_active : 21,
      enable_high_active : PIN_UNDEFINED,
      direction : 23,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 50,
      off_delay_ms : 1000
    },
    {
      step : 5,  // TCC1/WO[0]
      enable_low_active : 22,
      enable_high_active : PIN_UNDEFINED,
      direction : 24,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 500,
      off_delay_ms : 1000
    },
    {
      step : 6,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : 12,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : false,
      on_delay_us : 0,
      off_delay_ms : 0
    },
    {
      step : 9,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : 11,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : false,
      on_delay_us : 0,
      off_delay_ms : 0
    },
    {
      step : 4,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : 23,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : false,
      on_delay_us : 0,
      off_delay_ms : 0
    }};

#endif