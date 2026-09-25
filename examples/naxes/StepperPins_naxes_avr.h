#ifndef NAXES_PINS_AVR_H
#define NAXES_PINS_AVR_H

#include "StepperConfig.h"

// One unified config array, naxes_config_0, regardless of platform.
// ATmega168/328/328p expose only two step channels (OC1A / OC1B), so the third
// axis is omitted and NAXES_HW becomes 2. ATmega2560 and the 32U4 expose
// OC1A/OC1B/OC1C, giving a full three-axis config.

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || \
    defined(__AVR_ATmega168__)
const uint8_t naxes_led_pin = PIN_UNDEFINED;
const struct stepper_config_s naxes_config_0[] = {
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
#define NAXES_AXES_WIRED 2

#else
// ATmega2560 / ATmega32U4: three step channels OC1A / OC1B / OC1C.
const uint8_t naxes_led_pin = PIN_UNDEFINED;
const struct stepper_config_s naxes_config_0[] = {
    {
      step : stepPinStepper1A,  // OC1A
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : 5,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : false,
      on_delay_us : 0,
      off_delay_ms : 5000
    },
    {
      step : stepPinStepper1B,  // OC1B
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : 7,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : false,
      on_delay_us : 0,
      off_delay_ms : 5000
    },
    {
      step : stepPinStepper1C,  // OC1C
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : 6,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : false,
      on_delay_us : 0,
      off_delay_ms : 5000
    },
    STEPPER_CONFIG_END};
#define NAXES_AXES_WIRED 3

#endif

#endif
