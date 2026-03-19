#ifndef STEPPER_PINS_AVR_H
#define STEPPER_PINS_AVR_H

#include "StepperConfig.h"

#if (defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || \
     defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__))
// Example hardware configuration for Arduino Nano
// Please adapt to your configuration
const uint8_t led_pin = 13;  // turn off with PIN_UNDEFINED
const struct stepper_config_s stepper_config[MAX_STEPPER] = {
    {
      // stepper 1 shall be connected to OC1A
      step : stepPinStepper1A,
      enable_low_active : 6,
      enable_high_active : PIN_UNDEFINED,
      direction : 5,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 500000,
      off_delay_ms : 5000
    },
    {
      // stepper 2 shall be connected to OC1B
      step : stepPinStepper1B,
      enable_low_active : 8,
      enable_high_active : PIN_UNDEFINED,
      direction : 7,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    }};
#elif defined(__AVR_ATmega2560__)
// Example hardware configuration for Arduino ATmega2560
// Please adapt to your configuration
const uint8_t led_pin = PIN_UNDEFINED;  // turn off with PIN_UNDEFINED
const struct stepper_config_s stepper_config[MAX_STEPPER] = {
    {
      step : stepPinStepperA,
      enable_low_active : 19,
      enable_high_active : PIN_UNDEFINED,
      direction : 21,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 500000,
      off_delay_ms : 5000
    },
    {
      step : stepPinStepperB,
      enable_low_active : 18,
      enable_high_active : PIN_UNDEFINED,
      direction : 20,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    },
    {
      // stepper 3 shall be connected to OC4C
      step : stepPinStepperC,
      enable_low_active : 43,
      enable_high_active : PIN_UNDEFINED,
      direction : 42,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    }};
#elif defined(__AVR_ATmega32U4__)
// Example hardware configuration for Arduino ATmega32u4
// Please adapt to your configuration
const uint8_t led_pin = PIN_UNDEFINED;  // turn off with PIN_UNDEFINED
const struct stepper_config_s stepper_config[MAX_STEPPER] = {
    {
      step : stepPinStepperA,
      enable_low_active : 16,
      enable_high_active : PIN_UNDEFINED,
      direction : 26,  // PB4
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 500000,
      off_delay_ms : 5000
    },
    {
      step : stepPinStepperB,
      enable_low_active : 15,
      enable_high_active : PIN_UNDEFINED,
      direction : 14,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    }};
#endif

#endif
