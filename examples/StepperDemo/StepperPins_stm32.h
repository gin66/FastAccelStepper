// Stepper pin mapping for STM32
// Include this in your sketch to set default pins:
//   #include "StepperPins_stm32.h"
#ifndef STEPPERPINS_STM32_H
#define STEPPERPINS_STM32_H

#include "StepperConfig.h"

// ====================================================================
// STEP pins — PA0-PA3.
// On STM32, ANY GPIO can be a step pin (timer is used only for
// interrupt timing, not direct pin output).
// Compatible with: F103, G070, F401, H743, L476
// ====================================================================
#ifndef STEP_PIN_STEPPER_0
#define STEP_PIN_STEPPER_0 PA0
#endif
#ifndef STEP_PIN_STEPPER_1
#define STEP_PIN_STEPPER_1 PA1
#endif
#ifndef STEP_PIN_STEPPER_2
#define STEP_PIN_STEPPER_2 PA2
#endif
#ifndef STEP_PIN_STEPPER_3
#define STEP_PIN_STEPPER_3 PA3
#endif

// ====================================================================
// DIR pins — PB0-PB3.
// Available on all 5 CI boards (48-pin to 144-pin).
// ====================================================================
#ifndef DIR_PIN_STEPPER_0
#define DIR_PIN_STEPPER_0 PB0
#endif
#ifndef DIR_PIN_STEPPER_1
#define DIR_PIN_STEPPER_1 PB1
#endif
#ifndef DIR_PIN_STEPPER_2
#define DIR_PIN_STEPPER_2 PB2
#endif
#ifndef DIR_PIN_STEPPER_3
#define DIR_PIN_STEPPER_3 PB3
#endif

// ====================================================================
// ENABLE pin — PA4.
// PA4 is NOT LED_BUILTIN on any board, available on all 5 CI boards.
// ====================================================================
#ifndef ENABLE_PIN_STEPPER_0
#define ENABLE_PIN_STEPPER_0 PA4
#endif

// ====================================================================
// Config array for StepperDemo
// MAX_STEPPER = 4 (from pd_config.h)
// dir_change_delay = 1000µs (= 1ms) for stable direction change
// ====================================================================
const uint8_t led_pin = PIN_UNDEFINED;
const struct stepper_config_s stepper_config[MAX_STEPPER] = {
    {
      step : PA0,
      enable_low_active : PA4,
      enable_high_active : PIN_UNDEFINED,
      direction : PB0,
      dir_change_delay : 1000,
      direction_high_count_up : true,
      auto_enable : true,           // has valid enable pin PA4
      on_delay_us : 50,
      off_delay_ms : 1000,
    },
    {
      step : PA1,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : PB1,
      dir_change_delay : 1000,
      direction_high_count_up : true,
      auto_enable : false,          // no enable pin for stepper 1
      on_delay_us : 500,
      off_delay_ms : 1000,
    },
    {step : PIN_UNDEFINED},  // stepper 3
    {step : PIN_UNDEFINED},  // stepper 4
};
// Only 1 config variant — can add more later
#define NUM_CONFIGS 1
const struct stepper_config_set_s stepper_configs[NUM_CONFIGS] = {
    {"Default", stepper_config},
};

#endif /* STEPPERPINS_STM32_H */