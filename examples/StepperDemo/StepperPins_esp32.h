#ifndef STEPPER_PINS_ESP32_H
#define STEPPER_PINS_ESP32_H

#include "StepperConfig.h"

// Example hardware configuration for esp32 board.
// Please adapt to your configuration
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
const uint8_t led_pin = 8;
#else
const uint8_t led_pin = PIN_UNDEFINED;
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C3)
const struct stepper_config_s esp32_config_0[] = {
    {
      step : 6,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : PIN_UNDEFINED,
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
      step : 7,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : PIN_UNDEFINED,
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
    {"C3-Test", esp32_config_0},
};
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
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
#else
// clang-format off
    // Test-HW
    // Position 01 linked to atmega nano
    // 2: Enable Left Pin 13 GPIO13   , DIR Right Pin 7 GPIO18,    Step Right Pin 13 GPIO15
    // 3: Enable Left Pin 12 GPIO12   , DIR Right Pin 6 GPIO19,    Step Right Pin 12 GPIO2  blue LED
    // 4: Enable Left Pin 11 GPIO14   , DIR Right Pin 5 GPIO21,    Step Right Pin 11 GPIO4
    // 5: Enable Left Pin 10 GPIO27   , DIR Right Pin 4 GPIO3 RX0, Step Right Pin 10 GPIO16 RX2
    // 6: Enable Left Pin 9  GPIO26 A9, DIR Right Pin 3 GPIO1 TX0, Step Right Pin 9  GPIO17 TX2
    // 7: Enable Left Pin 8  GPIO25 A8, DIR Right Pin 2 GPIO22,    Step Right Pin 8  GPIO5
    //                          ALL Enable: Right Pin 1 GPIO23
    // Left Pin 15: +5V
// clang-format on
const struct stepper_config_s esp32_config_0[] = {
    {
      step : 17,
      enable_low_active : 26,
      enable_high_active : PIN_UNDEFINED,
      direction : 18,
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
      enable_low_active : 13,
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
#if MAX_STEPPER > 2
    {
      step : 2,
      enable_low_active : 12,
      enable_high_active : PIN_UNDEFINED,
      direction : 19,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 500,
      off_delay_ms : 1000,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
    {
      step : 5,
      enable_low_active : 25,
      enable_high_active : PIN_UNDEFINED,
      direction : 22,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
#endif
#if MAX_STEPPER > 4
    {
      step : 16,
      enable_low_active : 27,
      enable_high_active : PIN_UNDEFINED,
      direction : 21,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
    {
      step : 4,
      enable_low_active : 14,
      enable_high_active : PIN_UNDEFINED,
      direction : 21,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
#endif
#if MAX_STEPPER > 6
    {
      step : 14,
      enable_low_active : 26,
      enable_high_active : PIN_UNDEFINED,
      direction : 19,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
    {
      step : 23,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : 18,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
#endif
#if MAX_STEPPER > 8
    {
      step : 32,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : 18,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
    {
      step : 33,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : PIN_UNDEFINED,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
#endif
#if MAX_STEPPER == 14
    {
      step : 25,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : 18,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
    {
      step : 26,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : 18,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
    {
      step : 22,
      enable_low_active : 26,
      enable_high_active : PIN_UNDEFINED,
      direction : 18,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
    {
      step : 21,
      enable_low_active : PIN_UNDEFINED,
      enable_high_active : PIN_UNDEFINED,
      direction : 18,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10,
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      driver_type : DRIVER_DONT_CARE,
#endif
    },
#endif
    {step : PIN_UNDEFINED}};
#define NUM_CONFIGS 1
const struct stepper_config_set_s stepper_configs[NUM_CONFIGS] = {
    {"Test-HW", esp32_config_0},
};
#endif

#endif
