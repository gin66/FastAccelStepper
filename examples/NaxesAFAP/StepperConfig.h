#ifndef STEPPER_CONFIG_H
#define STEPPER_CONFIG_H

#include <stdint.h>

struct stepper_config_s {
  uint8_t step;
  uint8_t enable_low_active;
  uint8_t enable_high_active;
  uint8_t direction;
  uint16_t dir_change_delay;
  bool direction_high_count_up;
  bool auto_enable;
  uint32_t on_delay_us;
  uint16_t off_delay_ms;
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  FasDriver driver_type;
#endif
};

struct stepper_config_set_s {
  const char* name;
  const struct stepper_config_s* config;
};

// Last entry of a config list (step == PIN_UNDEFINED). All fields are set so
// -Wmissing-field-initializers stays quiet.
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
#define STEPPER_CONFIG_END              \
  {                                     \
    step : PIN_UNDEFINED,               \
    enable_low_active : PIN_UNDEFINED,  \
    enable_high_active : PIN_UNDEFINED, \
    direction : PIN_UNDEFINED,          \
    dir_change_delay : 0,               \
    direction_high_count_up : true,     \
    auto_enable : false,                \
    on_delay_us : 0,                    \
    off_delay_ms : 0,                   \
    driver_type : DRIVER_DONT_CARE      \
  }
#else
#define STEPPER_CONFIG_END              \
  {                                     \
    step : PIN_UNDEFINED,               \
    enable_low_active : PIN_UNDEFINED,  \
    enable_high_active : PIN_UNDEFINED, \
    direction : PIN_UNDEFINED,          \
    dir_change_delay : 0,               \
    direction_high_count_up : true,     \
    auto_enable : false,                \
    on_delay_us : 0,                    \
    off_delay_ms : 0                    \
  }
#endif

#endif
