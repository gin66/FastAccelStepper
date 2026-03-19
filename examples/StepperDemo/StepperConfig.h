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

#endif
