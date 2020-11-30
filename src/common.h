#ifndef COMMON_H
#define COMMON_H

struct stepper_command_s {
  uint32_t ticks;
  uint8_t steps;
  uint8_t state;
  bool count_up;
};

#endif
