#ifndef COMMON_H
#define COMMON_H

struct stepper_command_s {
  uint32_t ticks;
  uint8_t steps;
  uint8_t state;
  bool count_up;
};

struct queue_end_s {
  int32_t pos;     // in steps
  uint32_t ticks;  // in timer ticks, 0 on stopped stepper
  bool dir;
};
#endif
