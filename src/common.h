#ifndef COMMON_H
#define COMMON_H

//	ticks is multiplied by (1/TICKS_PER_S) in s
//	steps must be less than 128 aka 7 bits
//	If steps is 0, then a pause is generated
struct stepper_command_s {
  uint16_t ticks;
  uint8_t steps;
  bool count_up;
};

struct queue_end_s {
  int32_t pos;     // in steps
  uint32_t ticks;  // in timer ticks, 0 on stopped stepper
  uint32_t ticks_from_last_step;
  bool count_up;
  bool dir;
};
#endif
