#ifndef COMMON_H
#define COMMON_H

//	ticks is multiplied by (1/TICKS_PER_S) in s
//	If steps is 0, then a pause is generated
struct stepper_command_s {
  uint16_t ticks;
  uint8_t steps;
  bool count_up;
};

struct queue_end_s {
  int32_t pos;  // in steps
  bool count_up;
  bool dir;
};
#endif
