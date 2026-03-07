#include <stdint.h>

#include "fas_queue/stepper_queue.h"

// Here are the global variables to interface with the interrupts
#if defined(SUPPORT_DYNAMIC_ALLOCATION)
StepperQueue* fas_queue[NUM_QUEUES] = {nullptr};
#else
StepperQueue fas_queue[NUM_QUEUES];
#endif

void StepperQueue::_initVars() {
  dirPin = PIN_UNDEFINED;
  ignore_commands = false;
  read_idx = 0;
  next_write_idx = 0;
  queue_end.dir = true;
  queue_end.count_up = true;
  queue_end.pos = 0;
  dirHighCountsUp = true;
  _pd_initVars();
}
