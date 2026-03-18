#include <stdint.h>

#include "fas_queue/stepper_queue.h"

// Here are the global variables to interface with the interrupts
#if defined(SUPPORT_DYNAMIC_ALLOCATION)
StepperQueue* fas_queue[NUM_QUEUES] = {nullptr};
#else
StepperQueue fas_queue[NUM_QUEUES];
#endif

void StepperQueue::_initVars() {
  __builtin_memset(this, 0, sizeof(*this));
  _base_initVars();
  _pd_initVars();
}
