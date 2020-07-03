#include "StepperISR.h"

// Here are the global variables to interface with the interrupts
//StepperQueue fas_queue[NUM_QUEUES];

void StepperQueue::init(uint8_t step_pin) {
  dirPin = 255;
  autoEnablePin = 255;
  read_ptr = 0;
  next_write_ptr = 0;
}
