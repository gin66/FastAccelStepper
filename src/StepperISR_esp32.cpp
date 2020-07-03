#include "StepperISR.h"

#if defined(ARDUINO_ARCH_ESP32)

// Here are the global variables to interface with the interrupts
StepperQueue fas_queue[NUM_QUEUES];

void StepperQueue::init(uint8_t step_pin) {
  dirPin = 255;
  autoEnablePin = 255;
  read_ptr = 0;
  next_write_ptr = 0;
  digitalWrite(step_pin, LOW);
  pinMode(step_pin, OUTPUT);
}

#endif
