#include "StepperISR.h"

#include "FastAccelStepper.h"

// Here are the global variables to interface with the interrupts
// StepperQueue fas_queue[NUM_QUEUES];

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) { _initVars(); }
void StepperQueue::commandAddedToQueue(bool start) {
  _isRunning = start;
  next_write_idx++;
}
int8_t StepperQueue::startPreparedQueue() { return AQE_OK; }
void StepperQueue::forceStop() {}
void StepperQueue::connect() {}
void StepperQueue::disconnect() {}
bool StepperQueue::isValidStepPin(uint8_t step_pin) { return true; }
int8_t StepperQueue::queueNumForStepPin(uint8_t step_pin) { return -1; }
