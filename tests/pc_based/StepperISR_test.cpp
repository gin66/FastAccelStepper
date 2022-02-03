#include "StepperISR.h"

#include "FastAccelStepper.h"

// Here are the global variables to interface with the interrupts
// StepperQueue fas_queue[NUM_QUEUES];

void fas_init_engine(FastAccelStepperEngine* engine, uint8_t cpu_core) {}

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) { _initVars(); }
void StepperQueue::startQueue() { _isRunning = true; }
void StepperQueue::forceStop() {}
void StepperQueue::connect() {}
void StepperQueue::disconnect() {}
bool StepperQueue::isValidStepPin(uint8_t step_pin) { return true; }
int8_t StepperQueue::queueNumForStepPin(uint8_t step_pin) { return -1; }
void StepperQueue::adjustSpeedToStepperCount(uint8_t steppers) {
  max_speed_in_ticks = 80;  // This equals 200kHz @ 16MHz
}
