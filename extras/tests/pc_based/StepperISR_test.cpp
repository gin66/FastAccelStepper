#include "fas_queue/stepper_queue.h"

#include "FastAccelStepper.h"

static uint8_t stepper_allocated_mask = 0;

void fas_init_engine(FastAccelStepperEngine* engine) {}

void fas_reset_stepper_allocation() { stepper_allocated_mask = 0; }

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
  _initVars();
  max_speed_in_ticks = 80;  // This equals 200kHz @ 16MHz
}
void StepperQueue::startQueue() { _isRunning = true; }
void StepperQueue::forceStop() {}
void StepperQueue::connect() {}
void StepperQueue::disconnect() {}

StepperQueue* StepperQueue::tryAllocateQueue(FastAccelStepperEngine* engine,
                                             uint8_t step_pin) {
  (void)engine;
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    if ((stepper_allocated_mask & (1 << i)) == 0) {
      fas_queue[i].init(i, step_pin);
      stepper_allocated_mask |= (1 << i);
      return &fas_queue[i];
    }
  }
  return nullptr;
}
