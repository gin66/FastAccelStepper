#include "fas_queue/stepper_queue.h"

#include "FastAccelStepper.h"

static uint8_t stepper_allocated_mask = 0;

void fas_init_engine(FastAccelStepperEngine* engine) {}

void fas_reset_stepper_allocation() { stepper_allocated_mask = 0; }

bool StepperQueue::init(FastAccelStepperEngine* engine, uint8_t queue_num,
                        uint8_t step_pin) {
  _initVars();
  max_speed_in_ticks = 80;  // This equals 200kHz @ 16MHz
  return true;
}
void StepperQueue::startQueue() { _isRunning = true; }
void StepperQueue::forceStop() {}
void StepperQueue::connect() {}
void StepperQueue::disconnect() {}

StepperQueue* StepperQueue::tryAllocateQueue(uint8_t step_pin) {
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    if ((stepper_allocated_mask & (1 << i)) == 0) {
      if (!fas_queue[i].init(nullptr, i, step_pin)) {
        return nullptr;
      }
      stepper_allocated_mask |= (1 << i);
      return &fas_queue[i];
    }
  }
  return nullptr;
}
