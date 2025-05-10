#include "StepperISR.h"

#if defined(SUPPORT_RP_PICO)
#include <FreeRTOS.h>
#include <task.h>

// Here are the global variables to interface with the interrupts
StepperQueue fas_queue[NUM_QUEUES];

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
  uint8_t channel = queue_num;
}

void StepperQueue::connect() {
}

void StepperQueue::disconnect() {
}

bool StepperQueue::isReadyForCommands() {
  return false;
}

void StepperQueue::startQueue() {
}
void StepperQueue::forceStop() {
}
int32_t StepperQueue::getCurrentPosition() {
  return 0;
}

//*************************************************************************************************

bool StepperQueue::isValidStepPin(uint8_t step_pin) {
  return true;
}
int8_t StepperQueue::queueNumForStepPin(uint8_t step_pin) { return -1; }

//*************************************************************************************************
void StepperTask(void *parameter) {
  FastAccelStepperEngine *engine = (FastAccelStepperEngine *)parameter;
  while (true) {
    engine->manageSteppers();
    const TickType_t delay_time =
        (engine->_delay_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
    vTaskDelay(delay_time);
  }
}

void StepperQueue::adjustSpeedToStepperCount(uint8_t steppers) {
  max_speed_in_ticks = 80;  // This equals 200kHz @ 16MHz
}

void fas_init_engine(FastAccelStepperEngine *engine, uint8_t cpu_core) {
#define STACK_SIZE 3000
#define PRIORITY (configMAX_PRIORITIES - 1)
  engine->_delay_ms = DELAY_MS_BASE;
  xTaskCreate(StepperTask, "StepperTask", STACK_SIZE, engine, PRIORITY, NULL);
}
#endif
