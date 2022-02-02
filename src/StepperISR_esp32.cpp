
#include "StepperISR.h"

// Only since esp-idf v4.4 MCPWM_TIMER0_PHASE_DIRECTION_S is defined. So use
// this to distinguish between the two versions
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)

#ifdef __ESP32_IDF_V44__
#include <driver/periph_ctrl.h>
#include <soc/periph_defs.h>
#endif /* __ESP32_IDF_V44__ */

// Here are the global variables to interface with the interrupts
StepperQueue fas_queue[NUM_QUEUES];

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
#ifdef SUPPPORT_ESP32_MCPWM_PCNT
	init_mcpwm_pcnt(queue_num, step_pin);
#endif
}

void StepperQueue::connect() {
#ifdef SUPPPORT_ESP32_MCPWM_PCNT
	connect_mcpwm_pcnt();
#endif
}

void StepperQueue::disconnect() {
#ifdef SUPPPORT_ESP32_MCPWM_PCNT
	disconnect_mcpwm_pcnt();
#endif
}

void StepperQueue::startQueue() {
#ifdef SUPPPORT_ESP32_MCPWM_PCNT
	startQueue_mcpwm_pcnt();
#endif
}
void StepperQueue::forceStop() {
#ifdef SUPPPORT_ESP32_MCPWM_PCNT
	forceStop_mcpwm_pcnt();
#endif
}
uint16_t StepperQueue::_getPerformedPulses() {
#ifdef SUPPPORT_ESP32_MCPWM_PCNT
	return _getPerformedPulses_mcpwm_pcnt();
#else
	return 0;
#endif
}

//*************************************************************************************************

bool StepperQueue::isValidStepPin(uint8_t step_pin) { return true; }
int8_t StepperQueue::queueNumForStepPin(uint8_t step_pin) { return -1; }

//*************************************************************************************************
void StepperTask(void *parameter) {
  FastAccelStepperEngine *engine = (FastAccelStepperEngine *)parameter;
  const TickType_t delay_4ms =
      (DELAY_MS_BASE + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
  while (true) {
    engine->manageSteppers();
    esp_task_wdt_reset();
    vTaskDelay(delay_4ms);
  }
}

void StepperQueue::adjustSpeedToStepperCount(uint8_t steppers) {
	max_speed_in_ticks =  80; // This equals 200kHz @ 16MHz
}

void fas_init_engine(FastAccelStepperEngine *engine, uint8_t cpu_core) {
#define STACK_SIZE 1000
#define PRIORITY configMAX_PRIORITIES
  if (cpu_core > 1) {
    xTaskCreate(StepperTask, "StepperTask", STACK_SIZE, engine, PRIORITY, NULL);
  } else {
    xTaskCreatePinnedToCore(StepperTask, "StepperTask", STACK_SIZE, engine,
                            PRIORITY, NULL, cpu_core);
  }
}
#endif
