#include "StepperISR.h"

#if defined(SUPPORT_ESP32)

// Here are the global variables to interface with the interrupts
StepperQueue fas_queue[NUM_QUEUES];

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
  uint8_t channel = queue_num;
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  if (channel < QUEUES_MCPWM_PCNT) {
    use_rmt = false;
    init_mcpwm_pcnt(channel, step_pin);
    return;
  }
  channel -= QUEUES_MCPWM_PCNT;
#endif
#ifdef SUPPORT_ESP32_RMT
  use_rmt = true;
  init_rmt(channel, step_pin);
#endif
}

void StepperQueue::connect() {
#ifdef SUPPORT_ESP32_RMT
  if (use_rmt) {
    connect_rmt();
    return;
  }
#endif
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  connect_mcpwm_pcnt();
#endif
}

void StepperQueue::disconnect() {
#ifdef SUPPORT_ESP32_RMT
  if (use_rmt) {
    disconnect_rmt();
    return;
  }
#endif
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  disconnect_mcpwm_pcnt();
#endif
}

bool StepperQueue::isReadyForCommands() {
#if defined(SUPPORT_ESP32_RMT) && defined(SUPPORT_ESP32_MCPWM_PCNT)
  if (use_rmt) {
    return isReadyForCommands_rmt();
  }
  return isReadyForCommands_mcpwm_pcnt();
#elif defined(SUPPORT_ESP32_MCPWM_PCNT)
  return isReadyForCommands_mcpwm_pcnt();
#elif defined(SUPPORT_ESP32_RMT)
  return isReadyForCommands_rmt();
#else
#error "Nothing defined here"
#endif
}

void StepperQueue::startQueue() {
#ifdef SUPPORT_ESP32_RMT
  if (use_rmt) {
    startQueue_rmt();
    return;
  }
#endif
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  startQueue_mcpwm_pcnt();
#endif
}
void StepperQueue::forceStop() {
#ifdef SUPPORT_ESP32_RMT
  if (use_rmt) {
    forceStop_rmt();
    return;
  }
#endif
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  forceStop_mcpwm_pcnt();
#endif
}
uint16_t StepperQueue::_getPerformedPulses() {
#ifdef SUPPORT_ESP32_RMT
  if (use_rmt) {
    return _getPerformedPulses_rmt();
  }
#endif
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  return _getPerformedPulses_mcpwm_pcnt();
#endif
  return 0;
}

//*************************************************************************************************

bool StepperQueue::isValidStepPin(uint8_t step_pin) {
  gpio_drive_cap_t strength;
  esp_err_t res = gpio_get_drive_capability((gpio_num_t)step_pin, &strength);
  return res == ESP_OK;
}
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
  max_speed_in_ticks = 80;  // This equals 200kHz @ 16MHz
}

void fas_init_engine(FastAccelStepperEngine *engine, uint8_t cpu_core) {
#define STACK_SIZE 2000
#define PRIORITY configMAX_PRIORITIES
  if (cpu_core > 1) {
    xTaskCreate(StepperTask, "StepperTask", STACK_SIZE, engine, PRIORITY, NULL);
  } else {
    xTaskCreatePinnedToCore(StepperTask, "StepperTask", STACK_SIZE, engine,
                            PRIORITY, NULL, cpu_core);
  }
}

#endif
