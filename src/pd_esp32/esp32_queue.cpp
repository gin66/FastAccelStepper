#include "fas_queue/stepper_queue.h"

#if defined(SUPPORT_ESP32_I2S)
#include "pd_esp32/i2s_manager.h"
#endif

#if defined(SUPPORT_ESP32)

#include <esp_task_wdt.h>

#if defined(SUPPORT_DYNAMIC_ALLOCATION)
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
uint8_t StepperQueue::queues_allocated = 0;
#if defined(SUPPORT_ESP32_I2S)
bool StepperQueue::_i2s_mux_initialized = false;
uint32_t StepperQueue::_i2s_mux_allocated_bitmask = 0;
I2sManager* StepperQueue::_i2s_mux_manager = nullptr;
#endif
#ifdef SUPPORT_ESP32_MCPWM_PCNT
uint8_t StepperQueue::_mcpwm_pcnt_allocated = 0;
#endif
#ifdef SUPPORT_ESP32_RMT
uint8_t StepperQueue::_rmt_allocated = 0;
#endif
#else
uint8_t StepperQueue::queues_allocated = 0;
#endif  // SUPPORT_SELECT_DRIVER_TYPE
#endif

bool StepperQueue::init(FastAccelStepperEngine* engine, uint8_t queue_num,
                        uint8_t step_pin) {
  uint8_t channel = queue_num;
  max_speed_in_ticks = 80;
#ifdef DEBUG
  Serial.printf("init: queue_num=%d step_pin=%d\n", queue_num, step_pin);
#endif
#if defined(SUPPORT_DYNAMIC_ALLOCATION)
#else
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  use_mcpwm_pcnt = false;
  if (channel < QUEUES_MCPWM_PCNT) {
    use_mcpwm_pcnt = true;
  }
#endif
#ifdef SUPPORT_ESP32_RMT
  use_rmt = false;
  if ((queue_num >= QUEUES_MCPWM_PCNT) &&
      (queue_num < QUEUES_RMT + QUEUES_MCPWM_PCNT)) {
    use_rmt = true;
    channel = queue_num - QUEUES_MCPWM_PCNT;
  }
#endif
#endif
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  if (use_mcpwm_pcnt) {
    return init_mcpwm_pcnt(channel, step_pin);
  }
#endif
#ifdef SUPPORT_ESP32_RMT
  if (use_rmt) {
    return init_rmt(channel, step_pin);
  }
#endif
#if defined(SUPPORT_ESP32_I2S)
  if (use_i2s) {
    return init_i2s(step_pin);
  }
#endif
  return false;
}

void StepperQueue::connect() {
#ifdef SUPPORT_ESP32_I2S
  if (use_i2s) {
    return;
  }
#endif
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
#ifdef SUPPORT_ESP32_I2S
  if (use_i2s) {
    return;
  }
#endif
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
#ifdef SUPPORT_ESP32_I2S
  if (use_i2s) {
    return isReadyForCommands_i2s();
  }
#endif
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
#ifdef SUPPORT_ESP32_I2S
  if (use_i2s) {
    startQueue_i2s();
    return;
  }
#endif
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
#ifdef SUPPORT_ESP32_I2S
  if (use_i2s) {
    forceStop_i2s();
    return;
  }
#endif
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
#ifdef SUPPORT_ESP32_I2S
  if (use_i2s) {
    return _getPerformedPulses_i2s();
  }
#endif
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

bool StepperQueue::isValidStepPin(uint8_t step_pin) {
  gpio_drive_cap_t strength;
  esp_err_t res = gpio_get_drive_capability((gpio_num_t)step_pin, &strength);
  return res == ESP_OK;
}

#if defined(SUPPORT_SELECT_DRIVER_TYPE) && defined(SUPPORT_DYNAMIC_ALLOCATION)
// dynamic allocation only for espidf >=5.3, so no mcpwm/pcnt
StepperQueue* StepperQueue::tryAllocateQueue(FasDriver driver,
                                             uint8_t step_pin) {
  if (StepperQueue::queues_allocated >= MAX_STEPPER) {
    return nullptr;
  }

  if (step_pin & PIN_EXTERNAL_FLAG) {
    return nullptr;
  }
  if (step_pin & PIN_I2S_FLAG) {
#if !defined(SUPPORT_ESP32_I2S)
    return nullptr;
#endif
    if (!StepperQueue::_i2s_mux_initialized) {
      return nullptr;
    }
    uint8_t slot = step_pin & 0x1F;
    if (slot >= 32) {
      return nullptr;
    }
    uint32_t bit = 1UL << slot;
    if (StepperQueue::_i2s_mux_allocated_bitmask & bit) {
      return nullptr;
    }
    StepperQueue::_i2s_mux_allocated_bitmask |= bit;
    StepperQueue* q = new StepperQueue();
    q->use_i2s = true;
    q->i2s_mgr = StepperQueue::_i2s_mux_manager;
    q->_i2s_mux_step_byte_offset = i2s_mux_byte_offset(slot);
    q->_i2s_mux_step_bit_mask = i2s_mux_bit_mask(slot);
    return q;
  }

  if (!StepperQueue::isValidStepPin(step_pin)) {
    return nullptr;
  }

#if defined(SUPPORT_ESP32_RMT)
  if (driver == DRIVER_RMT) {
    if (StepperQueue::_rmt_allocated >= QUEUES_RMT) {
      return nullptr;
    }
    StepperQueue* q = new StepperQueue();
    q->use_rmt = true;
    StepperQueue::_rmt_allocated++;
    return q;
  }
#endif
#if defined(SUPPORT_ESP32_I2S)
  if (driver == DRIVER_I2S_DIRECT) {
    I2sManager* mgr = I2sManager::create((gpio_num_t)step_pin, I2S_GPIO_UNUSED,
                                         I2S_GPIO_UNUSED);
    if (mgr != nullptr) {
      StepperQueue* q = new StepperQueue();
      q->use_i2s = true;
      q->i2s_mgr = mgr;
      return q;
    }
  }
#endif

  // Just use the next possible one
  if (driver != DRIVER_DONT_CARE) {
    return nullptr;
  }
  StepperQueue* q = tryAllocateQueue(DRIVER_RMT, step_pin);
  if (q == nullptr) {
    q = tryAllocateQueue(DRIVER_I2S_DIRECT, step_pin);
  }
  return q;
}
#endif  // SUPPORT_SELECT_DRIVER_TYPE && SUPPORT_DYNAMIC_ALLOCATION

#if !defined(SUPPORT_SELECT_DRIVER_TYPE) && defined(SUPPORT_DYNAMIC_ALLOCATION)
StepperQueue* StepperQueue::tryAllocateQueue(uint8_t step_pin) {
  if (StepperQueue::queues_allocated >= MAX_STEPPER) {
    return nullptr;
  }

  if (step_pin & PIN_EXTERNAL_FLAG) {
    return nullptr;
  }

  if (!StepperQueue::isValidStepPin(step_pin)) {
    return nullptr;
  }

  StepperQueue* q = new StepperQueue();
#if defined(SUPPORT_ESP32_RMT)
  q->use_rmt = true;
#endif
  StepperQueue::queues_allocated++;
  return q;
}
#endif  // !SUPPORT_SELECT_DRIVER_TYPE && SUPPORT_DYNAMIC_ALLOCATION

void StepperTask(void* parameter) {
  FastAccelStepperEngine* engine = (FastAccelStepperEngine*)parameter;
  while (true) {
    engine->manageSteppers();
#if ESP_IDF_VERSION_MAJOR == 4
    esp_task_wdt_reset();
#endif
    const TickType_t delay_time =
        (engine->_delay_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
    vTaskDelay(delay_time);
  }
}

void fas_init_engine(FastAccelStepperEngine* engine, uint8_t cpu_core) {
#if ESP_IDF_VERSION_MAJOR == 4
#define STACK_SIZE 2000
#define PRIORITY configMAX_PRIORITIES
#else
// I2S adds i2s_channel_write() to StepperTask; that IDF blocking call
// consumes ~3 KB of stack by itself.  Use 6 KB to be safe.
#define STACK_SIZE 6000
#define PRIORITY (configMAX_PRIORITIES - 1)
#endif
  engine->_delay_ms = DELAY_MS_BASE;
  if (cpu_core > 1) {
    xTaskCreate(StepperTask, "StepperTask", STACK_SIZE, engine, PRIORITY, NULL);
  } else {
    xTaskCreatePinnedToCore(StepperTask, "StepperTask", STACK_SIZE, engine,
                            PRIORITY, NULL, cpu_core);
  }
}

#endif
