#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_I2S)
#include "pd_esp32/i2s_manager.h"
#endif

#if defined(SUPPORT_ESP32)
#include <Arduino.h>

#if defined(SUPPORT_SELECT_DRIVER_TYPE)
#if defined(SUPPORT_ESP32_I2S)
#define I2S_MODE_FREE 0
#define I2S_MODE_DIRECT 1
#define I2S_MODE_MUX 2
#endif

#ifdef SUPPORT_ESP32_MCPWM_PCNT
uint8_t StepperQueue::_mcpwm_pcnt_allocated = 0;
#endif
#ifdef SUPPORT_ESP32_RMT
uint8_t StepperQueue::_rmt_allocated = 0;
#endif
#if defined(SUPPORT_ESP32_I2S)
uint8_t StepperQueue::_i2s0_mode = I2S_MODE_FREE;
#if SOC_I2S_NUM >= 2
uint8_t StepperQueue::_i2s1_mode = I2S_MODE_FREE;
#endif
#if SOC_I2S_NUM >= 3
uint8_t StepperQueue::_i2s2_mode = I2S_MODE_FREE;
#endif
uint32_t StepperQueue::_i2s0_mux_allocated = 0;
#if SOC_I2S_NUM >= 2
uint32_t StepperQueue::_i2s1_mux_allocated = 0;
#endif
#if SOC_I2S_NUM >= 3
uint32_t StepperQueue::_i2s2_mux_allocated = 0;
#endif
#endif  // SUPPORT_ESP32_I2S
#endif  // SUPPORT_SELECT_DRIVER_TYPE

bool StepperQueue::init(FastAccelStepperEngine* engine, uint8_t queue_num,
                        uint8_t step_pin) {
  uint8_t channel = queue_num;
  max_speed_in_ticks = 80;
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  use_mcpwm_pcnt = false;
#endif
#ifdef SUPPORT_ESP32_RMT
  use_rmt = false;
#endif
#ifdef SUPPORT_ESP32_I2S
  use_i2s = false;
#endif
  Serial.printf("init: queue_num=%d step_pin=%d\n", queue_num, step_pin);
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  if (channel < QUEUES_MCPWM_PCNT) {
    return init_mcpwm_pcnt(channel, step_pin);
  }
  channel -= QUEUES_MCPWM_PCNT;
#endif
#ifdef SUPPORT_ESP32_RMT
  if (channel < QUEUES_RMT) {
    use_rmt = true;
    return init_rmt(channel, step_pin);
  }
  channel -= QUEUES_RMT;
#endif
#ifdef SUPPORT_ESP32_I2S
  if (channel < QUEUES_I2S) {
    use_i2s = true;
    return init_i2s(channel, step_pin);
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

#if defined(SUPPORT_SELECT_DRIVER_TYPE)

FasDriver StepperQueue::tryAllocateDriver(FasDriver driver) {
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  if (driver == FasDriver::MCPWM_PCNT) {
    if (_mcpwm_pcnt_allocated < QUEUES_MCPWM_PCNT) {
      return FasDriver::MCPWM_PCNT;
    }
  }
#endif

#ifdef SUPPORT_ESP32_RMT
  if (driver == FasDriver::RMT) {
    if (_rmt_allocated < QUEUES_RMT) {
      return FasDriver::RMT;
    }
  }
#endif

#if defined(SUPPORT_ESP32_I2S)
  if (driver == FasDriver::I2S_DIRECT) {
    if (_i2s0_mode == I2S_MODE_FREE) return FasDriver::I2S_DIRECT;
#if SOC_I2S_NUM >= 2
    if (_i2s1_mode == I2S_MODE_FREE) return FasDriver::I2S_DIRECT;
#endif
#if SOC_I2S_NUM >= 3
    if (_i2s2_mode == I2S_MODE_FREE) return FasDriver::I2S_DIRECT;
#endif
  }

  if (driver == FasDriver::I2S_MUX) {
    if (_i2s0_mode == I2S_MODE_MUX && _i2s0_mux_allocated != 0xFFFFFFFF) {
      return FasDriver::I2S_MUX;
    }
#if SOC_I2S_NUM >= 2
    if (_i2s1_mode == I2S_MODE_MUX && _i2s1_mux_allocated != 0xFFFFFFFF) {
      return FasDriver::I2S_MUX;
    }
#endif
#if SOC_I2S_NUM >= 3
    if (_i2s2_mode == I2S_MODE_MUX && _i2s2_mux_allocated != 0xFFFFFFFF) {
      return FasDriver::I2S_MUX;
    }
#endif
  }
#endif  // SUPPORT_ESP32_I2S

  return FasDriver::DONT_CARE;
}

FasDriver StepperQueue::selectAvailableDriverForPin(
    uint8_t step_pin, FasDriver preferred_driver) {
  if (!isValidStepPin(step_pin)) {
    return FasDriver::DONT_CARE;
  }

#if defined(SUPPORT_ESP32_I2S)
  if (step_pin & PIN_I2S0_FLAG) {
    return tryAllocateDriver(FasDriver::I2S_MUX);
  }
#endif

  if (preferred_driver != FasDriver::DONT_CARE) {
    FasDriver result = tryAllocateDriver(preferred_driver);
    if (result != FasDriver::DONT_CARE) {
      return result;
    }
    return FasDriver::DONT_CARE;
  }

#ifdef SUPPORT_ESP32_RMT
  FasDriver result = tryAllocateDriver(FasDriver::RMT);
  if (result != FasDriver::DONT_CARE) return result;
#endif

#if defined(SUPPORT_ESP32_I2S)
  result = tryAllocateDriver(FasDriver::I2S_DIRECT);
  if (result != FasDriver::DONT_CARE) return result;

  result = tryAllocateDriver(FasDriver::I2S_MUX);
  if (result != FasDriver::DONT_CARE) return result;
#endif

  return FasDriver::DONT_CARE;
}

#endif  // SUPPORT_SELECT_DRIVER_TYPE

void StepperTask(void* parameter) {
  FastAccelStepperEngine* engine = (FastAccelStepperEngine*)parameter;
  while (true) {
    engine->manageSteppers();
#if ESP_IDF_VERSION_MAJOR == 4
    esp_task_wdt_reset();
#endif
#ifdef SUPPORT_ESP32_I2S
    {
      I2sManager& mgr = I2sManager::instance();
      if (mgr.isInitialized() && !mgr.isDmaStarted()) {
        mgr.startDma();
      }
    }
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
