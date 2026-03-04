#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_I2S)

#include "pd_esp32/i2s_manager.h"
#include "pd_esp32/i2s_fill.h"
#include "pd_esp32/i2s_constants.h"

bool StepperQueue::init_i2s(uint8_t channel_num, uint8_t step_pin) {
  _initVars();
  _step_pin = step_pin;
  _i2s_step_slot = 0;
  _fill_state = {};
  _isRunning = false;

  I2sManager& mgr = I2sManager::instance();
  if (!mgr.isInitialized()) {
    return false;
  }

  max_speed_in_ticks = I2S_MIN_SPEED_TICKS;
  return true;
}

void StepperQueue::startQueue_i2s() { _isRunning = true; }

void StepperQueue::forceStop_i2s() {
  _isRunning = false;
  _fill_state = {};
}

bool StepperQueue::isReadyForCommands_i2s() {
  if (_isRunning) {
    return !isQueueFull();
  }
  return true;
}

uint16_t StepperQueue::_getPerformedPulses_i2s() { return 0; }

void StepperQueue::fill_i2s_buffer(uint8_t* buf) {
  if (!use_i2s) {
    return;
  }
  if (!_isRunning) {
    return;
  }

  bool buffer_full = i2s_fill_buffer(this, buf, &_fill_state);

  if (!buffer_full) {
    _isRunning = false;
  }
}

#endif
