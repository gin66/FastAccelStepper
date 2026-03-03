#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_I2S)

#include <string.h>
#include "pd_esp32/i2s_manager.h"
#include "pd_esp32/i2s_fill.h"
#include "pd_esp32/i2s_constants.h"

bool StepperQueue::init_i2s(uint8_t channel_num, uint8_t step_pin) {
  _initVars();
  _step_pin = step_pin;
  _i2s_step_slot = 0;
  _write_block = 0;
  _fill_state = {};
  _isRunning = false;

  I2sManager& mgr = I2sManager::instance();
  if (!mgr.isInitialized()) {
    return false;
  }

  uint8_t pulse_width = mgr.pulseWidthBits();
  if (pulse_width == 0) {
    pulse_width = 32;
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

void StepperQueue::clear_i2s_block(uint8_t *buf, bool first) {
  if (!use_i2s) {
    return;
  }
  I2sManager& mgr = I2sManager::instance();
  uint8_t pulse_width = mgr.pulseWidthBits();
  if (pulse_width == 0) pulse_width = 32;
  i2s_clear_block(buf, &_fill_state, first ? 1 : 0, pulse_width);
}

void StepperQueue::fill_i2s_buffer(uint8_t *buf, bool first) {
  if (!use_i2s) {
    return;
  }
  if (!_isRunning) {
    return;
  }

  //if (_write_block == busy_block) {
    // we were overrun
  //  _write_block = (busy_block + 2) % I2S_BLOCK_COUNT;
  //  _fill_state.tick_pos = _write_block * I2S_BLOCK_TICKS;
  //}

  I2sManager& mgr = I2sManager::instance();
  uint8_t pulse_width = mgr.pulseWidthBits();
  if (pulse_width == 0) pulse_width = 32;


  bool buffer_full =
        i2s_fill_buffer(this, buf, first ? 1 : 0, &_fill_state, pulse_width);

  // This is problematic
  if (isQueueEmpty()) {
    if (_isRunning) {
      _isRunning = false;
    }
  }
}

#endif
