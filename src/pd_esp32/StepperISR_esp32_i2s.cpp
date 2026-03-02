#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_I2S)

#include <string.h>
#include "pd_esp32/i2s_manager.h"
#include "pd_esp32/i2s_fill.h"
#include "pd_esp32/i2s_constants.h"

#define I2S_DEFAULT_BCLK_GPIO 33
#define I2S_DEFAULT_WS_GPIO -1
#define I2S_DEFAULT_DATA_GPIO 32

bool StepperQueue::init_i2s(uint8_t channel_num, uint8_t step_pin) {
  _initVars();
  _step_pin = step_pin;
  _i2s_step_slot = 0;
  _i2s_drain = 0;
  _write_block = 0;
  _fill_state = {};
  _isRunning = false;

  I2sManager& mgr = I2sManager::instance();
  if (!mgr.isInitialized()) {
    if (!mgr.init(I2S_DEFAULT_DATA_GPIO, I2S_DEFAULT_BCLK_GPIO,
                  I2S_DEFAULT_WS_GPIO)) {
      return false;
    }
  }

  max_speed_in_ticks = I2S_MIN_SPEED_TICKS;
  return true;
}

void StepperQueue::startQueue_i2s() { _isRunning = true; }

void StepperQueue::forceStop_i2s() {
  _isRunning = false;
  _i2s_drain = 0;
  _fill_state = {};
}

bool StepperQueue::isReadyForCommands_i2s() {
  if (_isRunning) {
    return !isQueueFull();
  }
  return true;
}

uint16_t StepperQueue::_getPerformedPulses_i2s() { return 0; }

void StepperQueue::fill_i2s_buffer(uint8_t busy_block) {
  if (!use_i2s) {
    return;
  }
  if (!_isRunning && (_i2s_drain == 0)) {
    return;
  }

  if (_write_block == busy_block) {
    _write_block = (busy_block + 1) % I2S_BLOCK_COUNT;
    _fill_state.tick_pos = 0;
  }

  I2sManager& mgr = I2sManager::instance();

  for (int i = 0; i < I2S_BLOCK_COUNT - 1; i++) {
    if (_write_block == busy_block) {
      _write_block = (_write_block + 1) % I2S_BLOCK_COUNT;
    }

    uint8_t* buf = mgr.blockBuf(_write_block);
    bool buffer_full = i2s_fill_buffer(this, buf, &_fill_state);

    if (buffer_full) {
      _write_block = (_write_block + 1) % I2S_BLOCK_COUNT;
    } else {
      break;
    }
  }

  if (isQueueEmpty()) {
    if (_isRunning) {
      _isRunning = false;
      _i2s_drain = I2S_DRAIN_TASKS;
    }
  }

  if (_i2s_drain > 0) {
    _i2s_drain--;
  }
}

#endif
