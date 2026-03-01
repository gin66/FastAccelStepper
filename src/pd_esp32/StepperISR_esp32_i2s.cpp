#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_I2S)

#include <string.h>
#include "pd_esp32/i2s_manager.h"

#define I2S_DEFAULT_BCLK_GPIO 33
#define I2S_DEFAULT_WS_GPIO I2S_GPIO_UNUSED
#define I2S_BLOCK_TICKS ((uint32_t)I2S_FRAMES_PER_BLOCK * I2S_TICKS_PER_FRAME)

bool StepperQueue::init_i2s(uint8_t channel_num, uint8_t step_pin) {
  _initVars();
  _step_pin = step_pin;
  _i2s_step_slot = 0;
  _i2s_tick_carry = 0;
  _i2s_tick_pos = 0;
  _i2s_drain = 0;
  _i2s_pulse_count = 0;
  _isRunning = false;

  I2sManager& mgr = I2sManager::instance();
  if (!mgr.isInitialized()) {
    if (!mgr.init((int)step_pin, I2S_DEFAULT_BCLK_GPIO,
                  (int)I2S_DEFAULT_WS_GPIO)) {
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
  _i2s_tick_carry = 0;
  _i2s_tick_pos = 0;
  _i2s_pulse_count = 0;
}

bool StepperQueue::isReadyForCommands_i2s() {
  if (_isRunning) {
    return !isQueueFull();
  }
  return true;
}

uint16_t StepperQueue::_getPerformedPulses_i2s() { return 0; }

void StepperQueue::fill_i2s_buffer() {
  if (!use_i2s) {
    return;
  }
  if (!_isRunning && (_i2s_drain == 0)) {
    return;
  }

  I2sManager& mgr = I2sManager::instance();
  uint8_t blk = mgr.writeBlock();
  uint8_t* buf = mgr.blockBuf(blk);
  mgr.clearBlock(blk);

  uint32_t tick_pos = _i2s_tick_pos;
  uint32_t tick_carry = _i2s_tick_carry;
  _i2s_pulse_count = 0;
  _i2s_tick_pos = 0;

  uint8_t rp = read_idx;

  while (tick_pos < I2S_BLOCK_TICKS &&
         _i2s_pulse_count < I2S_MAX_PULSES_PER_BLOCK) {
    if (rp == next_write_idx) {
      if (_isRunning) {
        _isRunning = false;
        _i2s_drain = I2S_DRAIN_TASKS;
      }
      break;
    }

    struct queue_entry* e = &entry[rp & QUEUE_LEN_MASK];

    if (e->toggle_dir) {
      LL_TOGGLE_PIN(dirPin);
      e->toggle_dir = 0;
    }

    if (e->steps == 0) {
      uint32_t total = (uint32_t)e->ticks + tick_carry;
      tick_pos += total;
      tick_carry = 0;
      rp++;
    } else {
      uint8_t steps = e->steps;
      const uint16_t ticks = e->ticks;

      uint8_t s = 0;
      while (s < steps) {
        uint32_t new_tick_pos = tick_pos + tick_carry + ticks;

        if (new_tick_pos >= I2S_BLOCK_TICKS) {
          if (s == 0) {
            _i2s_tick_pos = new_tick_pos - I2S_BLOCK_TICKS;
            _i2s_tick_carry = 0;
          } else {
            e->steps -= s;
            _i2s_tick_pos = new_tick_pos - I2S_BLOCK_TICKS;
            _i2s_tick_carry = 0;
          }
          tick_carry = 0;
          goto done;
        }

        tick_carry = 0;
        tick_pos = new_tick_pos;

        uint16_t byte_idx = (uint16_t)(tick_pos / I2S_TICKS_PER_FRAME);
        buf[byte_idx * I2S_BYTES_PER_FRAME] = 0xFF;
        buf[byte_idx * I2S_BYTES_PER_FRAME + 1] = 0xFF;

        if (_i2s_pulse_count < I2S_MAX_PULSES_PER_BLOCK) {
          _i2s_pulse_positions[_i2s_pulse_count] = byte_idx;
          _i2s_pulse_count++;
        }
        s++;
      }

      if (s == steps) {
        rp++;
      }
    }
  }

done:
  _i2s_tick_carry = tick_carry;
  read_idx = rp;

  mgr.flushBlock(blk);
  mgr.markWriteBlockPrepared();

  if (_i2s_drain > 0) {
    _i2s_drain--;
  }
}

#endif  // SUPPORT_ESP32_I2S
