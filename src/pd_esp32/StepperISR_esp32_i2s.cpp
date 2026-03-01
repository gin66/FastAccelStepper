#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_I2S)

#include <string.h>
#include <esp_task_wdt.h>
#include "pd_esp32/i2s_manager.h"

// Default GPIO assignments for the I2S clock signals.
// DATA_OUT (step pin) is taken from the stepper's step_pin argument.
#define I2S_DEFAULT_BCLK_GPIO 33
#define I2S_DEFAULT_WS_GPIO I2S_GPIO_UNUSED

bool StepperQueue::init_i2s(uint8_t channel_num, uint8_t step_pin) {
  _initVars();
  _step_pin = step_pin;
  _i2s_step_slot = 0;
  _i2s_tick_carry = 0;
  _i2s_drain = 0;
  _i2s_pulse_write_idx = 0;
  _i2s_pulse_read_idx = 0;
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
  _i2s_pulse_read_idx = 0;
  _i2s_pulse_write_idx = 0;
}

bool StepperQueue::isReadyForCommands_i2s() {
  if (_isRunning) {
    return !isQueueFull();
  }
  return true;
}

uint16_t StepperQueue::_getPerformedPulses_i2s() { return 0; }

// Fill the I2S work buffer with step pulses for this stepper.
// Called from StepperTask() after clearWorkBuf() and before flush().
void StepperQueue::fill_i2s_buffer() {
  if (!use_i2s) {
    return;
  }
  if (!_isRunning && (_i2s_drain == 0)) {
    return;
  }

  uint8_t* buf = I2sManager::instance().workBuf();
  uint32_t frame_pos = 0;
  uint32_t tick_carry = _i2s_tick_carry;

  uint8_t rp = read_idx;
  const uint8_t wp = next_write_idx;

  // Clear old pulses: from read_idx up to write_idx (wrapped)
  while (_i2s_pulse_read_idx != _i2s_pulse_write_idx) {
    uint16_t frame_idx = _i2s_pulse_positions[_i2s_pulse_read_idx];
    // Clear both L and R bytes for 2µs pulse
    buf[frame_idx * I2S_BYTES_PER_FRAME] = 0x00;
    buf[frame_idx * I2S_BYTES_PER_FRAME + 1] = 0x00;
    _i2s_pulse_read_idx = (_i2s_pulse_read_idx + 1) % I2S_MAX_PULSES;
  }

  while (frame_pos < I2S_FRAMES_PER_TASK) {
    if (rp == wp) {
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
      uint32_t frames = total / I2S_TICKS_PER_FRAME;
      tick_carry = total % I2S_TICKS_PER_FRAME;
      frame_pos += frames;
      rp++;
    } else {
      uint8_t steps = e->steps;
      const uint16_t ticks = e->ticks;
      uint8_t s = 0;

      while (s < steps) {
        uint32_t total = (uint32_t)ticks + tick_carry;
        uint32_t frames = total / I2S_TICKS_PER_FRAME;
        tick_carry = total % I2S_TICKS_PER_FRAME;
        frame_pos += frames;

        if (frame_pos >= I2S_FRAMES_PER_TASK) {
          break;
        }

        buf[frame_pos * I2S_BYTES_PER_FRAME] = 0xFF;
        buf[frame_pos * I2S_BYTES_PER_FRAME + 1] = 0xFF;

        _i2s_pulse_positions[_i2s_pulse_write_idx] = (uint16_t)frame_pos;
        _i2s_pulse_write_idx = (_i2s_pulse_write_idx + 1) % I2S_MAX_PULSES;
        s++;
      }

      if (s == steps) {
        rp++;
      } else {
        e->steps -= s;
        break;
      }
    }
  }

  _i2s_tick_carry = tick_carry;
  read_idx = rp;

  if (_i2s_drain > 0) {
    _i2s_drain--;
  }
}

#endif  // SUPPORT_ESP32_I2S
