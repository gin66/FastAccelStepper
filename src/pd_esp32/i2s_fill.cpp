#if defined(SUPPORT_ESP32_I2S)

#include "pd_esp32/i2s_fill.h"

#define I2S_BLOCK_TICKS ((uint16_t)(I2S_FRAMES_PER_BLOCK * I2S_TICKS_PER_FRAME))

void IRAM_ATTR i2s_fill_buffer(struct i2s_stepper_queue* q, uint8_t* buf,
                               struct i2s_fill_state* state) {
  uint16_t tick_pos = state->tick_pos;
  uint16_t tick_carry = state->tick_carry;
  state->pulse_count = 0;

  uint8_t rp = q->read_idx;

  while (tick_pos < I2S_BLOCK_TICKS &&
         state->pulse_count < I2S_MAX_PULSES_PER_BLOCK) {
    if (rp == q->next_write_idx) {
      break;
    }

    struct i2s_queue_entry* e = &q->entry[rp & QUEUE_LEN_MASK];

    if (e->toggle_dir) {
      LL_TOGGLE_PIN(q->dirPin);
      e->toggle_dir = 0;
    }

    if (e->steps == 0) {
      uint16_t total = (uint16_t)e->ticks + tick_carry;
      tick_pos += total;
      tick_carry = 0;
      rp++;
    } else {
      uint8_t steps = e->steps;
      const uint16_t ticks = e->ticks;

      uint8_t s = 0;
      while (s < steps) {
        uint16_t new_tick_pos = tick_pos + tick_carry + ticks;

        if (new_tick_pos >= I2S_BLOCK_TICKS) {
          if (s == 0) {
            state->tick_pos = new_tick_pos - I2S_BLOCK_TICKS;
            state->tick_carry = 0;
          } else {
            e->steps -= s;
            state->tick_pos = new_tick_pos - I2S_BLOCK_TICKS;
            state->tick_carry = 0;
          }
          tick_carry = 0;
          q->read_idx = rp;
          return;
        }

        tick_carry = 0;
        tick_pos = new_tick_pos;

        uint16_t byte_idx = (uint16_t)(tick_pos / I2S_TICKS_PER_FRAME);
        buf[byte_idx * I2S_BYTES_PER_FRAME] = 0xFF;
        buf[byte_idx * I2S_BYTES_PER_FRAME + 1] = 0xFF;

        if (state->pulse_count < I2S_MAX_PULSES_PER_BLOCK) {
          state->pulse_positions[state->pulse_count] = byte_idx;
          state->pulse_count++;
        }
        s++;
      }

      if (s == steps) {
        rp++;
      }
    }
  }

  state->tick_pos = 0;
  state->tick_carry = tick_carry;
  q->read_idx = rp;
}

#endif
