#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_I2S)

#include "pd_esp32/i2s_fill.h"

bool IRAM_ATTR i2s_fill_buffer(StepperQueueBase* q, uint8_t* buf,
                               struct i2s_fill_state* state) {
#ifdef DEBUG
  printf(
      "i2s_fill_buffer called remaining_high_ticks=%u "
      "remaining_low_ticks=%u\n",
      state->remaining_high_ticks, state->remaining_low_ticks);
#endif

  static const uint8_t pulse_width_bits = I2S_DEFAULT_PULSE_WIDTH_TICKS / 2;
  static const uint8_t ticks_per_bit = I2S_TICKS_PER_FRAME / I2S_BITS_PER_FRAME;
  static const uint32_t block_end = I2S_BLOCK_TICKS - 1;

  uint32_t tick_pos = 0;

  uint8_t rp = q->read_idx;

  do {
    while (state->remaining_high_ticks > 0) {
      uint16_t bit_pos = tick_pos / ticks_per_bit;

#ifdef DEBUG
      printf("  Writing pulse at tick_pos=%u remaining_ticks=%u bit_pos=%u\n",
             tick_pos, state->remaining_high_ticks, bit_pos);
#endif
      buf[bit_pos >> 3] |= (0x80 >> (bit_pos & 7));
      if (state->remaining_high_ticks >= ticks_per_bit) {
        state->remaining_high_ticks -= ticks_per_bit;
      } else {
        state->remaining_low_ticks -=
            (ticks_per_bit - state->remaining_high_ticks);
        state->remaining_high_ticks = 0;
      }

      tick_pos += ticks_per_bit;
      if (tick_pos > block_end) {
        q->read_idx = rp;
        return true;
      }
    }
    if (state->remaining_low_ticks > 0) {
      uint16_t ticks_in_block = block_end - tick_pos + 1;
#ifdef DEBUG
      printf(
          "  Advancing through low ticks at tick_pos=%u "
          "remaining_low_ticks=%u ticks_in_block=%u\n",
          tick_pos, state->remaining_low_ticks, ticks_in_block);
#endif
      if (state->remaining_low_ticks <= ticks_in_block) {
        tick_pos += state->remaining_low_ticks;
        state->remaining_low_ticks = 0;
      } else {
        tick_pos += ticks_in_block;
        state->remaining_low_ticks -= ticks_in_block;
      }
      if (tick_pos > block_end) {
        q->read_idx = rp;
        return true;
      }
    }

    if (rp == q->next_write_idx) {
      q->read_idx = rp;
      return false;
    }

    struct queue_entry* e = &q->entry[rp & QUEUE_LEN_MASK];
    if (e->toggle_dir) {
      LL_TOGGLE_PIN(q->dirPin);
      e->toggle_dir = 0;
    }
    if (e->steps == 0) {
      state->remaining_low_ticks = e->ticks;
      rp++;
    } else {
      state->remaining_low_ticks = e->ticks - pulse_width_bits;
      state->remaining_high_ticks = pulse_width_bits;
      e->steps--;
      if (e->steps == 0) {
        rp++;
      }
    }
  } while (true);
}

#endif
