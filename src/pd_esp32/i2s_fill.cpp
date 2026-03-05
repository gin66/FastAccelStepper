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

  static const uint8_t ticks_per_bit = I2S_TICKS_PER_FRAME / I2S_BITS_PER_FRAME;

  struct i2s_fill_state local_state = *state;
  uint8_t rp = q->read_idx;

  uint16_t bit_pos = 0;
  do {
    while (local_state.remaining_high_ticks > 0) {
#ifdef DEBUG
      printf("  Writing pulse at remaining_ticks=%u bit_pos=%u\n",
             local_state.remaining_high_ticks, bit_pos);
#endif
      // Byte order in I2S buffer is little-endian, but bits are sent MSB first,
      // so we need to XOR the byte index with 1.
      uint16_t byte_index = (bit_pos >> 3)^1;
      // always fill the full byte
      buf[byte_index] = 0xff >> (bit_pos & 7);
      uint8_t bits = 8 - (bit_pos & 7);
      uint8_t ticks = bits * ticks_per_bit;
      if (local_state.remaining_high_ticks >= ticks) {
        local_state.remaining_high_ticks -= ticks;
      } else {
        local_state.remaining_low_ticks -= 
            (ticks - local_state.remaining_high_ticks);
        local_state.remaining_high_ticks = 0;
      }

      bit_pos += bits;
      if (bit_pos >= I2S_BYTES_PER_BLOCK*8) {
        *state = local_state;
        return true;
      }
    }
    if (local_state.remaining_low_ticks > 0) {
      uint16_t ticks_in_block = I2S_BLOCK_TICKS - bit_pos * ticks_per_bit;
#ifdef DEBUG
      printf(
          "  Advancing through low ticks "
          "remaining_low_ticks=%u ticks_in_block=%u\n",
          local_state.remaining_low_ticks, ticks_in_block);
#endif
      if (local_state.remaining_low_ticks < ticks_in_block) {
        bit_pos += local_state.remaining_low_ticks / ticks_per_bit;
        local_state.off_ticks = local_state.remaining_low_ticks % ticks_per_bit;
        local_state.remaining_low_ticks = 0;
      } else {
        local_state.remaining_low_ticks -= ticks_in_block;
        *state = local_state;
        return true;
      }
    }

    if (rp == q->next_write_idx) {
      *state = local_state;
      return false;
    }

    struct queue_entry* e = &q->entry[rp & QUEUE_LEN_MASK];
    if (e->toggle_dir) {
      LL_TOGGLE_PIN(q->dirPin);
      e->toggle_dir = 0;
    }
    uint8_t steps = e->steps;
    local_state.remaining_low_ticks = e->ticks;
    if (local_state.remaining_low_ticks < 65535-ticks_per_bit) {
      local_state.remaining_low_ticks += local_state.off_ticks;
      local_state.off_ticks = 0;
    }
    if (steps > 0) {
      local_state.remaining_low_ticks -= I2S_DEFAULT_PULSE_WIDTH_TICKS;
      local_state.remaining_high_ticks = I2S_DEFAULT_PULSE_WIDTH_TICKS;
      steps--;
      e->steps = steps;
    }
    if (steps == 0) {
      rp++;
      q->read_idx = rp;
    }
  } while (true);
}

#endif
