#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_I2S)

#include "pd_esp32/i2s_fill.h"

void IRAM_ATTR i2s_clear_block(uint8_t* buf, struct i2s_fill_state* state,
                               uint8_t block, uint8_t pulse_width_bits) {
  uint8_t blk = block % I2S_BLOCK_COUNT;
  if (pulse_width_bits == 0) pulse_width_bits = 32;

  uint8_t pulses = state->pulse_count[blk];
  for (uint8_t i = 0; i < pulses; i++) {
    uint16_t bit_pos = state->pulse_positions[blk][i];
    while (true) {
      uint16_t byte_idx = bit_pos / 8;
      if (byte_idx >= I2S_BYTES_PER_BLOCK) {
        break;
      }
      uint8_t bit_mask = 1 << (7 - (bit_pos % 8));
      if (buf[byte_idx] & bit_mask) {
        buf[byte_idx] ^= bit_mask;
        bit_pos++;
      } else {
        break;
      }
    }
  }
  state->pulse_count[blk] = 0;
}

bool IRAM_ATTR i2s_fill_buffer(StepperQueueBase* q, uint8_t* buf, uint8_t blk,
                               struct i2s_fill_state* state,
                               uint8_t pulse_width_bits) {
  if (blk >= I2S_BLOCK_COUNT) {
#ifdef DEBUG
    puts("Invalid block number");
#endif
    abort();
    return false;
  }

#ifdef DEBUG
  printf(
      "i2s_fill_buffer called (block %u) remaining_high_ticks=%u "
      "remaining_low_ticks=%u\n",
      blk, state->remaining_high_ticks, state->remaining_low_ticks);
#endif
  uint32_t tick_pos = state->tick_pos;
  uint8_t tick_block = tick_pos / I2S_BLOCK_TICKS;
  if (tick_block != blk) {
    return true;
  }

  const uint32_t block_end = I2S_BLOCK_TICKS * (blk + 1) - 1;
  if (pulse_width_bits == 0) pulse_width_bits = 32;

  uint8_t rp = q->read_idx;

  do {
    bool new_pulse = true;
    while (state->remaining_high_ticks > 0) {
      // Write high bits for the current pulse
      uint16_t bit_pos = tick_pos % I2S_BLOCK_TICKS;
      uint8_t ticks_per_bit = I2S_TICKS_PER_FRAME / I2S_BITS_PER_FRAME;
      bit_pos /= ticks_per_bit;

#ifdef DEBUG
      printf(
          "  Writing pulse at tick_pos=%u (block %u) remaining_ticks=%u "
          "bit_pos=%u\n",
          tick_pos, blk, state->remaining_high_ticks, bit_pos);
#endif
      buf[bit_pos / 8] |= (1 << (7 - (bit_pos % 8)));
      if (state->remaining_high_ticks >= ticks_per_bit) {
        state->remaining_high_ticks -= ticks_per_bit;
      } else {
        state->remaining_low_ticks -=
            (ticks_per_bit - state->remaining_high_ticks);
        state->remaining_high_ticks = 0;
      }

      if (new_pulse) {
        state->pulse_positions[blk][state->pulse_count[blk]++] = bit_pos;
        new_pulse = false;
      }
      tick_pos += ticks_per_bit;
      if (tick_pos >= block_end) {
        state->tick_pos = tick_pos % (I2S_BLOCK_TICKS * I2S_BLOCK_COUNT);
        q->read_idx = rp;
        return true;
      }
    }

    if (state->remaining_low_ticks > 0) {
      uint16_t ticks_in_block = block_end - tick_pos + 1;
#ifdef DEBUG
      printf(
          "  Advancing through low ticks at tick_pos=%u (block %u) "
          "remaining_low_ticks=%u ticks_in_block=%u\n",
          tick_pos, blk, state->remaining_low_ticks, ticks_in_block);
#endif
      if (state->remaining_low_ticks <= ticks_in_block) {
        tick_pos += state->remaining_low_ticks;
        state->remaining_low_ticks = 0;
      } else {
        tick_pos += ticks_in_block;
        state->remaining_low_ticks -= ticks_in_block;
      }
      if (tick_pos >= block_end) {
        q->read_idx = rp;
        state->tick_pos = tick_pos % (I2S_BLOCK_TICKS * I2S_BLOCK_COUNT);
        return true;
      }
    }

    // no remaining ticks
    if (rp == q->next_write_idx) {
      state->tick_pos = tick_pos % (I2S_BLOCK_TICKS * I2S_BLOCK_COUNT);
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
