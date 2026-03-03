#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_I2S)

#include "pd_esp32/i2s_fill.h"

bool IRAM_ATTR i2s_fill_buffer(StepperQueueBase* q, uint8_t* buf, uint8_t block,
                               struct i2s_fill_state* state) {
  uint8_t blk = block % I2S_BLOCK_COUNT;
  uint32_t tick_pos = state->tick_pos;
  const uint32_t block_end = I2S_BLOCK_TICKS;

  for (uint8_t i = 0; i < state->pulse_count[blk]; i++) {
    uint16_t frame = state->pulse_positions[blk][i];
    if (frame < I2S_FRAMES_PER_BLOCK) {
      buf[frame * I2S_BYTES_PER_FRAME + 0] = 0x00;
      buf[frame * I2S_BYTES_PER_FRAME + 1] = 0x00;
      buf[frame * I2S_BYTES_PER_FRAME + 2] = 0x00;
      buf[frame * I2S_BYTES_PER_FRAME + 3] = 0x00;
    }
  }
  state->pulse_count[blk] = 0;

  uint8_t rp = q->read_idx;

  if (state->remaining_delay_ticks > 0) {
    uint32_t delay_end = tick_pos + state->remaining_delay_ticks;
    if (delay_end >= block_end) {
      state->remaining_delay_ticks = delay_end - block_end;
      state->tick_pos = 0;
      q->read_idx = rp;
      return true;
    }
    tick_pos = delay_end;
    state->remaining_delay_ticks = 0;
  }

  if (state->pending_pulse) {
    state->pending_pulse = false;
    uint16_t frame = tick_pos / I2S_TICKS_PER_FRAME;
    if (frame < I2S_FRAMES_PER_BLOCK) {
      buf[frame * I2S_BYTES_PER_FRAME + 0] = 0xFF;
      buf[frame * I2S_BYTES_PER_FRAME + 1] = 0xFF;
      buf[frame * I2S_BYTES_PER_FRAME + 2] = 0xFF;
      buf[frame * I2S_BYTES_PER_FRAME + 3] = 0xFF;
      state->pulse_positions[blk][state->pulse_count[blk]++] = frame;
    } else {
      state->pending_pulse = true;
    }
    if (frame < I2S_FRAMES_PER_BLOCK) {
      if (rp < q->next_write_idx) {
        struct queue_entry* e = &q->entry[rp & QUEUE_LEN_MASK];
        if (e->steps > 0) {
          e->steps--;
          if (e->steps == 0) {
            rp++;
          }
        }
      }
    }
  }

  while (tick_pos < block_end &&
         state->pulse_count[blk] < I2S_MAX_PULSES_PER_BLOCK) {
    if (rp == q->next_write_idx) {
      break;
    }

    struct queue_entry* e = &q->entry[rp & QUEUE_LEN_MASK];

    if (e->toggle_dir) {
      LL_TOGGLE_PIN(q->dirPin);
      e->toggle_dir = 0;
    }

    if (e->steps == 0) {
      uint32_t total = e->ticks;
      if (tick_pos + total >= block_end) {
        state->remaining_delay_ticks = tick_pos + total - block_end;
        state->pending_pulse = false;
        state->tick_pos = 0;
        q->read_idx = rp + 1;
        return true;
      }
      tick_pos += total;
      rp++;
    } else {
      uint8_t steps = e->steps;
      const uint16_t ticks = e->ticks;

      uint8_t s = 0;
      while (s < steps) {
        uint32_t new_tick_pos = tick_pos + ticks;

        if (new_tick_pos >= block_end) {
          e->steps -= s;
          state->remaining_delay_ticks = new_tick_pos - block_end;
          state->pending_pulse = true;
          state->tick_pos = 0;
          q->read_idx = rp;
          return true;
        }

        tick_pos = new_tick_pos;

        uint16_t frame = tick_pos / I2S_TICKS_PER_FRAME;
        if (frame < I2S_FRAMES_PER_BLOCK) {
          buf[frame * I2S_BYTES_PER_FRAME + 0] = 0xFF;
          buf[frame * I2S_BYTES_PER_FRAME + 1] = 0xFF;
          buf[frame * I2S_BYTES_PER_FRAME + 2] = 0xFF;
          buf[frame * I2S_BYTES_PER_FRAME + 3] = 0xFF;
          state->pulse_positions[blk][state->pulse_count[blk]++] = frame;
        }
        s++;
      }

      if (s == steps) {
        rp++;
      }
    }
  }

  state->tick_pos = tick_pos;
  q->read_idx = rp;
  return (tick_pos >= block_end);
}

#endif
