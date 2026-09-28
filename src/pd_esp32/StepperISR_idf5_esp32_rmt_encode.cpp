#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_RMT_V2)

#if (PART_SIZE & 1) != 0
#error "PART_SIZE must be even"
#endif

// rmt_fill_state is defined in esp32_queue.h for the target build. The PC
// test compiles this file against pd_test, which does not include
// esp32_queue.h, so provide the definition only when it is missing.
#if !defined(RMT_FILL_STATE_DEFINED)
struct rmt_fill_state {
  uint16_t remaining_low_ticks;
};
#endif

#include "pd_esp32/test_probe.h"

static void IRAM_ATTR emit_pause_symbols(uint32_t* data, uint16_t ticks) {
  for (uint8_t i = 0; i < PART_SIZE - 1; i++) {
    *data++ = 0x00040004;
    ticks -= 8;
  }
  uint16_t ticks_l = (uint16_t)(ticks >> 1);
  uint16_t ticks_r = (uint16_t)(ticks - ticks_l);
  uint32_t last = ticks_l;
  last <<= 16;
  last |= ticks_r;
  *data = last;
}

static uint8_t IRAM_ATTR emit_step_symbols(uint32_t* data, uint16_t ticks,
                                           uint32_t symbols_free) {
  if (ticks == 0xffff) {
    if (symbols_free < 2) {
      return 0;
    }
    data[0] = 0x40007fff | 0x8000;
    data[1] = 0x20002000;
    return 2;
  }
  if (symbols_free < 1) {
    return 0;
  }
  uint16_t ticks_high = (uint16_t)(ticks >> 1);
  uint16_t ticks_low = (uint16_t)(ticks - ticks_high);
  uint32_t symbol = ticks_low;
  symbol <<= 16;
  symbol |= (uint16_t)(ticks_high | 0x8000);
  data[0] = symbol;
  return 1;
}

uint32_t IRAM_ATTR rmt_encode_queue(StepperQueue* q, uint32_t* symbols,
                                    uint32_t symbols_free) {
  uint32_t written = 0;
  bool emitted = false;
  while (q->read_idx != q->next_write_idx) {
    struct queue_entry* e = &q->entry[q->read_idx & QUEUE_LEN_MASK];
    if (emitted && e->toggle_dir) {
      break;
    }
    if (e->steps == 0) {
      if (symbols_free < (uint32_t)PART_SIZE) {
        break;
      }
      if (e->toggle_dir) {
        LL_TOGGLE_PIN(q->dirPin);
        e->toggle_dir = 0;
      }
      emit_pause_symbols(symbols + written, e->ticks);
      written += (uint32_t)PART_SIZE;
      symbols_free -= (uint32_t)PART_SIZE;
      q->read_idx = (uint8_t)(q->read_idx + 1);
      PROBE_4_TOGGLE;
      emitted = true;
      continue;
    }
    if (symbols_free < 2) {
      break;
    }
    if (e->toggle_dir) {
      LL_TOGGLE_PIN(q->dirPin);
      e->toggle_dir = 0;
    }
    uint8_t steps_left = e->steps;
    uint8_t steps_done = 0;
    while (steps_left > 0) {
      uint8_t n = emit_step_symbols(symbols + written, e->ticks, symbols_free);
      if (n == 0) {
        break;
      }
      written += n;
      symbols_free -= n;
      steps_left--;
      steps_done++;
    }
    if (steps_done == 0) {
      break;
    }
    emitted = true;
    e->steps = steps_left;
    if (steps_left == 0) {
      q->read_idx = (uint8_t)(q->read_idx + 1);
      PROBE_4_TOGGLE;
    } else {
      break;
    }
  }
  return written;
}

//==========================================================================
// RMT V2 fill encoder (F2 - read-ahead bound)
//
// Walks the queue and emits the low phase in pieces. Every RMT sub-entry
// (including the step high) is capped at RMT_MAX_SYMBOL_TICKS, so any
// PART_SIZE-symbol window spans at most 2*PART_SIZE*RMT_MAX_SYMBOL_TICKS =
// RMT_MAX_INFLIGHT_TICKS (1 ms).
//
// Only the low phase is carried in state; the capped high is emitted with the
// step-start symbol and needs no state. Every sub-entry is >= 2 ticks
// (relation 1 floor).
//==========================================================================

static void IRAM_ATTR emit_pair(uint32_t* out, bool first_high, uint16_t a,
                                uint16_t b) {
  uint32_t word = ((uint32_t)b << 16) | a;
  if (first_high) {
    word |= 0x8000u;
  }
  *out = word;
}

// Emit low-only symbols until `remaining` is drained or symbols_free is used.
// Each symbol holds two low sub-entries in [2, RMT_MAX_SYMBOL_TICKS]. The
// split is chosen so the remaining ticks are never 1..3, so every sub-entry
// stays legal across calls.
static uint32_t IRAM_ATTR emit_low_only(uint32_t* symbols, uint32_t symbols_free,
                                        uint16_t* remaining_inout) {
  const uint16_t cap = (uint16_t)RMT_MAX_SYMBOL_TICKS;
  uint32_t written = 0;
  uint16_t rem = *remaining_inout;
  while (rem > 0 && symbols_free > 0) {
    uint16_t amt;
    if (rem <= (uint16_t)(2 * cap)) {
      if (rem < 4) {
        break;
      }
      amt = rem;
    } else {
      amt = (uint16_t)(2 * cap);
      if ((uint16_t)(rem - amt) < 4) {
        amt = (uint16_t)(rem - 4);
      }
    }
    uint16_t a = (uint16_t)(amt / 2);
    uint16_t b = (uint16_t)(amt - a);
    emit_pair(symbols + written, false, a, b);
    written++;
    symbols_free--;
    rem = (uint16_t)(rem - amt);
  }
  *remaining_inout = rem;
  return written;
}

uint32_t IRAM_ATTR rmt_encode_fill(StepperQueue* q,
                                   struct rmt_fill_state* state,
                                   uint32_t* symbols, uint32_t symbols_free) {
  const uint16_t cap = (uint16_t)RMT_MAX_SYMBOL_TICKS;
  uint32_t written = 0;
  uint32_t free_sym = symbols_free;

  while (free_sym > 0) {
    // 1) Drain low left over from the previous entry (or call).
    if (state->remaining_low_ticks > 0) {
      uint32_t n =
          emit_low_only(symbols + written, free_sym, &state->remaining_low_ticks);
      written += n;
      free_sym -= n;
      if (state->remaining_low_ticks > 0) {
        break;
      }
      continue;
    }

    if (q->read_idx == q->next_write_idx) {
      break;
    }

    struct queue_entry* e = &q->entry[q->read_idx & QUEUE_LEN_MASK];
    if (e->toggle_dir) {
      LL_TOGGLE_PIN(q->dirPin);
      e->toggle_dir = 0;
    }

    if (e->steps == 0) {
      // Pause: all low.
      state->remaining_low_ticks = e->ticks;
    } else {
      // Step: high = min(ticks >> 1, cap), low = ticks - high.
      uint16_t high = (uint16_t)fas_min((uint16_t)(e->ticks >> 1), cap);
      uint16_t low = (uint16_t)(e->ticks - high);
      uint16_t first_low;
      if (low <= cap) {
        first_low = low;
      } else {
        first_low = cap;
        if ((uint16_t)(low - first_low) < 4) {
          first_low = (uint16_t)(low - 4);
        }
      }
      emit_pair(symbols + written, true, high, first_low);
      written++;
      free_sym--;
      state->remaining_low_ticks = (uint16_t)(low - first_low);
      e->steps--;
    }

    if (e->steps == 0) {
      q->read_idx = (uint8_t)(q->read_idx + 1);
      PROBE_4_TOGGLE;
    }
  }

  return written;
}

#endif
