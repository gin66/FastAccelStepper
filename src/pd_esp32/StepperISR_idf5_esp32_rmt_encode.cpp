#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_RMT_V2)

#if (PART_SIZE & 1) != 0
#error "PART_SIZE must be even"
#endif

#include "pd_esp32/test_probe.h"

static void IRAM_ATTR emit_pause_symbols(uint32_t* data, uint16_t ticks) {
  for (uint8_t i = 0; i < PART_SIZE - 1; i++) {
    *data++ = 0x00040004;
    ticks = (uint16_t)(ticks - 8);
  }
  uint16_t ticks_l = (uint16_t)(ticks >> 1);
  uint16_t ticks_r = (uint16_t)(ticks - ticks_l);
  uint32_t last = ticks_l;
  last <<= 16;
  last |= ticks_r;
  *data = last;
}

static void IRAM_ATTR emit_step_symbols(uint32_t* data, uint16_t ticks) {
  if (ticks == 0xffff) {
    data[0] = 0x40007fff | 0x8000;
    data[1] = 0x20002000;
    return;
  }
  uint16_t ticks_high = (uint16_t)(ticks >> 1);
  uint16_t ticks_low = (uint16_t)(ticks - ticks_high);
  uint32_t symbol = ticks_low;
  symbol <<= 16;
  symbol |= (uint16_t)(ticks_high | 0x8000);
  data[0] = symbol;
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
    uint8_t per = (e->ticks == 0xffff) ? 2 : 1;
    uint8_t steps_left = e->steps;
    uint8_t steps_done = 0;
    while (steps_left > 0 && symbols_free >= per) {
      emit_step_symbols(symbols + written, e->ticks);
      written += per;
      symbols_free -= per;
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

#endif
