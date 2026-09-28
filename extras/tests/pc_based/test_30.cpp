// test_30 - ESP32 RMT V2 fill: read-ahead window invariant.
//
// The encoder splits long steps into RMT symbols so that no PART_SIZE-symbol
// window (one RMT half) spans more than RMT_MAX_INFLIGHT_TICKS (= 1 ms). That
// is the whole read-ahead guarantee: the buffer cannot hold more time than the
// ramp lookahead, so the queue cannot be drained mid-move.
//
// The fill carries only the remaining low phase. The step high is
// min(ticks >> 1, RMT_MAX_SYMBOL_TICKS), i.e. just another sub-entry.

#include <stdint.h>
#include <stdio.h>

uint16_t debug_part_size = 32;

#include "FastAccelStepper.h"

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

static int dir_toggles = 0;
#define SUPPORT_ESP32_RMT_V2
#define IRAM_ATTR
#define LL_TOGGLE_PIN(dirPin) dir_toggles++
#include "pd_esp32/StepperISR_idf5_esp32_rmt_encode.cpp"

static int failures = 0;

static void expect(bool ok, const char* msg) {
  if (!ok) {
    printf("FAIL: %s\n", msg);
    failures++;
  }
}

static void decode(uint32_t word, uint16_t* dur0, bool* lvl0, uint16_t* dur1,
                   bool* lvl1) {
  *dur0 = (uint16_t)(word & 0x7fff);
  *lvl0 = (word & 0x8000) != 0;
  *dur1 = (uint16_t)((word >> 16) & 0x7fff);
  *lvl1 = (word & 0x80000000) != 0;
}

static uint32_t symbol_ticks(uint32_t word) {
  return (uint32_t)(word & 0x7fff) + (uint32_t)((word >> 16) & 0x7fff);
}

static void reset_queue() {
  fas_queue[0]._initVars();
  fas_queue[0].read_idx = 0;
  fas_queue[0].next_write_idx = 0;
  dir_toggles = 0;
}

static void push(uint8_t steps, uint16_t ticks, bool toggle) {
  uint8_t idx = fas_queue[0].next_write_idx & QUEUE_LEN_MASK;
  fas_queue[0].entry[idx].steps = steps;
  fas_queue[0].entry[idx].ticks = ticks;
  fas_queue[0].entry[idx].toggle_dir = toggle ? 1 : 0;
  fas_queue[0].entry[idx].countUp = 1;
  fas_queue[0].entry[idx].hasSteps = steps != 0;
  fas_queue[0].next_write_idx++;
}

// --- sliding PART_SIZE-symbol window ---------------------------------------
static uint32_t win[64];
static uint32_t win_pos, win_count, win_sum, win_max;

static void window_reset() { win_pos = win_count = win_sum = win_max = 0; }

static void window_add(uint32_t ticks) {
  if (win_count == (uint32_t)PART_SIZE) {
    win_sum -= win[win_pos];
  } else {
    win_count++;
  }
  win[win_pos] = ticks;
  win_sum += ticks;
  if (++win_pos == (uint32_t)PART_SIZE) {
    win_pos = 0;
  }
  if (win_count == (uint32_t)PART_SIZE && win_sum > win_max) {
    win_max = win_sum;
  }
}

// --- one command: fill until the queue and the fill state are drained ------
static uint64_t commanded_ticks, emitted_ticks;
static uint32_t emitted_edges, expected_edges;
static uint16_t min_subentry;
static bool last_level;

static void emit(const uint32_t* sym, uint32_t n) {
  for (uint32_t i = 0; i < n; i++) {
    uint16_t d0, d1;
    bool l0, l1;
    decode(sym[i], &d0, &l0, &d1, &l1);
    if (d0 < min_subentry) min_subentry = d0;
    if (d1 < min_subentry) min_subentry = d1;
    if (l0 && !last_level) emitted_edges++;
    last_level = l0;
    if (l1 && !last_level) emitted_edges++;
    last_level = l1;
    emitted_ticks += symbol_ticks(sym[i]);
    window_add(symbol_ticks(sym[i]));
  }
}

static void run_case(uint8_t steps, uint16_t ticks, bool toggle) {
  reset_queue();
  window_reset();
  commanded_ticks = (uint64_t)ticks * (steps ? steps : 1);
  emitted_ticks = 0;
  emitted_edges = 0;
  expected_edges = steps;
  min_subentry = 0xffff;
  last_level = false;

  push(steps, ticks, toggle);

  struct rmt_fill_state state = {};
  uint32_t sym[2 * 64];
  uint32_t guard = 0;
  while (fas_queue[0].read_idx != fas_queue[0].next_write_idx ||
         state.remaining_low_ticks != 0) {
    uint32_t n =
        rmt_encode_fill(&fas_queue[0], &state, sym, 2 * (uint32_t)PART_SIZE);
    if (n == 0) {
      break;
    }
    emit(sym, n);
    if (++guard > 100000) {
      break;
    }
  }

  expect(emitted_ticks == commanded_ticks, "symbol ticks == commanded ticks");
  expect(emitted_edges == expected_edges, "one rising edge per step");
  expect(state.remaining_low_ticks == 0, "fill state drained");
  expect(min_subentry >= 2, "every sub-entry >= 2 ticks (relation 1)");
  expect(win_max <= RMT_MAX_INFLIGHT_TICKS,
         "every PART_SIZE-symbol window <= RMT_MAX_INFLIGHT_TICKS");
}

static void run_suite(uint16_t part) {
  debug_part_size = part;
  printf("\n=== PART_SIZE %u (cap=%u, window limit=%u) ===\n", part,
         (uint32_t)RMT_MAX_SYMBOL_TICKS, (uint32_t)RMT_MAX_INFLIGHT_TICKS);

  static const uint16_t ticks[] = {4,    5,     8,     99,   250,
                                   251,  500,   640,   3200, 5000,
                                   7500, 10000, 20000, 32767, 65535};
  static const uint8_t steps[] = {1, 2, 5, 51, 255};
  for (uint32_t t = 0; t < sizeof(ticks) / sizeof(ticks[0]); t++) {
    for (uint32_t s = 0; s < sizeof(steps) / sizeof(steps[0]); s++) {
      run_case(steps[s], ticks[t], false);
    }
  }
  run_case(0, 3200, false);   // pause
  run_case(0, 65535, false);  // long pause
  run_case(1, 640, true);     // direction toggle
  run_case(255, 65535, true);
}

int main() {
  run_suite(32);
  run_suite(24);
  if (failures != 0) {
    printf("%d failure(s)\n", failures);
    return 1;
  }
  printf("PASS\n");
  return 0;
}
