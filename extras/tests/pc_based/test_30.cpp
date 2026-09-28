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

static void decode(uint32_t word, bool* level0, uint16_t* dur0, bool* level1,
                   uint16_t* dur1) {
  uint16_t low = (uint16_t)(word & 0xffff);
  uint16_t high = (uint16_t)(word >> 16);
  *level0 = (low & 0x8000) != 0;
  *dur0 = (uint16_t)(low & 0x7fff);
  *level1 = (high & 0x8000) != 0;
  *dur1 = (uint16_t)(high & 0x7fff);
}

static uint32_t symbol_ticks(uint32_t word) {
  bool l0, l1;
  uint16_t d0, d1;
  decode(word, &l0, &d0, &l1, &d1);
  return (uint32_t)d0 + d1;
}

static bool any_zero_duration(const uint32_t* words, uint32_t count) {
  for (uint32_t i = 0; i < count; i++) {
    bool l0, l1;
    uint16_t d0, d1;
    decode(words[i], &l0, &d0, &l1, &d1);
    if (d0 == 0 || d1 == 0) {
      return true;
    }
  }
  return false;
}

static bool all_low(const uint32_t* words, uint32_t count) {
  for (uint32_t i = 0; i < count; i++) {
    bool l0, l1;
    uint16_t d0, d1;
    decode(words[i], &l0, &d0, &l1, &d1);
    if (l0 || l1) {
      return false;
    }
  }
  return true;
}

static uint32_t rising_edges(const uint32_t* words, uint32_t count) {
  bool high = false;
  uint32_t edges = 0;
  for (uint32_t i = 0; i < count; i++) {
    bool l0, l1;
    uint16_t d0, d1;
    decode(words[i], &l0, &d0, &l1, &d1);
    if (l0 && !high) {
      edges++;
    }
    high = l0;
    if (l1 && !high) {
      edges++;
    }
    high = l1;
  }
  return edges;
}

static uint32_t total_ticks(const uint32_t* words, uint32_t count) {
  uint32_t sum = 0;
  for (uint32_t i = 0; i < count; i++) {
    sum += symbol_ticks(words[i]);
  }
  return sum;
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

static void test_pause_fills_one_half() {
  printf("pause is PART_SIZE symbols\n");
  reset_queue();
  const uint16_t ticks = 3200;
  push(0, ticks, false);
  uint32_t sym[64];
  uint32_t n = rmt_encode_queue(&fas_queue[0], sym, 64);
  expect(n == (uint32_t)PART_SIZE, "pause width");
  expect(all_low(sym, n), "pause stays low");
  expect(!any_zero_duration(sym, n), "pause durations");
  expect(total_ticks(sym, n) == ticks, "pause tick sum");
  expect(fas_queue[0].read_idx == 1, "pause consumed");
  expect(dir_toggles == 0, "pause does not toggle");
}

static void test_pause_needs_a_full_half() {
  printf("pause waits for PART_SIZE free symbols\n");
  reset_queue();
  push(0, 3200, false);
  uint32_t sym[64];
  uint32_t n = rmt_encode_queue(&fas_queue[0], sym, (uint32_t)PART_SIZE - 1);
  expect(n == 0, "short buffer encodes nothing");
  expect(fas_queue[0].read_idx == 0, "pause not consumed");
  expect(fas_queue[0].entry[0].ticks == 3200, "pause ticks unchanged");
}

static void test_short_step_is_one_symbol() {
  printf("short step uses one symbol\n");
  reset_queue();
  push(1, 8000, false);
  uint32_t sym[8];
  uint32_t n = rmt_encode_queue(&fas_queue[0], sym, 8);
  expect(n == 1, "one symbol for 8000 ticks");
  bool l0, l1;
  uint16_t d0, d1;
  decode(sym[0], &l0, &d0, &l1, &d1);
  expect(l0 && !l1, "high then low");
  expect(d0 == 4000 && d1 == 4000, "even split");
  expect(rising_edges(sym, n) == 1, "one edge");
  expect(fas_queue[0].read_idx == 1, "step consumed");
  expect(fas_queue[0].entry[0].steps == 0, "steps cleared");
}

static void test_max_tick_step_is_two_symbols() {
  printf("65535-tick step uses two symbols\n");
  reset_queue();
  push(1, 65535, false);
  uint32_t sym[8];
  uint32_t n = rmt_encode_queue(&fas_queue[0], sym, 8);
  expect(n == 2, "two symbols");
  expect(!any_zero_duration(sym, n), "no zero duration");
  expect(total_ticks(sym, n) == 65535, "tick sum");
  expect(rising_edges(sym, n) == 1, "one edge");
  bool l0, l1;
  uint16_t d0, d1;
  decode(sym[0], &l0, &d0, &l1, &d1);
  expect(l0 && d0 == 32767, "leading high is 32767");
  expect(fas_queue[0].read_idx == 1, "step consumed");
}

static void test_step_needs_two_free_symbols() {
  printf("step entry needs two free symbols\n");
  reset_queue();
  push(4, 8000, false);
  uint32_t sym[8];
  uint32_t n = rmt_encode_queue(&fas_queue[0], sym, 1);
  expect(n == 0, "one free symbol encodes nothing");
  expect(fas_queue[0].read_idx == 0, "entry stays");
  expect(fas_queue[0].entry[0].steps == 4, "steps unchanged");
}

static void test_remaining_steps_written_back() {
  printf("partial step entry keeps the remainder\n");
  reset_queue();
  push(5, 8000, false);
  uint32_t sym[8];
  uint32_t n = rmt_encode_queue(&fas_queue[0], sym, 3);
  expect(n == 3, "three one-symbol steps");
  expect(rising_edges(sym, n) == 3, "three edges");
  expect(total_ticks(sym, n) == 24000, "three periods");
  expect(fas_queue[0].read_idx == 0, "entry still current");
  expect(fas_queue[0].entry[0].steps == 2, "two steps remain");
  n = rmt_encode_queue(&fas_queue[0], sym, 2);
  expect(n == 2, "rest of the entry");
  expect(fas_queue[0].read_idx == 1, "entry done");
  expect(fas_queue[0].entry[0].steps == 0, "no steps left");
}

static void test_max_tick_steps_pack_by_two() {
  printf("65535-tick steps stop when one symbol remains\n");
  reset_queue();
  push(3, 65535, false);
  uint32_t sym[8];
  uint32_t n = rmt_encode_queue(&fas_queue[0], sym, 5);
  expect(n == 4, "two steps, four symbols");
  expect(rising_edges(sym, n) == 2, "two edges");
  expect(fas_queue[0].entry[0].steps == 1, "one step remains");
  expect(fas_queue[0].read_idx == 0, "entry still current");
}

static void test_pause_then_steps_share_a_call() {
  printf("pause and following steps share one call\n");
  reset_queue();
  push(0, 4000, false);
  push(10, 8000, false);
  uint32_t sym[80];
  uint32_t free_symbols = (uint32_t)PART_SIZE + 4;
  uint32_t n = rmt_encode_queue(&fas_queue[0], sym, free_symbols);
  expect(n == free_symbols, "half plus four steps");
  expect(all_low(sym, (uint32_t)PART_SIZE), "leading pause");
  expect(total_ticks(sym, (uint32_t)PART_SIZE) == 4000, "pause sum");
  expect(rising_edges(sym + PART_SIZE, 4) == 4, "four steps");
  expect(fas_queue[0].read_idx == 1, "pause consumed, steps remain");
  expect(fas_queue[0].entry[1].steps == 6, "six steps remain");
}

static void test_toggle_waits_for_the_next_call() {
  printf("direction toggle is not in the pause call\n");
  reset_queue();
  push(0, 3200, false);
  push(0, 3200, false);
  push(1, 8000, true);
  uint32_t sym[80];
  uint32_t n = rmt_encode_queue(&fas_queue[0], sym, (uint32_t)PART_SIZE * 2);
  expect(n == (uint32_t)PART_SIZE * 2, "two drain pauses");
  expect(all_low(sym, n), "drains are low");
  expect(fas_queue[0].read_idx == 2, "toggle entry not started");
  expect(dir_toggles == 0, "dir still old");
  expect(fas_queue[0].entry[2].toggle_dir == 1, "toggle still pending");
  n = rmt_encode_queue(&fas_queue[0], sym, 4);
  expect(n == 1, "step on the next call");
  expect(dir_toggles == 1, "dir toggled once");
  expect(fas_queue[0].read_idx == 3, "toggle entry consumed");
  expect(rising_edges(sym, n) == 1, "one new-direction edge");
}

static void run_suite(uint16_t part) {
  debug_part_size = part;
  printf("\n=== PART_SIZE %u ===\n", part);
  test_pause_fills_one_half();
  test_pause_needs_a_full_half();
  test_short_step_is_one_symbol();
  test_max_tick_step_is_two_symbols();
  test_step_needs_two_free_symbols();
  test_remaining_steps_written_back();
  test_max_tick_steps_pack_by_two();
  test_pause_then_steps_share_a_call();
  test_toggle_waits_for_the_next_call();
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
