// test_22: ESP32 I2S MUX mode fill buffer tests
//
// Tests i2s_fill_buffer_mux() with frame-level resolution.
// MUX mode uses 74HC595 shift register for up to 32 steppers.

#define SUPPORT_ESP32_I2S

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pd_esp32/i2s_constants.h"
#include "fas_queue/stepper_queue.h"

static int test_passed = 0;
static int test_failed = 0;

static void test_result(const char* name, bool passed) {
  if (passed) {
    printf("PASS: %s\n", name);
    test_passed++;
  } else {
    printf("FAIL: %s\n", name);
    test_failed++;
  }
}

#define IRAM_ATTR
#include "pd_esp32/i2s_fill.cpp"

#define NO_PIN 255

static void add_command(StepperQueueBase* q, uint8_t steps, uint16_t ticks) {
  uint8_t idx = q->next_write_idx & QUEUE_LEN_MASK;
  q->entry[idx].steps = steps;
  q->entry[idx].ticks = ticks;
  q->entry[idx].toggle_dir = 0;
  q->next_write_idx++;
}

// MUX-specific helpers
static uint32_t countPulsesInSlot(const uint8_t* buf, uint8_t byte_offset,
                                  uint8_t bit_mask) {
  uint32_t count = 0;
  for (uint16_t frame = 0; frame < I2S_FRAMES_PER_BLOCK; frame++) {
    uint8_t val = buf[frame * I2S_BYTES_PER_FRAME + byte_offset];
    if (val & bit_mask) {
      count++;
    }
  }
  return count;
}

static uint8_t getByteOffset(uint8_t slot) { return slot / 8; }
static uint8_t getBitMask(uint8_t slot) { return 1 << (7 - (slot % 8)); }

static void setupMuxQueue(StepperQueue* q, uint8_t slot) {
  q->read_idx = 0;
  q->next_write_idx = 0;
  q->dirPin = NO_PIN;
}

static bool fillAndDetectPulses_mux(StepperQueue* q, uint8_t num_blocks,
                                    uint8_t slot, uint16_t* num_pulses_out) {
  uint8_t buf[I2S_BYTES_PER_BLOCK];
  struct i2s_fill_state state = {0, 0, 0};
  uint8_t byte_offset = getByteOffset(slot);
  uint8_t bit_mask = getBitMask(slot);
  bool last_result = false;
  uint16_t total_pulses = 0;

  for (uint8_t blk = 0; blk < num_blocks; blk++) {
    memset(buf, 0, I2S_BYTES_PER_BLOCK);
    last_result = i2s_fill_buffer_mux(q, buf, &state, byte_offset, bit_mask);
    total_pulses += countPulsesInSlot(buf, byte_offset, bit_mask);

    if (q->read_idx == q->next_write_idx && state.remaining_low_ticks == 0 &&
        state.remaining_high_ticks == 0) {
      break;
    }
  }

  *num_pulses_out = total_pulses;
  return last_result;
}

// ============================================================================
// Part 1: Single Stepper Frame-Level Tests
// ============================================================================

static void test_mux_single_step() {
  printf("Running: MUX single step @ 128 ticks\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q;
  setupMuxQueue(&q, 0);

  add_command(&q, 1, 128);

  struct i2s_fill_state state = {0, 0, 0};
  i2s_fill_buffer_mux(&q, buf, &state, getByteOffset(0), getBitMask(0));

  uint32_t pulses = countPulsesInSlot(buf, getByteOffset(0), getBitMask(0));
  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Pulses: %u (expected 1)\n", pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("MUX single step", pulses == 1 && consumed);
}

static void test_mux_multi_step() {
  printf("Running: MUX 5 steps @ 128 ticks\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q;
  setupMuxQueue(&q, 0);

  add_command(&q, 5, 128);

  struct i2s_fill_state state = {0, 0, 0};
  i2s_fill_buffer_mux(&q, buf, &state, getByteOffset(0), getBitMask(0));

  uint32_t pulses = countPulsesInSlot(buf, getByteOffset(0), getBitMask(0));
  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Pulses: %u (expected 5)\n", pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("MUX multi step", pulses == 5 && consumed);
}

static void test_mux_step_pause_step() {
  printf("Running: MUX step + pause + step\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q;
  setupMuxQueue(&q, 0);

  add_command(&q, 1, 128);
  add_command(&q, 0, 256);
  add_command(&q, 1, 128);

  struct i2s_fill_state state = {0, 0, 0};
  i2s_fill_buffer_mux(&q, buf, &state, getByteOffset(0), getBitMask(0));

  uint32_t pulses = countPulsesInSlot(buf, getByteOffset(0), getBitMask(0));
  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Pulses: %u (expected 2)\n", pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("MUX step+pause+step", pulses == 2 && consumed);
}

static void test_mux_empty_queue() {
  printf("Running: MUX empty queue\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q;
  setupMuxQueue(&q, 0);

  struct i2s_fill_state state = {0, 0, 0};
  bool result =
      i2s_fill_buffer_mux(&q, buf, &state, getByteOffset(0), getBitMask(0));

  uint32_t pulses = countPulsesInSlot(buf, getByteOffset(0), getBitMask(0));

  printf("  Pulses: %u (expected 0)\n", pulses);
  printf("  Return: %s (expected false)\n", result ? "true" : "false");

  test_result("MUX empty queue", pulses == 0 && !result);
}

static void test_mux_pause_only() {
  printf("Running: MUX pause only (steps=0)\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q;
  setupMuxQueue(&q, 0);

  add_command(&q, 0, 1000);

  struct i2s_fill_state state = {0, 0, 0};
  i2s_fill_buffer_mux(&q, buf, &state, getByteOffset(0), getBitMask(0));

  uint32_t pulses = countPulsesInSlot(buf, getByteOffset(0), getBitMask(0));
  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Pulses: %u (expected 0)\n", pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("MUX pause only", pulses == 0 && consumed);
}

// ============================================================================
// Part 2: off_ticks Compensation Tests
// ============================================================================

static void test_mux_off_ticks_carry() {
  printf("Running: MUX off_ticks carry (verify step at correct position)\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q;
  setupMuxQueue(&q, 0);

  add_command(&q, 1, 100);

  struct i2s_fill_state state = {0, 0, 0};
  i2s_fill_buffer_mux(&q, buf, &state, getByteOffset(0), getBitMask(0));

  uint32_t pulses = countPulsesInSlot(buf, getByteOffset(0), getBitMask(0));
  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Pulses: %u (expected 1)\n", pulses);
  printf("  off_ticks after: %u\n", state.off_ticks);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("MUX off_ticks carry", pulses == 1 && consumed);
}

static void test_mux_off_ticks_across_blocks() {
  printf("Running: MUX off_ticks across blocks\n");

  uint8_t bufs[I2S_BLOCK_COUNT][I2S_BYTES_PER_BLOCK];
  for (int i = 0; i < I2S_BLOCK_COUNT; i++) {
    memset(bufs[i], 0, I2S_BYTES_PER_BLOCK);
  }

  StepperQueue q;
  setupMuxQueue(&q, 0);

  add_command(&q, 1, 100);

  struct i2s_fill_state state = {0, 0, 0};

  i2s_fill_buffer_mux(&q, bufs[0], &state, getByteOffset(0), getBitMask(0));
  uint32_t pulses1 =
      countPulsesInSlot(bufs[0], getByteOffset(0), getBitMask(0));

  add_command(&q, 1, 100);
  i2s_fill_buffer_mux(&q, bufs[1], &state, getByteOffset(0), getBitMask(0));
  uint32_t pulses2 =
      countPulsesInSlot(bufs[1], getByteOffset(0), getBitMask(0));

  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Block 1 pulses: %u\n", pulses1);
  printf("  Block 2 pulses: %u (expected 1, with off_ticks=%u)\n", pulses2,
         state.off_ticks);
  printf("  Total pulses: %u (expected 2)\n", pulses1 + pulses2);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("MUX off_ticks across blocks",
              pulses1 + pulses2 == 2 && consumed);
}

static void test_mux_off_ticks_zero() {
  printf("Running: MUX off_ticks zero (exact frame multiple)\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q;
  setupMuxQueue(&q, 0);

  add_command(&q, 3, 128);

  struct i2s_fill_state state = {0, 0, 0};
  i2s_fill_buffer_mux(&q, buf, &state, getByteOffset(0), getBitMask(0));

  uint32_t pulses = countPulsesInSlot(buf, getByteOffset(0), getBitMask(0));
  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Pulses: %u (expected 3)\n", pulses);
  printf("  off_ticks: %u (expected 0)\n", state.off_ticks);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("MUX off_ticks zero",
              pulses == 3 && state.off_ticks == 0 && consumed);
}

static void test_mux_off_ticks_compensation() {
  printf("Running: MUX off_ticks compensation (255 steps @ 431 ticks)\n");

  StepperQueue q;
  setupMuxQueue(&q, 0);

  add_command(&q, 255, 431);

  uint16_t num_pulses = 0;
  fillAndDetectPulses_mux(&q, 20, 0, &num_pulses);

  bool consumed = (q.read_idx == q.next_write_idx);
  printf("  Total pulses: %u (expected 255)\n", num_pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("MUX off_ticks compensation 255@431",
              num_pulses == 255 && consumed);
}

static void test_mux_slot_0_isolation() {
  printf("Running: MUX slot 0 isolation\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q;
  setupMuxQueue(&q, 0);

  add_command(&q, 3, 128);

  struct i2s_fill_state state = {0, 0, 0};
  i2s_fill_buffer_mux(&q, buf, &state, getByteOffset(0), getBitMask(0));

  uint8_t other_byte = 1;
  uint8_t other_mask = 0xFF;
  uint32_t pulses_in_slot =
      countPulsesInSlot(buf, getByteOffset(0), getBitMask(0));
  uint32_t pulses_other = countPulsesInSlot(buf, other_byte, other_mask);

  printf("  Slot 0 pulses: %u (expected 3)\n", pulses_in_slot);
  printf("  Other slots: %u (expected 0)\n", pulses_other);

  test_result("MUX slot 0 isolation", pulses_in_slot == 3 && pulses_other == 0);
}

static void test_mux_slot_7_isolation() {
  printf("Running: MUX slot 7 isolation\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q;
  setupMuxQueue(&q, 7);

  add_command(&q, 3, 128);

  struct i2s_fill_state state = {0, 0, 0};
  i2s_fill_buffer_mux(&q, buf, &state, getByteOffset(7), getBitMask(7));

  uint32_t pulses_in_slot =
      countPulsesInSlot(buf, getByteOffset(7), getBitMask(7));
  uint32_t pulses_other = countPulsesInSlot(buf, 0, 0xFE);

  printf("  Slot 7 pulses: %u (expected 3)\n", pulses_in_slot);
  printf("  Other slots: %u (expected 0)\n", pulses_other);

  test_result("MUX slot 7 isolation", pulses_in_slot == 3 && pulses_other == 0);
}

static void test_mux_slot_15_isolation() {
  printf("Running: MUX slot 15 isolation\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q;
  setupMuxQueue(&q, 15);

  add_command(&q, 2, 128);

  struct i2s_fill_state state = {0, 0, 0};
  i2s_fill_buffer_mux(&q, buf, &state, getByteOffset(15), getBitMask(15));

  uint32_t pulses_in_slot =
      countPulsesInSlot(buf, getByteOffset(15), getBitMask(15));
  uint32_t pulses_byte0 = countPulsesInSlot(buf, 0, 0xFF);
  uint32_t pulses_byte2 = countPulsesInSlot(buf, 2, 0xFF);

  printf("  Slot 15 pulses: %u (expected 2)\n", pulses_in_slot);
  printf("  Byte 0: %u, Byte 2: %u (expected 0)\n", pulses_byte0, pulses_byte2);

  test_result("MUX slot 15 isolation",
              pulses_in_slot == 2 && pulses_byte0 == 0 && pulses_byte2 == 0);
}

static void test_mux_slot_31_isolation() {
  printf("Running: MUX slot 31 isolation\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q;
  setupMuxQueue(&q, 31);

  add_command(&q, 2, 128);

  struct i2s_fill_state state = {0, 0, 0};
  i2s_fill_buffer_mux(&q, buf, &state, getByteOffset(31), getBitMask(31));

  uint32_t pulses_in_slot =
      countPulsesInSlot(buf, getByteOffset(31), getBitMask(31));
  uint32_t pulses_other = countPulsesInSlot(buf, 0, 0xFF);

  printf("  Slot 31 pulses: %u (expected 2)\n", pulses_in_slot);
  printf("  Other: %u (expected 0)\n", pulses_other);

  test_result("MUX slot 31 isolation",
              pulses_in_slot == 2 && pulses_other == 0);
}

static void test_mux_two_steppers_same_buffer() {
  printf("Running: MUX two steppers same buffer (OR operation)\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q1, q2;
  setupMuxQueue(&q1, 0);
  setupMuxQueue(&q2, 1);

  add_command(&q1, 2, 128);
  add_command(&q2, 3, 128);

  struct i2s_fill_state state1 = {0, 0, 0};
  struct i2s_fill_state state2 = {0, 0, 0};

  i2s_fill_buffer_mux(&q1, buf, &state1, getByteOffset(0), getBitMask(0));
  i2s_fill_buffer_mux(&q2, buf, &state2, getByteOffset(1), getBitMask(1));

  uint32_t pulses_slot0 =
      countPulsesInSlot(buf, getByteOffset(0), getBitMask(0));
  uint32_t pulses_slot1 =
      countPulsesInSlot(buf, getByteOffset(1), getBitMask(1));

  printf("  Slot 0 pulses: %u (expected 2)\n", pulses_slot0);
  printf("  Slot 1 pulses: %u (expected 3)\n", pulses_slot1);

  test_result("MUX two steppers OR", pulses_slot0 == 2 && pulses_slot1 == 3);
}

// ============================================================================
// Part 4: Block Boundary Tests
// ============================================================================

static void test_mux_block_boundary() {
  printf("Running: MUX block boundary\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q;
  setupMuxQueue(&q, 0);

  uint16_t block_ticks = I2S_FRAMES_PER_BLOCK * I2S_TICKS_PER_FRAME;
  add_command(&q, 1, block_ticks);

  struct i2s_fill_state state = {0, 0, 0};
  bool full =
      i2s_fill_buffer_mux(&q, buf, &state, getByteOffset(0), getBitMask(0));

  uint32_t pulses = countPulsesInSlot(buf, getByteOffset(0), getBitMask(0));

  printf("  Pulses: %u (expected 1)\n", pulses);
  printf("  Return: %s (expected true)\n", full ? "true" : "false");

  test_result("MUX block boundary", pulses == 1 && full);
}

static void test_mux_pause_spans_block() {
  printf("Running: MUX step spans multiple blocks\n");

  StepperQueue q;
  setupMuxQueue(&q, 0);

  uint16_t block_ticks = I2S_FRAMES_PER_BLOCK * I2S_TICKS_PER_FRAME;
  add_command(&q, 1, block_ticks + 256);

  uint16_t num_pulses = 0;
  fillAndDetectPulses_mux(&q, 3, 0, &num_pulses);

  bool consumed = (q.read_idx == q.next_write_idx);
  printf("  Total pulses: %u (expected 1)\n", num_pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("MUX step spans blocks", num_pulses == 1 && consumed);
}

static void test_mux_return_value_full() {
  printf("Running: MUX return value full\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q;
  setupMuxQueue(&q, 0);

  uint16_t block_ticks = I2S_FRAMES_PER_BLOCK * I2S_TICKS_PER_FRAME;
  add_command(&q, 1, block_ticks);

  struct i2s_fill_state state = {0, 0, 0};
  bool result =
      i2s_fill_buffer_mux(&q, buf, &state, getByteOffset(0), getBitMask(0));

  printf("  Return: %s (expected true)\n", result ? "true" : "false");

  test_result("MUX return value full", result);
}

static void test_mux_return_value_partial() {
  printf("Running: MUX return value partial\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q;
  setupMuxQueue(&q, 0);

  add_command(&q, 2, 128);

  struct i2s_fill_state state = {0, 0, 0};
  bool result =
      i2s_fill_buffer_mux(&q, buf, &state, getByteOffset(0), getBitMask(0));

  printf("  Return: %s (expected false)\n", result ? "true" : "false");

  test_result("MUX return value partial", !result);
}

// ============================================================================
// Part 5: Edge Cases
// ============================================================================

static void test_mux_max_steps() {
  printf("Running: MUX max steps (255 @ 64 ticks)\n");

  uint8_t bufs[3][I2S_BYTES_PER_BLOCK];
  for (int i = 0; i < 3; i++) {
    memset(bufs[i], 0, I2S_BYTES_PER_BLOCK);
  }

  StepperQueue q;
  setupMuxQueue(&q, 0);

  add_command(&q, 255, 64);

  struct i2s_fill_state state = {0, 0, 0};
  uint32_t total_pulses = 0;

  for (int blk = 0; blk < 3; blk++) {
    i2s_fill_buffer_mux(&q, bufs[blk], &state, getByteOffset(0), getBitMask(0));
    total_pulses +=
        countPulsesInSlot(bufs[blk], getByteOffset(0), getBitMask(0));
  }

  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Total pulses: %u (expected 255)\n", total_pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("MUX max steps 255@64", total_pulses == 255 && consumed);
}

static void test_mux_long_pause() {
  printf("Running: MUX long pause (65535 ticks) + step\n");

  StepperQueue q;
  setupMuxQueue(&q, 0);

  add_command(&q, 0, 65535);
  add_command(&q, 1, 128);

  uint16_t num_pulses = 0;
  fillAndDetectPulses_mux(&q, 15, 0, &num_pulses);

  bool consumed = (q.read_idx == q.next_write_idx);
  printf("  Total pulses: %u (expected 1)\n", num_pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("MUX long pause", num_pulses == 1 && consumed);
}

static void test_mux_min_speed() {
  printf("Running: MUX min speed (400 ticks)\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueue q;
  setupMuxQueue(&q, 0);

  add_command(&q, 3, I2S_MUX_MIN_SPEED_TICKS);

  struct i2s_fill_state state = {0, 0, 0};
  i2s_fill_buffer_mux(&q, buf, &state, getByteOffset(0), getBitMask(0));

  uint32_t pulses = countPulsesInSlot(buf, getByteOffset(0), getBitMask(0));
  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Pulses: %u (expected 3)\n", pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("MUX min speed", pulses == 3 && consumed);
}

static void test_mux_partial_steps() {
  printf("Running: MUX partial steps (100 @ 100 ticks)\n");

  uint8_t bufs[I2S_BLOCK_COUNT][I2S_BYTES_PER_BLOCK];
  for (int i = 0; i < I2S_BLOCK_COUNT; i++) {
    memset(bufs[i], 0, I2S_BYTES_PER_BLOCK);
  }

  StepperQueue q;
  setupMuxQueue(&q, 0);

  add_command(&q, 100, 100);

  struct i2s_fill_state state = {0, 0, 0};

  i2s_fill_buffer_mux(&q, bufs[0], &state, getByteOffset(0), getBitMask(0));
  uint32_t pulses1 =
      countPulsesInSlot(bufs[0], getByteOffset(0), getBitMask(0));

  i2s_fill_buffer_mux(&q, bufs[1], &state, getByteOffset(0), getBitMask(0));
  uint32_t pulses2 =
      countPulsesInSlot(bufs[1], getByteOffset(0), getBitMask(0));

  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Block 1 pulses: %u\n", pulses1);
  printf("  Block 2 pulses: %u (expected > 0)\n", pulses2);
  printf("  Total: %u (expected 100)\n", pulses1 + pulses2);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("MUX partial steps", pulses1 + pulses2 == 100 && consumed);
}

// ============================================================================
// Main
// ============================================================================

void basic_test() {
  puts("=== I2S MUX Fill Buffer Tests ===\n");
  fflush(stdout);

  puts("=== Part 1: Single Stepper Frame-Level ===");
  test_mux_single_step();
  test_mux_multi_step();
  test_mux_step_pause_step();
  test_mux_empty_queue();
  test_mux_pause_only();

  puts("\n=== Part 2: off_ticks Compensation ===");
  test_mux_off_ticks_carry();
  test_mux_off_ticks_across_blocks();
  test_mux_off_ticks_zero();
  test_mux_off_ticks_compensation();

  puts("\n=== Part 3: Slot Isolation (Representative: 0, 7, 15, 31) ===");
  test_mux_slot_0_isolation();
  test_mux_slot_7_isolation();
  test_mux_slot_15_isolation();
  test_mux_slot_31_isolation();
  test_mux_two_steppers_same_buffer();

  puts("\n=== Part 4: Block Boundary ===");
  test_mux_block_boundary();
  test_mux_pause_spans_block();
  test_mux_return_value_full();
  test_mux_return_value_partial();

  puts("\n=== Part 5: Edge Cases ===");
  test_mux_max_steps();
  test_mux_long_pause();
  test_mux_min_speed();
  test_mux_partial_steps();

  printf("\n=== Test Summary ===\n");
  printf("Total: %d  Passed: %d  Failed: %d\n", test_passed + test_failed,
         test_passed, test_failed);
  if (test_failed == 0) {
    puts("All tests PASSED");
  } else {
    printf("%d test(s) FAILED\n", test_failed);
  }
}

int main() {
  basic_test();
  return test_failed ? 1 : 0;
}
