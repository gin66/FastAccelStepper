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

static uint8_t getByteOffset(uint8_t slot) { return i2s_mux_byte_offset(slot); }
static uint8_t getBitMask(uint8_t slot) { return i2s_mux_bit_mask(slot); }

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

static bool onlyTargetSlotUsed(const uint8_t* buf, uint8_t byte_offset,
                               uint8_t bit_mask) {
  for (uint16_t frame = 0; frame < I2S_FRAMES_PER_BLOCK; frame++) {
    for (uint8_t b = 0; b < I2S_BYTES_PER_FRAME; b++) {
      uint8_t v = buf[frame * I2S_BYTES_PER_FRAME + b];
      if (b == byte_offset) {
        if (v & ~bit_mask) {
          return false;
        }
      } else if (v != 0) {
        return false;
      }
    }
  }
  return true;
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

  uint32_t pulses_in_slot =
      countPulsesInSlot(buf, getByteOffset(0), getBitMask(0));
  bool isolated = onlyTargetSlotUsed(buf, getByteOffset(0), getBitMask(0));

  printf("  Slot 0 pulses: %u (expected 3)\n", pulses_in_slot);
  printf("  Isolated: %s\n", isolated ? "yes" : "no");

  test_result("MUX slot 0 isolation", pulses_in_slot == 3 && isolated);
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
  bool isolated = onlyTargetSlotUsed(buf, getByteOffset(7), getBitMask(7));

  printf("  Slot 7 pulses: %u (expected 3)\n", pulses_in_slot);
  printf("  Isolated: %s\n", isolated ? "yes" : "no");

  test_result("MUX slot 7 isolation", pulses_in_slot == 3 && isolated);
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
  bool isolated = onlyTargetSlotUsed(buf, getByteOffset(15), getBitMask(15));

  printf("  Slot 15 pulses: %u (expected 2)\n", pulses_in_slot);
  printf("  Isolated: %s\n", isolated ? "yes" : "no");

  test_result("MUX slot 15 isolation", pulses_in_slot == 2 && isolated);
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
  bool isolated = onlyTargetSlotUsed(buf, getByteOffset(31), getBitMask(31));

  printf("  Slot 31 pulses: %u (expected 2)\n", pulses_in_slot);
  printf("  Isolated: %s\n", isolated ? "yes" : "no");

  test_result("MUX slot 31 isolation", pulses_in_slot == 2 && isolated);
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
// Part 6: Position-exact, many queues, one shared buffer
// ============================================================================
//
// Everything above asserts pulse COUNTS, which cannot see a pulse landing one
// frame early or late: the count still matches and only the timing is wrong.
// They also memset(0) the buffer, while the driver seeds every frame from
// _mux_state before any queue touches it (init_mux_buffer), and at most two
// queues ever share a buffer. So the three things the failing sweep actually
// exercised -- the seed, the OR of many slots, and the frame each pulse lands
// in -- were untested.
//
// This drives N queues into one shared buffer the way handleTxDone() does and
// compares every byte of every frame against a model built from the tick
// arithmetic alone, so a fill that is off by a frame fails here rather than on
// a logic analyzer 300 ms later.

#define MUX_POS_MAX_SLOTS 32
// Sized for the slowest case below: 64 steps at 1600 ticks is 1600 frames.
#define MUX_POS_MAX_FRAMES (I2S_FRAMES_PER_BLOCK * 16)

// init_mux_buffer() from i2s_manager.cpp, which test_22 does not link.
static void seed_mux_buffer(uint8_t* buf, uint32_t mux_state) {
  uint32_t* b = reinterpret_cast<uint32_t*>(buf);
  uint8_t i = I2S_BYTES_PER_BLOCK / 4;
  do {
    b[--i] = mux_state;
  } while (i);
}

// A step's pulse sits in the frame containing its start tick, and start tick of
// step k is k * ticks. 400 ticks is 6.25 frames, so the frame sequence is
// 6, 6, 6, 7, ... -- the frame-quantised period the harness has to allow for.
static uint16_t ref_frame(uint16_t k, uint16_t ticks) {
  return (uint16_t)((uint32_t)k * ticks / I2S_TICKS_PER_FRAME);
}

static uint8_t seed_byte(uint32_t seed, uint8_t byte_index) {
  return (uint8_t)((seed >> (8 * byte_index)) & 0xFF);
}

static bool check_mux_positions(uint8_t nslots, uint16_t ticks, uint16_t steps,
                                uint32_t seed, bool verbose) {
  static StepperQueue q[MUX_POS_MAX_SLOTS];
  static struct i2s_fill_state st[MUX_POS_MAX_SLOTS];
  static uint8_t bufs[I2S_BLOCK_COUNT][I2S_BYTES_PER_BLOCK];
  static uint8_t want[MUX_POS_MAX_SLOTS][MUX_POS_MAX_FRAMES];
  bool ok = true;

  for (uint8_t s = 0; s < nslots; s++) {
    setupMuxQueue(&q[s], s);
    st[s].remaining_low_ticks = 0;
    st[s].remaining_high_ticks = 0;
    st[s].off_ticks = 0;
    add_command(&q[s], steps, ticks);
    memset(want[s], 0, MUX_POS_MAX_FRAMES);
    for (uint16_t k = 0; k < steps; k++) {
      uint16_t f = ref_frame(k, ticks);
      if (f < MUX_POS_MAX_FRAMES) {
        want[s][f] = 1;
      }
    }
  }
  memset(bufs, 0, sizeof(bufs));

  uint32_t total_frames = (uint32_t)steps * ticks / I2S_TICKS_PER_FRAME;
  uint16_t blocks = (uint16_t)(total_frames / I2S_FRAMES_PER_BLOCK) + 2;

  uint16_t seen_total = 0;
  for (uint16_t blk = 0; blk < blocks; blk++) {
    uint8_t* buf = bufs[blk % I2S_BLOCK_COUNT];
    seed_mux_buffer(buf, seed);
    for (uint8_t s = 0; s < nslots; s++) {
      i2s_fill_buffer_mux(&q[s], buf, &st[s], getByteOffset(s), getBitMask(s));
    }

    uint16_t f0 = (uint16_t)(blk * I2S_FRAMES_PER_BLOCK);
    for (uint16_t s = 0; s < nslots; s++) {
      uint16_t seen = 0;
      for (uint16_t f = 0; f < I2S_FRAMES_PER_BLOCK; f++) {
        uint16_t abs = f0 + f;
        uint8_t got =
            (buf[f * I2S_BYTES_PER_FRAME + getByteOffset(s)] & getBitMask(s))
                ? 1
                : 0;
        uint8_t exp = (abs < MUX_POS_MAX_FRAMES) ? want[s][abs] : 0;
        seen += got;
        if (got != exp) {
          if (ok && verbose) {
            printf("  slot %2u: frame %u (abs %u) is %u, expected %u\n", s, f,
                   abs, got, exp);
          }
          ok = false;
        }
      }
      seen_total += seen;
    }

    // Bytes the steppers do not own must come through the fill untouched, i.e.
    // still be exactly what init_mux_buffer seeded.
    for (uint16_t f = 0; f < I2S_FRAMES_PER_BLOCK; f++) {
      for (uint8_t b = 0; b < I2S_BYTES_PER_FRAME; b++) {
        uint8_t exp = seed_byte(seed, b);
        for (uint8_t s = 0; s < nslots; s++) {
          if (getByteOffset(s) == b) {
            uint16_t abs = (uint16_t)(f0 + f);
            if (abs < MUX_POS_MAX_FRAMES && want[s][abs]) {
              exp |= getBitMask(s);
            }
          }
        }
        uint8_t got = buf[f * I2S_BYTES_PER_FRAME + b];
        if (got != exp) {
          if (ok && verbose) {
            printf("  block %u frame %u byte %u is 0x%02x, expected 0x%02x\n",
                   blk, f, b, got, exp);
          }
          ok = false;
        }
      }
    }
  }

  uint16_t exp_per_slot = steps;
  if (verbose) {
    printf("  slots %u, %u steps @ %u ticks = %u frames over %u blocks\n",
           nslots, steps, ticks, total_frames, blocks);
    printf("  pulses seen in slot 0: %u (expected %u)\n", seen_total / nslots,
           exp_per_slot);
    printf("  seed word 0x%08lx preserved outside step slots: %s\n",
           (unsigned long)seed, ok ? "yes" : "NO");
  }

  for (uint8_t s = 0; s < nslots; s++) {
    if (q[s].read_idx != q[s].next_write_idx) {
      if (verbose) {
        printf("  slot %u: queue not fully consumed\n", s);
      }
      ok = false;
    }
  }
  return ok;
}

static void test_mux_positions_20_slots() {
  printf("Running: MUX positions, 20 slots @ 400 ticks, 64 steps\n");
  // Slots 20..31 are unused at n=20 and stand in for the dir bits a `dir` mode
  // run would hold there, so the seed is not trivially zero.
  test_result(
      "MUX positions 20 slots",
      check_mux_positions(20, I2S_MUX_MIN_SPEED_TICKS, 64, 0xFFF00000u, true));
}

static void test_mux_positions_26_slots() {
  printf("Running: MUX positions, 26 slots @ 400 ticks, 64 steps\n");
  // n=26 is the sweep point that lost slot 16 alone.
  test_result(
      "MUX positions 26 slots",
      check_mux_positions(26, I2S_MUX_MIN_SPEED_TICKS, 64, 0xFC000000u, true));
}

static void test_mux_positions_all_slots() {
  printf("Running: MUX positions, all 32 slots @ 400 ticks, 64 steps\n");
  test_result("MUX positions 32 slots",
              check_mux_positions(32, I2S_MUX_MIN_SPEED_TICKS, 64, 0u, true));
}

static void test_mux_positions_slow_and_fast() {
  printf("Running: MUX positions at other periods\n");
  bool a = check_mux_positions(20, 401, 64, 0xFFF00000u, false);
  bool b = check_mux_positions(20, 640, 64, 0xFFF00000u, false);
  bool c = check_mux_positions(20, 1000, 64, 0xFFF00000u, false);
  bool d = check_mux_positions(8, 1600, 64, 0u, false);
  test_result("MUX positions 401/640/1000/1600 ticks", a && b && c && d);
}

static void test_mux_positions_queue_drains_midrun() {
  printf("Running: MUX positions with the queue draining mid-block\n");
  // The path test_22 never takes: the fill returns false part way through a
  // block (queue empty), _isRunning goes false, and a later addQueueEntry has
  // to resume from the saved tick state rather than from the start of a block.
  StepperQueue q;
  setupMuxQueue(&q, 0);
  uint8_t buf[I2S_BYTES_PER_BLOCK];
  struct i2s_fill_state st = {0, 0, 0};
  bool ok = true;

  // 2 steps at 400 ticks: the block is 125 frames = 8000 ticks, so this fits
  // and the block ends with the queue already empty.
  add_command(&q, 2, 400);
  seed_mux_buffer(buf, 0);
  bool full =
      i2s_fill_buffer_mux(&q, buf, &st, getByteOffset(0), getBitMask(0));
  uint32_t first = countPulsesInSlot(buf, getByteOffset(0), getBitMask(0));
  // false means "queue drained", which is what happened: 2 steps at 400 ticks
  // is 800 ticks and the block is 8000. The unfilled rest of the block stays at
  // the seed, which is correct.
  if (first != 2 || full) {
    printf("  block 1: %u pulses, full=%d (expected 2, false)\n", first, full);
    ok = false;
  }

  // Now a pause long enough to span a block boundary, then a step. The step
  // must land on the frame its tick arithmetic names, not on the first frame.
  add_command(&q, 0, 9000);  // 9000 ticks = 140 frames > one block
  add_command(&q, 1, 400);
  memset(buf, 0, sizeof(buf));
  seed_mux_buffer(buf, 0);
  i2s_fill_buffer_mux(&q, buf, &st, getByteOffset(0), getBitMask(0));
  memset(buf, 0, sizeof(buf));
  seed_mux_buffer(buf, 0);
  i2s_fill_buffer_mux(&q, buf, &st, getByteOffset(0), getBitMask(0));
  uint32_t second = countPulsesInSlot(buf, getByteOffset(0), getBitMask(0));
  if (second != 1) {
    printf("  after the pause: %u pulses in the next block (expected 1)\n",
           second);
    ok = false;
  }
  if (q.read_idx != q.next_write_idx) {
    printf("  queue not consumed\n");
    ok = false;
  }
  test_result("MUX positions across a drained queue", ok);
}

// Do independently-filled queues stay in phase?
//
// Measured on hardware at 20 slots: the bus carried two distinct words per
// period instead of one -- 0x0000FFFF (slots 0-15) and 0x000F0000 (slots 16-19)
// -- each internally perfect at 6,6,6,7 frames, but offset from each other by
// about one step. That offset is what makes a dropped frame hit "16 of 20" and
// then "1 of 26, slot 16": the victims are a phase group, not a slot range.
//
// Nothing in i2s_fill_buffer_mux() can create that -- it is called per queue
// with its own state and touches only its own byte_offset/bit_mask. So either
// the fill introduces skew, or the queues were started at different times. This
// starts 20 queues from the same instant with the same command and asserts
// every one of them puts its pulses in the same frames. If this passes, the
// skew is not in the fill and the queues did not start together.
static void test_mux_queues_start_in_phase() {
  printf("Running: MUX 20 queues started together stay in phase\n");

  enum { N = 20 };
  StepperQueue q[N];
  struct i2s_fill_state st[N];
  uint8_t bufs[I2S_BLOCK_COUNT][I2S_BYTES_PER_BLOCK];

  for (uint8_t s = 0; s < N; s++) {
    setupMuxQueue(&q[s], s);
    st[s].remaining_low_ticks = 0;
    st[s].remaining_high_ticks = 0;
    st[s].off_ticks = 0;
    add_command(&q[s], 255, I2S_MUX_MIN_SPEED_TICKS);
  }
  memset(bufs, 0, sizeof(bufs));

  // One shared tick timeline: every queue runs the same 255 steps at the same
  // period, so a bit is set in frame f exactly when some step starts in f.
  static uint8_t common[MUX_POS_MAX_FRAMES];
  memset(common, 0, sizeof(common));
  for (uint16_t k = 0; k < 255; k++) {
    uint16_t f = ref_frame(k, I2S_MUX_MIN_SPEED_TICKS);
    if (f < MUX_POS_MAX_FRAMES) {
      common[f] = 1;
    }
  }

  bool ok = true;
  uint16_t blk = 0;
  uint16_t pulses_per_block = 0;
  while (blk < 6) {
    uint8_t* buf = bufs[blk % I2S_BLOCK_COUNT];
    seed_mux_buffer(buf, 0);
    for (uint8_t s = 0; s < N; s++) {
      i2s_fill_buffer_mux(&q[s], buf, &st[s], getByteOffset(s), getBitMask(s));
    }

    for (uint16_t f = 0; f < I2S_FRAMES_PER_BLOCK; f++) {
      uint16_t abs = (uint16_t)(blk * I2S_FRAMES_PER_BLOCK + f);
      if (abs < MUX_POS_MAX_FRAMES && common[abs]) {
        pulses_per_block++;
      }
      // All 20 slots must agree: byte b is 0xFF/0x0F/... in a pulse frame and
      // exactly the seed everywhere else. Any disagreement between slots is a
      // phase split, which is the hardware symptom this test exists to catch.
      for (uint8_t b = 0; b < I2S_BYTES_PER_FRAME; b++) {
        uint8_t expect = 0;
        if (abs < MUX_POS_MAX_FRAMES && common[abs]) {
          for (uint8_t s = 0; s < N; s++) {
            if (getByteOffset(s) == b) {
              expect |= getBitMask(s);
            }
          }
        }
        uint8_t got = buf[f * I2S_BYTES_PER_FRAME + b];
        if (got != expect) {
          if (ok) {
            printf(
                "  block %u frame %u (abs %u) byte %u is 0x%02x, "
                "expected 0x%02x\n",
                blk, f, abs, b, got, expect);
          }
          ok = false;
        }
      }
    }
    blk++;
  }

  if (ok) {
    printf(
        "  all %u slots agree on every frame of %u blocks "
        "(%u pulse frames per block, 6.25-frame period)\n",
        N, blk, pulses_per_block / blk);
  }
  test_result("MUX 20 queues start in phase", ok);
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

  puts("\n=== Part 6: Position-exact, Many Queues, One Buffer ===");
  test_mux_positions_20_slots();
  test_mux_positions_26_slots();
  test_mux_positions_all_slots();
  test_mux_positions_slow_and_fast();
  test_mux_positions_queue_drains_midrun();
  test_mux_queues_start_in_phase();

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
