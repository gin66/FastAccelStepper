// test_21: ESP32 I2S tests.
//
// Part 1: Pure DMA infrastructure tests - callbacks, triple buffering,
// bit stream analysis. No driver code involved.
//
// Part 2: Driver fill function tests - i2s_fill_buffer() with queue commands.

#define SUPPORT_ESP32_I2S

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pd_esp32/i2s_constants.h"

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

// ---------------------------------------------------------------------------
// Mock I2sManager with callback support
// ---------------------------------------------------------------------------

typedef void (*i2s_dma_callback_t)(uint8_t block);

class I2sManager {
 public:
  static I2sManager& instance();

  bool init(int, int, int) {
    _initialized = true;
    for (int i = 0; i < I2S_BLOCK_COUNT; i++) {
      memset(_bufs[i], 0, I2S_BYTES_PER_BLOCK);
    }
    _dma_block = 0;
    _write_block = 0;
    _callback_count = 0;
    return true;
  }

  bool isInitialized() const { return _initialized; }

  uint8_t* blockBuf(uint8_t block) { return _bufs[block % I2S_BLOCK_COUNT]; }

  void clearBlock(uint8_t block) {
    if (block < I2S_BLOCK_COUNT) {
      memset(_bufs[block], 0x00, I2S_BYTES_PER_BLOCK);
    }
  }

  uint8_t dmaBlock() const { return _dma_block; }
  uint8_t writeBlock() const { return _write_block; }

  bool isWriteBlockAvailable() const {
    uint8_t next_write = (_write_block + 1) % I2S_BLOCK_COUNT;
    return next_write != _dma_block;
  }

  void markWriteBlockPrepared() {
    _write_block = (_write_block + 1) % I2S_BLOCK_COUNT;
  }

  void registerDmaCallback(i2s_dma_callback_t cb) { _dma_callback = cb; }

  void dmaConsumeBlock() {
    if (_dma_callback) {
      _dma_callback(_dma_block);
      _callback_count++;
    }
    memset(_bufs[_dma_block], 0, I2S_BYTES_PER_BLOCK);
    _dma_block = (_dma_block + 1) % I2S_BLOCK_COUNT;
  }

  uint32_t callbackCount() const { return _callback_count; }

  I2sManager()
      : _initialized(false),
        _dma_block(0),
        _write_block(0),
        _callback_count(0),
        _dma_callback(nullptr) {
    for (int i = 0; i < I2S_BLOCK_COUNT; i++) {
      memset(_bufs[i], 0, I2S_BYTES_PER_BLOCK);
    }
  }

 private:
  bool _initialized;
  uint8_t _bufs[I2S_BLOCK_COUNT][I2S_BYTES_PER_BLOCK];
  uint8_t _dma_block;
  uint8_t _write_block;
  uint32_t _callback_count;
  i2s_dma_callback_t _dma_callback;
};

static I2sManager s_i2s_mgr;
inline I2sManager& I2sManager::instance() { return s_i2s_mgr; }

// ---------------------------------------------------------------------------
// Bit stream analyzer - receives tick-level pin states
// ---------------------------------------------------------------------------
// Bit stream analyzer - receives tick-level pin states
// ---------------------------------------------------------------------------
// Bit stream analyzer - receives tick-level pin states
// ---------------------------------------------------------------------------

class BitStreamAnalyzer {
 public:
  uint32_t rising_edges;
  uint32_t falling_edges;
  uint32_t total_ticks;
  uint32_t last_rising_tick;
  uint32_t last_falling_tick;
  bool last_level;
  uint8_t ticks_per_bit;

  void reset() {
    rising_edges = 0;
    falling_edges = 0;
    total_ticks = 0;
    last_rising_tick = 0;
    last_falling_tick = 0;
    last_level = false;
  }

  void processBit(bool level) {
    for (uint8_t i = 0; i < ticks_per_bit; i++) {
      processTick(level);
    }
  }

  void processTick(bool level) {
    if (level && !last_level) {
      last_rising_tick = total_ticks;
      rising_edges++;
    } else if (!level && last_level) {
      last_falling_tick = total_ticks;
      falling_edges++;
    }
    last_level = level;
    total_ticks++;
  }

  uint32_t pulseCount() const { return rising_edges; }
};

static BitStreamAnalyzer s_analyzer;

// ---------------------------------------------------------------------------
// DMA callback - processes buffer and sends to bit stream analyzer
//
// TODO: For driver integration tests (test_22), this callback should invoke
// the driver's fill_i2s_buffer() function to simulate real hardware behavior.
// For these low-level infrastructure tests, we only analyze the buffer content.
// ---------------------------------------------------------------------------

static void dmaCallback(uint8_t block) {
  I2sManager& mgr = I2sManager::instance();
  const uint8_t* buf = mgr.blockBuf(block);

  for (uint16_t frame = 0; frame < I2S_FRAMES_PER_BLOCK; frame++) {
    uint8_t l_msb = buf[frame * I2S_BYTES_PER_FRAME + 0];
    uint8_t l_lsb = buf[frame * I2S_BYTES_PER_FRAME + 1];
    uint8_t r_msb = buf[frame * I2S_BYTES_PER_FRAME + 2];
    uint8_t r_lsb = buf[frame * I2S_BYTES_PER_FRAME + 3];

    for (int8_t bit = 7; bit >= 0; bit--) {
      s_analyzer.processBit((l_msb & (1 << bit)) != 0);
    }
    for (int8_t bit = 7; bit >= 0; bit--) {
      s_analyzer.processBit((l_lsb & (1 << bit)) != 0);
    }
    for (int8_t bit = 7; bit >= 0; bit--) {
      s_analyzer.processBit((r_msb & (1 << bit)) != 0);
    }
    for (int8_t bit = 7; bit >= 0; bit--) {
      s_analyzer.processBit((r_lsb & (1 << bit)) != 0);
    }
  }
}

// ---------------------------------------------------------------------------
// Helper to write pulses to buffer
// ---------------------------------------------------------------------------

static void write_pulse_to_buffer(uint8_t* buf, uint16_t frame) {
  buf[frame * I2S_BYTES_PER_FRAME + 0] = 0xFF;
  buf[frame * I2S_BYTES_PER_FRAME + 1] = 0xFF;
}

// ---------------------------------------------------------------------------
// Low-level DMA tests
// ---------------------------------------------------------------------------

static void test_dma_callback_invoked() {
  printf("Running: DMA callback invoked on consume\n");

  s_analyzer.reset();
  I2sManager::instance().init(32, 33, -1);
  I2sManager::instance().registerDmaCallback(dmaCallback);

  I2sManager::instance().dmaConsumeBlock();
  I2sManager::instance().dmaConsumeBlock();
  I2sManager::instance().dmaConsumeBlock();

  uint32_t callbacks = I2sManager::instance().callbackCount();
  printf("  Callbacks: %u (expected 3)\n", callbacks);

  test_result("DMA callback invoked", callbacks == 3);
}

static void test_dma_block_rotation() {
  printf("Running: DMA block rotation\n");

  I2sManager::instance().init(32, 33, -1);

  uint8_t b0 = I2sManager::instance().dmaBlock();
  I2sManager::instance().dmaConsumeBlock();
  uint8_t b1 = I2sManager::instance().dmaBlock();
  I2sManager::instance().dmaConsumeBlock();
  uint8_t b2 = I2sManager::instance().dmaBlock();
  I2sManager::instance().dmaConsumeBlock();
  uint8_t b3 = I2sManager::instance().dmaBlock();

  printf("  Blocks: %u -> %u -> %u -> %u\n", b0, b1, b2, b3);
  printf("  Expected: 0 -> 1 -> 2 -> 0 (modulo 3)\n");

  bool correct = (b0 == 0) && (b1 == 1) && (b2 == 2) && (b3 == 0);
  test_result("DMA block rotation", correct);
}

static void test_dma_write_block_advance() {
  printf("Running: DMA write block advance\n");

  I2sManager::instance().init(32, 33, -1);

  uint8_t w0 = I2sManager::instance().writeBlock();
  I2sManager::instance().markWriteBlockPrepared();
  uint8_t w1 = I2sManager::instance().writeBlock();
  I2sManager::instance().markWriteBlockPrepared();
  uint8_t w2 = I2sManager::instance().writeBlock();
  I2sManager::instance().markWriteBlockPrepared();
  uint8_t w3 = I2sManager::instance().writeBlock();

  printf("  Write blocks: %u -> %u -> %u -> %u\n", w0, w1, w2, w3);
  printf("  Expected: 0 -> 1 -> 2 -> 0 (modulo 3)\n");

  bool correct = (w0 == 0) && (w1 == 1) && (w2 == 2) && (w3 == 0);
  test_result("DMA write block advance", correct);
}

static void test_dma_buffer_clearing() {
  printf("Running: DMA buffer clearing on consume\n");

  I2sManager::instance().init(32, 33, -1);
  uint8_t* buf = I2sManager::instance().blockBuf(0);

  memset(buf, 0xFF, I2S_BYTES_PER_BLOCK);
  printf("  Buffer 0 filled with 0xFF\n");

  I2sManager::instance().dmaConsumeBlock();

  buf = I2sManager::instance().blockBuf(0);

  bool all_zero = true;
  for (uint16_t i = 0; i < I2S_BYTES_PER_BLOCK; i++) {
    if (buf[i] != 0x00) {
      all_zero = false;
      break;
    }
  }

  printf("  After consume, block 0 all zeros: %s\n", all_zero ? "yes" : "no");
  test_result("DMA buffer clearing", all_zero);
}

static void test_dma_block_availability() {
  printf("Running: DMA write block availability\n");

  I2sManager::instance().init(32, 33, -1);

  bool avail0 = I2sManager::instance().isWriteBlockAvailable();
  printf("  Initial (dma=0, write=0): available=%s\n", avail0 ? "yes" : "no");

  I2sManager::instance().markWriteBlockPrepared();
  bool avail1 = I2sManager::instance().isWriteBlockAvailable();
  printf("  After 1 prepare (dma=0, write=1): available=%s\n",
         avail1 ? "yes" : "no");

  I2sManager::instance().markWriteBlockPrepared();
  bool avail2 = I2sManager::instance().isWriteBlockAvailable();
  printf("  After 2 prepares (dma=0, write=2): available=%s\n",
         avail2 ? "yes" : "no");

  I2sManager::instance().dmaConsumeBlock();
  bool avail3 = I2sManager::instance().isWriteBlockAvailable();
  printf("  After 1 consume (dma=1, write=2): available=%s\n",
         avail3 ? "yes" : "no");

  bool correct = avail0 && avail1 && !avail2 && avail3;
  test_result("DMA write block availability", correct);
}

static void test_bit_stream_single_pulse() {
  printf("Running: Bit stream single pulse detection\n");

  s_analyzer.reset();

  for (int i = 0; i < 100; i++) {
    s_analyzer.processTick(false);
  }
  for (int i = 0; i < 64; i++) {
    s_analyzer.processTick(true);
  }
  for (int i = 0; i < 100; i++) {
    s_analyzer.processTick(false);
  }

  printf("  Rising edges: %u (expected 1)\n", s_analyzer.rising_edges);
  printf("  Falling edges: %u (expected 1)\n", s_analyzer.falling_edges);
  printf("  Total ticks: %u (expected 264)\n", s_analyzer.total_ticks);

  bool correct = (s_analyzer.rising_edges == 1) &&
                 (s_analyzer.falling_edges == 1) &&
                 (s_analyzer.total_ticks == 264);
  test_result("Bit stream single pulse", correct);
}

static void test_bit_stream_multiple_pulses() {
  printf("Running: Bit stream 7 pulses detection\n");

  s_analyzer.reset();

  for (int pulse = 0; pulse < 7; pulse++) {
    for (int i = 0; i < 64; i++) {
      s_analyzer.processTick(true);
    }
    for (int i = 0; i < 64 * 49; i++) {
      s_analyzer.processTick(false);
    }
  }

  printf("  Pulses detected: %u (expected 7)\n", s_analyzer.pulseCount());
  printf("  Total ticks: %u (expected %u)\n", s_analyzer.total_ticks,
         7 * 64 * 50);

  bool correct =
      (s_analyzer.pulseCount() == 7) && (s_analyzer.total_ticks == 7 * 64 * 50);
  test_result("Bit stream 7 pulses", correct);
}

static void test_dma_10_rounds_7_pulses() {
  printf("Running: DMA 10 rounds with 7 pulses each\n");

  s_analyzer.reset();
  I2sManager::instance().init(32, 33, -1);
  I2sManager::instance().registerDmaCallback(dmaCallback);

  for (int round = 0; round < 10; round++) {
    uint8_t blk = I2sManager::instance().dmaBlock();
    uint8_t* buf = I2sManager::instance().blockBuf(blk);

    for (int pulse = 0; pulse < 7; pulse++) {
      uint16_t frame = (pulse * 35) % I2S_FRAMES_PER_BLOCK;
      write_pulse_to_buffer(buf, frame);
    }

    I2sManager::instance().dmaConsumeBlock();
  }

  printf("  DMA rounds: %u (expected 10)\n",
         I2sManager::instance().callbackCount());
  printf("  Total pulses detected: %u (expected 70)\n",
         s_analyzer.pulseCount());

  bool correct = (I2sManager::instance().callbackCount() == 10) &&
                 (s_analyzer.pulseCount() == 70);
  test_result("DMA 10 rounds 7 pulses", correct);
}

static void test_dma_buffer_content() {
  printf("Running: DMA buffer content verification\n");

  I2sManager::instance().init(32, 33, -1);
  uint8_t* buf = I2sManager::instance().blockBuf(0);

  write_pulse_to_buffer(buf, 0);
  write_pulse_to_buffer(buf, 50);
  write_pulse_to_buffer(buf, 100);
  write_pulse_to_buffer(buf, 150);
  write_pulse_to_buffer(buf, 200);
  write_pulse_to_buffer(buf, 249);

  uint32_t pulse_frames = 0;
  for (uint16_t f = 0; f < I2S_FRAMES_PER_BLOCK; f++) {
    if (buf[f * I2S_BYTES_PER_FRAME] & 0x80) {
      pulse_frames++;
    }
  }

  printf("  Wrote 6 pulse frames\n");
  printf("  Detected pulse frames: %u\n", pulse_frames);

  test_result("DMA buffer content", pulse_frames == 6);
}

static void test_dma_total_ticks_per_block() {
  printf("Running: DMA total ticks per block\n");

  s_analyzer.reset();
  s_analyzer.ticks_per_bit = 2;  // 8MHz BCLK = 2 ticks at 16MHz per bit
  I2sManager::instance().init(32, 33, -1);
  I2sManager::instance().registerDmaCallback(dmaCallback);

  uint8_t* buf =
      I2sManager::instance().blockBuf(I2sManager::instance().dmaBlock());
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  I2sManager::instance().dmaConsumeBlock();

  uint32_t expected_ticks = I2S_FRAMES_PER_BLOCK * I2S_TICKS_PER_FRAME;
  printf("  Ticks per block: %u (expected %u)\n", s_analyzer.total_ticks,
         expected_ticks);

  test_result("DMA total ticks per block",
              s_analyzer.total_ticks == expected_ticks);
}

// ---------------------------------------------------------------------------
// Part 2: Driver fill function tests
// ---------------------------------------------------------------------------

#define IRAM_ATTR
#include "pd_esp32/i2s_fill.cpp"

#define NO_PIN 255

static uint32_t countPulsesInBuffer(const uint8_t* buf) {
  uint32_t count = 0;
  for (uint16_t f = 0; f < I2S_FRAMES_PER_BLOCK; f++) {
    if (buf[f * I2S_BYTES_PER_FRAME + 0] == 0xFF &&
        buf[f * I2S_BYTES_PER_FRAME + 1] == 0xFF) {
      count++;
    }
  }
  return count;
}

static void add_command(StepperQueueBase* q, uint8_t steps, uint16_t ticks) {
  uint8_t idx = q->next_write_idx & QUEUE_LEN_MASK;
  q->entry[idx].steps = steps;
  q->entry[idx].ticks = ticks;
  q->entry[idx].toggle_dir = 0;
  q->next_write_idx++;
}

static void test_fill_single_step() {
  printf("Running: Fill single step at 256 ticks\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 1, 256);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  i2s_fill_buffer(&q, buf, &state);

  uint32_t pulses = countPulsesInBuffer(buf);
  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Pulses: %u (expected 1)\n", pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("Fill single step", pulses == 1 && consumed);
}

static void test_fill_multi_step() {
  printf("Running: Fill 5 steps @ 128 ticks each\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 5, 128);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  i2s_fill_buffer(&q, buf, &state);

  uint32_t pulses = countPulsesInBuffer(buf);
  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Pulses: %u (expected 5)\n", pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("Fill multi step", pulses == 5 && consumed);
}

static void test_fill_pause() {
  printf("Running: Fill pause command (steps=0, ticks=5000)\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 0, 5000);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  i2s_fill_buffer(&q, buf, &state);

  uint32_t pulses = countPulsesInBuffer(buf);
  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Pulses: %u (expected 0)\n", pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("Fill pause", pulses == 0 && consumed);
}

static void test_fill_step_pause_step() {
  printf("Running: Fill step + pause + step\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 1, 256);
  add_command(&q, 0, 512);
  add_command(&q, 1, 256);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  i2s_fill_buffer(&q, buf, &state);

  uint32_t pulses = countPulsesInBuffer(buf);
  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Pulses: %u (expected 2)\n", pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  if (state.pulse_count >= 2) {
    uint16_t tick0 = state.pulse_positions[0] * I2S_TICKS_PER_FRAME;
    uint16_t tick1 = state.pulse_positions[1] * I2S_TICKS_PER_FRAME;
    printf("  Pulse 0 at tick: %u\n", tick0);
    printf("  Pulse 1 at tick: %u\n", tick1);
    printf("  Tick diff: %u (expected %u)\n", tick1 - tick0, 256 + 512 + 256);
  }

  test_result("Fill step+pause+step", pulses == 2 && consumed);
}

static void test_fill_block_boundary() {
  printf("Running: Fill across block boundary (16000 ticks)\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 1, 16000);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  i2s_fill_buffer(&q, buf, &state);

  uint32_t pulses = countPulsesInBuffer(buf);
  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Pulses: %u (expected 0, pulse doesn't fit)\n", pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");
  printf("  tick_pos after: %u (expected 0 for next block)\n", state.tick_pos);

  test_result("Fill block boundary",
              pulses == 0 && !consumed && state.tick_pos == 0);
}

static void test_fill_two_blocks() {
  printf("Running: Fill across two blocks (16000 ticks pulse)\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 1, 16000);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  memset(buf, 0, I2S_BYTES_PER_BLOCK);
  i2s_fill_buffer(&q, buf, &state);

  uint32_t pulses1 = countPulsesInBuffer(buf);
  uint16_t tick_pos1 = state.tick_pos;
  printf("  Block 1: pulses=%u, tick_pos=%u\n", pulses1, tick_pos1);

  memset(buf, 0, I2S_BYTES_PER_BLOCK);
  i2s_fill_buffer(&q, buf, &state);

  uint32_t pulses2 = countPulsesInBuffer(buf);
  bool consumed = (q.read_idx == q.next_write_idx);
  printf("  Block 2: pulses=%u, consumed=%s\n", pulses2,
         consumed ? "yes" : "no");

  test_result("Fill two blocks",
              pulses1 == 0 && tick_pos1 == 0 && pulses2 == 1 && consumed);
}

static void test_fill_partial_steps() {
  printf("Running: Fill partial steps (100 steps @ 200 ticks)\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 100, 200);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  memset(buf, 0, I2S_BYTES_PER_BLOCK);
  i2s_fill_buffer(&q, buf, &state);

  uint32_t pulses1 = countPulsesInBuffer(buf);
  uint8_t remaining = q.entry[q.read_idx & QUEUE_LEN_MASK].steps;
  uint16_t block_ticks = I2S_FRAMES_PER_BLOCK * I2S_TICKS_PER_FRAME;
  uint8_t expected_in_block = block_ticks / 200;
  if (expected_in_block * 200 >= block_ticks) expected_in_block--;
  uint8_t expected_remaining = 100 - expected_in_block;
  printf("  Block 1: pulses=%u (expected %u)\n", pulses1, expected_in_block);
  printf("  Remaining steps: %u (expected %u)\n", remaining,
         expected_remaining);

  memset(buf, 0, I2S_BYTES_PER_BLOCK);
  i2s_fill_buffer(&q, buf, &state);

  uint32_t pulses2 = countPulsesInBuffer(buf);
  bool consumed = (q.read_idx == q.next_write_idx);
  printf("  Block 2: pulses=%u (expected %u), consumed=%s\n", pulses2,
         expected_remaining, consumed ? "yes" : "no");

  test_result("Fill partial steps",
              pulses1 == expected_in_block && remaining == expected_remaining &&
                  pulses2 == expected_remaining && consumed);
}

static void test_fill_carry_ticks() {
  printf("Running: Fill carry ticks (pause that spans block)\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 1, 1000);
  add_command(&q, 0, 20000);
  add_command(&q, 1, 1000);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  memset(buf, 0, I2S_BYTES_PER_BLOCK);
  i2s_fill_buffer(&q, buf, &state);

  uint32_t pulses1 = countPulsesInBuffer(buf);
  printf("  Block 1: pulses=%u (expected 1)\n", pulses1);

  memset(buf, 0, I2S_BYTES_PER_BLOCK);
  i2s_fill_buffer(&q, buf, &state);

  uint32_t pulses2 = countPulsesInBuffer(buf);
  printf("  Block 2: pulses=%u (expected 1, pause ends then step 2)\n",
         pulses2);

  memset(buf, 0, I2S_BYTES_PER_BLOCK);
  i2s_fill_buffer(&q, buf, &state);

  uint32_t pulses3 = countPulsesInBuffer(buf);
  bool consumed = (q.read_idx == q.next_write_idx);
  printf("  Block 3: pulses=%u (expected 0), consumed=%s\n", pulses3,
         consumed ? "yes" : "no");

  test_result("Fill carry ticks",
              pulses1 == 1 && pulses2 == 1 && pulses3 == 0 && consumed);
}

static void test_fill_empty_queue() {
  printf("Running: Fill with empty queue\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  i2s_fill_buffer(&q, buf, &state);

  uint32_t pulses = countPulsesInBuffer(buf);
  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Pulses: %u (expected 0)\n", pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("Fill empty queue", pulses == 0 && consumed);
}

static void test_fill_min_speed() {
  printf("Running: Fill at minimum speed (128 ticks)\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 3, I2S_MIN_SPEED_TICKS);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  i2s_fill_buffer(&q, buf, &state);

  uint32_t pulses = countPulsesInBuffer(buf);
  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Pulses: %u (expected 3)\n", pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("Fill min speed", pulses == 3 && consumed);
}

static void test_fill_max_steps() {
  printf("Running: Fill max steps in one command (255 steps)\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 255, 128);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  memset(buf, 0, I2S_BYTES_PER_BLOCK);
  i2s_fill_buffer(&q, buf, &state);
  uint32_t pulses1 = countPulsesInBuffer(buf);
  uint8_t remaining1 = q.entry[q.read_idx & QUEUE_LEN_MASK].steps;
  printf("  Block 1: pulses=%u, remaining=%u\n", pulses1, remaining1);

  memset(buf, 0, I2S_BYTES_PER_BLOCK);
  i2s_fill_buffer(&q, buf, &state);
  uint32_t pulses2 = countPulsesInBuffer(buf);
  uint8_t remaining2 = q.entry[q.read_idx & QUEUE_LEN_MASK].steps;
  printf("  Block 2: pulses=%u, remaining=%u\n", pulses2, remaining2);

  uint32_t total = pulses1 + pulses2;
  while (q.read_idx != q.next_write_idx) {
    memset(buf, 0, I2S_BYTES_PER_BLOCK);
    i2s_fill_buffer(&q, buf, &state);
    total += countPulsesInBuffer(buf);
  }
  printf("  Total pulses: %u (expected 255)\n", total);

  test_result("Fill max steps", total == 255);
}

static void test_fill_long_pause() {
  printf("Running: Fill long pause spanning multiple blocks\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 0, 50000);
  add_command(&q, 1, 256);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  uint32_t total_pulses = 0;
  int blocks = 0;
  while (q.read_idx != q.next_write_idx && blocks < 10) {
    memset(buf, 0, I2S_BYTES_PER_BLOCK);
    i2s_fill_buffer(&q, buf, &state);
    total_pulses += countPulsesInBuffer(buf);
    blocks++;
  }

  printf("  Blocks processed: %d\n", blocks);
  printf("  Total pulses: %u (expected 1)\n", total_pulses);
  bool consumed = (q.read_idx == q.next_write_idx);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("Fill long pause", blocks >= 3 && total_pulses == 1 && consumed);
}

static void test_fill_pulse_at_exact_boundary() {
  printf("Running: Fill pulse at exact block boundary\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];
  uint16_t block_ticks = I2S_FRAMES_PER_BLOCK * I2S_TICKS_PER_FRAME;

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 1, block_ticks);
  add_command(&q, 1, 256);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  memset(buf, 0, I2S_BYTES_PER_BLOCK);
  i2s_fill_buffer(&q, buf, &state);
  uint32_t pulses1 = countPulsesInBuffer(buf);
  printf("  Block 1: pulses=%u (expected 0, pulse at boundary)\n", pulses1);

  memset(buf, 0, I2S_BYTES_PER_BLOCK);
  i2s_fill_buffer(&q, buf, &state);
  uint32_t pulses2 = countPulsesInBuffer(buf);
  printf("  Block 2: pulses=%u (expected 2, pending + next)\n", pulses2);

  bool consumed = (q.read_idx == q.next_write_idx);
  test_result("Fill pulse at exact boundary",
              pulses1 == 0 && pulses2 == 2 && consumed);
}

static void test_fill_multiple_commands() {
  printf("Running: Fill multiple mixed commands\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 2, 200);
  add_command(&q, 0, 1000);
  add_command(&q, 3, 300);
  add_command(&q, 0, 500);
  add_command(&q, 1, 150);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  memset(buf, 0, I2S_BYTES_PER_BLOCK);
  i2s_fill_buffer(&q, buf, &state);

  uint32_t pulses = countPulsesInBuffer(buf);
  bool consumed = (q.read_idx == q.next_write_idx);

  printf("  Pulses: %u (expected 6 = 2+3+1)\n", pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("Fill multiple commands", pulses == 6 && consumed);
}

static void test_fill_consecutive_partial() {
  printf("Running: Fill consecutive partial steps across blocks\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 200, 150);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  uint32_t total_pulses = 0;
  int blocks = 0;
  while (q.read_idx != q.next_write_idx && blocks < 10) {
    memset(buf, 0, I2S_BYTES_PER_BLOCK);
    i2s_fill_buffer(&q, buf, &state);
    total_pulses += countPulsesInBuffer(buf);
    blocks++;
  }

  bool consumed = (q.read_idx == q.next_write_idx);
  printf("  Blocks: %d\n", blocks);
  printf("  Total pulses: %u (expected 200)\n", total_pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("Fill consecutive partial",
              total_pulses == 200 && consumed && blocks >= 2);
}

static void test_fill_max_ticks_single_step() {
  printf("Running: Fill two steps at max ticks (65535)\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 2, 65535);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  uint32_t total_pulses = 0;
  int blocks = 0;

  while (q.read_idx != q.next_write_idx && blocks < 20) {
    memset(buf, 0, I2S_BYTES_PER_BLOCK);
    i2s_fill_buffer(&q, buf, &state);
    total_pulses += countPulsesInBuffer(buf);
    blocks++;
  }

  bool consumed = (q.read_idx == q.next_write_idx);
  printf("  Blocks: %d\n", blocks);
  printf("  Total pulses: %u (expected 2)\n", total_pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("Fill max ticks single step", total_pulses == 2 && consumed);
}

static void test_fill_max_ticks_max_steps() {
  printf("Running: Fill 4 steps at max ticks (65535 each)\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 4, 65535);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  uint32_t total_pulses = 0;
  int blocks = 0;
  while (q.read_idx != q.next_write_idx && blocks < 50) {
    memset(buf, 0, I2S_BYTES_PER_BLOCK);
    i2s_fill_buffer(&q, buf, &state);
    total_pulses += countPulsesInBuffer(buf);
    blocks++;
  }

  bool consumed = (q.read_idx == q.next_write_idx);
  printf("  Blocks: %d\n", blocks);
  printf("  Total pulses: %u (expected 4)\n", total_pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("Fill max ticks max steps", total_pulses == 4 && consumed);
}

static void test_fill_max_pause() {
  printf("Running: Fill max pause (65535 ticks) between two steps\n");

  uint8_t buf[I2S_BYTES_PER_BLOCK];

  StepperQueueBase q;
  q.read_idx = 0;
  q.next_write_idx = 0;
  q.dirPin = NO_PIN;

  add_command(&q, 1, 1000);
  add_command(&q, 0, 65535);
  add_command(&q, 1, 1000);

  struct i2s_fill_state state = {0, 0, 0, {0}, 0};

  uint32_t total_pulses = 0;
  int blocks = 0;
  uint16_t block_ticks = I2S_FRAMES_PER_BLOCK * I2S_TICKS_PER_FRAME;
  int min_expected_blocks =
      (1000 + 65535 + 1000 + block_ticks - 1) / block_ticks;

  while (q.read_idx != q.next_write_idx && blocks < 20) {
    memset(buf, 0, I2S_BYTES_PER_BLOCK);
    i2s_fill_buffer(&q, buf, &state);
    total_pulses += countPulsesInBuffer(buf);
    blocks++;
  }

  bool consumed = (q.read_idx == q.next_write_idx);
  printf("  Blocks: %d (expected >= %d)\n", blocks, min_expected_blocks);
  printf("  Total pulses: %u (expected 2)\n", total_pulses);
  printf("  Queue consumed: %s\n", consumed ? "yes" : "no");

  test_result("Fill max pause",
              total_pulses == 2 && consumed && blocks >= min_expected_blocks);
}

void basic_test() {
  puts("=== I2S Low-Level DMA Tests ===\n");

  puts("=== DMA Callback Tests ===");
  test_dma_callback_invoked();

  puts("\n=== DMA Block Management ===");
  test_dma_block_rotation();
  test_dma_write_block_advance();
  test_dma_buffer_clearing();
  test_dma_block_availability();

  puts("\n=== Bit Stream Analysis ===");
  test_bit_stream_single_pulse();
  test_bit_stream_multiple_pulses();
  test_dma_total_ticks_per_block();

  puts("\n=== DMA Round Tests ===");
  test_dma_10_rounds_7_pulses();
  test_dma_buffer_content();

  puts("\n=== Part 2: Driver Fill Function ===");
  test_fill_single_step();
  test_fill_multi_step();
  test_fill_pause();
  test_fill_step_pause_step();
  test_fill_block_boundary();
  test_fill_two_blocks();
  test_fill_partial_steps();
  test_fill_carry_ticks();

  puts("\n=== Part 2: Edge Cases ===");
  test_fill_empty_queue();
  test_fill_min_speed();
  test_fill_max_steps();
  test_fill_long_pause();
  test_fill_pulse_at_exact_boundary();
  test_fill_multiple_commands();
  test_fill_consecutive_partial();
  test_fill_max_ticks_single_step();
  test_fill_max_ticks_max_steps();
  test_fill_max_pause();

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
