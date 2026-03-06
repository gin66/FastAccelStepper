#ifndef I2S_TEST_COMMON_H
#define I2S_TEST_COMMON_H

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SUPPORT_ESP32_I2S
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
    _callback_count = 0;
    return true;
  }

  bool isInitialized() const { return _initialized; }

  uint8_t* blockBuf(uint8_t block) { return _bufs[block % I2S_BLOCK_COUNT]; }

  uint8_t dmaBlock() const { return _dma_block; }

  void registerDmaCallback(i2s_dma_callback_t cb) { _dma_callback = cb; }

  void dmaConsumeBlock() {
    if (_dma_callback) {
      _dma_callback(_dma_block);
      _callback_count++;
    }
    _dma_block = (_dma_block + 1) % I2S_BLOCK_COUNT;
  }

  uint32_t callbackCount() const { return _callback_count; }

  I2sManager()
      : _initialized(false),
        _dma_block(0),
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
  uint32_t _callback_count;
  i2s_dma_callback_t _dma_callback;
};

static I2sManager s_i2s_mgr;
inline I2sManager& I2sManager::instance() { return s_i2s_mgr; }

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
// Pulse analysis helpers
// ---------------------------------------------------------------------------

static uint32_t countPulsesInBuffer(const uint8_t* buf) {
  uint32_t count = 0;
  bool last_bit = false;
  uint16_t total_bits = I2S_FRAMES_PER_BLOCK * I2S_BITS_PER_FRAME;
  for (uint16_t bit_pos = 0; bit_pos < total_bits; bit_pos++) {
    uint16_t byte_pos = (bit_pos >> 3) ^ 1;
    uint8_t bit_offset = bit_pos & 7;
    bool current = (buf[byte_pos] >> (7 - bit_offset)) & 1;
    if (current && !last_bit) {
      count++;
    }
    last_bit = current;
  }
  return count;
}

static uint32_t detectPulsePositions(const uint8_t* buf, uint32_t buf_size,
                                     uint32_t* positions) {
  uint32_t count = 0;
  uint8_t prev_bit = 0;
  uint32_t total_bits = buf_size * 8;

  for (uint32_t bit_pos = 0; bit_pos < total_bits; bit_pos++) {
    uint32_t byte_pos = (bit_pos >> 3) ^ 1;
    uint8_t bit_offset = bit_pos & 7;
    uint8_t curr_bit = (buf[byte_pos] >> (7 - bit_offset)) & 1;
    if (curr_bit == 1 && prev_bit == 0) {
      positions[count++] = bit_pos;
    }
    prev_bit = curr_bit;
  }
  return count;
}

#endif  // I2S_TEST_COMMON_H
