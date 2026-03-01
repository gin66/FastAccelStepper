#pragma once
#if defined(SUPPORT_ESP32_I2S)

#include <driver/i2s_std.h>
#include <stdint.h>

// I2S timing: BCLK=8MHz, stereo 8-bit
// Frame = L-byte (8 BCLK) + R-byte (8 BCLK) = 2µs = 32 ticks at 16MHz
#define I2S_SAMPLE_RATE_HZ 500000UL
#define I2S_TICKS_PER_FRAME 32
#define I2S_BYTES_PER_FRAME 2
#define I2S_TASK_MS 4
#define I2S_FRAMES_PER_TASK 2000
#define I2S_BYTES_PER_TASK (I2S_FRAMES_PER_TASK * I2S_BYTES_PER_FRAME)
// Min step period: 2 frames = 4µs = 64 ticks
#define I2S_MIN_SPEED_TICKS 64
// DMA: 16 descriptors x 500 frames = 8000 frames = 16ms total buffer
#define I2S_DMA_DESC_NUM 16
#define I2S_DMA_FRAME_NUM 500
// Extra task cycles to stream after queue empties (ensures DMA flushes)
#define I2S_DRAIN_TASKS 4

class I2sManager {
 public:
  static I2sManager& instance();

  bool init(int data_pin, int bclk_pin, int ws_pin);
  bool isInitialized() const { return _initialized; }

  uint8_t* workBuf() { return _work_buf; }
  void clearWorkBuf();
  bool flush();

 private:
  I2sManager() {}
  bool _initialized = false;
  i2s_chan_handle_t _chan = nullptr;
  uint8_t _work_buf[I2S_BYTES_PER_TASK];
};

#endif  // SUPPORT_ESP32_I2S
