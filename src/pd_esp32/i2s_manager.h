#ifndef PD_ESP32_I2S_MANAGER_H
#define PD_ESP32_I2S_MANAGER_H
#if defined(SUPPORT_ESP32_I2S)

#include <stdint.h>
#include "i2s_constants.h"
#include <driver/i2s_std.h>

class I2sManager {
 public:
  static I2sManager& instance();

  bool init(int data_pin, int bclk_pin, int ws_pin);
  bool isInitialized() const { return _initialized; }

  uint8_t* blockBuf(uint8_t block) { return _bufs[block % I2S_BLOCK_COUNT]; }

  uint8_t dmaBlock() const { return _dma_block; }

  bool startDma();
  bool isDmaStarted() const { return _dma_started; }
  void handleTxDone();
  void queueBlockToDma(uint8_t block);

  void setPulseWidthBits(uint8_t bits) { _pulse_width_bits = bits; }
  uint8_t pulseWidthBits() const { return _pulse_width_bits; }

  volatile uint32_t _callback_count = 0;

  i2s_chan_handle_t channel() const { return _chan; }

 private:
  I2sManager() : _pulse_width_bits(32) {}
  bool _initialized = false;
  bool _dma_started = false;
  i2s_chan_handle_t _chan = nullptr;
  uint8_t _bufs[I2S_BLOCK_COUNT][I2S_BYTES_PER_BLOCK];
  uint8_t _dma_block = 0;
  uint8_t _pulse_width_bits;
};

#endif  // SUPPORT_ESP32_I2S
#endif  // PD_ESP32_I2S_MANAGER_H
