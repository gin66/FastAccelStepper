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
  void clearBlock(uint8_t block);
  bool flushBlock(uint8_t block);

  uint8_t dmaBlock() const { return _dma_block; }
  uint8_t preparedBlock() const { return _prepared_block; }
  uint8_t writeBlock() const { return _write_block; }
  bool isBlockPrepared(uint8_t block) const {
    return _block_prepared[block % I2S_BLOCK_COUNT];
  }

  void markWriteBlockPrepared();
  void advanceDmaBlock();

 private:
  I2sManager() {}
  bool _initialized = false;
  i2s_chan_handle_t _chan = nullptr;
  uint8_t _bufs[I2S_BLOCK_COUNT][I2S_BYTES_PER_BLOCK];
  bool _block_prepared[I2S_BLOCK_COUNT] = {false, false, false};
  uint8_t _dma_block = 0;
  uint8_t _prepared_block = 1;
  uint8_t _write_block = 2;
};

#endif  // SUPPORT_ESP32_I2S
#endif  // PD_ESP32_I2S_MANAGER_H
