#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_I2S)

#include <string.h>
#include "pd_esp32/i2s_manager.h"

I2sManager& I2sManager::instance() {
  static I2sManager mgr;
  return mgr;
}

bool I2sManager::init(int data_pin, int bclk_pin, int ws_pin) {
  if (_initialized) {
    return true;
  }

  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = I2S_DMA_DESC_NUM;
  chan_cfg.dma_frame_num = I2S_DMA_FRAME_NUM;

  esp_err_t rc = i2s_new_channel(&chan_cfg, &_chan, NULL);
  if (rc != ESP_OK) {
    return false;
  }

  i2s_std_config_t std_cfg = {
      .clk_cfg = {.sample_rate_hz = I2S_SAMPLE_RATE_HZ,
                  .clk_src = I2S_CLK_SRC_DEFAULT,
                  .mclk_multiple = I2S_MCLK_MULTIPLE_128},
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_8BIT,
                                                  I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {.mclk = I2S_GPIO_UNUSED,
                   .bclk = (gpio_num_t)bclk_pin,
                   .ws = (gpio_num_t)ws_pin,
                   .dout = (gpio_num_t)data_pin,
                   .din = I2S_GPIO_UNUSED,
                   .invert_flags =
                       {
                           .mclk_inv = false,
                           .bclk_inv = false,
                           .ws_inv = false,
                       }},
  };

  rc = i2s_channel_init_std_mode(_chan, &std_cfg);
  if (rc != ESP_OK) {
    i2s_del_channel(_chan);
    _chan = nullptr;
    return false;
  }

  rc = i2s_channel_enable(_chan);
  if (rc != ESP_OK) {
    i2s_del_channel(_chan);
    _chan = nullptr;
    return false;
  }

  for (int i = 0; i < I2S_BLOCK_COUNT; i++) {
    memset(_bufs[i], 0, I2S_BYTES_PER_BLOCK);
    _block_prepared[i] = false;
  }
  _dma_block = 0;
  _prepared_block = 1;
  _write_block = 2;
  _initialized = true;
  return true;
}

void I2sManager::clearBlock(uint8_t block) {
  if (block < I2S_BLOCK_COUNT) {
    memset(_bufs[block], 0x00, I2S_BYTES_PER_BLOCK);
    _block_prepared[block] = false;
  }
}

bool I2sManager::flushBlock(uint8_t block) {
  if (!_initialized || block >= I2S_BLOCK_COUNT) {
    return false;
  }
  size_t written = 0;
  esp_err_t rc = i2s_channel_write(_chan, _bufs[block], I2S_BYTES_PER_BLOCK,
                                   &written, portMAX_DELAY);
  return (rc == ESP_OK) && (written == I2S_BYTES_PER_BLOCK);
}

void I2sManager::markWriteBlockPrepared() {
  _block_prepared[_write_block] = true;
  _prepared_block = _write_block;
  _write_block = (_write_block + 1) % I2S_BLOCK_COUNT;
}

void I2sManager::advanceDmaBlock() {
  _block_prepared[_dma_block] = false;
  if (_block_prepared[_prepared_block]) {
    _dma_block = _prepared_block;
    _prepared_block = (_prepared_block + 1) % I2S_BLOCK_COUNT;
  }
}

#endif  // SUPPORT_ESP32_I2S
