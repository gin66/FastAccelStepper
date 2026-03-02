#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_I2S)

#include <string.h>
#include <Arduino.h>
#include "pd_esp32/i2s_manager.h"

static IRAM_ATTR bool i2s_tx_done_callback(i2s_chan_handle_t handle,
                                           i2s_event_data_t* event,
                                           void* user_ctx) {
  I2sManager* mgr = static_cast<I2sManager*>(user_ctx);
  mgr->handleTxDone();
  return false;
}

I2sManager& I2sManager::instance() {
  static I2sManager mgr;
  return mgr;
}

bool I2sManager::init(int data_pin, int bclk_pin, int ws_pin) {
  if (_initialized) {
    return true;
  }

  // ESP32 I2S 16-bit stereo mode - VERIFIED CONFIGURATION (measured on scope):
  //
  // Buffer layout (16-bit slots, all bytes transmitted):
  //   Bytes 0-1: L channel (16 bits MSB-first: byte 0 = MSB, byte 1 = LSB)
  //   Bytes 2-3: R channel (16 bits MSB-first: byte 2 = MSB, byte 3 = LSB)
  //
  // Timing (measured):
  //   BCLK: 8MHz (32 bits × 250kHz sample rate)
  //   Frame: 32 bits = 4µs (L: 16 bits + R: 16 bits)
  //   Block: 250 frames × 4µs = 1ms (I2S_BYTES_PER_BLOCK = 1000)
  //
  // Test patterns verified:
  //   0x55 → 4MHz output (01010101 at 8MHz bit rate)
  //   0x0F → 1MHz output (00001111 = 4 HIGH + 4 LOW bits)
  //   0xFF → DC HIGH
  //
  // For stepper pulse generation:
  // - Single stepper: Fill all 16 bits of L channel (or both L+R same pattern)
  // - Multi-stepper: L channel for stepper 1, R channel for stepper 2
  // - Each bit = 125ns (1/8MHz), provides fine-grained pulse timing

  Serial.printf("I2S init: data=%d, bclk=%d, ws=%d\n", data_pin, bclk_pin,
                ws_pin);

  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = I2S_DMA_DESC_NUM;
  chan_cfg.dma_frame_num = I2S_DMA_FRAME_NUM;
  chan_cfg.auto_clear = true;

  esp_err_t rc = i2s_new_channel(&chan_cfg, &_chan, NULL);
  if (rc != ESP_OK) {
    return false;
  }

  i2s_std_config_t std_cfg = {
      .clk_cfg = {.sample_rate_hz = I2S_SAMPLE_RATE_HZ,
                  .clk_src = I2S_CLK_SRC_DEFAULT,
                  .mclk_multiple = I2S_MCLK_MULTIPLE_128},
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                  I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {.mclk = I2S_GPIO_UNUSED,
                   .bclk = (gpio_num_t)bclk_pin,
                   .ws = I2S_GPIO_UNUSED,
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

  i2s_event_callbacks_t cbs = {};
  cbs.on_sent = i2s_tx_done_callback;
  rc = i2s_channel_register_event_callback(_chan, &cbs, this);
  if (rc != ESP_OK) {
    i2s_del_channel(_chan);
    _chan = nullptr;
    return false;
  }

  for (int i = 0; i < I2S_BLOCK_COUNT; i++) {
    memset(_bufs[i], 0, I2S_BYTES_PER_BLOCK);
  }
  _dma_block = 0;
  _write_block = 0;
  _initialized = true;
  return true;
}

void I2sManager::clearBlock(uint8_t block) {
  if (block < I2S_BLOCK_COUNT) {
    memset(_bufs[block], 0x00, I2S_BYTES_PER_BLOCK);
  }
}

bool I2sManager::flushBlock(uint8_t block) {
  if (!_initialized || block >= I2S_BLOCK_COUNT) {
    return false;
  }
  size_t written = 0;
  esp_err_t rc =
      i2s_channel_write(_chan, _bufs[block], I2S_BYTES_PER_BLOCK, &written, 0);
  return (rc == ESP_OK) && (written == I2S_BYTES_PER_BLOCK);
}

void I2sManager::registerDmaCallback(i2s_dma_callback_t cb, void* user_data) {
  _dma_callback = cb;
  _dma_callback_data = user_data;
}

bool I2sManager::startDma() {
  if (!_initialized || _dma_started) {
    return _dma_started;
  }
  Serial.printf("I2S startDma: enabling channel\n");
  if (i2s_channel_enable(_chan) != ESP_OK) {
    Serial.printf("I2S startDma: channel enable FAILED\n");
    return false;
  }
  _dma_started = true;

  vTaskDelay(1);

  Serial.printf("I2S startDma: writing %d blocks with timeout=100\n",
                I2S_BLOCK_COUNT);
  for (int i = 0; i < I2S_BLOCK_COUNT; i++) {
    size_t written = 0;
    esp_err_t rc =
        i2s_channel_write(_chan, _bufs[i], I2S_BYTES_PER_BLOCK, &written, 0);
    Serial.printf("I2S startDma: block %d written=%d rc=%d\n", i, (int)written,
                  rc);
  }
  Serial.printf("I2S startDma: filling remaining DMA buffer\n");
  int total_written = 0;
  for (int j = 0; j < 100; j++) {
    for (int i = 0; i < I2S_BLOCK_COUNT; i++) {
      size_t written = 0;
      esp_err_t rc =
          i2s_channel_write(_chan, _bufs[i], I2S_BYTES_PER_BLOCK, &written, 0);
      if (rc == ESP_OK && written > 0) {
        total_written += written;
      }
    }
  }
  Serial.printf("I2S startDma: filled %d bytes\n", total_written);
  return true;
}

void IRAM_ATTR I2sManager::handleTxDone() {
  _callback_count++;
  advanceDmaBlock();
  if (_dma_callback) {
    _dma_callback(_dma_callback_data);
  }
}

#endif  // SUPPORT_ESP32_I2S
