#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_I2S)

#include <string.h>
#include <Arduino.h>
#include "pd_esp32/i2s_manager.h"

static IRAM_ATTR bool i2s_tx_done_callback(i2s_chan_handle_t handle,
                                           i2s_event_data_t* event,
                                           void* user_ctx) {
  I2sManager* mgr = static_cast<I2sManager*>(user_ctx);
  uint8_t* buf = (uint8_t*)event->dma_buf;
  mgr->handleTxDone(buf);
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

  Serial.printf("I2S init: data=%d, bclk=%d, ws=%d\n", data_pin, bclk_pin,
                ws_pin);

  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = I2S_DMA_DESC_NUM;
  chan_cfg.dma_frame_num = I2S_DMA_FRAME_NUM;
  chan_cfg.auto_clear = false;

  esp_err_t rc = i2s_new_channel(&chan_cfg, &_chan, NULL);
  if (rc != ESP_OK) {
    return false;
  }

  i2s_std_config_t std_cfg = {
      .clk_cfg = {.sample_rate_hz = I2S_SAMPLE_RATE_HZ,
                  .clk_src = I2S_CLK_SRC_DEFAULT,
                  .mclk_multiple = I2S_MCLK_MULTIPLE_128},  // lowest multiplier
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
  _initialized = true;
  return true;
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

  Serial.printf("I2S startDma: DMA started\n");
  return true;
}

void IRAM_ATTR I2sManager::handleTxDone(uint8_t* buf) {
  _callback_count++;

  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  for (uint8_t i = 0; i < QUEUES_I2S; i++) {
    uint8_t queue_idx = QUEUES_MCPWM_PCNT + QUEUES_RMT + i;
    FAS_QUEUE(queue_idx).fill_i2s_buffer(buf);
  }
}

#endif  // SUPPORT_ESP32_I2S
