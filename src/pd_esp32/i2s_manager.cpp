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

I2sManager* I2sManager::create(gpio_num_t data_pin, gpio_num_t bclk_pin,
                               gpio_num_t ws_pin) {
  Serial.printf("I2S create: data=%d, bclk=%d, ws=%d\n", data_pin, bclk_pin,
                ws_pin);

  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = I2S_DMA_DESC_NUM;
  chan_cfg.dma_frame_num = I2S_DMA_FRAME_NUM;
  chan_cfg.auto_clear = false;

  i2s_chan_handle_t chan = nullptr;
  esp_err_t rc = i2s_new_channel(&chan_cfg, &chan, NULL);
  if (rc != ESP_OK) {
    return nullptr;
  }

  i2s_std_config_t std_cfg = {
      .clk_cfg = {.sample_rate_hz = I2S_SAMPLE_RATE_HZ,
                  .clk_src = I2S_CLK_SRC_DEFAULT,
                  .mclk_multiple = I2S_MCLK_MULTIPLE_128},
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                  I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {.mclk = I2S_GPIO_UNUSED,
                   .bclk = bclk_pin,
                   .ws = I2S_GPIO_UNUSED,
                   .dout = data_pin,
                   .din = I2S_GPIO_UNUSED,
                   .invert_flags =
                       {
                           .mclk_inv = false,
                           .bclk_inv = false,
                           .ws_inv = false,
                       }},
  };

  rc = i2s_channel_init_std_mode(chan, &std_cfg);
  if (rc != ESP_OK) {
    i2s_del_channel(chan);
    return nullptr;
  }

  I2sManager* mgr = new I2sManager();
  mgr->_chan = chan;
  if (!mgr->init()) {
    i2s_del_channel(mgr->_chan);
    mgr->_chan = nullptr;
    delete mgr;
    return nullptr;
  }

  Serial.printf("I2S startDma: enabling channel\n");
  if (i2s_channel_enable(mgr->_chan) != ESP_OK) {
    Serial.printf("I2S startDma: channel enable FAILED\n");
    i2s_del_channel(mgr->_chan);
    mgr->_chan = nullptr;
    delete mgr;
    return nullptr;
  }
  
  return mgr;
}

bool I2sManager::init() {
  i2s_event_callbacks_t cbs = {};
  cbs.on_sent = i2s_tx_done_callback;
  esp_err_t rc = i2s_channel_register_event_callback(_chan, &cbs, this);
  if (rc != ESP_OK) {
    return false;
  }
  return true;
}

void IRAM_ATTR I2sManager::handleTxDone(uint8_t* buf) {
  _callback_count++;

        pinMode(14, OUTPUT);
        digitalWrite(14, HIGH);
  // clear the buffer for the next round
  // around 18us
  memset(buf, 0, I2S_BYTES_PER_BLOCK);

  for (uint8_t i = 0; i < NUM_QUEUES; i++) {
    StepperQueue* q = fas_queue[i];
    if (q) {
      if (q->use_i2s) {
        if (q->i2s_mgr == this) {
          pinMode(33, OUTPUT);
          digitalWrite(33, HIGH);
          // 138us @ 10us speed
          q->fill_i2s_buffer(buf);
          digitalWrite(33, LOW);
        }
      }
    }
  }
        digitalWrite(14, LOW);
}

#endif  // SUPPORT_ESP32_I2S
