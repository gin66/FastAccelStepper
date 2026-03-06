#ifndef PD_ESP32_I2S_MANAGER_H
#define PD_ESP32_I2S_MANAGER_H
#if defined(SUPPORT_ESP32_I2S)

#include <stdint.h>
#include "i2s_constants.h"
#include <driver/i2s_std.h>

class I2sManager {
 public:
  static I2sManager* create(gpio_num_t data_pin, gpio_num_t bclk_pin,
                            gpio_num_t ws_pin);

  void handleTxDone(uint8_t* buf);

  bool _is_mux = false;

  volatile uint32_t _callback_count = 0;

 private:
  I2sManager() {}
  bool init();
  void init_mux_buffer(uint8_t* buf);
  i2s_chan_handle_t _chan = nullptr;
};

#endif  // SUPPORT_ESP32_I2S
#endif  // PD_ESP32_I2S_MANAGER_H
