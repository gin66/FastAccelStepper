#ifndef PD_ESP32_I2S_FILL_H
#define PD_ESP32_I2S_FILL_H
#if defined(SUPPORT_ESP32_I2S)

#include <stdint.h>
#include "pd_esp32/i2s_constants.h"
#include "fas_queue/base.h"

#ifndef LL_TOGGLE_PIN
#define LL_TOGGLE_PIN(dirPin)
#endif

struct i2s_fill_state {
  uint16_t remaining_low_ticks;
  uint8_t remaining_high_ticks;
  uint8_t off_ticks;
};

bool i2s_fill_buffer(StepperQueueBase* q, uint8_t* buf,
                     struct i2s_fill_state* state);

#endif
#endif
