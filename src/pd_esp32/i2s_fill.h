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
  uint32_t tick_pos;  // tick position along the three blocks
  uint16_t remaining_high_ticks;
  uint16_t remaining_low_ticks;
  uint16_t pulse_positions[I2S_BLOCK_COUNT][I2S_MAX_PULSES_PER_BLOCK];
  uint8_t pulse_count[I2S_BLOCK_COUNT];
};

void i2s_clear_block(uint8_t* buf, struct i2s_fill_state* state, uint8_t block,
                     uint8_t pulse_width_bits);

bool i2s_fill_buffer(StepperQueueBase* q, uint8_t* buf, uint8_t block,
                     struct i2s_fill_state* state, uint8_t pulse_width_bits);

#endif
#endif
