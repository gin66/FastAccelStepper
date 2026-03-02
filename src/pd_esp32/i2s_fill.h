#ifndef PD_ESP32_I2S_FILL_H
#define PD_ESP32_I2S_FILL_H
#if defined(SUPPORT_ESP32_I2S)

#include <stdint.h>
#include "pd_esp32/i2s_constants.h"

#ifndef QUEUE_LEN_MASK
#define QUEUE_LEN_MASK 63
#endif

#ifndef LL_TOGGLE_PIN
#define LL_TOGGLE_PIN(dirPin)
#endif

struct i2s_queue_entry {
  uint8_t steps;
  uint8_t toggle_dir : 1;
  uint16_t ticks;
};

struct i2s_stepper_queue {
  struct i2s_queue_entry entry[64];
  volatile uint8_t read_idx;
  uint8_t next_write_idx;
  uint8_t dirPin;
};

struct i2s_fill_state {
  uint16_t tick_pos;
  uint16_t tick_carry;
  uint16_t remaining_high_ticks;
  uint16_t remaining_low_ticks;
  uint16_t pulse_positions[I2S_MAX_PULSES_PER_BLOCK];
  uint8_t pulse_count;
};

void i2s_fill_buffer(struct i2s_stepper_queue* q, uint8_t* buf,
                     struct i2s_fill_state* state);

#endif
#endif
