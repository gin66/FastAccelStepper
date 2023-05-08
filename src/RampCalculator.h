#ifndef RAMP_CALCULATOR_H
#define RAMP_CALCULATOR_H

#include <stdint.h>

#include "PoorManFloat.h"
#include "common.h"

#ifdef TEST_TIMING
uint32_t calculate_ticks_v1(uint32_t steps, float acceleration);
uint32_t calculate_ticks_v2(uint32_t steps, float acceleration);
uint32_t calculate_ticks_v3(uint32_t steps, float pre_calc);
uint32_t calculate_ticks_v4(uint32_t steps, uint32_t acceleration);
uint32_t calculate_ticks_v5(uint32_t steps, pmf_logarithmic pre_calc);
uint32_t calculate_ticks_v6(uint32_t steps, pmf_logarithmic pre_calc);
uint32_t calculate_ticks_v7(uint32_t steps, pmf_logarithmic pre_calc);
uint32_t calculate_ticks_v8(uint32_t steps, pmf_logarithmic pre_calc);
#endif

uint32_t calculate_ticks(uint32_t steps, pmf_logarithmic pmfl_accel);
uint32_t calculate_ramp_steps(uint32_t ticks, pmf_logarithmic pmfl_accel);

struct ramp_config_s {
  uint32_t min_travel_ticks;
  uint32_t max_ramp_up_steps;
  pmf_logarithmic pmfl_accel;
  uint8_t accel_change_cnt;

  void init() {
    accel_change_cnt = 0;
    min_travel_ticks = 0;
    max_ramp_up_steps = 0;
    pmfl_accel = PMF_CONST_INVALID;
  }
  inline int8_t checkValidConfig() {
    if (min_travel_ticks == 0) {
      return MOVE_ERR_SPEED_IS_UNDEFINED;
    }
    if (pmfl_accel == PMF_CONST_INVALID) {
      return MOVE_ERR_ACCELERATION_IS_UNDEFINED;
    }
    return MOVE_OK;
  }
  inline void setSpeedInTicks(uint32_t min_step_ticks) {
    if (min_travel_ticks != min_step_ticks) {
      min_travel_ticks = min_step_ticks;
      if (checkValidConfig() == MOVE_OK) {
        max_ramp_up_steps = calculate_ramp_steps(min_step_ticks, pmfl_accel);
      }
    }
  }
  inline void setAcceleration(int32_t accel) {
    pmf_logarithmic new_pmfl_accel = pmfl_from((uint32_t)accel);
    if (pmfl_accel != new_pmfl_accel) {
      pmfl_accel = new_pmfl_accel;

      if (checkValidConfig() == MOVE_OK) {
        max_ramp_up_steps = calculate_ramp_steps(min_travel_ticks, pmfl_accel);
      }
      accel_change_cnt++;
    }
  }
};

#endif
