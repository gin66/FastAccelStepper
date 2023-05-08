#ifndef RAMP_CALCULATOR_H
#define RAMP_CALCULATOR_H

#include <stdint.h>

#include "PoorManFloat.h"
#include "common.h"

#if (TICKS_PER_S == 16000000L)
#define PMF_TICKS_PER_S PMF_CONST_16E6
#define PMF_TICKS_PER_S_DIV_SQRT_OF_2 PMF_CONST_16E6_DIV_SQRT_OF_2
#define PMF_ACCEL_FACTOR PMF_CONST_128E12
#define US_TO_TICKS(u32) (u32 * 16)
#define TICKS_TO_US(u32) (u32 / 16)
#elif (TICKS_PER_S == 21000000L)
#define PMF_TICKS_PER_S PMF_CONST_21E6
#define PMF_TICKS_PER_S_DIV_SQRT_OF_2 PMF_CONST_21E6_DIV_SQRT_OF_2
#define PMF_ACCEL_FACTOR PMF_CONST_2205E11
#define US_TO_TICKS(u32) (u32 * 21)
#define TICKS_TO_US(u32) (u32 / 21)
#else
#define SUPPORT_PMF_TIMER_FREQ_VARIABLES
#define PMF_TICKS_PER_S pmfl_timer_freq
#define PMF_TICKS_PER_S_DIV_SQRT_OF_2 pmfl_timer_freq_div_sqrt_of_2
#define PMF_ACCEL_FACTOR pmfl_timer_freq_square_div_2
// This overflows for approx. 1s at 40 MHz, only
#define US_TO_TICKS(u32) \
  ((uint32_t)((((uint32_t)((u32) * (TICKS_PER_S / 10000L))) / 100L)))

// This calculation needs more work
#define TICKS_TO_US(u32) \
  ((uint32_t)((((uint32_t)((u32) / (TICKS_PER_S / 1000000L))) / 1L)))

#endif

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
  inline int8_t checkValidConfig() const {
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
        max_ramp_up_steps = calculate_ramp_steps(min_step_ticks);
      }
    }
  }
  inline void setAcceleration(int32_t accel) {
    pmf_logarithmic new_pmfl_accel = pmfl_from((uint32_t)accel);
    if (pmfl_accel != new_pmfl_accel) {
      pmfl_accel = new_pmfl_accel;

      if (checkValidConfig() == MOVE_OK) {
        max_ramp_up_steps = calculate_ramp_steps(min_travel_ticks);
      }
      accel_change_cnt++;
    }
  }

  uint32_t calculate_ticks(uint32_t steps) const {
	  // s = 1/2 * a * t^2
	  // 2*a*s = (a*t)^2 = v^2 = (TICKS_PER_S/ticks)^2
	  // ticks = TICKS_PER_S / sqrt(2*a*s) 
	  // ticks = TICKS_PER_S/sqrt(2) / sqrt(a*s) 
	  pmf_logarithmic pmfl_steps = pmfl_from(steps);
	  pmf_logarithmic pmfl_steps_mul_accel = pmfl_multiply(pmfl_steps, pmfl_accel);
	  pmf_logarithmic pmfl_sqrt_steps_mul_accel = pmfl_sqrt(pmfl_steps_mul_accel);
	  pmf_logarithmic pmfl_res = pmfl_divide(PMF_TICKS_PER_S_DIV_SQRT_OF_2, pmfl_sqrt_steps_mul_accel);
	  uint32_t res = pmfl_to_u32(pmfl_res);
	  return res;
  }
  uint32_t calculate_ramp_steps(uint32_t ticks) const {
	  // pmfl is in range -64..<64 due to shift by 1
	  // pmfl_ticks is in range 0..<32
	  // pmfl_accel is in range 0..<32
	  // PMF_ACCEL_FACTOR is approx. 47 for 16 Mticks/s
	  // pmfl_ticks squared is in range 0..<64
	  pmf_logarithmic pmfl_ticks = pmfl_from(ticks);
	  pmf_logarithmic pmfl_inv_accel2 = pmfl_divide(PMF_ACCEL_FACTOR, pmfl_accel);
	  return pmfl_to_u32(pmfl_divide(pmfl_inv_accel2, pmfl_square(pmfl_ticks)));
  }
};

#endif
