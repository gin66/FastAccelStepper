#ifndef RAMP_CALCULATOR_H
#define RAMP_CALCULATOR_H

#include <stdint.h>

#include "Log2Representation.h"
#include "fas_arch/common.h"

#if (TICKS_PER_S == 16000000L)
#define LOG2_TICKS_PER_S LOG2_CONST_16E6
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 LOG2_CONST_16E6_DIV_SQRT_OF_2
#define LOG2_ACCEL_FACTOR LOG2_CONST_128E12
#define US_TO_TICKS(u32) ((u32) * 16)
#define TICKS_TO_US(u32) ((u32) / 16)
#elif (TICKS_PER_S == 21000000L)
#define LOG2_TICKS_PER_S LOG2_CONST_21E6
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 LOG2_CONST_21E6_DIV_SQRT_OF_2
#define LOG2_ACCEL_FACTOR LOG2_CONST_2205E11
#define US_TO_TICKS(u32) ((u32) * 21)
#define TICKS_TO_US(u32) ((u32) / 21)
#else
#define SUPPORT_LOG2_TIMER_FREQ_VARIABLES
#define LOG2_TICKS_PER_S log2_timer_freq
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 log2_timer_freq_div_sqrt_of_2
#define LOG2_ACCEL_FACTOR log2_timer_freq_square_div_2
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

struct ramp_parameters_s {
  int32_t move_value;
  uint32_t min_travel_ticks;
  uint32_t s_h;
  uint32_t s_jump;
  pmf_logarithmic log2_accel;
  bool apply : 1;              // clear on read by stepper task. Triggers read !
  bool any_change : 1;         // clear on read by stepper task
  bool recalc_ramp_steps : 1;  // clear on read by stepper task
  bool valid_acceleration : 1;
  bool valid_speed : 1;
  bool move_absolute : 1;
  bool keep_running : 1;
  bool keep_running_count_up : 1;

  void init() {
    move_value = 0;
    valid_acceleration = false;
    valid_speed = false;
    apply = false;
    any_change = false;
    recalc_ramp_steps = false;
    move_absolute = true;
    keep_running = false;
    keep_running_count_up = true;
    s_h = 0;
    s_jump = 0;
    min_travel_ticks = 0;
  }
  inline void applyParameters() {
    if (any_change) {
      fasDisableInterrupts();
      apply = true;
      fasEnableInterrupts();
    }
  }
  inline void setRunning(bool count_up) {
    fasDisableInterrupts();
    keep_running = true;
    keep_running_count_up = count_up;
    any_change = true;
    apply = true;
    fasEnableInterrupts();
  }
  inline void setTargetPosition(int32_t pos) {
    fasDisableInterrupts();
    move_value = pos;
    move_absolute = true;
    keep_running = false;
    keep_running_count_up = true;
    any_change = true;
    apply = true;
    fasEnableInterrupts();
  }
  inline void setTargetRelativePosition(int32_t move) {
    fasDisableInterrupts();
    if (any_change && !move_absolute) {
      move_value += move;
    } else {
      move_value = move;
    }
    move_absolute = false;
    keep_running = false;
    keep_running_count_up = true;
    any_change = true;
    apply = true;
    fasEnableInterrupts();
  }
  inline void setCubicAccelerationSteps(uint32_t s_cubic_steps) {
    if (s_h != s_cubic_steps) {
      fasDisableInterrupts();
      s_h = s_cubic_steps;
      recalc_ramp_steps = true;
      any_change = true;
      fasEnableInterrupts();
    }
  }
  inline void setSpeedInTicks(uint32_t min_step_ticks) {
    if (!valid_speed || (min_travel_ticks != min_step_ticks)) {
      fasDisableInterrupts();
      min_travel_ticks = min_step_ticks;
      valid_speed = true;
      any_change = true;
      fasEnableInterrupts();
    }
  }
  inline void setAcceleration(int32_t accel) {
    pmf_logarithmic new_log2_accel = log2_from((uint32_t)accel);
    if (!valid_acceleration || (log2_accel != new_log2_accel)) {
      fasDisableInterrupts();
      valid_acceleration = true;
      any_change = true;
      recalc_ramp_steps = true;
      log2_accel = new_log2_accel;
      fasEnableInterrupts();
    }
  }
  inline void setJumpStart(uint32_t jump_step) { s_jump = jump_step; }
  inline MoveResultCode checkValidConfig() {
    if (!valid_speed) {
      return MOVE_ERR_SPEED_IS_UNDEFINED;
    }
    if (!valid_acceleration) {
      return MOVE_ERR_ACCELERATION_IS_UNDEFINED;
    }
    return MOVE_OK;
  }
};

struct ramp_config_s {
  struct ramp_parameters_s parameters;

  // These three variables are derived
  uint32_t max_ramp_up_steps;
  pmf_logarithmic log2_ticks_h;
  pmf_logarithmic cubic;

  void init() { parameters.init(); }
  inline void update() {
    if (parameters.s_h > 0) {
      pmf_logarithmic log2_s_h = log2_from(parameters.s_h);
      // 1/cubic = sqrt(3/2 * a) / s_h^(1/6) / TICKS_PER_S
      //         = sqrt(3/2 * a / s_h^(1/3)) / TICKS_PER_S
      // cubic = TICKS_PER_S / sqrt(s_h^(1/3) / (3/2 * a))
      cubic = log2_multiply(LOG2_CONST_3_DIV_2, parameters.log2_accel);
      cubic = log2_sqrt(log2_divide(log2_pow_div_3(log2_s_h), cubic));
      cubic = log2_multiply(LOG2_TICKS_PER_S, cubic);

      // calculate_ticks(s_h)
      log2_ticks_h = log2_divide(cubic, log2_pow_2_div_3(log2_s_h));
    } else {
      log2_ticks_h = LOG2_CONST_MAX;
    }
    max_ramp_up_steps = calculate_ramp_steps(parameters.min_travel_ticks);
    if (max_ramp_up_steps == 0) {
      max_ramp_up_steps = 1;
    }
#ifdef TEST
    printf("MAX_RAMP_UP_STEPS=%d from %d ticks\n", max_ramp_up_steps,
           parameters.min_travel_ticks);
#endif
  }

  uint32_t calculate_ticks(uint32_t steps) const {
    // s = 1/2 * a * t^2
    // 2*a*s = (a*t)^2 = v^2 = (TICKS_PER_S/ticks)^2
    // ticks = TICKS_PER_S / sqrt(2*a*s)
    // ticks = TICKS_PER_S/sqrt(2) / sqrt(a*s)
    if (steps >= parameters.s_h) {
      steps -= (parameters.s_h + 2) >> 2;
      pmf_logarithmic log2_steps = log2_from(steps);
      pmf_logarithmic log2_steps_mul_accel =
          log2_multiply(log2_steps, parameters.log2_accel);
      pmf_logarithmic log2_sqrt_steps_mul_accel =
          log2_sqrt(log2_steps_mul_accel);
      pmf_logarithmic log2_res =
          log2_divide(LOG2_TICKS_PER_S_DIV_SQRT_OF_2, log2_sqrt_steps_mul_accel);
      uint32_t res = log2_to_u32(log2_res);
      return res;
    }
    // ticks = cubic / s^(2/3)
    pmf_logarithmic log2_steps = log2_from(steps);
    pmf_logarithmic log2_res = log2_divide(cubic, log2_pow_2_div_3(log2_steps));
    uint32_t res = log2_to_u32(log2_res);
    return res;
  }
  uint32_t calculate_ramp_steps(uint32_t ticks) const {
    // log2 is in range -64..<64 due to shift by 1
    // log2_ticks is in range 0..<32
    // log2_accel is in range 0..<32
    // LOG2_ACCEL_FACTOR is approx. 47 for 16 Mticks/s
    // log2_ticks squared is in range 0..<64
    pmf_logarithmic log2_ticks = log2_from(ticks);
    if (log2_ticks <= log2_ticks_h) {
      pmf_logarithmic log2_inv_accel2 =
          log2_divide(LOG2_ACCEL_FACTOR, parameters.log2_accel);
      uint32_t steps =
          log2_to_u32(log2_divide(log2_inv_accel2, log2_square(log2_ticks)));
      steps += (parameters.s_h + 2) >> 2;
      return steps;
    }
    // s = (cubic/ticks)^(3/2)
    pmf_logarithmic log2_res = log2_divide(cubic, log2_ticks);
    log2_res = log2_pow_3_div_2(log2_res);
    uint32_t steps = log2_to_u32(log2_res);
    return steps;
  }
};
#endif
