#ifndef RAMP_CALCULATOR_H
#define RAMP_CALCULATOR_H

#include <stdint.h>

#include "log2/Log2Representation.h"
#include "fas_arch/common.h"

#if (TICKS_PER_S == 16000000L)
#define LOG2_TICKS_PER_S LOG2_CONST_16E6
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 LOG2_CONST_16E6_DIV_SQRT_OF_2
#define LOG2_ACCEL_FACTOR LOG2_CONST_128E12
#define US_TO_TICKS(u32) ((u32) * 16)
#define TICKS_TO_US(u32) ((u32) / 16)

// === STM32H743 @400MHz default path ===
// TIM2 @200MHz, PSC=11 → timer actual = 200M/12 = 16.666.667 Hz
// Avoids ~4% timing error when user doesn't override TICKS_PER_S
// Note: modulo check gives false positive (200M % 16666666 = 8),
// but timing is <0.0001% error. Use 20000000 for error=0.
#elif (TICKS_PER_S == 16666666L)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x2FFB)  // VERIFIED: log2(16666666)*512 = 12283.41
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x2EFB)  // VERIFIED: log2(16666666/√2)*512 = 12027.41
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x5DF6)  // VERIFIED: 2×0x2FFB−512 = 24054 = 0x5DF6
#define US_TO_TICKS(u32)                ((uint32_t)((u32) * 50 / 3))
#define TICKS_TO_US(u32)                ((uint32_t)((u32) * 3 / 50))

// === STM32F103: 72MHz÷4(PSC=3)=18MHz ===
#elif (TICKS_PER_S == 18000000L)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x3034)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x2F34)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x5E68)
#define US_TO_TICKS(u32)                ((u32) * 18)
#define TICKS_TO_US(u32)                ((u32) / 18)

// === STM32F401: 84MHz÷5(PSC=4)=16.8MHz ===
#elif (TICKS_PER_S == 16800000L)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x3001)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x2F01)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x5E02)
#define US_TO_TICKS(u32)                ((uint32_t)((u32) * 168 / 10))
#define TICKS_TO_US(u32)                ((uint32_t)((u32) * 10 / 168))

// === STM32H743 @400MHz: 200MHz÷10(PSC=9)=20MHz ===
#elif (TICKS_PER_S == 20000000L)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x3082)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x2F82)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x5F04)
#define US_TO_TICKS(u32)                ((u32) * 20)
#define TICKS_TO_US(u32)                ((u32) / 20)

#elif (TICKS_PER_S == 21000000L)
#define LOG2_TICKS_PER_S LOG2_CONST_21E6
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 LOG2_CONST_21E6_DIV_SQRT_OF_2
#define LOG2_ACCEL_FACTOR LOG2_CONST_2205E11
#define US_TO_TICKS(u32) ((u32) * 21)
#define TICKS_TO_US(u32) ((u32) / 21)
#elif (TICKS_PER_S == 32000000L)
// STM32L0, RP2040
#define LOG2_TICKS_PER_S               ((log2_value_t)0x31dd)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x30dd)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x61ba)
#define US_TO_TICKS(u32)                ((u32) * 32)
#define TICKS_TO_US(u32)                ((u32) / 32)
#elif (TICKS_PER_S == 48000000L)
// STM32F0/G0/WL — STM32 fork: UNVERIFIED_IN_CI (prescaled to 16M)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x3308)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x3208)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x6411)
#define US_TO_TICKS(u32)                ((u32) * 48)
#define TICKS_TO_US(u32)                ((u32) / 48)
#elif (TICKS_PER_S == 64000000L)
// STM32G0/WB — STM32 fork: UNVERIFIED_IN_CI (prescaled to 16M)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x33dd)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x32dd)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x65ba)
#define US_TO_TICKS(u32)                ((u32) * 64)
#define TICKS_TO_US(u32)                ((u32) / 64)
#elif (TICKS_PER_S == 72000000L)
// STM32F1/L1 — STM32 fork: UNVERIFIED_IN_CI (prescaled to 18M)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x3434)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x3334)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x6668)
#define US_TO_TICKS(u32)                ((u32) * 72)
#define TICKS_TO_US(u32)                ((u32) / 72)
#elif (TICKS_PER_S == 80000000L)
// STM32L4 — STM32 fork: UNVERIFIED_IN_CI (prescaled to 16M)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x3482)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x3382)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x6704)
#define US_TO_TICKS(u32)                ((u32) * 80)
#define TICKS_TO_US(u32)                ((u32) / 80)
#elif (TICKS_PER_S == 84000000L)
// STM32F401/411 — STM32 fork: UNVERIFIED_IN_CI (prescaled to 16.8M)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x34a6)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x33a6)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x674c)
#define US_TO_TICKS(u32)                ((u32) * 84)
#define TICKS_TO_US(u32)                ((u32) / 84)
#elif (TICKS_PER_S == 100000000L)
// STM32F411/746 — STM32 fork: UNVERIFIED_IN_CI (prescaled to ≤20M)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x3527)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x3427)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x684d)
#define US_TO_TICKS(u32)                ((u32) * 100)
#define TICKS_TO_US(u32)                ((u32) / 100)
#elif (TICKS_PER_S == 120000000L)
// STM32L4+/F4 — STM32 fork: UNVERIFIED_IN_CI (prescaled to ≤20M)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x35ad)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x34ad)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x695a)
#define US_TO_TICKS(u32)                ((u32) * 120)
#define TICKS_TO_US(u32)                ((u32) / 120)
#elif (TICKS_PER_S == 168000000L)
// STM32F405/407 — STM32 fork: UNVERIFIED_IN_CI (prescaled to ≤20M)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x36a6)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x35a6)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x6b4c)
#define US_TO_TICKS(u32)                ((u32) * 168)
#define TICKS_TO_US(u32)                ((u32) / 168)
#elif (TICKS_PER_S == 170000000L)
// STM32F3/G4 — STM32 fork: UNVERIFIED_IN_CI (prescaled to ≤20M)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x36af)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x35af)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x6b5e)
#define US_TO_TICKS(u32)                ((u32) * 170)
#define TICKS_TO_US(u32)                ((u32) / 170)
#elif (TICKS_PER_S == 216000000L)
// STM32F7 — STM32 fork: UNVERIFIED_IN_CI (prescaled to ≤20M)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x375f)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x365f)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x6cbe)
#define US_TO_TICKS(u32)                ((u32) * 216)
#define TICKS_TO_US(u32)                ((u32) / 216)
#elif (TICKS_PER_S == 480000000L)
// STM32H7 (default) — STM32 fork: UNVERIFIED_IN_CI (prescaled to 20M)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x39ad)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x38ad)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x715a)
#define US_TO_TICKS(u32)                ((u32) * 480)
#define TICKS_TO_US(u32)                ((u32) / 480)
#elif (TICKS_PER_S == 550000000L)
// STM32H7 (overclock) — STM32 fork: UNVERIFIED_IN_CI (prescaled to ≤20M)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x3a12)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x3912)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x7224)
#define US_TO_TICKS(u32)                ((u32) * 550)
#define TICKS_TO_US(u32)                ((u32) / 550)
#else
// V17: Replaced SUPPORT_LOG2_TIMER_FREQ_VARIABLES with #error.
// All CI boards use predefined entries — this branch is never reached.
// Supported values: 16M, 18M, 20M, 21M, 32M, 48M, 64M, 72M, 80M, 84M,
// 100M, 120M, 168M, 170M, 216M, 480M, 550M
#error "Unsupported TICKS_PER_S. Use timer prescaler to match a supported value."
#endif

#ifdef TEST_TIMING
uint32_t calculate_ticks_v1(uint32_t steps, float acceleration);
uint32_t calculate_ticks_v2(uint32_t steps, float acceleration);
uint32_t calculate_ticks_v3(uint32_t steps, float pre_calc);
uint32_t calculate_ticks_v4(uint32_t steps, uint32_t acceleration);
uint32_t calculate_ticks_v5(uint32_t steps, log2_value_t pre_calc);
uint32_t calculate_ticks_v6(uint32_t steps, log2_value_t pre_calc);
uint32_t calculate_ticks_v7(uint32_t steps, log2_value_t pre_calc);
uint32_t calculate_ticks_v8(uint32_t steps, log2_value_t pre_calc);
#endif

struct ramp_parameters_s {
  int32_t move_value;
  uint32_t min_travel_ticks;
  uint32_t s_h;
  uint32_t s_jump;
  log2_value_t log2_accel;
  bool apply : 1;              // clear on read by stepper task. Triggers read !
  bool any_change : 1;         // clear on read by stepper task
  bool recalc_ramp_steps : 1;  // clear on read by stepper task
  bool valid_acceleration : 1;
  bool valid_speed : 1;
  bool move_absolute : 1;
  bool keep_running : 1;
  bool keep_running_count_up : 1;

  void init() {
    __builtin_memset(this, 0, sizeof(*this));
    move_absolute = true;
    keep_running_count_up = true;
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
    log2_value_t new_log2_accel = log2_from((uint32_t)accel);
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
  inline MoveResultCode checkValidConfig() const {
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
  log2_value_t log2_ticks_h;
  log2_value_t cubic;

  void init() { parameters.init(); }
  inline void update() {
    if (parameters.s_h > 0) {
      log2_value_t log2_s_h = log2_from(parameters.s_h);
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
      log2_value_t log2_steps = log2_from(steps);
      log2_value_t log2_steps_mul_accel =
          log2_multiply(log2_steps, parameters.log2_accel);
      log2_value_t log2_sqrt_steps_mul_accel = log2_sqrt(log2_steps_mul_accel);
      log2_value_t log2_res = log2_divide(LOG2_TICKS_PER_S_DIV_SQRT_OF_2,
                                          log2_sqrt_steps_mul_accel);
      uint32_t res = log2_to_u32(log2_res);
      return res;
    }
    // ticks = cubic / s^(2/3)
    log2_value_t log2_steps = log2_from(steps);
    log2_value_t log2_res = log2_divide(cubic, log2_pow_2_div_3(log2_steps));
    uint32_t res = log2_to_u32(log2_res);
    return res;
  }
  uint32_t calculate_ramp_steps(uint32_t ticks) const {
    // log2 is in range -64..<64 due to shift by 1
    // log2_ticks is in range 0..<32
    // log2_accel is in range 0..<32
    // LOG2_ACCEL_FACTOR is approx. 47 for 16 Mticks/s
    // log2_ticks squared is in range 0..<64
    log2_value_t log2_ticks = log2_from(ticks);
    if (log2_ticks <= log2_ticks_h) {
      log2_value_t log2_inv_accel2 =
          log2_divide(LOG2_ACCEL_FACTOR, parameters.log2_accel);
      uint32_t steps =
          log2_to_u32(log2_divide(log2_inv_accel2, log2_square(log2_ticks)));
      steps += (parameters.s_h + 2) >> 2;
      return steps;
    }
    // s = (cubic/ticks)^(3/2)
    log2_value_t log2_res = log2_divide(cubic, log2_ticks);
    log2_res = log2_pow_3_div_2(log2_res);
    uint32_t steps = log2_to_u32(log2_res);
    return steps;
  }
};
#endif
