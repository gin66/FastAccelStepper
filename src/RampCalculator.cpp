#include "RampCalculator.h"

#include <math.h>

#include "StepperISR.h"

#ifdef TEST_TIMING
// This module has only one purpose:
//
// -  To calculate for a given step the corresponding speed.
//
// With constant acceleration the relation between the steps s. acceleration a,
// time t and speed v will fulfill this equation during acceleration and
// deceleration:
//
//		s = 0.5 * a * t² = 0.5 * v² / a
//
// Acceleration is just counting up the steps and deceleration counting down
// towards 0.
//
// From this equation the actual speed v at a given step can be calculated by:
//
//		v = sqrt(2*s*a)
//
// For the pwm, the speed needs to be translated into time ticks T = n *
// timeticks = 1/v:
//
//		n * timeticks = 1 / sqrt(2*s*a)
//
// If the µC would be fast, then the solution would be in floating point:
uint32_t calculate_ticks_v1(uint32_t steps, float acceleration) {
  float n = TICKS_PER_S / sqrt(2.0 * steps * acceleration);
  return n;
}
// Just this takes approx 92-4 = 88 us. Which is pretty slow.
//
// Little optimization improves to 87-4 = 83 us
uint32_t calculate_ticks_v2(uint32_t steps, float acceleration) {
  float n = TICKS_PER_S / sqrt(2 * steps * acceleration);
  return n;
}
//
// Precalculate TICKS_PER_S / sqrt(2 * acceleration)
// results in 80-4 = 76us
uint32_t calculate_ticks_v3(uint32_t steps, float pre_calc) {
  float n = pre_calc / sqrt(steps);
  return n;
}
//
// Using pmf_logarithmic improves to 22-4 = 18us, but less precision
uint32_t calculate_ticks_v4(uint32_t steps, uint32_t acceleration) {
  pmf_logarithmic pmfl_a = pmfl_from(acceleration);
  pmf_logarithmic pmfl_s = pmfl_from(steps);
  pmf_logarithmic pmfl_res =
      pmfl_divide(PMF_TICKS_PER_S, pmfl_sqrt(pmfl_multiply(pmfl_s, pmfl_a)));
  uint32_t res = pmfl_to_u32(pmfl_res);
  return res;
}
//
// Precalculating the acceleration related constant improves further to 12-4 =
// 8us. This measurement was actually done with ump_float.
// So this routine does not fit to that measurement anymore
//
uint32_t calculate_ticks_v5(uint32_t steps, pmf_logarithmic pre_calc) {
  pmf_logarithmic pmfl_s = pmfl_from(steps);
  pmf_logarithmic pmfl_res = pmfl_divide(pre_calc, pmfl_sqrt(pmfl_s));
  uint32_t res = pmfl_to_u32(pmfl_res);
  return res;
}
//
// using the combined function yields actually no measureable improvement
uint32_t calculate_ticks_v6(uint32_t steps, pmf_logarithmic pre_calc) {
  //  pmf_logarithmic pmfl_s = pmfl_from(steps);
  //  pmf_logarithmic pmfl_res = pmfl_sqrt_after_divide(pre_calc, pmfl_s);
  //  uint32_t res = pmfl_to_u32(pmfl_res);
  //  return res;
  return calculate_ticks_v5(steps, pre_calc);
}
//
// In order to increase the precision to more than 8 bits and avoid
// the penalty of slow float operations different approach will be used.
//
// in order to avoid division+sqrt operation, the following equation will be
// rearranged:
//
//		n * timeticks = 1 / sqrt(2*s*a)
//
// into:
//
//		n² * s = 1 / sqrt(2*a) / timeticks²
//
// with the right part being constant for a given acceleration a
//
//		n² * s = const(a)
//
// This can be solved iteratively
//
//      n_0 = 0
//	                n_i
//      n_[i+1] = {		         	with n_i² * s <= const(a) < (n_i
//      + mask_i)² * s
//                  n_i + mask_i
//
//      mask_[i+1] = mask_i >> 1
//
// In order to avoid the square calculation in each step several temporary
// values need to be calculated
//                                n_i² * s = f_i
//      f_[i+1] = n_[i+1]² * s = {                         with same condition
//      as before
//						          (n_i  + mask_i)² * s
//
// For the lower case the calculation continues:
//
//		= n_i² * s + 2 * n_i * mask_i * s + mask_i² * s
//
//		= f_i + g_i + h_i
//
// With g_i and h_i defined like this:
//
//		g_0 = 0
//
//		g_[i+1] = 2 * n_[i+1] * mask_[i+1] * s
//
//					2 * n_i * (mask_i >> 1) * s = g_i >> 1
//				= {
//				    2 * (n_i + mask_i) * (mask_i >> 1) * s =
//(g_i
//>> 1) + (mask_i² * s)
//                                                         = (g_i >> 1) + h_i
//
//		h_0 = mask_0 ² * s
//
//		h_[i+1] = mask_[i+1]² * s
//				= (mask_i >> 1)² * s
//				= h_i >> 2
//
// Actually this is a dead end, because below routines already needs 35us,
// while several variables below ought to be 32bit
uint32_t calculate_ticks_v7(uint32_t steps, pmf_logarithmic pre_calc) {
  // initial values for i = 0
  uint16_t mask_i = 0x800;
  uint16_t n_i = 0;
  uint16_t f_i = 0;
  uint16_t g_i = 0;
  uint16_t h_i = steps;
  uint16_t const_a = pre_calc;

  while (mask_i) {
    uint16_t f_x = f_i + g_i + h_i;
    // Serial.println(f_x);
    if (f_x < const_a) {
      n_i += mask_i;
      f_i = f_x;
      g_i >>= 1;
      g_i += h_i;
      h_i >>= 2;
      mask_i >>= 1;
    } else {
      g_i >>= 1;
      h_i >>= 2;
      mask_i >>= 1;
    }
  }
  return n_i;
}

// This is used and requires 9.5us.
//
// Measure with:
//   cd extras/tests/simavr_based
//   make -C off_test_timing
//   From the results subtract port B from port A measurement
uint32_t calculate_ticks_v8(uint32_t steps, pmf_logarithmic pre_calc) {
  pmf_logarithmic pmfl_steps = pmfl_from(steps);
  pmf_logarithmic pmfl_sqrt_steps = pmfl_sqrt(pmfl_steps);
  pmf_logarithmic pmfl_res = pmfl_divide(pre_calc, pmfl_sqrt_steps);
  uint32_t res = pmfl_to_u32(pmfl_res);
  return res;
}
#endif
