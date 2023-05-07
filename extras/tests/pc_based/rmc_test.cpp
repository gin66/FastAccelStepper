#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "RampCalculator.h"

// Not a real test case

int main() {
  uint32_t res;

  // Calculation is pre_calc/sqrt(steps)
  //
  uint32_t pre_calc = 10000000;
  uint32_t steps = 100000;
  float correct;
  float err;
  pmf_logarithmic pmfl_pre_calc = pmfl_from(pre_calc);
  res = calculate_ticks_v8(steps, pmfl_pre_calc);
  correct = pmfl_to_u32(pmfl_pre_calc) * 1.0 / sqrt(steps);
  err = res - correct;
  printf("%d %f  %f\n", res, correct, err);

  //	assert(false);
  return 0;
}
