#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "RampCalculator.h"

// Not a real test case

int main() {
  uint32_t res;

  // Calculation is pre_calc/sqrt(steps)
  //
  struct ramp_config_s c;
  c.init();
  c.setAcceleration(100);
  c.setSpeedInTicks(16 * 1000);

  char fname[100];
  sprintf(fname, "ramp.gnuplot");
  FILE *gp_file = fopen(fname, "w");
  fprintf(gp_file, "$data <<EOF\n");

  uint64_t sum_ticks = 0;
  for (uint32_t s = 1; s <= c.max_ramp_up_steps; s++) {
    uint32_t ticks = c.calculate_ticks(s);
    sum_ticks += ticks;
    uint32_t rs = c.calculate_ramp_steps(ticks);
    uint32_t err = rs >= s ? rs - s : s - rs;
    printf("%d: %d %d %ld %d delta=%d\n", s, 16000000 / ticks, ticks, sum_ticks,
           rs, err);
    fprintf(gp_file, "%d %ld %d %d %d\n", s, sum_ticks, 16000000 / ticks, ticks,
            rs);
  }
  fprintf(gp_file, "EOF\n");
  fprintf(gp_file, "plot $data using 2:3 with linespoints\n");
  fprintf(gp_file, "pause -1\n");
  fclose(gp_file);
  //	assert(false);
  return 0;
}
