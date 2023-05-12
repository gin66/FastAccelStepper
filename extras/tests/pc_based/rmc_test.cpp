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
  float ramp_acceleration = 10000.0;
  uint32_t max_speed_in_ticks = 1600;

  struct ramp_config_s c;
  c.init();
  c.parameters.setAcceleration(ramp_acceleration);
  c.parameters.setSpeedInTicks(max_speed_in_ticks);

  char fname[100];
  sprintf(fname, "ramp.gnuplot");
  FILE *gp_file = fopen(fname, "w");
  fprintf(gp_file, "$data <<EOF\n");

  uint64_t sum_ticks = 0;
  float old_speed = 0;
  for (uint32_t s = 1; s <= c.max_ramp_up_steps; s++) {
    uint32_t ticks = c.calculate_ticks(s);
    sum_ticks += ticks;
    float ideal_speed = float(sum_ticks) / 16000000.0 * ramp_acceleration;
    uint32_t rs = c.calculate_ramp_steps(ticks);
    uint32_t ticks_back = c.calculate_ticks(rs);
    uint32_t err = rs >= s ? rs - s : s - rs;
    uint32_t err_ticks =
        ticks >= ticks_back ? ticks - ticks_back : ticks_back - ticks;
    float speed = 16000000.0 / float(ticks);
    float speed_back = 16000000.0 / float(ticks_back);
    old_speed = speed;
    float err_speed =
        speed <= speed_back ? speed - speed_back : speed_back - speed;
    printf("%d: %d %d %f %d delta=%d delta_ticks=%d speed=%f\n", s,
           16000000 / ticks, ticks, float(sum_ticks) / 16000000.0, rs, err,
           err_ticks, err_speed);
    fprintf(gp_file, "%d %f %d %d %d %d %d %f %f\n", s,
            float(sum_ticks) / 16000000.0, 16000000 / ticks, ticks, rs, err,
            err_ticks, err_speed, ideal_speed);
  }
  fprintf(gp_file, "EOF\n");
  // fprintf(gp_file, "plot $data using 2:3 with linespoints\n");
  // fprintf(gp_file, "set terminal pngcairo size 1024,768\n");
  // fprintf(gp_file, "set output \"ramp.png\"\n");
  fprintf(gp_file, "set terminal qt\n");
  fprintf(gp_file, "set term qt size 1024,768\n");
  fprintf(gp_file,
          "set multiplot title \"Acceleration=%f max speed=%d steps/s\" layout "
          "2,2 columnsfirst margins 0.1,0.9,0.1,0.9 spacing 0.1 columnsfirst\n",
          ramp_acceleration, 16000000 / max_speed_in_ticks);

  fprintf(gp_file, "set xlabel \"ramp steps\"\n");
  fprintf(gp_file, "set ylabel \"speed in steps/s\"\n");
  fprintf(
      gp_file,
      "plot $data using 1:3 with line title \"step to speed dependency\"\n");

  fprintf(gp_file, "set xlabel \"ramp steps\"\n");
  fprintf(gp_file, "set ylabel \"recovered ramp steps\"\n");
  fprintf(gp_file,
          "plot $data using 1:5 with line title \"steps(speed(steps))\"\n");

  fprintf(gp_file, "set xlabel \"time in s\"\n");
  fprintf(gp_file, "set ylabel \"speed in steps/s\"\n");
  fprintf(gp_file, "plot $data using 2:3 with line title \"speed over time\",");
  fprintf(gp_file, "     $data using 2:9 with line title \"ideal speed\"\n");

  fprintf(gp_file, "set xlabel \"time in s\"\n");
  fprintf(gp_file, "set ylabel \"speed error in steps/s\"\n");
  fprintf(gp_file, "set yrange [-10:10]\n");
  fprintf(
      gp_file,
      "plot $data using 2:8 with line title \"speed error on ramp change\",");
  fprintf(
      gp_file,
      "     $data using 2:($3-$9) with line title \"speed error to ideal\"\n");

  fprintf(gp_file, "unset multiplot\n");

  fprintf(gp_file, "pause -1\n");
  fclose(gp_file);
  //	assert(false);
  return 0;
}
