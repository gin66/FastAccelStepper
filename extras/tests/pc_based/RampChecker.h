#include <stdlib.h>

class RampChecker {
 public:
  uint64_t total_ticks;
  uint32_t last_dt;
  uint32_t min_dt;
  bool increase_ok;
  bool flat_ok;
  bool decrease_ok;
  bool first;
  bool dir_high;
  bool reversing_allowed;
  uint32_t accelerate_till;
  uint32_t coast_till;
  uint32_t time_coasting;
  uint32_t pos;
  uint32_t ticks_since_last_step = 0;
  float avg_accel = 0;
  FILE *gp_file = NULL;

  void next_ramp() {
    increase_ok = true;
    decrease_ok = false;
    last_dt = ~0;
    min_dt = ~0;
    first = true;
    dir_high = true;
    coast_till = 0;
    time_coasting = 0;
    accelerate_till = 0;
    reversing_allowed = false;
  }
  RampChecker() {
    total_ticks = 0;
    pos = 0;
    next_ramp();
  }
  void start_plot(char *fname) {
    gp_file = fopen(fname, "w");
    fprintf(gp_file, "$data <<EOF\n");
  }
  void finish_plot() {
    if (gp_file != NULL) {
      fprintf(gp_file, "EOF\n");
      fprintf(gp_file, "set term x11 size 1600, 800\n");
      fprintf(gp_file, "set multiplot layout 2,2\n");
      fprintf(gp_file, "set title \"speed [steps/s] over time [s]\"\n");
      fprintf(gp_file, "plot $data using 1:2 with lines notitle\n");
      fprintf(gp_file, "set title \"speed [steps/s] over position\"\n");
      fprintf(gp_file, "plot $data using 4:2 with lines notitle\n");
      fprintf(gp_file, "set title \"position over time [s]\"\n");
      fprintf(gp_file, "plot $data using 1:4 with lines notitle\n");
      fprintf(gp_file,
              "set title \"averaged (!) acceleration [steps/s*s] over time "
              "[s]\"\n");
      fprintf(gp_file, "plot $data using 1:5 with lines notitle\n");
      fprintf(gp_file, "pause -1\n");
      fclose(gp_file);
      gp_file = NULL;
    }
  }
  void check_section(struct queue_entry *e) {
    uint8_t steps = e->steps;
    if (steps == 0) {
      // Just a pause
      if (ticks_since_last_step <= 0xffff0000) {
        ticks_since_last_step += e->ticks;
      }
      total_ticks += e->ticks;
      printf("process pause %d => %u\n", e->ticks, ticks_since_last_step);
      return;
    }
    if (e->toggle_dir) {
      assert(reversing_allowed);
      dir_high = !dir_high;
      increase_ok = true;
      last_dt = ~0;
      decrease_ok = false;
    }
    if (dir_high) {
      pos += steps;
    } else {
      pos -= steps;
    }
    uint32_t curr_dt = ticks_since_last_step;
    total_ticks += steps * e->ticks;
    if (!first) {
      min_dt = min(min_dt, curr_dt);
    }
    float accel = 0;
    if (last_dt != ~0) {
      accel = (16000000.0 / float(curr_dt) - 16000000.0 / float(last_dt)) /
              (1.0 / 16000000.0 * float(steps * curr_dt));
      avg_accel += (accel - avg_accel) / (steps * 20);
    }
    printf(
        "process command in ramp checker @%.6fs: steps = %d last = %u current "
        "= %u "
        " min_dt "
        "= %u   accel=%.6f inc=%s dec=%s\n",
        total_ticks / 16000000.0, steps, last_dt, curr_dt, min_dt, accel,
        increase_ok ? "ALLOW" : "NO", decrease_ok ? "ALLOW" : "NO");

    if (gp_file != NULL) {
      fprintf(gp_file, "%.6f %.2f %d %d %f\n", total_ticks / 16000000.0,
              16000000.0 / last_dt, last_dt, pos, avg_accel);
    }

    assert(first || (steps * curr_dt > 0));

    if (last_dt > curr_dt) {
      assert(increase_ok);
      accelerate_till = total_ticks;
      decrease_ok = true;
    } else if (last_dt < curr_dt) {
      if (increase_ok) {
        coast_till = total_ticks - curr_dt;
      }
      assert(decrease_ok);
      increase_ok = false;
    } else {
      time_coasting += steps * curr_dt;
    }

    if (!first) {
      last_dt = curr_dt;
    }

    ticks_since_last_step = e->ticks;
    first = false;
  }
};
