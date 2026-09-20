// test_26 — FasNAxis TDD suite (Step 0: harness only, no planner yet).
//
// This is the harness that the white-paper steps
// (extras/doc/n_axes_whitepaper.md) build up test-first. At Step 0 there is no
// planner: the suite proves the Makefile wildcard picks up test_26, the gnuplot
// helper compiles under -Werror and writes a well-formed multi-panel file, and
// main() exits 0.
//
// Later steps add F1..F19 sections to main() in order; each writes
// test_26_fN.gnuplot via NaxisPlot. Failure aborts the run through the
// test() macro.
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "fas_arch/test_pc.h"  // test() macro
#include "fas_naxis/ramp_map.h"
#include "naxis_plot.h"

// The normal test_% rule links LIB_O (FastAccelStepper.o), which references the
// PC interrupt hooks below. The other tests define these; Step 15's real-queue
// FAS runs use them. (Defined here, not in the header.)
void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

// Smoke-test the gnuplot helper on a trivial 2-axis run so the header is
// compiled and exercised even before a planner exists. No kinematics.
void plot_smoke() {
  NaxisPlot plot;
  plot.start_plot("harness", "FasNAxis harness smoke", 2);
  // Commanded polyline: a small square, the grey reference in panel 1.
  plot.poly_point(0.0, 0.0);
  plot.poly_point(0.0, 1600.0);
  plot.poly_point(1600.0, 1600.0);
  plot.poly_point(1600.0, 0.0);
  plot.poly_done();
  for (int i = 0; i <= 4; i++) {
    double t = 0.1 * i;
    double x = 400.0 * i;
    double y = (i % 2 == 0) ? 0.0 : 1600.0;
    double speed[2] = {1000.0 * (5 - i), 1000.0 * (5 - i)};
    double P[2] = {(double)i, (double)i};
    double R[2] = {4000.0 - 1000.0 * i, 4000.0 - 1000.0 * i};
    double ticks[2] = {16000.0, 16000.0};
    // Synthetic commanded-minus-realized deviation so the fifth panel shows a
    // non-trivial line; a real planner supplies the actual value.
    double deviation = (i % 2 == 0) ? 1.0 : -1.0;
    plot.row(t, x, y, deviation, speed, P, R, ticks);
  }
  plot.finish_plot();
  test(plot.is_open() == false, "plot should be closed after finish_plot");
  printf("harness gnuplot written: test_26_harness.gnuplot\n");
}

// F1 kernel (whitepaper section 7.1): the single-axis period/step law is the
// FAS ramp itself. The RampMap wrapper must be a bit-for-bit identity with the
// ramp_config_s object the library uses (same ticks_cfg + log2_from(accel)),
// round-trip within the documented log2 error band, be monotone non-increasing
// in P, and treat P = 0 as "stopped" (never feed it to calculate_ticks).
//
// Numbers from section 14.1: ticks_cfg = 4000 step/s => 4000 ticks, a = 2000
// step/s^2, P_coast = calculate_ramp_steps(ticks_cfg) ~ 4000 (log2-rounded).
void f1_kernel() {
  const uint32_t ticks_cfg = 4000;
  const uint32_t accel = 2000;

  // The oracle is the exact ramp_config_s the library builds the same way.
  ramp_config_s ref;
  ref.init();
  ref.parameters.setSpeedInTicks(ticks_cfg);
  ref.parameters.setAcceleration(accel);
  ref.update();

  RampMap map(ticks_cfg, accel);

  // P_coast matches the reference (section 14.1: 4000).
  test(map.P_coast() == ref.max_ramp_up_steps, "F1 P_coast identity");
  test(abs((int32_t)map.P_coast() - 4000) <= 16, "F1 P_coast ~ 4000");
  test(map.ticks_cfg() == ticks_cfg, "F1 ticks_cfg retained");

  uint32_t prev_ticks = 0;
  uint32_t max_round_err = 0;
  uint32_t max_round_err_ticks = 0;
  for (uint32_t P = 1; P <= map.P_coast(); P *= 2) {
    // Identity: the wrapper forwards to ramp_config_s, so it is exact.
    uint32_t map_ticks = map.calculate_ticks(P);
    uint32_t ref_ticks = ref.calculate_ticks(P);
    test(map_ticks == ref_ticks, "F1 calculate_ticks identity");

    // Monotone non-increasing in P: more ramp steps => shorter or equal period.
    if (prev_ticks != 0) {
      test(map_ticks <= prev_ticks, "F1 period monotone non-increasing in P");
    }
    prev_ticks = map_ticks;

    // Round-trip within the documented log2 error band. RampCalculator.cpp's
    // own round-trip check treats up to 1 step / 1 tick as acceptable, so the
    // wrapper inherits the same tolerance.
    uint32_t back = map.calculate_ramp_steps(map_ticks);
    uint32_t err = back >= P ? back - P : P - back;
    uint32_t back_ticks = map.calculate_ticks(back);
    uint32_t err_ticks = map_ticks >= back_ticks ? map_ticks - back_ticks
                                                 : back_ticks - map_ticks;
    if (err > max_round_err) {
      max_round_err = err;
    }
    if (err_ticks > max_round_err_ticks) {
      max_round_err_ticks = err_ticks;
    }
    test(err <= 1, "F1 ramp-steps round-trip within 1 step");
    test(err_ticks <= 1, "F1 period round-trip within 1 tick");
  }
  printf("F1 kernel: P_coast=%u max_round_err=%u max_round_err_ticks=%u\n",
         map.P_coast(), max_round_err, max_round_err_ticks);

  // P = 0 is "stopped": FAS starts ramps at P >= 1, so calculate_ticks(0) is
  // never called. Guard the contract by asserting the smallest P we ever pass
  // is 1 and that P_coast >= 1 (a degenerate ramp would violate F1).
  test(map.P_coast() >= 1, "F1 P_coast >= 1 (non-degenerate ramp)");

  // Plot: period vs P, overlay of wrapper vs reference. They are identical by
  // construction, so the overlay is a flat check that the wrapper forwards. The
  // F1 kernel is a period-vs-P identity check with no motion, so it uses the
  // single-panel scalar overlay rather than a time-series trace.
  NaxisPlot plot;
  plot.start_scalar("f1_map", "FasNAxis F1 ramp map identity");
  for (uint32_t P = 1; P <= map.P_coast(); P *= 2) {
    double map_ticks = (double)map.calculate_ticks(P);
    double ref_ticks = (double)ref.calculate_ticks(P);
    plot.scalar_row((double)P, map_ticks, ref_ticks);
  }
  plot.finish_scalar(1.0, (double)map.P_coast(), "P [ramp steps]",
                     "period [ticks]", "wrapper", "RampCalculator");
  printf("F1 kernel plot written: test_26_f1_map.gnuplot\n");
}

int main() {
  puts("FasNAxis TDD");
  plot_smoke();
  f1_kernel();
  printf("TEST_26 PASSED\n");
  return 0;
}
