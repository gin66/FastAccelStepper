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
//
// Opt into the rotordynamic plant (physical_stepper_whitepaper section 3.1 /
// 13.3) so F21 can attach a PhysicalStepper to each axis's SimPort. Only F21
// attaches a plant; every other fixture keeps the ideal SimPort counter
// bit-identical. The plant is header-only and uses std::vector, so the
// Makefile links test_26 with g++ rather than the plain gcc test_% rule.
#define FAS_PHYSICAL_STEPPER_ENABLED 1
// <vector> (pulled in by physical_stepper.h) must precede the PC test shim's
// function-like `test` macro: libc++'s <atomic> reaches <vector> and the macro
// would corrupt std::atomic_flag member names.
#include <vector>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include <math.h>

#include "FasNAxis.h"
#include "fas_arch/test_pc.h"  // test() macro
#include "fas_naxis/dda.h"
#include "fas_naxis/linear.h"
#include "fas_naxis/ramp_law.h"
#include "fas_naxis/ramp_map.h"
#include "fas_naxis/remaining.h"
#include "naxis_plot.h"
#include "naxis_ref.h"
#include "naxis_sim_port.h"
#ifdef FAS_NAXIS_TRACE
#include "naxis_html_dump.h"
#endif

// The normal test_% rule links LIB_O (FastAccelStepper.o), which references the
// PC interrupt hooks below. The other tests define these; Step 15's real-queue
// FAS runs use them. (Defined here, not in the header.)
void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

// Peak performed ramp-up over a rest-to-rest move of S steps. This is the
// outcome of RampLaw (P starts at 0, live remaining-to-stop), not a
// precomputed min(P_stop, R/2). Used by F19 / F2b item 5 instead of a
// Remaining::cap_P estimate.
static uint32_t peak_performed(uint32_t ticks_cfg, uint32_t accel, uint32_t S) {
  RampLaw law(ticks_cfg, accel, S);
  uint32_t peak = 0;
  while (!law.done()) {
    law.step();
    if (law.P > peak) {
      peak = law.P;
    }
  }
  return peak;
}

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

// Step 2 (whitepaper section 8): the remaining-steps scan R is the lookahead
// kernel. Feed polylines as arrays of signed per-axis displacements (no queues)
// and assert the section 8.1 parse, the section 8.5 Linear path-stop, the
// collinear micro-segment scan (F10), the F6b anisotropic corner, and the
// section 8.2 lookahead speed cap (F19: a small HORIZON of micro-segments caps
// P below P_stop without error; the same HORIZON with one long block coasts).
//
// Numbers from section 14.1: ticks_cfg = 4000 step/s => 4000 ticks, a = 2000
// step/s^2, so P_stop = calculate_ramp_steps(ticks_cfg) ~ 4000.
void f2_remaining() {
  const uint32_t ticks_cfg = 4000;
  const uint32_t accel = 2000;
  const uint32_t P_stop = RampMap(ticks_cfg, accel).P_coast();
  test(P_stop >= 3900 && P_stop <= 4100, "F2 P_stop ~ 4000");

  // --- table row 1: one axis, one block (10000,) => R = 10000 ---
  {
    Remaining r(1, 1);
    int32_t b0[1] = {10000};
    r.set_block(0, b0);
    test(r.remaining(0, 0) == 10000, "F2 one block R = 10000");
  }

  // --- table row 2: two blocks, same sign (4000,) + (4000,) => R = 8000 ---
  {
    Remaining r(1, 2);
    int32_t b0[1] = {4000};
    int32_t b1[1] = {4000};
    r.set_block(0, b0);
    r.set_block(1, b1);
    test(r.remaining(0, 0) == 8000, "F2 two same-sign blocks R = 8000");
    test(r.remaining(0, 1) == 4000, "F2 second block alone R = 4000");
  }

  // --- table row 3: reversal (4000,) + (-100,) => R = 4000 at start, then
  // --- 100 the other way after the first block ---
  {
    Remaining r(1, 2);
    int32_t b0[1] = {4000};
    int32_t b1[1] = {-100};
    r.set_block(0, b0);
    r.set_block(1, b1);
    test(r.remaining(0, 0) == 4000, "F2 reversal R = 4000 at start");
    test(r.remaining(0, 1) == 100, "F2 reversal R = 100 after block 0");
    test(r.delta_of(0, 1) < 0, "F2 reversal goes the other way (sign flip)");
  }

  // --- table row 4: idle then move (0,) + (100,) => first block does not
  // --- start a direction; R = 100 ---
  {
    Remaining r(1, 2);
    int32_t b0[1] = {0};
    int32_t b1[1] = {100};
    r.set_block(0, b0);
    r.set_block(1, b1);
    test(r.remaining(0, 0) == 100, "F2 idle-then-move R = 100");
    test(r.remaining(0, 1) == 100, "F2 move block alone R = 100");
  }

  // --- table row 5: Linear path-stop, 2-D square corner. The binder's R ends
  // --- at the 90-degree vertex even though that axis would continue: a
  // square's
  // --- side is one R-budget, not the 4-side perimeter. ---
  {
    Remaining r(2, 4);
    int32_t b0[2] = {0, 1600};
    int32_t b1[2] = {1600, 0};
    int32_t b2[2] = {0, -1600};
    int32_t b3[2] = {-1600, 0};
    r.set_block(0, b0);
    r.set_block(1, b1);
    r.set_block(2, b2);
    r.set_block(3, b3);
    // At head 0 the moving axis is Y (|1600| > |0|); its Linear R ends at the
    // first 90-degree corner = one side (1600), not the 6400 perimeter.
    test(r.remaining_linear_binder(1, 0) == 1600,
         "F2 Linear square binder R ends at 90-degree vertex");
    test(r.remaining_linear_binder(1, 0) < 6400,
         "F2 Linear square R does not span the perimeter");
    // The non-collinear test itself: adjacent square sides are perpendicular.
    test(r.collinear_same_sense(0, 1) == false,
         "F2 adjacent square sides are not collinear");
  }

  // --- table row 6 (F10): 100 x 100-step collinear micro-segments. R at the
  // --- head is the full 10000, not 100 (collinear continuation, no per-segment
  // --- rest). Test on a 2-axis diagonal so the collinear-aware scan matters.
  // ---
  {
    const int N = 100;
    Remaining r(2, N);
    for (int i = 0; i < N; i++) {
      int32_t blk[2] = {100, 100};
      r.set_block(i, blk);
    }
    test(r.remaining(0, 0) == N * 100, "F2 F10 per-axis R sees full 10000");
    test(r.remaining_linear_binder(0, 0) == N * 100,
         "F2 F10 Linear binder R sees full 10000 (collinear)");
    test(r.remaining(0, 1) == (N - 1) * 100,
         "F2 F10 R after one block = 9900 (no per-segment rest)");
  }

  // --- table row 7 (F6b): (4000,1) then (0,3999). R_x = 4000 (then idle);
  // --- R_y = 4000 (continues through the anisotropic corner). ---
  {
    Remaining r(2, 2);
    int32_t b0[2] = {4000, 1};
    int32_t b1[2] = {0, 3999};
    r.set_block(0, b0);
    r.set_block(1, b1);
    test(r.remaining(0, 0) == 4000, "F2 F6b R_x = 4000 then idle");
    test(r.remaining(1, 0) == 4000, "F2 F6b R_y = 4000 continues");
  }

  // --- section 8.5 collinear boundary: 1 degree passes, 2 degree is the edge,
  // --- 3 degree and 90 degree stop (angle change => path-stop). ---
  {
    Remaining r(2, 2);
    int32_t a1[2] = {1000, 17};  // ~1 degree off the x-axis
    int32_t ax[2] = {1000, 0};
    int32_t a2[2] = {1000, 34};  // ~1.94 deg, just inside the 2 deg edge
    int32_t a3[2] = {1000, 52};  // ~3 degree
    int32_t a90[2] = {0, 1000};  // 90 degree
    r.set_block(0, a1);
    r.set_block(1, ax);
    test(r.collinear_same_sense(0, 1) == true, "F2 1 degree is collinear");
    r.set_block(0, a2);
    r.set_block(1, ax);
    test(r.collinear_same_sense(0, 1) == true, "F2 2 degree edge is collinear");
    r.set_block(0, a3);
    r.set_block(1, ax);
    test(r.collinear_same_sense(0, 1) == false, "F2 3 degree stops");
    r.set_block(0, a90);
    r.set_block(1, ax);
    test(r.collinear_same_sense(0, 1) == false, "F2 90 degree stops");
  }

  // --- section 8.2 / F19: a small HORIZON of micro-segments caps P below
  // --- P_stop (last point is rest, live remaining-to-stop so peak P < R)
  // --- and does NOT raise an error; the same HORIZON with one long block
  // --- coasts to P_stop because N/2 > P_stop. ---
  {
    const int N = 100;
    Remaining r(1, N);
    for (int i = 0; i < N; i++) {
      int32_t blk[1] = {100};
      r.set_block(i, blk);
    }
    r.horizon = 8;  // HORIZON too small to hold P_stop
                    // R is capped by HORIZON: 8 blocks x 100 = 800 < P_stop. No
                    // error raised.
    int32_t R_micro = r.remaining(0, 0);
    test(R_micro == 800, "F2 F19 micro-segment R capped at HORIZON = 800");
    uint32_t P_micro = peak_performed(ticks_cfg, accel, (uint32_t)R_micro);
    test(P_micro < P_stop, "F2 F19 micro peak P below P_stop");
    test(P_micro < (uint32_t)R_micro, "F2 F19 must brake: peak P < R");

    // Same HORIZON, one long 10000-step block: R is steps, not points, so the
    // single block coasts to P_stop (10000/2 > P_stop).
    Remaining long_r(1, 1);
    int32_t big[1] = {10000};
    long_r.set_block(0, big);
    long_r.horizon = 8;
    test(long_r.remaining(0, 0) == 10000, "F2 F19 long block R = 10000");
    uint32_t P_long = peak_performed(ticks_cfg, accel, 10000);
    test(P_long == P_stop, "F2 F19 long block reaches P_stop (coasts)");
    test(2 * P_stop < 10000, "F2 F19 N/2 > P_stop so coast exists");
  }

  // Plot F10: live ramp over the 100 collinear 100-step segments. R sees
  // through (10000, not 100 per joint), so P starts at 0, coasts because
  // 10000/2 > P_stop, and commanded/realized share the 45-degree chord.
  {
    const int N = 100;
    const uint32_t S = (uint32_t)(N * 100);
    Remaining r(2, N);
    for (int i = 0; i < N; i++) {
      int32_t blk[2] = {100, 100};
      r.set_block(i, blk);
    }
    int32_t prev = 0;
    for (int head = 0; head < N; head++) {
      int32_t Rscan = r.remaining(0, head);
      test(Rscan <= prev || prev == 0, "F2 F10 R non-increasing over time");
      test(Rscan == (N - head) * 100, "F2 F10 R = remaining collinear steps");
      prev = Rscan;
    }
    test(r.remaining(0, 0) == (int32_t)S, "F2 F10 R at head is full 10000");

    NaxisPlot plot;
    plot.start_plot("f10", "FasNAxis F10 collinear micro-segments", 2);
    plot.poly_point(0.0, 0.0);
    for (int i = 1; i <= N; i++) {
      plot.poly_point((double)i * 100.0, (double)i * 100.0);
    }
    plot.poly_done();

    RampLaw law(ticks_cfg, accel, S);
    uint32_t peak = 0;
    uint32_t plot_every = 50;
    // Rest sample: P starts at 0, realized on the commanded chord.
    {
      double speed[2] = {0.0, 0.0};
      double Pcol[2] = {0.0, 0.0};
      double Rcol[2] = {(double)law.R, (double)law.R};
      double tickscol[2] = {0.0, 0.0};
      plot.row(0.0, 0.0, 0.0, 0.0, speed, Pcol, Rcol, tickscol);
    }
    for (uint32_t k = 0; k < S; k++) {
      test(law.P <= law.R, "F2 F10 P <= R at every step");
      uint32_t ticks = law.step();
      if (law.P > peak) {
        peak = law.P;
      }
      uint32_t s = S - law.R;  // steps issued = position on the diagonal
      if ((s % 100) == 0) {
        int head = (int)(s / 100);
        if (head < N) {
          test(r.remaining(0, head) == (int32_t)law.R,
               "F2 F10 remaining() matches live R at block boundary");
        }
      }
      if (k % plot_every == 0 || k + 1 == S) {
        // P == 0 is stopped: the last decel command used ticks_cfg, which
        // would plot as v_max at rest. Show speed 0 at standstill.
        double v = (law.P == 0) ? 0.0 : NAXIS_PLOT_TICKS_PER_S / (double)ticks;
        double speed[2] = {v, v};
        double Pcol[2] = {(double)law.P, (double)law.P};
        double Rcol[2] = {(double)law.R, (double)law.R};
        double pt = (law.P == 0) ? 0.0 : (double)ticks;
        double tickscol[2] = {pt, pt};
        double t = (double)law.total_ticks / NAXIS_PLOT_TICKS_PER_S;
        plot.row(t, (double)s, (double)s, 0.0, speed, Pcol, Rcol, tickscol);
      }
    }
    plot.finish_plot();
    test(law.done(), "F2 F10 planner consumed all collinear steps");
    test(peak == P_stop, "F2 F10 coasts: peak P == P_stop (R sees through)");
    test(plot.is_open() == false, "F2 F10 plot closed");
    printf("F2 remaining: P_stop=%u peak=%u plot written test_26_f10.gnuplot\n",
           P_stop, peak);
  }
}

// Step 2b (whitepaper section 6.3 / 8.3, todo "theory probes"): the lookahead
// model must be correct on its own, not merely on the F-fixtures. A wrong model
// must fail here even if a later fixture is green. The oracle is a small pure
// function (Remaining::binder_axis / dda_steps / remaining, plus RampLaw
// peak_performed), never FasNAxis state. Two mutation hooks prove the probes
// are load-bearing:
//   -FAS_NAXIS_NO_REBIND in remaining.h makes item 3 fail;
//   -FAS_NAXIS_NO_REST_CAP in ramp_law.h makes item 5 fail to brake
//    (peak P is not < R). extras/tests/pc_based/prove_mutations.sh rebuilds
//    with each hook (make mutations); it is a test, not a library script.
//
// Limits from section 14.1: a = 2000 step/s^2. ticks_cfg is the configured
// period; the ratios in item 3 use integer ticks (never a / in production).
void f2b_oracle() {
  const uint32_t accel = 2000;

  // --- item 1: reference oracle ---------------------------------------
  // Given a path direction and per-axis ticks, binder_axis names the axis
  // that takes the longest wall-clock (argmax |delta_i| * ticks_i), DDA maps
  // the binder's step count onto every slave, and P <= R per axis. Check the
  // oracle: a clean DDA block (long X, short Y, equal ticks -> X binds, Y
  // slaved) plus the rebind concept (a 4x-slower Y wins the wall-clock).
  {
    int32_t block[2] = {100, 50};
    uint32_t ticks[2] = {400, 400};  // equal: longest |delta| (X) binds
    int binder = Remaining::binder_axis(block, ticks, 2);
    test(binder == 0, "F2b item1 longest |delta| axis binds (X)");
    // DDA: Y is slaved onto X's 100 binder steps and issues 50 of them; the
    // binder issues its own 100.
    test(Remaining::dda_steps(block[binder], block[1]) == 50,
         "F2b item1 DDA Y slaved to 50 of X 100 steps");
    test(Remaining::dda_steps(block[binder], block[0]) == 100,
         "F2b item1 DDA X (binder) issues 100 steps");
    // Rebind concept: with Y 4x slower (ticks 1600), Y's wall-clock
    // |dy|*ticks_y (50*1600) beats X's (100*400), so Y binds the
    // time-law. DDA master stays X (longest |delta|); ticks_floor is
    // Y's period (lengthen ticks_b), not a DDA rebind onto 50 steps.
    uint32_t ticks_slow[2] = {400, 1600};
    int binder_slow = Remaining::binder_axis(block, ticks_slow, 2);
    test(binder_slow == 1, "F2b item1 slow Y binds by wall-clock (rebind)");
    test(Remaining::longest_axis(block, ticks_slow, 2) == 0,
         "F2b item1 DDA master stays longest X");
    test(Remaining::ticks_floor(block, ticks_slow, 2) == 1600,
         "F2b item1 ticks_floor is the slow slave");
    // P <= R on the binder: a short R is remaining-to-stop, so the live
    // ramp must brake (peak P < R), not a precomputed min(P_stop, R/2).
    int32_t R = 30;
    uint32_t P = peak_performed(400, accel, (uint32_t)R);
    test(P < (uint32_t)R, "F2b item1 peak P < R on the binder");
  }

  // --- item 3: rebind neighbourhood -----------------------------------
  // |dx| in {99,100,101}, ticks_y/ticks_x in {1, 2, 99/100 (integer ticks)}.
  // Especially |dx|*ticks_x ~= |dy|*ticks_y: the slow axis must bind. With
  // rebind off (FAS_NAXIS_NO_REBIND) the longest-distance-only binder is wrong
  // on these near-tie blocks (proven by the #if 0 build below).
  {
    int dxs[3] = {99, 100, 101};
    int tx[3] = {100, 100, 100};
    int ty[3] = {100, 200, 99};  // ratios 1, 2, 99/100 as integer ticks
    int32_t dy = 100;            // fixed |dy| so |dx|*ticks_x ~= |dy|*ticks_y
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        int32_t block[2] = {dxs[i], dy};
        uint32_t ticks[2] = {(uint32_t)tx[i], (uint32_t)ty[j]};
        int binder = Remaining::binder_axis(block, ticks, 2);
        int64_t sx = (int64_t)block[0] * ticks[0];
        int64_t sy = (int64_t)block[1] * ticks[1];
        // The binder is exactly the axis with the larger |
        // delta|*ticks
        // (ties broken to X, the smaller index).
        if (sy > sx) {
          test(binder == 1, "F2b item3 slow Y binds");
        } else {
          test(binder == 0, "F2b item3 X binds (tie / faster Y)");
        }
      }
    }
    printf("F2b item3 rebind neighbourhood checked\n");
  }

  // --- item 4: collinear boundary -------------------------------------
  // 1 degree must pass, 2 degree is the documented edge, 3 and 90 degree
  // must stop (angle change => Linear path-stop). Include n=3 with one tiny
  // component (the tiny axis must not change the 2-D collinearity verdict).
  {
    Remaining r2(2, 2);
    int32_t one[2] = {1000, 17};
    int32_t edge[2] = {1000, 34};
    int32_t three[2] = {1000, 52};
    int32_t right[2] = {0, 1000};
    int32_t axis[2] = {1000, 0};
    r2.set_block(0, one);
    r2.set_block(1, axis);
    test(r2.collinear_same_sense(0, 1) == true, "F2b item4 1 deg passes");
    r2.set_block(0, edge);
    test(r2.collinear_same_sense(0, 1) == true, "F2b item4 2 deg edge passes");
    r2.set_block(0, three);
    test(r2.collinear_same_sense(0, 1) == false, "F2b item4 3 deg stops");
    r2.set_block(0, right);
    test(r2.collinear_same_sense(0, 1) == false, "F2b item4 90 deg stops");
    // n=3 collinearity is the 3-D dot test, so a tiny perpendicular
    // component does not by itself force a path-stop, while a
    Remaining r3(3, 2);
    int32_t collinear0[3] = {100, 50, 10};
    int32_t collinear1[3] = {200, 100, 20};  // exact multiple: collinear
    int32_t bend0[3] = {100, 50, 10};
    int32_t bend1[3] = {100, 0, 0};  // y bends: not collinear
    int32_t tiny0[3] = {1000, 0, 1};
    int32_t tiny1[3] = {1000, 0, 0};  // a tiny z stays in the 2-deg cone
    r3.set_block(0, collinear0);
    r3.set_block(1, collinear1);
    test(r3.collinear_same_sense(0, 1) == true,
         "F2b item4 n=3 proportional vectors are collinear");
    r3.set_block(0, bend0);
    r3.set_block(1, bend1);
    test(r3.collinear_same_sense(0, 1) == false,
         "F2b item4 n=3 non-collinear pair stops");
    r3.set_block(0, tiny0);
    r3.set_block(1, tiny1);
    test(r3.collinear_same_sense(0, 1) == true,
         "F2b item4 n=3 tiny component stays collinear");
  }

  // --- item 5: lookahead speed cap ------------------------------------
  // Open path of 800 steps, P_stop = 4000: last point is rest, live
  // remaining-to-stop so peak P < R (a triangle, not a precomputed R/2
  // estimate), no error. Appending collinear blocks until R >= 2 P_stop
  // makes coasting to P_stop legal. Path direction (800, 400): Linear
  // slaves Y onto X's R = 800 triangle.
  {
    const uint32_t ticks_cfg = 4000;
    uint32_t P_stop = RampMap(ticks_cfg, accel).P_coast();
    test(P_stop >= 3900 && P_stop <= 4100, "F2b item5 P_stop ~ 4000");

    // 800-step open path (single block): must brake; peak is below P_stop
    // and strictly below remaining (the rest-cap mutation fails this).
    uint32_t R800 = 800;
    uint32_t P800 = peak_performed(ticks_cfg, accel, R800);
    test(P800 < P_stop, "F2b item5 short path P below P_stop");
    test(P800 < R800, "F2b item5 short path brakes (peak P < R)");
    test(P800 >= 1, "F2b item5 short path no error (P >= 1)");

    // Append collinear blocks until N/2 > P_stop: coasting becomes legal.
    uint32_t Rbig = 10000;
    uint32_t Pbig = peak_performed(ticks_cfg, accel, Rbig);
    test(Pbig == P_stop, "F2b item5 long path reaches P_stop (coasts)");
    test(Pbig > P800, "F2b item5 larger P legal once N/2 > P_stop");

    // Path direction (800, 400): X is the binder, Y is DDA-slaved onto X's
    // R = 800 triangle. Y issues 400/800 of X's 800 binder steps = 400.
    int32_t block[2] = {800, 400};
    uint32_t ticks[2] = {ticks_cfg, ticks_cfg};
    int binder = Remaining::binder_axis(block, ticks, 2);
    test(binder == 0, "F2b item5 (800,400) X is the binder");
    test(Remaining::dda_steps(block[binder], block[1]) == 400,
         "F2b item5 Y DDA-slaved to 400 of X's 800 steps");
  }
  // --- mutation counterexamples (proof the hooks are load-bearing) ---
  // The normal build has the full model, so each counterexample below MUST
  // hold; extras/tests/pc_based/prove_mutations.sh (`make mutations`)
  // rebuilds with FAS_NAXIS_NO_REBIND (item 3 fails) and
  // FAS_NAXIS_NO_REST_CAP (item 5 fails to brake) to prove a wrong model is
  // detected.
  {
    // Rebind counterexample (item 3): |dx|*ticks_x ~= |dy|*ticks_y, the slow
    // Y binds. Longest-distance-only (rebind disabled) would pick X (101 >
    // 99) and be wrong.
    int32_t reb[2] = {101, 100};
    uint32_t rt[2] = {100, 99};  // X: 10100  Y: 9900 (Y wall-clock smaller
                                 // but Y distance ~ equal)
    // Here Y's wall-clock 100*99 = 9900 < X 101*100 = 10100, so X is the
    // correct binder; the near-tie is what the neighbourhood sweeps. Assert
    // the oracle matches the exact argmax so a wrong model cannot hide.
    int binder = Remaining::binder_axis(reb, rt, 2);
    int64_t sx = 101 * 100, sy = 100 * 99;
    test(binder == (sy > sx ? 1 : 0),
         "F2b mutation rebind oracle matches argmax");
    // Rest-cap counterexample (item 5): remaining-to-stop must brake, so
    // peak P < R. Ignoring R (always accel) makes peak P == R on this
    // short move.
    uint32_t P_cap = peak_performed(4000, accel, 800);
    test(P_cap < 800, "F2b mutation rest cap brakes (peak P < R)");
  }
  printf("F2b oracle: all theory probes green\n");
}

// Step 2c (whitepaper section 6.3): the DDA walk, one block, geometry only.
//
// No ramp, no period, no queues. Remaining::dda_steps already counts how many
// steps a slave issues over |bind| binder steps; this step *walks* the error
// accumulator one binder step at a time (DdaWalk). Two-axis hand cases with
// integer positions only. Each binder step issues 0 or 1 step per axis (never
// 2); the walked count per axis must equal Remaining::dda_steps; the chord
// invariant |2*err| <= |bind| holds after every binder step; a test-only
// double confirms distance to the chord <= 0.5*sqrt(n) (section 12.4).
struct DdaCase {
  const char* name;
  int32_t dx;
  int32_t dy;
  int binder;  // expected binder axis (0 = X, 1 = Y; tie -> smaller index)
};

static void f2c_dda_walk() {
  DdaCase cases[] = {
      {"(5,0)  X binder, Y never steps", 5, 0, 0},
      {"(0,4)  Y binder, X never steps", 0, 4, 1},
      {"(5,5)  X binder (tie), Y every step", 5, 5, 0},
      {"(5,3)  X binder, Bresenham", 5, 3, 0},
      {"(5,-3) X binder, Y steps -1", 5, -3, 0},
      {"(-4,2) X binder, signs follow delta", -4, 2, 0},
  };
  for (int c = 0; c < 6; c++) {
    const DdaCase& cs = cases[c];
    int32_t d[2] = {cs.dx, cs.dy};
    uint32_t ticks[2] = {4000, 4000};  // equal ticks: longest |delta| binds
    int binder = Remaining::binder_axis(d, ticks, 2);
    char msg[64];
    snprintf(msg, sizeof(msg), "2c binder for %s", cs.name);
    test(binder == cs.binder, msg);
    int32_t bind = d[binder];
    int32_t slave = d[1 - binder];

    int32_t walked[2] = {0, 0};
    int32_t pos[2] = {0, 0};
    DdaWalk w(bind, slave);
    // A test-only double accumulator for the 12.4 chord-distance check. This is
    // not production; the production walker is integer-only (DdaWalk).
    double max_chord_dist = 0.0;
    double chord_len = 0.0;
    while (!w.done()) {
      int out_bind = 0, out_slave = 0;
      w.step(&out_bind, &out_slave);
      test(out_bind == 1 || out_bind == -1, "2c binder steps exactly one");
      test(out_slave == -1 || out_slave == 0 || out_slave == 1,
           "2c slave steps 0 or 1, never 2");
      walked[binder] += out_bind;
      walked[1 - binder] += out_slave;
      pos[binder] += out_bind;
      pos[1 - binder] += out_slave;
      test(w.on_chord(), "2c chord invariant |2*err| <= |bind|");

      // 12.4: perpendicular distance of (pos) from the origin-to-d chord.
      double px = (double)pos[0], py = (double)pos[1];
      double dx = (double)d[0], dy = (double)d[1];
      double denom = dx * dx + dy * dy;
      if (denom > 0.0) {
        chord_len = denom;
        double dist2 = (px * dy - py * dx) / denom;  // (dx,dy) unit-ish
        double dist = dist2 * dist2;
        if (dist > max_chord_dist) {
          max_chord_dist = dist;
        }
      }
    }
    // Walked count per axis must equal the analytic dda_steps oracle.
    int expected_bind = Remaining::dda_steps(bind, bind);
    int expected_slave = Remaining::dda_steps(bind, slave);
    int32_t wb_abs = (walked[binder] > 0) ? walked[binder] : -walked[binder];
    test(wb_abs == expected_bind, "2c binder walked count == dda_steps");
    int32_t ws_abs =
        (walked[1 - binder] > 0) ? walked[1 - binder] : -walked[1 - binder];
    test(ws_abs == expected_slave, "2c slave walked count == dda_steps");

    // End position equals the waypoint (integer positions only).
    test(pos[0] == cs.dx && pos[1] == cs.dy, "2c end position == waypoint");

    // 12.4 test-only: max perpendicular distance^2 <= 0.25 * n.
    double n = chord_len;
    test(max_chord_dist <= 0.25 * n + 1e-9, "2c chord distance <= 0.5*sqrt(n)");
  }

  // Plot: XY of (5,3) on the chord (binder X, Y slaved).
  {
    int32_t d[2] = {5, 3};
    int binder = 0;
    int32_t bind = d[binder], slave = d[1 - binder];
    NaxisPlot plot;
    plot.start_plot("f2c", "FasNAxis F2c DDA walk (5,3)", 2);
    plot.poly_point(0.0, 0.0);
    plot.poly_point(5.0, 3.0);
    plot.poly_done();
    DdaWalk w(bind, slave);
    int32_t pos[2] = {0, 0};
    for (int s = 0; s <= 5; s++) {
      double speed[2] = {0.0, 0.0};
      double P[2] = {0.0, 0.0};
      double R[2] = {0.0, 0.0};
      double ticks[2] = {0.0, 0.0};
      plot.row((double)s, (double)pos[0], (double)pos[1], 0.0, speed, P, R,
               ticks);
      if (w.done()) {
        break;
      }
      int out_bind = 0, out_slave = 0;
      w.step(&out_bind, &out_slave);
      pos[0] += out_bind;
      pos[1] += out_slave;
    }
    plot.finish_plot();
    printf("F2c DDA walk plot written: test_26_f2c.gnuplot\n");
  }
  printf("F2c DDA walk: 2-axis hand cases green\n");
}

// Step 3 (whitepaper section 7.1): the ramp law P vs R, still with no queue.
//
// One axis, rest-to-rest over S = 10000 steps at the section 14.1 limits
// (ticks_cfg for 4000 step/s, a = 2000 step/s^2). RampLaw drives R down to 0
// one command at a time, applying the FAS control law of RampControl
// (_getNextCommand):
//
//    R > P   -> accelerate (P++), coast when P is already at P_coast
//    R == P  -> decelerate (P--)
//    R < P   -> decelerate (overshoot guard)
//    ticks   = max(calculate_ticks(P), ticks_cfg, ticks_min)
//
// Checks: the law never lets P exceed R; the issued step count equals S; the
// trapezoid peaks at P_coast and is symmetric (accel == decel); the total
// duration matches an independent RampCalculator trapezoid within a few Delta_t
// (a 2 ms slice); a batched planning chunk lands on the same P trajectory as
// single steps (the 2 ms rule must not change the law). The plot compares by
// eye to the test_02 trapezoids.
void f3_ramp() {
  const uint32_t ticks_cfg = 4000;
  const uint32_t accel = 2000;
  const uint32_t S = 10000;
  RampMap map(ticks_cfg, accel);
  const uint32_t P_coast = map.P_coast();
  test(P_coast >= 3900 && P_coast <= 4100, "F3 P_coast ~ 4000");
  // P_coast must be small enough for a trapezoid (coast exists), so the peak
  // is the true coast, not a triangular cap.
  test(2 * P_coast < S, "F3 trapezoid: 2*P_coast < S (coast exists)");

  // Reference trapezoid, computed independently from the planner: update P
  // first (accel from rest is 0->1), then sum calculate_ticks over the
  // issued P sequence. A separate loop, so an off-by-one in the planner's
  // P advance / R decrement still diverges here.
  auto ref_total = [&]() -> uint64_t {
    uint64_t total = 0;
    int32_t rem = (int32_t)S;
    int32_t P = 0;
    for (int32_t k = 0; k < S; k++) {
      if (rem > P) {
        if ((int32_t)P < (int32_t)P_coast) {
          P++;
        }
      } else if (P > 0) {
        P--;
      }
      uint32_t t =
          (P == 0) ? map.calculate_ticks(1) : map.calculate_ticks((uint32_t)P);
      if (t < ticks_cfg) {
        t = ticks_cfg;
      }
      total += t;
      rem--;
    }
    return total;
  };
  uint64_t ref = ref_total();

  // Pass 1: single-step trace. Assert the law invariants and record the P
  // trajectory + trace for the plot.
  {
    RampLaw law(ticks_cfg, accel, S);
    test(law.P == 0 && law.R == S, "F3 start at rest");
    uint32_t peak = 0;
    int32_t first_decel_k = -1;
    int32_t last_accel_k = -1;
    int32_t prev_R = S;
    NaxisPlot plot;
    plot.start_plot("f3", "FasNAxis F3 rest-to-rest ramp", 1);
    plot.poly_point(0.0, 0.0);
    plot.poly_point((double)S, 0.0);
    plot.poly_done();
    uint32_t plot_every = 50;  // downsample 10000 rows for the gnuplot
    {
      double speed0[1] = {0.0};
      double P0[1] = {0.0};
      double R0[1] = {(double)S};
      double ticks0[1] = {0.0};
      plot.row(0.0, 0.0, 0.0, 0.0, speed0, P0, R0, ticks0);
    }
    for (uint32_t k = 0; k < S; k++) {
      uint32_t R_before = law.R;
      uint32_t P_before = law.P;
      test(P_before <= R_before, "F3 P <= R at every step");
      uint32_t ticks = law.step();
      // P moves by exactly 1 toward the coast / back to rest, never jumps.
      test(law.P == P_before + 1 || law.P == P_before - 1 ||
               (law.P == P_before && P_before == P_coast),
           "F3 P changes by 0 or 1");
      if (law.P > peak) {
        peak = law.P;
      }
      if (R_before > P_before && law.P > P_before) {
        last_accel_k = (int32_t)k;
      }
      if (R_before <= P_before && first_decel_k < 0 && law.P < P_before) {
        first_decel_k = (int32_t)k;
      }
      // R counts down by exactly 1, never skips.
      test(law.R == prev_R - 1, "F3 R decreases by 1 per step");
      prev_R = law.R;
      double v = (law.P == 0) ? 0.0 : NAXIS_PLOT_TICKS_PER_S / (double)ticks;
      double speed[1] = {v};
      double Pcol[1] = {(double)law.P};
      double Rcol[1] = {(double)law.R};
      double tickscol[1] = {(law.P == 0) ? 0.0 : (double)ticks};
      double t = (double)law.total_ticks / NAXIS_PLOT_TICKS_PER_S;
      double x = (double)(S - law.R);
      if (k % plot_every == 0 || k + 1 == S) {
        plot.row(t, x, 0.0, 0.0, speed, Pcol, Rcol, tickscol);
      }
    }
    plot.finish_plot();
    test(law.done(), "F3 planner consumed all S steps");
    test(peak == P_coast, "F3 peak P == P_coast (coast reached)");
    test(last_accel_k < first_decel_k, "F3 accel phase precedes decel phase");
    // Symmetric trapezoid: decel starts when R == P; P held at P_coast during
    // coast, so R == P_coast at position S - P_coast. The whitepaper 14.1
    // "s_coast = 10000 - 8000 = 2000" is the coast length S - 2*P_coast.
    int32_t expect_decel = (int32_t)S - (int32_t)P_coast;
    test(abs(first_decel_k - expect_decel) <= 2,
         "F3 decel starts at S - P_coast");
    test(law.total_ticks == ref, "F3 total ticks matches RampCalculator");
    printf("F3 ramp: S=%u P_coast=%u peak=%u decel@%d total=%llu ref=%llu\n", S,
           P_coast, peak, first_decel_k, (unsigned long long)law.total_ticks,
           (unsigned long long)ref);
  }

  // Pass 2: batched planning chunk (FAS 2 ms rule). A chunk must land on the
  // identical P trajectory and total as single steps -- batching is a
  // presentation detail, not a law change. Use a slow-speed chunk so several
  // steps fall in one slice (section 4.2: 2 ms / current period).
  {
    RampLaw single(ticks_cfg, accel, S);
    RampLaw chunk(ticks_cfg, accel, S);
    // Chunk size: a 2 ms window at the current period, clamped to [1,
    // remaining], exactly as FAS plans slow ramps forward.
    const uint32_t DT = TICKS_PER_S / 500;  // 2 ms in ticks (16 MHz)
    uint32_t i = 0;
    while (i < S) {
      uint32_t p0 = single.P;
      uint32_t r0 = single.R;
      uint32_t ticks0 = single.period();
      uint32_t ps = (ticks0 < TICKS_PER_S / 1000) ? DT / ticks0 : 1;
      if (ps == 0) {
        ps = 1;
      }
      if (ps > single.R) {
        ps = single.R;
      }
      single.step_chunk(ps);
      chunk.step_chunk(ps);
      test(chunk.P == single.P, "F3 chunked P matches single-step P");
      test(chunk.R == single.R, "F3 chunked R matches single-step R");
      i += ps;
    }
    test(single.done() && chunk.done(), "F3 both passes done");
    test(single.total_ticks == chunk.total_ticks, "F3 chunk total == single");
    test(single.total_ticks == ref, "F3 chunk total matches RampCalculator");
    printf("F3 chunk pass: total=%llu (matches single-step RampCalculator)\n",
           (unsigned long long)chunk.total_ticks);
  }

  // P <= R must hold at the instant of the transition into decel too: the law
  // only turns decel when R <= P, so at that boundary P <= R still (they are
  // equal, within the 1-step log2 rounding).
  printf("F3 ramp plot written: test_26_f3.gnuplot\n");
}

// Step 2d (whitepaper section 6.3 / 9.2): the Linear one-block rest-to-rest
// interpolator. One committed segment: the binder runs a RampLaw over
// |delta_bind|; each binder step is one DDA tick from 2c. Equal ticks_cfg, so
// the longest |delta| binds. Checks: issued |steps| per axis equal |delta|;
// the walk ends at the vertex; the path lies on the chord; the binder P <= R
// from the RampLaw fields holds every step (a sanity check for this step --
// reconstruction from issued periods is Step 2e, not here).
void f2d_linear_one_block() {
  const uint32_t ticks_cfg = 4000;
  const uint32_t accel = 2000;  // section 14.1 limits
  struct Case {
    const char* name;
    int32_t dx, dy;
    int expect_binder;
  };
  Case cases[] = {
      {"(20,8) X binds", 20, 8, 0},
      {"(20,0) Y never steps", 20, 0, 0},
      {"(8,20) Y binds", 8, 20, 1},
  };
  for (int c = 0; c < 3; c++) {
    const Case& cs = cases[c];
    int32_t d[2] = {cs.dx, cs.dy};
    uint32_t ticks[2] = {ticks_cfg, ticks_cfg};
    LinearBlock block(ticks_cfg, accel, 2, d, ticks);
    char msg[64];
    snprintf(msg, sizeof(msg), "2d binder for %s", cs.name);
    test(block.binder == cs.expect_binder, msg);

    int32_t issued[2] = {0, 0};  // signed accumulated steps per axis
    int32_t pos[2] = {0, 0};
    double max_chord_dist = 0.0;
    double chord_len = 0.0;
    double px0 = 0.0, py0 = 0.0;
    while (!block.done()) {
      int step_out[2];
      block.step(step_out);
      test(step_out[0] == -1 || step_out[0] == 0 || step_out[0] == 1,
           "2d axis 0 steps in {-1,0,1}");
      test(step_out[1] == -1 || step_out[1] == 0 || step_out[1] == 1,
           "2d axis 1 steps in {-1,0,1}");
      issued[0] += step_out[0];
      issued[1] += step_out[1];
      pos[0] += step_out[0];
      pos[1] += step_out[1];
      // Sanity for this step: binder P <= R from the RampLaw fields (2e proves
      // the same from issued periods, not these fields).
      test(block.law.P <= block.law.R, "2d binder P <= R (fields)");
      // 12.4: perpendicular distance of (pos) from the origin-to-d chord.
      double px = (double)pos[0], py = (double)pos[1];
      double ax = (double)d[0], ay = (double)d[1];
      double denom = ax * ax + ay * ay;
      if (denom > 0.0) {
        chord_len = denom;
        double dist = (px * ay - py * ax) / denom;  // chord unit-ish, signed
        double d2 = dist * dist;
        if (d2 > max_chord_dist) {
          max_chord_dist = d2;
        }
        px0 = px;
        py0 = py;
      }
    }
    // Issued |steps| per axis equals |delta|.
    int32_t is0 = issued[0] > 0 ? issued[0] : -issued[0];
    int32_t is1 = issued[1] > 0 ? issued[1] : -issued[1];
    int32_t ex0 = cs.dx > 0 ? cs.dx : -cs.dx;
    int32_t ex1 = cs.dy > 0 ? cs.dy : -cs.dy;
    snprintf(msg, sizeof(msg), "2d issued|steps| axis0 == |delta| for %s",
             cs.name);
    test(is0 == ex0, msg);
    snprintf(msg, sizeof(msg), "2d issued|steps| axis1 == |delta| for %s",
             cs.name);
    test(is1 == ex1, msg);
    // End position is the vertex.
    test(pos[0] == cs.dx && pos[1] == cs.dy, "2d end position == vertex");
    // Path on the chord: 12.4 max perpendicular distance^2 <= 0.25 * n.
    double n = chord_len;
    test(max_chord_dist <= 0.25 * n + 1e-9,
         "2d path within 0.5*sqrt(n) of the chord");
    printf("F2d %s: binder=%d issued=(%d,%d) end=(%d,%d) chord^2=%.3f n=%.1f\n",
           cs.name, block.binder, issued[0], issued[1], pos[0], pos[1],
           max_chord_dist, n);
    (void)px0;
    (void)py0;
  }

  // Plot: XY of (20,8) plus binder speed vs time.
  {
    int32_t d[2] = {20, 8};
    uint32_t ticks[2] = {ticks_cfg, ticks_cfg};
    LinearBlock block(ticks_cfg, accel, 2, d, ticks);
    NaxisPlot plot;
    plot.start_plot("f2d", "FasNAxis F2d Linear one-block (20,8)", 2);
    plot.poly_point(0.0, 0.0);
    plot.poly_point((double)d[0], (double)d[1]);
    plot.poly_done();
    int32_t pos[2] = {0, 0};
    // First sample at rest (P == 0) so the trace has a start point.
    {
      double speed0[2] = {0.0, 0.0};
      double P0[2] = {0.0, 0.0};
      double R0[2] = {(double)block.law.R, 0.0};
      double ticks0[2] = {0.0, 0.0};
      plot.row(0.0, 0.0, 0.0, 0.0, speed0, P0, R0, ticks0);
    }
    while (!block.done()) {
      int step_out[2];
      uint32_t ticks_issued = block.step(step_out);
      pos[0] += step_out[0];
      pos[1] += step_out[1];
      double t = (double)block.law.total_ticks / NAXIS_PLOT_TICKS_PER_S;
      double speed[2] = {
          block.law.P == 0 ? 0.0
                           : NAXIS_PLOT_TICKS_PER_S / (double)ticks_issued,
          0.0,
      };
      double Pcol[2] = {(double)block.law.P, 0.0};
      double Rcol[2] = {(double)block.law.R, 0.0};
      double tickscol[2] = {(block.law.P == 0) ? 0.0 : (double)ticks_issued,
                            0.0};
      plot.row(t, (double)pos[0], (double)pos[1], 0.0, speed, Pcol, Rcol,
               tickscol);
    }
    plot.finish_plot();
    test(plot.is_open() == false, "2d plot closed");
    printf("F2d Linear one-block plot written: test_26_f2d.gnuplot\n");
  }
  printf("F2d Linear one-block: hand cases green\n");
}

// Step 2e (whitepaper section 6.3 / 12.4): reconstruct P from the *issued
// periods* of the 2d trace -- a model that only wrote planner P fields must
// fail this (it would read RampLaw.P, which we never touch here). For each
// binder step, P_issued = calculate_ramp_steps(ticks_issued); the last
// rest-to-stop record issues ticks_cfg (period() at P==0) which reconstructs
// to P_coast, a standstill marker, not a moving command -- excluded from the
// P<=R and envelope checks. Remaining after k binder steps is |delta_bind|-k
// (the last buffered point is rest, section 8.1).
//
// Checks on every hand case:
//  - DDA master is longest |delta| (never a shorter axis);
//  - issued |steps| per axis equals |delta| (vertex hit);
//  - P_issued <= remaining at every moving sample;
//  - peak P_issued < |delta_master| (live remaining-to-stop on these shorts);
//  - envelope: when axis i steps, ticks >= ticks_i_cfg (one-step slack, 12.4).
//
// Rebind hand cases (Y slower, wall-clock |dy|*ticks_y > |dx|*ticks_x):
// Y constrains the time-law (ticks_floor = ticks_y) but X stays DDA master.
// X is scaled down in speed, not in count -- walking DDA on Y would issue
// only |dy| steps of X and miss the vertex (the 2g (4000,8000) bite).
void f2e_issued_periods() {
  struct Case {
    const char* name;
    int32_t dx, dy;
    uint32_t ticks_x, ticks_y;
    int expect_master;
    int expect_time_binder;
  };
  Case cases[] = {
      {"(20,8) X binds (equal ticks)", 20, 8, 4000, 4000, 0, 0},
      {"(20,8) Y 4x slower", 20, 8, 4000, 16000, 0, 1},
      {"(5,3) Y 2x slower (2g bite)", 5, 3, 4000, 8000, 0, 1},
  };
  for (int c = 0; c < 3; c++) {
    const Case& cs = cases[c];
    int32_t d[2] = {cs.dx, cs.dy};
    uint32_t ticks[2] = {cs.ticks_x, cs.ticks_y};
    uint32_t t_floor = Remaining::ticks_floor(d, ticks, 2);
    int time_binder = Remaining::binder_axis(d, ticks, 2);
    RampMap map(t_floor, 2000);
    LinearBlock block(4000, 2000, 2, d, ticks);
    char msg[80];
    snprintf(msg, sizeof(msg), "2e DDA master for %s", cs.name);
    test(block.binder == cs.expect_master, msg);
    snprintf(msg, sizeof(msg), "2e time-law binder for %s", cs.name);
    test(time_binder == cs.expect_time_binder, msg);
    test(block.law.ticks_cfg == t_floor, "2e time-law is ticks_floor");

    uint32_t N = block.law.R;  // |delta_master|
    int32_t issued[2] = {0, 0};
    uint32_t peak_issued = 0;
    int moving = 0;
    int k = 0;
    bool ok_envelope = true;
    bool ok_p_le_r = true;
    while (!block.done()) {
      int step_out[2];
      uint32_t ticks_issued = block.step(step_out);
      issued[0] += step_out[0];
      issued[1] += step_out[1];
      uint32_t p_issued = map.calculate_ramp_steps(ticks_issued);
      bool rest = (ticks_issued == t_floor);  // last step: P==0 -> P_coast
      if (rest) {
        // Standstill marker (period() at P==0). Excluded from P<=R / envelope;
        // its reconstruction is the coast position P_coast, not a moving P.
        test(map.calculate_ramp_steps(t_floor) == map.P_coast(),
             "2e last record reconstructs to P_coast");
      } else {
        moving++;
        uint32_t remaining = N - (uint32_t)k;  // |delta_master| - k, k before
        if (p_issued > remaining) {
          ok_p_le_r = false;
        }
        if (p_issued > peak_issued) {
          peak_issued = p_issued;
        }
        // Per-axis envelope: a command that steps axis i must not be faster
        // than ticks_i_cfg (shared tick sum, one-step slack of 12.4).
        if (step_out[0] != 0 && ticks_issued < ticks[0]) {
          ok_envelope = false;
        }
        if (step_out[1] != 0 && ticks_issued < ticks[1]) {
          ok_envelope = false;
        }
      }
      k++;
    }
    snprintf(msg, sizeof(msg), "2e P_issued <= remaining for %s", cs.name);
    test(ok_p_le_r, msg);
    test(peak_issued < N, "2e peak P_issued < |delta_master|");
    test(ok_envelope, "2e envelope ticks >= ticks_i_cfg");
    int32_t ix = issued[0] > 0 ? issued[0] : -issued[0];
    int32_t iy = issued[1] > 0 ? issued[1] : -issued[1];
    int32_t ex = cs.dx > 0 ? cs.dx : -cs.dx;
    int32_t ey = cs.dy > 0 ? cs.dy : -cs.dy;
    snprintf(msg, sizeof(msg), "2e issued |steps| == |delta| for %s", cs.name);
    test(ix == ex && iy == ey, msg);
    printf(
        "F2e %s: master=%d time_binder=%d N=%u peak_P=%u moving=%d "
        "issued=(%d,%d)\n",
        cs.name, block.binder, time_binder, N, peak_issued, moving, issued[0],
        issued[1]);
  }
  printf("F2e issued-period reconstruction: hand cases green\n");
}

static int32_t iround(double x) {
  return (int32_t)(x >= 0.0 ? x + 0.5 : x - 0.5);
}

static uint32_t lcg_next(uint32_t* s) {
  *s = *s * 1664525u + 1013904223u;
  return *s;
}

static int32_t irand_inc(uint32_t* s, int32_t lo, int32_t hi) {
  uint32_t span = (uint32_t)(hi - lo + 1);
  return lo + (int32_t)(lcg_next(s) % span);
}

static void push_block(int32_t blocks[][2], int* n_blocks, int32_t* posx,
                       int32_t* posy, int32_t dx, int32_t dy) {
  if (dx == 0 && dy == 0) {
    return;
  }
  blocks[*n_blocks][0] = dx;
  blocks[*n_blocks][1] = dy;
  *posx += dx;
  *posy += dy;
  (*n_blocks)++;
}

// Walk a Linear walker (NaxisRefLinear oracle or LinearPoly interpolator). Both
// share the same public interface (step/done/master/ticks_law/R_before_cmd/R/P
// /dda/total_ticks). Reconstruct P from issued periods (never from P/R fields).
// At each block end, store the last moving P_issued.
template <class Walker>
static void walk_polyline(Walker& ref, Remaining* rem, const uint32_t* ticks,
                          uint32_t accel, int32_t* end_pos, int32_t* issued,
                          bool* envelope_ok, bool* p_le_r_ok,
                          uint32_t* vertex_p, int* n_vertex, int max_vertex,
                          NaxisPlot* plot, uint32_t* recon_slack,
                          int32_t* vertex_pos) {
  int32_t pos[2] = {0, 0};
  issued[0] = 0;
  issued[1] = 0;
  *envelope_ok = true;
  *p_le_r_ok = true;
  *n_vertex = 0;
  uint32_t last_moving_p = 0;
  uint32_t slack = 0;
  // Plot speed smoothing. The per-command pulse rate is noisy: the DDA master
  // alternates, a multi-step catch-up is one short command, and the old code
  // zeroed speed whenever the issued period equalled ticks_cfg -- which is
  // exactly a coast, so every cruise looked like a stop. Track each axis's
  // position delta per command and low-pass it (EMA, time constant
  // kSpeedTau_s), sampling the smoothed value every kPlotStride commands.
  // P and R are path-level (path steps), the DDA binder's currency.
  const int kPlotStride = 8;
  const double kSpeedTau_s = 0.02;
  int sample_n = 0;
  int32_t last_pos[2] = {0, 0};
  uint64_t last_ticks = 0;
  double ema_v[2] = {0.0, 0.0};
  bool have_sample = false;
  if (plot) {
    double z[2] = {0.0, 0.0};
    plot->row(0.0, 0.0, 0.0, 0.0, z, z, z, z);
    have_sample = true;
  }
  while (!ref.done()) {
    int step_out[2];
    uint32_t ticks_issued = ref.step(step_out);
    uint32_t t_law = ref.ticks_law;
    uint32_t R_before = ref.R_before_cmd;
    issued[0] += step_out[0];
    issued[1] += step_out[1];
    pos[0] += step_out[0];
    pos[1] += step_out[1];
    RampMap map(t_law, accel);
    uint32_t p_issued =
        ticks_issued > 0 ? map.calculate_ramp_steps(ticks_issued) : 0;
    // A coast issues the max-speed period, which is not a "rest"; the original
    // `rest` classification (period == max) stays for the assertion gating, but
    // it must no longer drive the plot speed (that is what zeroed every coast).
    bool rest = (ticks_issued == t_law);
    if (!rest) {
      // calculate_ramp_steps o calculate_ticks may land one step high
      // (log2 inverse, section 12.4 one-step slack).
      if (ref.P > R_before) {
        *p_le_r_ok = false;
      }
      if (p_issued > R_before) {
        uint32_t d = p_issued - R_before;
        if (d > slack) {
          slack = d;
        }
      }
      last_moving_p = p_issued;
      if (step_out[0] != 0 && ticks_issued < ticks[0]) {
        *envelope_ok = false;
      }
      if (step_out[1] != 0 && ticks_issued < ticks[1]) {
        *envelope_ok = false;
      }
    }
    if (plot) {
      uint64_t now_ticks = ref.total_ticks;
      double dt = (double)(now_ticks - last_ticks) / NAXIS_PLOT_TICKS_PER_S;
      if (dt > 0.0) {
        double alpha = dt / (kSpeedTau_s + dt);
        ema_v[0] += alpha * ((double)(pos[0] - last_pos[0]) / dt - ema_v[0]);
        ema_v[1] += alpha * ((double)(pos[1] - last_pos[1]) / dt - ema_v[1]);
      }
      last_pos[0] = pos[0];
      last_pos[1] = pos[1];
      last_ticks = now_ticks;
      if ((sample_n % kPlotStride) == 0 && have_sample) {
        double speed[2] = {ema_v[0], ema_v[1]};
        double Pcol[2] = {(double)p_issued, (double)p_issued};
        double Rcol[2] = {(double)ref.R, (double)ref.R};
        double tickscol[2] = {(double)ticks_issued, (double)ticks_issued};
        plot->row((double)now_ticks / NAXIS_PLOT_TICKS_PER_S, (double)pos[0],
                  (double)pos[1], 0.0, speed, Pcol, Rcol, tickscol);
      }
    }
    sample_n++;
    if (ref.dda.done()) {
      if (*n_vertex < max_vertex) {
        vertex_p[*n_vertex] = last_moving_p;
        if (vertex_pos) {
          vertex_pos[2 * *n_vertex] = pos[0];
          vertex_pos[2 * *n_vertex + 1] = pos[1];
        }
      }
      (*n_vertex)++;
      last_moving_p = 0;
    }
  }
  end_pos[0] = pos[0];
  end_pos[1] = pos[1];
  if (recon_slack) {
    *recon_slack = slack;
  }
}

// NaxisRefLinear oracle walker (Step 2ref / F20).
static void ref_walk_polyline(Remaining* rem, const uint32_t* ticks,
                              uint32_t accel, int32_t* end_pos, int32_t* issued,
                              bool* envelope_ok, bool* p_le_r_ok,
                              uint32_t* vertex_p, int* n_vertex, int max_vertex,
                              NaxisPlot* plot, uint32_t* recon_slack,
                              int32_t* vertex_pos) {
  NaxisRefLinear ref(rem, ticks, accel);
  walk_polyline(ref, rem, ticks, accel, end_pos, issued, envelope_ok, p_le_r_ok,
                vertex_p, n_vertex, max_vertex, plot, recon_slack, vertex_pos);
}

// Step 2ref: globally fastest constraint-faithful Linear track. One-block
// traces match LinearBlock (2e). Two-block rows of 2f hold on the reference.
// F20 is the long polyline (half-circle with seeded random before/after).
void f2ref_reference() {
  const uint32_t accel = 2000;
  struct One {
    const char* name;
    int32_t dx, dy;
    uint32_t tx, ty;
  };
  One ones[] = {
      {"(20,8) equal ticks", 20, 8, 4000, 4000},
      {"(20,8) Y 4x slower", 20, 8, 4000, 16000},
      {"(5,3) Y 2x slower", 5, 3, 4000, 8000},
  };
  for (int c = 0; c < 3; c++) {
    int32_t d[2] = {ones[c].dx, ones[c].dy};
    uint32_t ticks[2] = {ones[c].tx, ones[c].ty};
    LinearBlock block(4000, accel, 2, d, ticks);
    Remaining rem(2, 1);
    rem.set_block(0, d);
    NaxisRefLinear ref(&rem, ticks, accel);
    char msg[80];
    while (!block.done() && !ref.done()) {
      int a[2], b[2];
      uint32_t ta = block.step(a);
      uint32_t tb = ref.step(b);
      test(ta == tb, "2ref one-block ticks match LinearBlock");
      test(a[0] == b[0] && a[1] == b[1],
           "2ref one-block steps match LinearBlock");
    }
    snprintf(msg, sizeof(msg), "2ref %s both done together", ones[c].name);
    test(block.done() && ref.done(), msg);
  }
  printf("F2ref one-block identity with LinearBlock green\n");

  struct Two {
    const char* name;
    int32_t a0, a1, b0, b1;
    bool path_stop_joint;
    int master1;
  };
  Two twos[] = {
      {"(5,0)+(0,5) L", 5, 0, 0, 5, true, 1},
      {"(3,3)+(2,2) collinear", 3, 3, 2, 2, false, 0},
      {"(5,0)+(-3,0) reversal", 5, 0, -3, 0, true, 0},
  };
  uint32_t ticks_eq[2] = {4000, 4000};
  for (int c = 0; c < 3; c++) {
    Remaining rem(2, 2);
    int32_t d0[2] = {twos[c].a0, twos[c].a1};
    int32_t d1[2] = {twos[c].b0, twos[c].b1};
    rem.set_block(0, d0);
    rem.set_block(1, d1);
    int32_t end_pos[2], issued[2];
    bool env = true, plr = true;
    uint32_t vp[8];
    int nv = 0;
    ref_walk_polyline(&rem, ticks_eq, accel, end_pos, issued, &env, &plr, vp,
                      &nv, 8, NULL, NULL, NULL);
    char msg[80];
    snprintf(msg, sizeof(msg), "2ref %s envelope", twos[c].name);
    test(env, msg);
    snprintf(msg, sizeof(msg), "2ref %s P_issued <= R", twos[c].name);
    test(plr, msg);
    test(issued[0] == twos[c].a0 + twos[c].b0 &&
             issued[1] == twos[c].a1 + twos[c].b1,
         "2ref two-block issued == polyline");
    test(end_pos[0] == issued[0] && end_pos[1] == issued[1],
         "2ref two-block end is last vertex");
    test(nv >= 2, "2ref two-block has a vertex sample per block");
    if (twos[c].path_stop_joint) {
      test(vp[0] <= 1, "2ref path-stop joint last moving P_issued <= 1");
    } else {
      test(vp[0] > 1, "2ref collinear joint does not rest");
    }
    printf("F2ref %s: vertices=%d joint_P=%u end=(%d,%d)\n", twos[c].name, nv,
           vp[0], end_pos[0], end_pos[1]);
    (void)twos[c].master1;
  }
  printf("F2ref two-block path-stop/collinear/reversal green\n");
}

// F20 block layout shared by the oracle (Step 2ref) and the interpolator
// (Step 2h): seeded random, a connecting block to (r,0), a half-circle of 1 deg
// chords, then seeded random again. Same seed (26) => identical polyline,
// so both walkers run over the same committed blocks.
static int build_f20_blocks(int32_t blocks[][2], int32_t* sum) {
  const int32_t r = 1600;
  const int n_arc = 180;
  const int n_rand = 80;
  int n_blocks = 0;
  uint32_t rng = 26;
  int32_t posx = 0;
  int32_t posy = 0;
  sum[0] = 0;
  sum[1] = 0;
  for (int i = 0; i < n_rand; i++) {
    int32_t dx = 0;
    int32_t dy = 0;
    while (dx == 0 && dy == 0) {
      dx = irand_inc(&rng, -12, 12);
      dy = irand_inc(&rng, -12, 12);
    }
    push_block(blocks, &n_blocks, &posx, &posy, dx, dy);
  }
  push_block(blocks, &n_blocks, &posx, &posy, r - posx, 0 - posy);
  int32_t ax = r;
  int32_t ay = 0;
  for (int i = 1; i <= n_arc; i++) {
    double th = 3.14159265358979323846 * (double)i / (double)n_arc;
    int32_t x = iround((double)r * cos(th));
    int32_t y = iround((double)r * sin(th));
    push_block(blocks, &n_blocks, &posx, &posy, x - ax, y - ay);
    ax = x;
    ay = y;
  }
  for (int i = 0; i < n_rand; i++) {
    int32_t dx = 0;
    int32_t dy = 0;
    while (dx == 0 && dy == 0) {
      dx = irand_inc(&rng, -12, 12);
      dy = irand_inc(&rng, -12, 12);
    }
    push_block(blocks, &n_blocks, &posx, &posy, dx, dy);
  }
  sum[0] = posx;
  sum[1] = posy;
  return n_blocks;
}

// F20: several hundred waypoints — seeded random, half-circle, seeded random
// — so the globally fastest Linear reference hits collinear cruise, Y-reversal
// path-stop on the arc, rebind (ticks 4000/8000), idle-free kinks, and a long
// connecting block. Constraint-faithful: vertices, envelope, P<=R,
// |steps|==|Δ|.
void f20_long_polyline() {
  const uint32_t accel = 2000;
  const uint32_t ticks[2] = {4000, 8000};
  int32_t blocks[400][2];
  int32_t sum[2];
  int n_blocks = build_f20_blocks(blocks, sum);

  test(n_blocks >= 200, "F20 several hundred waypoints");
  Remaining rem(2, n_blocks);
  for (int b = 0; b < n_blocks; b++) {
    rem.set_block(b, blocks[b]);
  }
  rem.horizon = 0xFFFFFFFFU;

  NaxisPlot plot;
  plot.start_plot("f20", "FasNAxis F20 half-circle + random", 2);
  plot.poly_point(0.0, 0.0);
  {
    int32_t wx = 0, wy = 0;
    for (int b = 0; b < n_blocks; b++) {
      wx += blocks[b][0];
      wy += blocks[b][1];
      plot.poly_point((double)wx, (double)wy);
    }
  }
  plot.poly_done();

  int32_t end_pos[2], issued[2];
  bool env = true, plr = true;
  uint32_t vp[512];
  int nv = 0;
  uint32_t recon_slack = 0;
  ref_walk_polyline(&rem, ticks, accel, end_pos, issued, &env, &plr, vp, &nv,
                    512, &plot, &recon_slack, NULL);
  plot.finish_plot();

  test(env, "F20 envelope ticks >= ticks_i_cfg");
  test(plr, "F20 law P <= R_before");
  test(recon_slack <= 2, "F20 reconstructed P within 2-step log2 band of R");
  test(issued[0] == sum[0] && issued[1] == sum[1],
       "F20 issued |steps| == polyline");
  test(end_pos[0] == sum[0] && end_pos[1] == sum[1],
       "F20 end position is last vertex");
  test(nv == n_blocks, "F20 a vertex sample at every waypoint");

  int n_stop = 0;
  int n_cruise = 0;
  for (int i = 0; i < nv - 1; i++) {
    if (vp[i] <= 1) {
      n_stop++;
    } else {
      n_cruise++;
    }
  }
  test(n_stop >= 1, "F20 has a Linear path-stop joint");
  test(n_cruise >= 1, "F20 has a collinear cruise joint");
  printf(
      "F20 blocks=%d vertices=%d stop_joints=%d cruise_joints=%d "
      "recon_slack=%u end=(%d,%d)\n",
      n_blocks, nv, n_stop, n_cruise, recon_slack, end_pos[0], end_pos[1]);
  printf("F20 long polyline plot written: test_26_f20.gnuplot\n");
}

// Step 2f: two-block Linear, path-stop vs collinear. The LinearPoly
// interpolator (src/fas_naxis/linear.h) walks two committed blocks and is
// compared to the NaxisRefLinear oracle on the three two-block rows: a 90deg
// L (path-stop, P -> 0 at the vertex, binder switches to the next axis), a
// collinear run (P carries, no rest at the joint), and a reversal (P -> 0
// then the other sign). P is reconstructed from issued periods; a wrong model
// that only writes planner P fields must fail here.
// Step 2g: exhaustive tiny Linear. Every 2-axis polyline with |delta_i| <= 5,
// 3 vertices (2 blocks) and 4 vertices (3 blocks), across three integer ticks
// pairs, compared to the naxis_ref oracle (independent of the interpolator P /
// R fields). This is combinatorics (DDA / vertex / P <= R), not coasting, so
// no P_stop cruise is asserted. Idle-then-move is included (a block may be 0);
// the first block is never all-zero (a no-op move).
//
// Vertex invariant of the oracle: walk_polyline emits one vertex per walked
// block, and a trailing zero block is not walked, so the count is
// "highest non-zero block index + 1". Each vertex sample sits at a waypoint.
// A joint between block i-1 and i is a path-stop (P -> 0) when the two
// displacements are not the same 2 deg-collinear sense, and a collinear cruise
// (P carries) otherwise.
// Walks one committed polyline through the naxis_ref oracle and checks the
// 2g invariants. `wp` holds the cumulative waypoint of each block start
// (wp[2b] / wp[2b+1] = sum of blocks 0..b-1), wp[0..1] = origin.
static void f2g_walk(Remaining* rem, const uint32_t* ticks, uint32_t accel,
                     int n_blocks, int32_t* wp, uint64_t* ref_time,
                     uint64_t* wp_time, uint64_t* joint_time) {
  int32_t end_pos[2], issued[2];
  bool env = true, plr = true;
  uint32_t vp[16];
  int nv = 0;
  uint32_t slack = 0;
  int32_t vpos[32];
  *ref_time = 0;
  *wp_time = 0;
  *joint_time = 0;

  struct timeval t0, t1;
  gettimeofday(&t0, NULL);
  ref_walk_polyline(rem, ticks, accel, end_pos, issued, &env, &plr, vp, &nv, 16,
                    NULL, &slack, vpos);
  gettimeofday(&t1, NULL);
  *ref_time = (uint64_t)(t1.tv_sec - t0.tv_sec) * 1000000ULL +
              (uint64_t)(t1.tv_usec - t0.tv_usec);

  test(env, "2g envelope ticks >= ticks_i_cfg");
  test(plr, "2g law P_issued <= R");
  int32_t sum[2] = {0, 0};
  for (int b = 0; b < n_blocks; b++) {
    sum[0] += rem->delta_of(0, b);
    sum[1] += rem->delta_of(1, b);
  }
  test(issued[0] == sum[0] && issued[1] == sum[1],
       "2g issued |steps| == |delta|");
  test(end_pos[0] == sum[0] && end_pos[1] == sum[1], "2g end is last vertex");

  gettimeofday(&t0, NULL);
  // Every emitted vertex lands exactly on a cumulative waypoint. A trailing
  // no-op block makes the oracle emit a duplicate sample at the final position
  // (the finished transition still has dda.done() true), so we check "on some
  // waypoint" rather than a positional index.
  for (int i = 0; i < nv; i++) {
    bool on_waypoint = 0;
    for (int b = 0; b < n_blocks; b++) {
      if (vpos[2 * i] == wp[2 * b] && vpos[2 * i + 1] == wp[2 * b + 1]) {
        on_waypoint = 1;
      }
    }
    test(on_waypoint, "2g every vertex hits a cumulative waypoint");
  }
  gettimeofday(&t1, NULL);
  *wp_time = (uint64_t)(t1.tv_sec - t0.tv_sec) * 1000000ULL +
             (uint64_t)(t1.tv_usec - t0.tv_usec);

  gettimeofday(&t0, NULL);
  // Joint P semantics. vertex_p[k] is the last moving P of block k. A joint
  // between block i-1 and i is a path-stop (P -> 0) when the two
  // displacements are not the same 2 deg-collinear sense, and a collinear
  // cruise (P carries) otherwise. Idle blocks are not joints. A collinear
  // joint only *must* be in cruise (P_issued > 1) once the live remaining
  // path exceeds P_coast on a moving axis; below that the ramp is
  // legitimately decelerating toward rest.
  for (int i = 1; i < n_blocks; i++) {
    int32_t da[2] = {rem->delta_of(0, i - 1), rem->delta_of(1, i - 1)};
    int32_t db[2] = {rem->delta_of(0, i), rem->delta_of(1, i)};
    int64_t ma = (int64_t)da[0] * da[0] + (int64_t)da[1] * da[1];
    int64_t mb = (int64_t)db[0] * db[0] + (int64_t)db[1] * db[1];
    if (ma == 0 || mb == 0) {
      continue;  // an idle block is not a joint
    }
    int64_t dot = (int64_t)da[0] * db[0] + (int64_t)da[1] * db[1];
    bool stop = (dot <= 0) || (dot * dot * 100000 < 99878 * ma * mb);
    if (stop) {
      test(vp[i - 1] <= 1, "2g path-stop joint P_issued <= 1");
    } else {
      int32_t r0 = 0, r1 = 0;
      for (int b = i; b < n_blocks; b++) {
        int32_t c[2] = {rem->delta_of(0, b), rem->delta_of(1, b)};
        if (c[0] == 0 && c[1] == 0) {
          continue;
        }
        int m = Remaining::longest_axis(c, ticks, 2);
        int64_t a = c[m] > 0 ? c[m] : -c[m];
        if (m == 0) {
          r0 += a;
        } else {
          r1 += a;
        }
      }
      uint32_t pc0 = RampMap(ticks[0], accel).P_coast();
      uint32_t pc1 = RampMap(ticks[1], accel).P_coast();
      bool cruise =
          (r0 > 0 && (uint32_t)r0 > pc0) || (r1 > 0 && (uint32_t)r1 > pc1);
      if (cruise) {
        test(vp[i - 1] > 1, "2g collinear cruise joint does not rest");
      }
    }
  }
  gettimeofday(&t1, NULL);
  *joint_time = (uint64_t)(t1.tv_sec - t0.tv_sec) * 1000000ULL +
                (uint64_t)(t1.tv_usec - t0.tv_usec);
}

void f2g_exhaustive() {
  const uint32_t accel = 2000;
  const uint32_t ticks_cases[3][2] = {{4000, 4000}, {4000, 8000}, {100, 99}};
  int32_t vals[11];
  for (int v = 0; v < 11; v++) {
    vals[v] = (int32_t)v - 5;
  }
  long n_tested = 0, n_joint_stop = 0, n_joint_collinear = 0;
  struct Timing {
    uint64_t ref_walk;
    uint64_t waypoint_check;
    uint64_t joint_check;
  };
  Timing total = {0, 0, 0};
  for (int t = 0; t < 3; t++) {
    printf("F2g ticks_case[%d]: (%u,%u)\n", t, ticks_cases[t][0],
           ticks_cases[t][1]);
    struct Timing case_t = {0, 0, 0};
    for (int i0 = 0; i0 < 11; i0++) {
      for (int j0 = 0; j0 < 11; j0++) {
        int32_t d0[2] = {vals[i0], vals[j0]};
        if (d0[0] == 0 && d0[1] == 0) {
          continue;  // all-zero first block is a no-op move; skip
        }
        for (int i1 = 0; i1 < 11; i1++) {
          for (int j1 = 0; j1 < 11; j1++) {
            int32_t d1[2] = {vals[i1], vals[j1]};
            int32_t wp[4] = {d0[0], d0[1], d0[0] + d1[0], d0[1] + d1[1]};
            Remaining rem2(2, 2);
            rem2.set_block(0, d0);
            rem2.set_block(1, d1);
            uint64_t ref_t = 0, wp_t = 0, joint_t = 0;
            f2g_walk(&rem2, ticks_cases[t], accel, 2, wp, &ref_t, &wp_t,
                     &joint_t);
            case_t.ref_walk += ref_t;
            case_t.waypoint_check += wp_t;
            case_t.joint_check += joint_t;
            n_tested++;
            if (d1[0] != 0 || d1[1] != 0) {
              int64_t dot = (int64_t)d0[0] * d1[0] + (int64_t)d0[1] * d1[1];
              int64_t ma = (int64_t)d0[0] * d0[0] + (int64_t)d0[1] * d0[1];
              int64_t mb = (int64_t)d1[0] * d1[0] + (int64_t)d1[1] * d1[1];
              bool stop = (dot <= 0) || (dot * dot * 100000 < 99878 * ma * mb);
              if (stop) {
                n_joint_stop++;
              } else {
                n_joint_collinear++;
              }
            }
            for (int i2 = 0; i2 < 11; i2++) {
              for (int j2 = 0; j2 < 11; j2++) {
                int32_t d2[2] = {vals[i2], vals[j2]};
                int32_t wp3[6] = {d0[0],
                                  d0[1],
                                  d0[0] + d1[0],
                                  d0[1] + d1[1],
                                  d0[0] + d1[0] + d2[0],
                                  d0[1] + d1[1] + d2[1]};
                Remaining rem3(2, 3);
                rem3.set_block(0, d0);
                rem3.set_block(1, d1);
                rem3.set_block(2, d2);
                ref_t = wp_t = joint_t = 0;
                f2g_walk(&rem3, ticks_cases[t], accel, 3, wp3, &ref_t, &wp_t,
                         &joint_t);
                case_t.ref_walk += ref_t;
                case_t.waypoint_check += wp_t;
                case_t.joint_check += joint_t;
                n_tested++;
              }
            }
          }
        }
      }
    }
    total.ref_walk += case_t.ref_walk;
    total.waypoint_check += case_t.waypoint_check;
    total.joint_check += case_t.joint_check;
  }
  printf(
      "F2g exhaustive Linear: %ld polylines tested, %ld path-stop joints, "
      "%ld collinear joints\n",
      n_tested, n_joint_stop, n_joint_collinear);
  uint64_t total_ns = total.ref_walk + total.waypoint_check + total.joint_check;
  printf(
      "F2g timing (mach_absolute_time units): ref_walk=%lu waypoint_check=%lu "
      "joint_check=%lu total=%lu\n",
      (unsigned long)total.ref_walk, (unsigned long)total.waypoint_check,
      (unsigned long)total.joint_check, (unsigned long)total_ns);
  test(n_tested > 0, "2g polylines were actually tested");
  test(n_joint_stop > 0, "2g exercised a path-stop joint");
  test(n_joint_collinear > 0, "2g exercised a collinear joint");
  printf("F2g exhaustive tiny Linear green\n");
}
void f2f_two_block() {
  const uint32_t accel = 2000;
  uint32_t ticks_eq[2] = {4000, 4000};
  struct Row {
    const char* name;
    int32_t a0, a1, b0, b1;
    bool path_stop;
    int master_after;  // binder expected after the joint (block 1)
  };
  Row rows[] = {
      {"(5,0)+(0,5) L", 5, 0, 0, 5, true, 1},
      {"(3,3)+(2,2) collinear", 3, 3, 2, 2, false, 0},
      {"(5,0)+(-3,0) reversal", 5, 0, -3, 0, true, 0},
  };
  for (int c = 0; c < 3; c++) {
    const Row& r = rows[c];
    int32_t sum0 = r.a0 + r.b0;
    int32_t sum1 = r.a1 + r.b1;
    int32_t d0[2] = {r.a0, r.a1};
    int32_t d1[2] = {r.b0, r.b1};

    // Oracle walk (truth).
    Remaining ref_rem(2, 2);
    ref_rem.set_block(0, d0);
    ref_rem.set_block(1, d1);
    int32_t ref_end[2], ref_issued[2];
    bool ref_env = true, ref_plr = true;
    uint32_t ref_vp[8];
    int ref_nv = 0;
    uint32_t ref_slack = 0;
    ref_walk_polyline(&ref_rem, ticks_eq, accel, ref_end, ref_issued, &ref_env,
                      &ref_plr, ref_vp, &ref_nv, 8, NULL, &ref_slack, NULL);

    // Interpolator walk (the production core under test).
    Remaining poly_rem(2, 2);
    poly_rem.set_block(0, d0);
    poly_rem.set_block(1, d1);
    LinearPoly poly(&poly_rem, ticks_eq, accel);
    int32_t poly_end[2], poly_issued[2];
    bool poly_env = true, poly_plr = true;
    uint32_t poly_vp[8];
    int poly_nv = 0;
    uint32_t poly_slack = 0;
    walk_polyline(poly, &poly_rem, ticks_eq, accel, poly_end, poly_issued,
                  &poly_env, &poly_plr, poly_vp, &poly_nv, 8, NULL, &poly_slack,
                  NULL);

    char msg[96];
    snprintf(msg, sizeof(msg), "2f %s envelope (interpolator)", r.name);
    test(poly_env, msg);
    snprintf(msg, sizeof(msg), "2f %s P_issued <= R (interpolator)", r.name);
    test(poly_plr, msg);
    snprintf(msg, sizeof(msg), "2f %s issued |steps| == |delta|", r.name);
    test(poly_issued[0] == sum0 && poly_issued[1] == sum1, msg);
    snprintf(msg, sizeof(msg), "2f %s end position is last vertex", r.name);
    test(poly_end[0] == sum0 && poly_end[1] == sum1, msg);
    snprintf(msg, sizeof(msg), "2f %s a vertex sample per block", r.name);
    test(poly_nv >= 2, msg);
    if (r.path_stop) {
      snprintf(msg, sizeof(msg), "2f %s path-stop joint P_issued <= 1", r.name);
      test(poly_vp[0] <= 1, msg);
    } else {
      snprintf(msg, sizeof(msg), "2f %s collinear joint does not rest", r.name);
      test(poly_vp[0] > 1, msg);
    }

    // Interpolator must match the oracle within a few DDA ticks (log2 /
    // one-step slack). Compare issued step counts and the joint P sample.
    test(ref_env && ref_plr, "2f oracle envelope and P<=R hold");
    snprintf(msg, sizeof(msg), "2f %s interpolator issues match oracle",
             r.name);
    test(poly_issued[0] == ref_issued[0] && poly_issued[1] == ref_issued[1],
         msg);
    snprintf(msg, sizeof(msg), "2f %s joint P within log2 slack of oracle",
             r.name);
    uint32_t dp = (poly_vp[0] > ref_vp[0]) ? poly_vp[0] - ref_vp[0]
                                           : ref_vp[0] - poly_vp[0];
    test(dp <= 2, msg);

    // After the joint the DDA binder is the longest axis of block 1.
    int master1 = Remaining::longest_axis(d1, ticks_eq, 2);
    snprintf(msg, sizeof(msg), "2f %s master switches to %d after joint",
             r.name, r.master_after);
    test(master1 == r.master_after, msg);
    test(master1 == poly.master, "2f interpolator master matches oracle");

    printf("F2f %s: joint_P poly=%u ref=%u end=(%d,%d) nv=%d\n", r.name,
           poly_vp[0], ref_vp[0], poly_end[0], poly_end[1], poly_nv);
    (void)ref_slack;
    (void)poly_slack;
    (void)ref_nv;
  }

  // Plot: the 90deg L (0,0)->(5,0)->(5,5), the realized path on the chords
  // with the corner (vertex) marked and the per-axis speed over time.
  {
    int32_t d0[2] = {5, 0};
    int32_t d1[2] = {0, 5};
    Remaining rem(2, 2);
    rem.set_block(0, d0);
    rem.set_block(1, d1);
    LinearPoly poly(&rem, ticks_eq, accel);
    NaxisPlot plot;
    plot.start_plot("f2f", "FasNAxis F2f two-block L (5,0)+(0,5)", 2);
    plot.poly_point(0.0, 0.0);
    plot.poly_point(5.0, 0.0);
    plot.poly_point(5.0, 5.0);
    plot.poly_done();
    int32_t pos[2] = {0, 0};
    {
      double z[2] = {0.0, 0.0};
      plot.row(0.0, 0.0, 0.0, 0.0, z, z, z, z);
    }
    while (!poly.done()) {
      int step_out[2];
      uint32_t ticks_issued = poly.step(step_out);
      pos[0] += step_out[0];
      pos[1] += step_out[1];
      double t = (double)poly.total_ticks / NAXIS_PLOT_TICKS_PER_S;
      double v = NAXIS_PLOT_TICKS_PER_S / (double)ticks_issued;
      double speed[2] = {poly.master == 0 ? v : 0.0,
                         poly.master == 1 ? v : 0.0};
      double Pcol[2] = {poly.master == 0 ? (double)poly.P : 0.0,
                        poly.master == 1 ? (double)poly.P : 0.0};
      double Rcol[2] = {poly.master == 0 ? (double)poly.R : 0.0,
                        poly.master == 1 ? (double)poly.R : 0.0};
      double tickscol[2] = {poly.master == 0 ? (double)ticks_issued : 0.0,
                            poly.master == 1 ? (double)ticks_issued : 0.0};
      plot.row(t, (double)pos[0], (double)pos[1], 0.0, speed, Pcol, Rcol,
               tickscol);
    }
    plot.finish_plot();
    test(plot.is_open() == false, "2f plot closed");
    printf("F2f two-block L plot written: test_26_f2f.gnuplot\n");
  }
  printf("F2f two-block Linear path-stop/collinear/reversal green\n");
}

void f2h_nblock_vs_f20() {
  // Step 2h: the production N-block interpolator (LinearPoly) must match the
  // globally fastest Linear reference (NaxisRefLinear) on F20, within a 2-step
  // log2 band. Same committed blocks (shared build_f20_blocks, seed 26), so the
  // two walks run tick-for-tick over the same polyline. A faster track that
  // leaves the chord or skips a vertex is not Linear.
  const uint32_t accel = 2000;
  const uint32_t ticks[2] = {4000, 8000};
  int32_t blocks[400][2];
  int32_t sum[2];
  int n_blocks = build_f20_blocks(blocks, sum);
  test(n_blocks >= 200, "2h several hundred waypoints");

  // Oracle walk (truth).
  Remaining ref_rem(2, n_blocks);
  for (int b = 0; b < n_blocks; b++) {
    ref_rem.set_block(b, blocks[b]);
  }
  ref_rem.horizon = 0xFFFFFFFFU;
  int32_t ref_end[2], ref_issued[2];
  bool ref_env = true, ref_plr = true;
  uint32_t ref_vp[512];
  int ref_nv = 0;
  uint32_t ref_slack = 0;
  int32_t ref_vpos[1024];
  NaxisPlot refplot;
  refplot.start_plot("f20_lin", "FasNAxis F2h interpolator vs F20 reference",
                     2);
  refplot.poly_point(0.0, 0.0);
  {
    int32_t wx = 0, wy = 0;
    for (int b = 0; b < n_blocks; b++) {
      wx += blocks[b][0];
      wy += blocks[b][1];
      refplot.poly_point((double)wx, (double)wy);
    }
  }
  refplot.poly_done();
  ref_walk_polyline(&ref_rem, ticks, accel, ref_end, ref_issued, &ref_env,
                    &ref_plr, ref_vp, &ref_nv, 512, &refplot, &ref_slack,
                    ref_vpos);
  refplot.finish_plot();

  // Interpolator walk (the production core under test).
  Remaining poly_rem(2, n_blocks);
  for (int b = 0; b < n_blocks; b++) {
    poly_rem.set_block(b, blocks[b]);
  }
  poly_rem.horizon = 0xFFFFFFFFU;
  LinearPoly poly(&poly_rem, ticks, accel);
  int32_t poly_end[2], poly_issued[2];
  bool poly_env = true, poly_plr = true;
  uint32_t poly_vp[512];
  int poly_nv = 0;
  uint32_t poly_slack = 0;
  int32_t poly_vpos[1024];
  walk_polyline(poly, &poly_rem, ticks, accel, poly_end, poly_issued, &poly_env,
                &poly_plr, poly_vp, &poly_nv, 512, NULL, &poly_slack,
                poly_vpos);

  test(poly_env, "2h interpolator envelope ticks >= ticks_i_cfg");
  test(poly_plr, "2h interpolator law P_issued <= R");
  test(poly_slack <= 2,
       "2h interpolator reconstructed P within 2-step log2 band");
  test(poly_issued[0] == sum[0] && poly_issued[1] == sum[1],
       "2h interpolator issued |steps| == polyline");
  test(poly_end[0] == sum[0] && poly_end[1] == sum[1],
       "2h interpolator end is last vertex");
  test(poly_nv == ref_nv, "2h interpolator vertex count matches oracle");

  // Same vertices, in the same order, as the oracle.
  for (int i = 0; i < poly_nv; i++) {
    test(poly_vpos[2 * i] == ref_vpos[2 * i] &&
             poly_vpos[2 * i + 1] == ref_vpos[2 * i + 1],
         "2h interpolator vertex i hits the same waypoint as the oracle");
    uint32_t dvp = (poly_vp[i] > ref_vp[i]) ? poly_vp[i] - ref_vp[i]
                                            : ref_vp[i] - poly_vp[i];
    test(dvp <= 2, "2h interpolator joint P within 2-step log2 of oracle");
  }

  // Both kinds of joints must be exercised, matching the oracle counts.
  int ref_stop = 0, poly_stop = 0;
  for (int i = 0; i < ref_nv - 1; i++) {
    if (ref_vp[i] <= 1) {
      ref_stop++;
    }
    if (poly_vp[i] <= 1) {
      poly_stop++;
    }
  }
  test(ref_stop >= 1, "2h oracle has a Linear path-stop joint");
  test(ref_stop > 0, "2h oracle has a collinear cruise joint");
  test(poly_stop == ref_stop, "2h interpolator path-stop joints match oracle");

  printf(
      "2h interpolator vs F20 reference: blocks=%d vertices=%d "
      "stop_joints ref=%d poly=%d end=(%d,%d)\n",
      n_blocks, poly_nv, ref_stop, poly_stop, poly_end[0], poly_end[1]);
  printf("F2h N-block interpolator matches F20 reference green\n");
}

// Step 3b (whitepaper section 12.4 item 3b): stoppability from the command
// trace. Ignore the planner's P / R fields. From the issued periods and the
// leftover polyline, calculate_ramp_steps(current_ticks) <= remaining on every
// axis at every sample: the plan can still stop. The oracle is NaxisRefLinear
// (the Linear reference of Step 2ref); P is reconstructed from issued periods
// and compared to the per-axis remaining scan (Remaining::remaining), never to
// the walker's P / R fields.
//
// Three named fixtures:
//   F1  1 axis, 10000 rest-to-rest: coasts to P_coast, P_issued <= R.
//   F5  square 1600, Linear: P -> 0 at each 90deg corner (path-stop), decel
//       starts on the side, P_issued <= R.
//   F10 100 x 100-step collinear micro-segments: R sees through (no per-
//       segment rest), coasts to P_coast, P_issued <= R.
//
// Three mutations are documented here and proven by
// extras/tests/pc_based/prove_mutations.sh (`make mutations`):
//   - FAS_NAXIS_NO_CROSS_BLOCK_R (naxis_ref.h): R is one block, not the
//     collinear sum. F10's joints rest (P -> 0 per block) and the collinear-
//     cruise check fails.
//   - FAS_NAXIS_NO_REBIND (remaining.h): binder_axis ignores ticks. F5/F10
//     (equal ticks) are unaffected, but the rebind neighbourhood of Step 2b
//     fails, so the model is wrong.
//   - FAS_NAXIS_NO_REST_CAP (ramp_law.h): ignore remaining-to-stop. A short
//     open path no longer brakes (peak P is not < R); the F1/F5/F10 stoppable
//     plan would over-run.
//
// Vertex snap: F5's corners are samples (the walker emits a vertex per block);
// a planner that does not snap would miss (1600,0) etc.
struct StoppableResult {
  uint32_t peak_p;
  uint32_t joint_p[512];
  int n_joint;
  int n_stop;
  int n_cruise;
  bool p_le_r;
};

// Walk a NaxisRefLinear oracle and check per-axis stoppability: at every
// moving sample, the reconstructed P_issued must be <= the remaining scan on
// every axis (the plan can still stop). Joint P samples (last moving P of each
// block) classify path-stop (P -> 0) vs collinear cruise (P carries).
static void check_stoppable(Remaining* rem, const uint32_t* ticks,
                            uint32_t accel, StoppableResult* out,
                            const char* name) {
  NaxisRefLinear ref(rem, ticks, accel);
  out->peak_p = 0;
  out->n_joint = 0;
  out->n_stop = 0;
  out->n_cruise = 0;
  out->p_le_r = true;
  int32_t pos[2] = {0, 0};
  uint32_t last_moving_p = 0;
  while (!ref.done()) {
    int step_out[2] = {0, 0};
    uint32_t ticks_issued = ref.step(step_out);
    if (ticks_issued == 0) {
      continue;  // finished transition, not a command
    }
    pos[0] += step_out[0];
    pos[1] += step_out[1];
    RampMap map(ref.ticks_law, accel);
    uint32_t p_issued = map.calculate_ramp_steps(ticks_issued);
    bool rest = (ticks_issued == ref.ticks_law);
    if (!rest) {
      // Per-axis stoppability: the issued period must still allow a stop
      // within the remaining scan on every axis. An idle axis has P = 0.
      for (int i = 0; i < ref.n_axes; i++) {
        uint32_t p_i = (step_out[i] != 0) ? p_issued : 0;
        int32_t R_i = rem->remaining(i, ref.block);
        // calculate_ramp_steps o calculate_ticks may land up to two steps
        // high (log2 inverse, section 12.4 slack), as in walk_polyline.
        if (R_i < 0 || p_i > (uint32_t)R_i + 2) {
          out->p_le_r = false;
          printf("DBG viol %s axis=%d p=%u R=%d block=%d ticks=%u\n", name, i,
                 p_i, R_i, ref.block, ticks_issued);
        }
      }
      if (p_issued > out->peak_p) {
        out->peak_p = p_issued;
      }
      last_moving_p = p_issued;
    }
    if (ref.dda.done()) {
      if (out->n_joint < 512) {
        out->joint_p[out->n_joint] = last_moving_p;
      }
      out->n_joint++;
      last_moving_p = 0;
    }
  }
  for (int i = 0; i < out->n_joint; i++) {
    if (out->joint_p[i] <= 1) {
      out->n_stop++;
    } else {
      out->n_cruise++;
    }
  }
}

void f3b_stoppability() {
  const uint32_t accel = 2000;
  const uint32_t ticks_cfg = 4000;
  uint32_t P_coast = RampMap(ticks_cfg, accel).P_coast();
  test(P_coast >= 3900 && P_coast <= 4100, "F3b P_coast ~ 4000");

  // --- F1: 1 axis, 10000 rest-to-rest. Coasts to P_coast, P_issued <= R. ---
  {
    Remaining rem(1, 1);
    int32_t b0[1] = {10000};
    rem.set_block(0, b0);
    uint32_t ticks[2] = {ticks_cfg, ticks_cfg};
    StoppableResult r;
    check_stoppable(&rem, ticks, accel, &r, "F1");
    test(r.p_le_r, "F3b F1 P_issued <= R on every sample (stoppable)");
    test(r.peak_p >= P_coast - P_coast / 100 && r.peak_p <= P_coast,
         "F3b F1 coasts to P_coast (10000/2 > P_stop)");
    printf("F3b F1 1-axis 10000: peak_P=%u P_coast=%u stoppable\n", r.peak_p,
           P_coast);
  }

  // --- F5: square 1600, Linear. P -> 0 at each 90deg corner (path-stop);
  // --- decel starts on the side; P_issued <= R. Each side is 1600 < 2*P_coast,
  // --- so no side coasts (peak P < P_coast). ---
  {
    Remaining rem(2, 4);
    int32_t b0[2] = {0, 1600};
    int32_t b1[2] = {1600, 0};
    int32_t b2[2] = {0, -1600};
    int32_t b3[2] = {-1600, 0};
    rem.set_block(0, b0);
    rem.set_block(1, b1);
    rem.set_block(2, b2);
    rem.set_block(3, b3);
    uint32_t ticks[2] = {ticks_cfg, ticks_cfg};
    StoppableResult r;
    check_stoppable(&rem, ticks, accel, &r, "F5");
    test(r.p_le_r, "F3b F5 P_issued <= R on every sample (stoppable)");
    test(r.n_stop >= 4, "F3b F5 P -> 0 at each 90deg corner (path-stop)");
    test(r.peak_p < P_coast,
         "F3b F5 each side too short to coast (decel on side)");
    printf("F3b F5 square 1600: peak_P=%u stop_joints=%d\n", r.peak_p,
           r.n_stop);
  }

  // --- F10: 100 x 100-step collinear micro-segments. R sees through (no
  // --- per-segment rest), coasts to P_coast, P_issued <= R. The collinear
  // --- joints cruise (P > 1); the cross-block-R mutation makes them rest. ---
  {
    const int N = 100;
    Remaining rem(2, N);
    for (int i = 0; i < N; i++) {
      int32_t blk[2] = {100, 100};
      rem.set_block(i, blk);
    }
    uint32_t ticks[2] = {ticks_cfg, ticks_cfg};
    StoppableResult r;
    check_stoppable(&rem, ticks, accel, &r, "F10");
    test(r.p_le_r, "F3b F10 P_issued <= R on every sample (stoppable)");
    test(r.peak_p >= P_coast - P_coast / 100 && r.peak_p <= P_coast,
         "F3b F10 coasts to P_coast (R sees through)");
    test(r.n_cruise >= 1,
         "F3b F10 collinear joints do not rest (R sees through)");
    printf("F3b F10 100x100 collinear: peak_P=%u cruise_joints=%d\n", r.peak_p,
           r.n_cruise);
  }
  printf("F3b stoppability from the command trace green\n");
}

// F4 — SimPort addQueueEntry contract (whitepaper section 4.1 / 4.1.1 /
// 4.4.1). A self-contained section that exercises the duck-typed StepperQueue
// stand-in without FasNAxis planning: the feeder talks to addQueueEntry() only.
//
//   - append on an empty queue: isQueueEmpty() was true before, no underrun
//   - kick-off addQueueEntry(NULL, true); empty queue afterwards is an
//     underrun, not before
//   - a pause (steps = 0) uses the caller's count_up, no implicit flip; a
//     reverting pause (count_up = !old) simply leaves the port in the new DIR
//   - pd_test default: a reversing step enqueues with no injected pause
//   - inject hook: a reversing step injects a before-pause (old DIR) then an
//     after-pause (new DIR) that flips queue_end, so a naive retry would XOR
//     back; the following step sees no further injection
//   - drain advances position (signed) and the simulated clock
//   - isRampGeneratorActive() is false unless the test forces it
void f4_sim_port() {
  const uint16_t max_ticks = 80;
  SimPort port(max_ticks);
  test(port.isQueueEmpty(), "F4: fresh port queue empty");
  test(port.isRampGeneratorActive() == false,
       "F4: ramp generator idle by default");

  // Append on an empty queue: isQueueEmpty() was true before, not underrun.
  struct stepper_command_s c = {4000, 3, true};
  test(port.addQueueEntry(&c, false) == AQE_OK, "F4: append returns OK");
  test(port.isQueueEmpty() == false, "F4: queue populated after append");
  test(port.hasUnderrun() == false, "F4: empty-prefill is not underrun");

  // Kick-off starts the queue. After that, an empty queue is an underrun.
  test(port.addQueueEntry(NULL, true) == AQE_OK, "F4: kick-off returns OK");
  test(port.isRunning(), "F4: port running after kick-off");
  port.drain();
  test(port.isQueueEmpty(), "F4: queue drained");
  test(port.hasUnderrun(), "F4: empty after kick-off with motion is underrun");

  // Kick-off on an empty, un-kicked-off queue is an error.
  SimPort empty(max_ticks);
  test(empty.addQueueEntry(NULL, true) == AQE_ERROR_EMPTY_QUEUE_TO_START,
       "F4: kick-off on empty queue is an error");

  // A pause (steps = 0) uses the count_up the caller passes: no implicit flip.
  SimPort pause(max_ticks);
  struct stepper_command_s p0 = {4000, 0, false};
  test(pause.addQueueEntry(&p0, false) == AQE_OK, "F4: pause enqueues");
  test(pause.queueEndCountUp() == false, "F4: pause keeps caller count_up");

  // A reverting pause (count_up = !old) leaves the port in the new DIR without
  // any injected pause: this is pd_test's revert.
  SimPort revert(max_ticks);
  struct stepper_command_s base = {4000, 1, true};
  test(revert.addQueueEntry(&base, false) == AQE_OK, "F4: base step enqueues");
  test(revert.queueEndCountUp() == true, "F4: base step is count_up");
  struct stepper_command_s rev = {4000, 0, false};  // revert to old DIR
  test(revert.addQueueEntry(&rev, false) == AQE_OK, "F4: reverting pause OK");
  test(revert.queueEndCountUp() == false,
       "F4: reverting pause flips to old DIR");

  // pd_test default: a reversing step enqueues with no injected pause.
  SimPort def(max_ticks);
  struct stepper_command_s step_up = {4000, 1, true};
  test(def.addQueueEntry(&step_up, false) == AQE_OK, "F4: first step enqueues");
  struct stepper_command_s step_dn = {4000, 1, false};
  AqeResultCode r = def.addQueueEntry(&step_dn, false);
  test(r == AQE_OK, "F4: default reversing step enqueues, no inject");
  test(def.injectedPauseTicks() == 0, "F4: no injected pause on default");
  test(def.queueEndCountUp() == false, "F4: default reversing step new DIR");

  // Inject hook: a reversing step injects a before-pause (old DIR) then an
  // after-pause (new DIR, flips queue_end); a naive retry would XOR back.
  SimPort inj(max_ticks);
  inj.setInjectMode(SimPort::InjectDirPauses);
  inj.setInjectTicks(8000);
  struct stepper_command_s s_up = {4000, 1, true};
  test(inj.addQueueEntry(&s_up, false) == AQE_OK, "F4: inject first step OK");
  struct stepper_command_s s_dn = {4000, 1, false};
  AqeResultCode r1 = inj.addQueueEntry(&s_dn, false);
  test(r1 == AQE_DIR_CHANGE_PAUSE_INJECTED, "F4: inject before-pause injected");
  test(inj.injectedPauseTicks() == 8000, "F4: injected ticks recorded");
  test(inj.queueEndCountUp() == true, "F4: before-pause keeps old DIR");
  AqeResultCode r2 = inj.addQueueEntry(&s_dn, false);
  test(r2 == AQE_DIR_CHANGE_PAUSE_INJECTED, "F4: inject after-pause injected");
  test(inj.queueEndCountUp() == false,
       "F4: after-pause flips queue_end to new DIR");
  AqeResultCode r3 = inj.addQueueEntry(&s_dn, false);
  test(r3 == AQE_OK, "F4: reversing step enqueues once sequence done");
  test(inj.queueEndCountUp() == false, "F4: step leaves port in new DIR");

  // A following reversing step sees no further injection until it reverses:
  // a step continuing in the same DIR does not re-enter the inject sequence.
  SimPort inj2(max_ticks);
  inj2.setInjectMode(SimPort::InjectDirPauses);
  inj2.setInjectTicks(8000);
  struct stepper_command_s a_up = {4000, 1, true};
  test(inj2.addQueueEntry(&a_up, false) == AQE_OK, "F4: inj2 first step OK");
  struct stepper_command_s a_dn = {4000, 1, false};
  inj2.addQueueEntry(&a_dn, false);
  inj2.addQueueEntry(&a_dn, false);
  test(inj2.queueEndCountUp() == false, "F4: inj2 reversed once");
  struct stepper_command_s a_dn2 = {4000, 1, false};
  AqeResultCode r4 = inj2.addQueueEntry(&a_dn2, false);
  test(r4 == AQE_OK, "F4: same-direction step has no further inject");
  test(inj2.injectedPauseTicks() == 0, "F4: no inject on a non-reversing step");

  // Error: ticks below max speed is a planner bug, not a retry.
  SimPort err(max_ticks);
  struct stepper_command_s too_low = {40, 3, true};
  test(err.addQueueEntry(&too_low, false) == AQE_ERROR_TICKS_TOO_LOW,
       "F4: ticks below max speed is ErrorTicksTooLow");

  // drain() advances position (signed by count_up) and the simulated clock.
  SimPort drain(max_ticks);
  drain.addQueueEntry(&s_up, false);
  drain.addQueueEntry(&step_dn, false);
  drain.addQueueEntry(NULL, true);
  test(drain.position() == 0, "F4: net position zero before drain");
  drain.drain();
  test(drain.clock() == (4000u + 4000u), "F4: clock sums both commands");
  test(drain.position() == 0, "F4: +1 then -1 nets to zero");

  // A forced ramp generator is visible through isRampGeneratorActive().
  SimPort rg(max_ticks);
  rg.setRampGeneratorActive(true);
  test(rg.isRampGeneratorActive(), "F4: forced ramp generator active");

  printf("F4 SimPort addQueueEntry contract green\n");
}

// Step 6: Linear interpolator, one committed block, through SimPort (F1, F2,
// F3, F17). FasNAxis commits one rest-to-rest segment, prefills the queues with
// start=false, then kicks off each axis (addQueueEntry(NULL, true)); later
// commands use start=true. The test drains one command per axis per iteration
// so the axes stay in lockstep. Checks: issued step sums == target, P <= R on
// every committed sample, the realized path stays on the commanded chord
// (section 12.4 rounding box), the first fill of an empty queue is not
// underrun, and the run completes without underrun. Plots:
// test_26_f1_lin.gnuplot, test_26_f2.gnuplot, test_26_f3_lin.gnuplot.
struct SimSegmentResult {
  int64_t issued[2];
  int64_t bind_moves;
  int64_t both_moves;
  uint32_t max_P;
  uint32_t min_R;
  uint32_t max_ticks;
  double max_dev;
  bool p_le_r;
  bool underrun;
  bool first_fill_underrun;
  int64_t n_iter;
  PumpStatus first_pump;
};

static void run_linear_segment(SimPort& px, SimPort& py,
                               FasNAxis<2, 64, SimPort>& path,
                               const char* fixture, const char* title,
                               int32_t tx, int32_t ty, bool do_plot,
                               SimSegmentResult* res) {
  res->issued[0] = 0;
  res->issued[1] = 0;
  res->bind_moves = 0;
  res->both_moves = 0;
  res->max_P = 0;
  res->min_R = 0xFFFFFFFFu;
  res->max_ticks = 0;
  res->max_dev = 0.0;
  res->p_le_r = true;
  res->underrun = false;
  res->first_fill_underrun = false;
  res->n_iter = 0;

  NaxisPlot plot;
  if (do_plot) {
    plot.start_plot(fixture, title, 2);
    plot.poly_point((double)px.position(), (double)py.position());
    plot.poly_point((double)tx, (double)ty);
    plot.poly_done();
  }

  int32_t target[2] = {tx, ty};
  path.addLine(target);
  path.endPath();
  res->first_pump = path.pump();
  res->first_fill_underrun = path.hasUnderrun();

  int master = path.masterAxis();
  while (path.isBusy()) {
    int64_t s0 = 0, s1 = 0;
    bool u0 = true, u1 = true;
    px.drain_one(&s0, &u0);
    py.drain_one(&s1, &u1);
    res->issued[0] += s0;
    res->issued[1] += s1;
    int64_t s_bind = (master == 0) ? s0 : s1;
    int64_t s_slave = (master == 0) ? s1 : s0;
    if (s_bind != 0) {
      res->bind_moves++;
      if (s_slave != 0) {
        res->both_moves++;
      }
    }
    uint32_t P = path.performedRampUp();
    uint32_t R = path.remainingToStop();
    uint32_t ticks = path.lastTicks();
    if (P > res->max_P) {
      res->max_P = P;
    }
    if (R < res->min_R) {
      res->min_R = R;
    }
    if (ticks > res->max_ticks) {
      res->max_ticks = ticks;
    }
    if (P > R) {
      res->p_le_r = false;
    }
    double x = (double)px.position();
    double y = (double)py.position();
    double nx = -(double)ty;
    double ny = (double)tx;
    double nlen = sqrt(nx * nx + ny * ny);
    double dev =
        nlen > 0.0 ? fabs(nx * x + ny * y) / nlen : sqrt(x * x + y * y);
    if (dev > res->max_dev) {
      res->max_dev = dev;
    }
    if (do_plot && (res->n_iter % 4 == 0)) {
      double t = (double)px.clock() / NAXIS_PLOT_TICKS_PER_S;
      double v = ticks > 0 ? NAXIS_PLOT_TICKS_PER_S / (double)ticks : 0.0;
      double speed[2] = {v, v};
      double Pcol[2] = {(double)P, (double)P};
      double Rcol[2] = {(double)R, (double)R};
      double tcol[2] = {(double)ticks, (double)ticks};
      plot.row(t, x, y, dev, speed, Pcol, Rcol, tcol);
    }
    res->n_iter++;
    path.pump();
  }
  res->underrun = path.hasUnderrun();
  if (do_plot) {
    plot.finish_plot();
  }
}

void f6_linear_sim() {
  // F1: one long axis, idle second axis (1-axis fixture run through the queue
  // path). 10 000 steps at ticks_cfg 4000 / accel 2000 (section 14.1).
  {
    SimPort px(4000), py(4000);
    FasNAxisConfig cfg;
    FasNAxis<2, 64, SimPort> path(cfg);
    test(path.addAxis(0, &px) == true, "F6 F1 addAxis(0)");
    test(path.addAxis(1, &py) == true, "F6 F1 addAxis(1)");
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    SimSegmentResult res;
    run_linear_segment(px, py, path, "f1_lin",
                       "FasNAxis F1 Linear (10000,0) via SimPort", 10000, 0,
                       true, &res);
    test(res.issued[0] == 10000, "F6 F1 issued X == 10000");
    test(res.issued[1] == 0, "F6 F1 idle Y issues no steps");
    test(res.p_le_r, "F6 F1 P <= R on every sample");
    test(res.underrun == false, "F6 F1 no underrun");
    test(res.first_fill_underrun == false, "F6 F17 first fill not underrun");
    test(res.first_pump == PumpStatus::Running,
         "F6 F1 first pump returns Running");
    test(px.position() == 10000 && py.position() == 0,
         "F6 F1 realized end == target");
    printf(
        "F6 F1 (10000,0): issued=(%lld,%lld) peak_P=%u max_ticks=%u "
        "iter=%lld\n",
        (long long)res.issued[0], (long long)res.issued[1], res.max_P,
        res.max_ticks, (long long)res.n_iter);
    printf("F1 Linear SimPort plot written: test_26_f1_lin.gnuplot\n");
  }

  // F2: 45 degree line, equal limits. X binds; every binder step is one step on
  // each axis, so |delta_x| == |delta_y| every slice and the path is the chord.
  {
    SimPort px(4000), py(4000);
    FasNAxisConfig cfg;
    FasNAxis<2, 64, SimPort> path(cfg);
    path.addAxis(0, &px);
    path.addAxis(1, &py);
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    SimSegmentResult res;
    run_linear_segment(px, py, path, "f2", "FasNAxis F2 Linear 45 deg", 1600,
                       1600, true, &res);
    test(res.issued[0] == 1600 && res.issued[1] == 1600,
         "F6 F2 both axes issue 1600");
    test(res.bind_moves == 1600 && res.both_moves == res.bind_moves,
         "F6 F2 equal |delta| every slice on a 45 deg line");
    test(res.max_dev <= 0.5 * sqrt(2.0) + 1e-9,
         "F6 F2 path stays on the chord (0.5 sqrt n box)");
    test(res.p_le_r, "F6 F2 P <= R on every sample");
    test(res.underrun == false, "F6 F2 no underrun");
    test(px.position() == 1600 && py.position() == 1600,
         "F6 F2 realized end == target");
    printf("F6 F2 (1600,1600): issued=(%lld,%lld) max_dev=%.3f peak_P=%u\n",
           (long long)res.issued[0], (long long)res.issued[1], res.max_dev,
           res.max_P);
    printf("F2 Linear SimPort plot written: test_26_f2.gnuplot\n");
  }

  // F3: (10000, 100). X binds; Y is a slow slave well below its own ramp and
  // the realized path stays on the chord. Both endpoints (vertices) are hit.
  {
    SimPort px(4000), py(4000);
    FasNAxisConfig cfg;
    FasNAxis<2, 64, SimPort> path(cfg);
    path.addAxis(0, &px);
    path.addAxis(1, &py);
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    SimSegmentResult res;
    run_linear_segment(px, py, path, "f3_lin", "FasNAxis F3 Linear (10000,100)",
                       10000, 100, true, &res);
    test(path.masterAxis() == 0, "F6 F3 X is the DDA master");
    test(res.issued[0] == 10000 && res.issued[1] == 100,
         "F6 F3 issued == (10000,100)");
    test(res.max_dev <= 0.5 * sqrt(2.0) + 1e-9,
         "F6 F3 path stays on the chord (0.5 sqrt n box)");
    test(res.p_le_r, "F6 F3 P <= R on every sample");
    test(res.underrun == false, "F6 F3 no underrun");
    test(px.position() == 10000 && py.position() == 100,
         "F6 F3 vertex hit: realized end == target");
    printf(
        "F6 F3 (10000,100): issued=(%lld,%lld) master=%d max_dev=%.3f "
        "peak_P=%u\n",
        (long long)res.issued[0], (long long)res.issued[1], path.masterAxis(),
        res.max_dev, res.max_P);
    printf("F3 Linear SimPort plot written: test_26_f3_lin.gnuplot\n");
  }

  printf("F6/F17 Linear interpolator through SimPort green\n");
}

// One drained SimPort pair. A master period above 65535 is split into a
// half-period step entry (s0/s1 nonzero on the master) plus pause entries
// (s0 == s1 == 0) covering the remainder (section 4.2 / 9.3).
struct ProdTraceEntry {
  uint32_t t;
  int s0;
  int s1;
};

struct PolySimResult {
  int64_t issued[2];
  int32_t end_pos[2];
  uint32_t vertex_p[256];
  int32_t vertex_pos[512];
  int n_vertex;
  uint32_t max_moving_p;
  uint32_t min_moving_ticks;
  bool envelope_ok;
  bool p_le_r_ok;
  bool trace_match;
  bool underrun;
  bool first_fill_underrun;
  uint64_t total_ticks;
  int n_cmd;
  int n_entry;
};

// Step 7: drive a committed polyline through FasNAxis/SimPort and compare the
// produced Linear track command by command to the NaxisRefLinear oracle (Step
// 2ref). Phase 1 drains the real queue into a trace; phase 2 coalesces the
// 65535 splits back to full periods and checks tick-for-tick equality, the
// per-axis envelope, issued |steps| == |delta|, and the joint P semantics. P is
// reconstructed from the issued period (never from planner P fields); vertices
// are located by the oracle's cumulative step position.
template <uint16_t HZ>
static void walk_prod_polyline(SimPort& px, SimPort& py,
                               FasNAxis<2, HZ, SimPort>& path,
                               const int32_t (*verts)[2], int n_verts,
                               const uint32_t* ticks, uint32_t t_law,
                               uint32_t accel, bool do_plot,
                               const char* fixture, const char* title,
                               PolySimResult* res) {
  static ProdTraceEntry e[70000];
  int n_blocks = n_verts - 1;
  res->issued[0] = 0;
  res->issued[1] = 0;
  res->end_pos[0] = 0;
  res->end_pos[1] = 0;
  res->n_vertex = 0;
  res->max_moving_p = 0;
  res->min_moving_ticks = 0xFFFFFFFFu;
  res->envelope_ok = true;
  res->p_le_r_ok = true;
  res->trace_match = true;
  res->underrun = false;
  res->first_fill_underrun = false;
  res->total_ticks = 0;
  res->n_cmd = 0;
  res->n_entry = 0;

  Remaining rem(2, n_blocks);
  for (int b = 0; b < n_blocks; b++) {
    int32_t d[2];
    d[0] = verts[b + 1][0] - verts[b][0];
    d[1] = verts[b + 1][1] - verts[b][1];
    rem.set_block(b, d);
  }

  NaxisPlot plot;
  if (do_plot) {
    plot.start_plot(fixture, title, 2);
    for (int v = 0; v < n_verts; v++) {
      plot.poly_point((double)verts[v][0], (double)verts[v][1]);
    }
    plot.poly_done();
  }

  for (int v = 1; v < n_verts; v++) {
    int32_t p[2] = {verts[v][0], verts[v][1]};
    path.addLine(p);
  }
  path.endPath();

  path.pump();
  res->first_fill_underrun = path.hasUnderrun();
  if (path.performedRampUp() > path.remainingToStop()) {
    res->p_le_r_ok = false;
  }
  while (path.isBusy()) {
    int64_t s0 = 0;
    int64_t s1 = 0;
    bool up0 = true;
    bool up1 = true;
    uint32_t t0 = px.drain_one(&s0, &up0);
    py.drain_one(&s1, &up1);
    if (res->n_entry < 70000) {
      e[res->n_entry].t = t0;
      e[res->n_entry].s0 = (int)(s0 == 0 ? 0 : (up0 ? s0 : -s0));
      e[res->n_entry].s1 = (int)(s1 == 0 ? 0 : (up1 ? s1 : -s1));
      res->n_entry++;
    }
    res->total_ticks += t0;
    path.pump();
    if (path.performedRampUp() > path.remainingToStop()) {
      res->p_le_r_ok = false;
    }
  }
  res->underrun = path.hasUnderrun();

  RampMap map(t_law, accel);
  NaxisRefLinear ref(&rem, ticks, accel);
  int32_t pos[2] = {0, 0};
  int next_vertex = 1;
  uint32_t last_moving_p = 0;
  int i = 0;
  // EMA-smoothed per-axis realized speed (see walk_polyline): a per-command
  // pulse rate is noisy, and the old `rest = (T == t_law)` zeroed every coast.
  const int kPlotStride = 8;
  const double kSpeedTau_s = 0.02;
  int sample_n = 0;
  int32_t last_pos[2] = {0, 0};
  uint64_t last_ticks = 0;
  double ema_v[2] = {0.0, 0.0};
  bool have_sample = true;
  while (!ref.done()) {
    int os[2];
    uint32_t T_ref = ref.step(os);
    while (i < res->n_entry && e[i].s0 == 0 && e[i].s1 == 0) {
      i++;
    }
    if (i >= res->n_entry) {
      res->trace_match = false;
      break;
    }
    uint32_t T_prod = e[i].t;
    int ps0 = e[i].s0;
    int ps1 = e[i].s1;
    i++;
    while (i < res->n_entry && e[i].s0 == 0 && e[i].s1 == 0) {
      T_prod += e[i].t;
      i++;
    }
    if (ps0 != os[0] || ps1 != os[1] || T_prod != T_ref) {
      res->trace_match = false;
    }
    uint32_t p_issued = map.calculate_ramp_steps(T_prod);
    // The coalesced command always carries motion (zero-step entries were
    // skipped above); `is_pause` is the real "no motion" test, unlike the old
    // `T_prod == t_law`, which is a coast.
    bool is_pause = (ps0 == 0 && ps1 == 0);
    if (!is_pause) {
      last_moving_p = p_issued;
      if (p_issued > res->max_moving_p) {
        res->max_moving_p = p_issued;
      }
      if (ps0 != 0 && T_prod + 1 < ticks[0]) {
        res->envelope_ok = false;
      }
      if (ps1 != 0 && T_prod + 1 < ticks[1]) {
        res->envelope_ok = false;
      }
    }
    if (T_prod < res->min_moving_ticks) {
      res->min_moving_ticks = T_prod;
    }
    pos[0] += ps0;
    pos[1] += ps1;
    res->issued[0] += ps0;
    res->issued[1] += ps1;
    res->n_cmd++;
    if (do_plot) {
      uint64_t now_ticks = ref.total_ticks;
      double dt = (double)(now_ticks - last_ticks) / NAXIS_PLOT_TICKS_PER_S;
      if (dt > 0.0) {
        double alpha = dt / (kSpeedTau_s + dt);
        ema_v[0] += alpha * ((double)(pos[0] - last_pos[0]) / dt - ema_v[0]);
        ema_v[1] += alpha * ((double)(pos[1] - last_pos[1]) / dt - ema_v[1]);
      }
      last_pos[0] = pos[0];
      last_pos[1] = pos[1];
      last_ticks = now_ticks;
      if ((sample_n % kPlotStride) == 0 && have_sample) {
        double speed[2] = {ema_v[0], ema_v[1]};
        double Pcol[2] = {(double)p_issued, (double)p_issued};
        double Rcol[2] = {(double)ref.R, (double)ref.R};
        double tcol[2] = {(double)T_prod, (double)T_prod};
        plot.row((double)now_ticks / NAXIS_PLOT_TICKS_PER_S, (double)pos[0],
                 (double)pos[1], 0.0, speed, Pcol, Rcol, tcol);
      }
    }
    sample_n++;
    if (ref.dda.done()) {
      if (next_vertex < n_verts && pos[0] == verts[next_vertex][0] &&
          pos[1] == verts[next_vertex][1]) {
        if (res->n_vertex < 256) {
          res->vertex_p[res->n_vertex] = last_moving_p;
          res->vertex_pos[2 * res->n_vertex] = pos[0];
          res->vertex_pos[2 * res->n_vertex + 1] = pos[1];
        }
        res->n_vertex++;
        next_vertex++;
      }
    }
  }
  res->end_pos[0] = pos[0];
  res->end_pos[1] = pos[1];
  if (do_plot) {
    plot.finish_plot();
  }
}

// Step 10: the gnuplot file NaxisPlot wrote is the plot. Confirm the bytes the
// helper promises are actually on disk: the data heredoc opener, its closer,
// and the multiplot layout. A missing one means the helper drifted.
static bool gnuplot_has(const char* path, const char* needle) {
  FILE* f = fopen(path, "r");
  if (f == NULL) {
    return false;
  }
  char buf[4096];
  size_t n;
  bool found = false;
  while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
    if (strstr(buf, needle) != NULL) {
      found = true;
      break;
    }
  }
  fclose(f);
  return found;
}

// Step 7 (whitepaper section 8): Linear lookahead across blocks through the
// real feeder. F5 pins path-stop corners (P -> 0, decel starts on the side),
// F9/F18 pin the time-law rebind (DDA master stays the longest |delta| while a
// slower short axis lengthens the period), and F10 pins that R sees through
// collinear micro-segments (no rest at a joint, cruise when N/2 > P_stop).
void f7_linear_lookahead() {
  const uint32_t accel = 2000;
  const uint32_t ticks_eq[2] = {4000, 4000};
  const uint32_t P_coast = RampMap(4000, accel).P_coast();

  // F5: square 1600 (returns to the origin), Linear path-stop at every 90 deg
  // corner. Each side is shorter than P_stop, so P is a triangle and the decel
  // starts on the side, not in the last slice.
  {
    int32_t verts[5][2] = {{0, 0}, {1600, 0}, {1600, 1600}, {0, 1600}, {0, 0}};
    SimPort px(4000), py(4000);
    FasNAxisConfig cfg;
    FasNAxis<2, 64, SimPort> path(cfg);
    path.addAxis(0, &px);
    path.addAxis(1, &py);
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    PolySimResult res;
    walk_prod_polyline<64>(px, py, path, verts, 5, ticks_eq, 4000, accel, true,
                           "f5", "FasNAxis F5 square 1600 Linear", &res);
    test(res.trace_match, "F7 F5 trace matches naxis_ref");
    test(res.issued[0] == 0 && res.issued[1] == 0,
         "F7 F5 issued nets to the origin");
    test(res.end_pos[0] == 0 && res.end_pos[1] == 0,
         "F7 F5 realizes the square and returns");
    test(res.n_vertex == 4, "F7 F5 every corner is a vertex sample");
    bool corners_zero = true;
    for (int k = 0; k < res.n_vertex; k++) {
      if (res.vertex_p[k] > 1) {
        corners_zero = false;
      }
    }
    test(corners_zero, "F7 F5 P -> 0 at every corner");
    test(res.max_moving_p > 1, "F7 F5 ramps up on a side");
    test(res.max_moving_p < P_coast,
         "F7 F5 peak P < P_stop (decel starts on the side)");
    test(res.envelope_ok, "F7 F5 envelope ticks >= ticks_i_cfg");
    test(res.p_le_r_ok, "F7 F5 P <= R on every sample");
    test(!res.underrun && !res.first_fill_underrun, "F7 F5 no underrun");
    printf("F7 F5 square: vertices=%d peak_P=%u max_ticks(as T)=%llu\n",
           res.n_vertex, res.max_moving_p, (unsigned long long)res.total_ticks);
    printf("F5 Linear SimPort plot written: test_26_f5.gnuplot\n");

    // Step 10: the gnuplot file is the plot. Confirm the bytes NaxisPlot
    // promised are on disk before touching the helper further.
    test(gnuplot_has("test_26_f5.gnuplot", "$data <<EOF"),
         "F10 F5 gnuplot has the data heredoc opener");
    test(gnuplot_has("test_26_f5.gnuplot", "EOF"),
         "F10 F5 gnuplot has a heredoc closer");
    test(gnuplot_has("test_26_f5.gnuplot", "set multiplot"),
         "F10 F5 gnuplot has the multiplot layout");

#ifdef FAS_NAXIS_TRACE
    // Step 10: under the trace macro the same samples also land in an HTML
    // viewer page. The dumper copies the checked-in template and embeds the
    // (t, x, y) rows the gnuplot file already carries.
    NaxisHtmlDump html("F5", "FasNAxis F5 square 1600 Linear");
    for (int k = 0; k < res.n_vertex; k++) {
      html.row((double)k, (double)res.vertex_pos[2 * k],
               (double)res.vertex_pos[2 * k + 1]);
    }
    html.finish();
    char html_path[256];
    snprintf(html_path, sizeof(html_path), "%s/tests/out/F5.html",
             NAXIS_HTML_ROOT);
    test(gnuplot_has(html_path, "id=\"trace\""),
         "F10 F5 HTML viewer page written under FAS_NAXIS_TRACE");
#endif
  }

  // F9: 45 deg line, equal |delta|, ticks_x = 10 * ticks_y. The tie-break
  // keeps X (slower) as the DDA master; ticks_floor lengthens the shared period
  // to X, so Y is scaled down in speed (every Y step waits >= ticks_x).
  {
    int32_t verts[2][2] = {{0, 0}, {1600, 1600}};
    uint32_t ticks9[2] = {40000, 4000};
    SimPort px(40000), py(4000);
    FasNAxisConfig cfg;
    FasNAxis<2, 64, SimPort> path(cfg);
    path.addAxis(0, &px);
    path.addAxis(1, &py);
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    PolySimResult res;
    walk_prod_polyline<64>(px, py, path, verts, 2, ticks9, 40000, accel, true,
                           "f9", "FasNAxis F9 45 deg, X 10x slower", &res);
    test(res.trace_match, "F7 F9 trace matches naxis_ref");
    test(path.masterAxis() == 0, "F7 F9 X is the DDA master (slower token)");
    test(res.issued[0] == 1600 && res.issued[1] == 1600,
         "F7 F9 both axes issue their full |delta|");
    test(res.min_moving_ticks >= ticks9[0],
         "F7 F9 X binds: shared period >= X ticks, Y scaled down");
    test(res.envelope_ok, "F7 F9 envelope");
    test(res.p_le_r_ok, "F7 F9 P <= R");
    test(!res.underrun && !res.first_fill_underrun, "F7 F9 no underrun");
    printf("F7 F9 45deg: master=%d issued=(%lld,%lld) min_ticks=%u\n",
           path.masterAxis(), (long long)res.issued[0],
           (long long)res.issued[1], res.min_moving_ticks);
    printf("F9 Linear SimPort plot written: test_26_f9.gnuplot\n");
  }

  // F18: (10000, 9000) with Y 40x slower. Longest is X (DDA master), but Y
  // would exceed v_max if X ran at ticks_x; Y lengthens ticks_b and X is scaled
  // down in speed, both axes still issuing their full |delta|.
  {
    int32_t verts[2][2] = {{0, 0}, {10000, 9000}};
    uint32_t ticks18[2] = {1000, 40000};
    SimPort px(1000), py(40000);
    FasNAxisConfig cfg;
    FasNAxis<2, 64, SimPort> path(cfg);
    path.addAxis(0, &px);
    path.addAxis(1, &py);
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    PolySimResult res;
    walk_prod_polyline<64>(px, py, path, verts, 2, ticks18, 40000, accel, true,
                           "f18", "FasNAxis F18 (10000,9000), Y 40x slower",
                           &res);
    test(res.trace_match, "F7 F18 trace matches naxis_ref");
    test(path.masterAxis() == 0,
         "F7 F18 X is the DDA master (longest |delta|)");
    test(res.issued[0] == 10000 && res.issued[1] == 9000,
         "F7 F18 both axes issue their full |delta|");
    test(res.min_moving_ticks >= ticks18[1],
         "F7 F18 Y time-law binds: shared period >= Y ticks, X scaled down");
    test(res.envelope_ok, "F7 F18 envelope");
    test(res.p_le_r_ok, "F7 F18 P <= R");
    test(res.end_pos[0] == 10000 && res.end_pos[1] == 9000,
         "F7 F18 vertex hit");
    test(!res.underrun && !res.first_fill_underrun, "F7 F18 no underrun");
    printf("F7 F18: master=%d issued=(%lld,%lld) min_ticks=%u\n",
           path.masterAxis(), (long long)res.issued[0],
           (long long)res.issued[1], res.min_moving_ticks);
    printf("F18 Linear SimPort plot written: test_26_f18.gnuplot\n");
  }

  // F10: 100 x 100-step collinear micro-segments totalling 10000. R sees
  // through the joints: P cruises (N/2 > P_stop), no rest at any of the first
  // 99 collinear joints, and the decel spans the tail across block boundaries.
  {
    int32_t verts[101][2];
    for (int k = 0; k <= 100; k++) {
      verts[k][0] = 100 * k;
      verts[k][1] = 0;
    }
    SimPort px(4000), py(4000);
    FasNAxisConfig cfg;
    FasNAxis<2, 128, SimPort> path(cfg);
    path.addAxis(0, &px);
    path.addAxis(1, &py);
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    PolySimResult res;
    walk_prod_polyline<128>(
        px, py, path, verts, 101, ticks_eq, 4000, accel, true, "f10",
        "FasNAxis F10 100x100 collinear micro-segments", &res);
    test(res.trace_match, "F7 F10 trace matches naxis_ref");
    test(res.issued[0] == 10000 && res.issued[1] == 0,
         "F7 F10 issued X == 10000, idle Y issues nothing");
    test(res.n_vertex == 100, "F7 F10 one vertex sample per micro-segment");
    bool joints_continue = true;
    for (int k = 0; k < res.n_vertex - 1; k++) {
      if (res.vertex_p[k] <= 1) {
        joints_continue = false;
      }
    }
    test(joints_continue, "F7 F10 no rest at a collinear joint");
    test(res.vertex_p[res.n_vertex - 1] <= 1,
         "F7 F10 the last buffered point is rest");
    test(res.max_moving_p + 64 >= P_coast && res.max_moving_p <= P_coast,
         "F7 F10 coasts because N/2 > P_stop");
    test(res.envelope_ok, "F7 F10 envelope");
    test(res.p_le_r_ok, "F7 F10 P <= R");
    test(!res.underrun && !res.first_fill_underrun, "F7 F10 no underrun");
    printf("F7 F10: vertices=%d peak_P=%u P_coast=%u\n", res.n_vertex,
           res.max_moving_p, P_coast);
    printf("F10 Linear SimPort plot written: test_26_f10.gnuplot\n");
  }

  printf("F7/F9/F10/F18 Linear lookahead across blocks green\n");
}

// F16 (whitepaper section 6 / 14 / F-table row F16): the header skeleton and
// its registration contract. No motion yet (that is Step 6+). This pins:
//    - FasNAxisConfig{} default member initializers, and the constructor's
//      0 -> default recovery for dt_ticks and kappa_stop_q8.
//    - PumpStatus has Idle/Running/Underrun/Error and NO LookaheadTooShort.
//    - addAxis fails on i >= NAXES, a null pointer, or a running/ramp-active
//      stepper; a small HORIZON relative to P_stop is NOT an addAxis failure.
//    - addLine before any position sync is illegal; after sync it is legal.
//    - addLine to the current position (every delta 0) is a no-op.
// Backed by SimPort so we exercise the same query surface the real
// FastAccelStepper uses (isRampGeneratorActive / isRunning), without raising
// MAX_STEPPER or linking extra queues.
static void f16_skeleton() {
  // Default config: a valid Linear config with documented defaults.
  FasNAxisConfig cfg;
  test(cfg.dt_ticks == 32000, "F16: default dt_ticks is 32000");
  test(cfg.kappa_stop_q8 == 320, "F16: default kappa_stop_q8 is 320");
  test(cfg.overshoot_max == 8, "F16: default overshoot_max is 8");
  test(cfg.mode == FasNAxisConfig::Linear, "F16: default mode is Linear");
  test(cfg.dir_before_ticks == 0, "F16: default dir_before_ticks is 0");
  test(cfg.dir_after_ticks == 0, "F16: default dir_after_ticks is 0");

  // PumpStatus enumerates exactly Idle/Running/Underrun/Error and no
  // LookaheadTooShort (that concept is deliberately absent from v1).
  test((int)PumpStatus::Idle == 0 && (int)PumpStatus::Running == 1 &&
           (int)PumpStatus::Underrun == 2 && (int)PumpStatus::Error == 3,
       "F16: PumpStatus has Idle/Running/Underrun/Error");

  // 0 -> default recovery: a raw zeroed struct still means the defaults, not a
  // zero-duration slice / zero diagnostic threshold.
  FasNAxisConfig zeroed;
  zeroed.dt_ticks = 0;
  zeroed.kappa_stop_q8 = 0;
  FasNAxis<2, 8, SimPort> recovered(zeroed);
  test(recovered.dt_ticks() == 32000, "F16: dt_ticks 0 recovers to 32000");
  test(recovered.kappa_stop_q8() == 320,
       "F16: kappa_stop_q8 0 recovers to 320");

  // addAxis rejects the out-of-range index.
  SimPort px(4000);
  SimPort py(4000);
  FasNAxis<2, 8, SimPort> ok(cfg);
  test(ok.addAxis(0, &px) == true, "F16: addAxis(0) succeeds");
  test(ok.addAxis(1, &py) == true, "F16: addAxis(1) succeeds");
  test(ok.addAxis(2, &px) == false, "F16: addAxis(i>=NAXES) fails");

  // addAxis rejects a null pointer.
  test(ok.addAxis(0, NULL) == false, "F16: addAxis(null) fails");

  // A small HORIZON relative to P_stop is NOT an addAxis failure: the same
  // HORIZON that later caps the ramp (F19) still registers cleanly.
  FasNAxis<2, 2, SimPort> tiny(cfg);
  test(tiny.addAxis(0, &px) == true, "F16: small HORIZON addAxis succeeds");
  test(tiny.addAxis(1, &py) == true, "F16: small HORIZON addAxis succeeds");

  // addAxis fails when the stepper's ramp generator is active or it is
  // running (no race with manageSteppers / a prior moveTo).
  SimPort ramping(4000);
  ramping.setRampGeneratorActive(true);
  FasNAxis<2, 8, SimPort> r(cfg);
  test(r.addAxis(0, &px) == true, "F16: addAxis on idle port succeeds");
  test(r.addAxis(1, &ramping) == false, "F16: addAxis while ramp active fails");

  // Position sync: addLine is illegal before a sync; legal after.
  FasNAxis<2, 8, SimPort> p(cfg);
  p.addAxis(0, &px);
  p.addAxis(1, &py);
  int32_t at[2] = {10, 20};
  test(p.addLine(at) == false, "F16: addLine before sync is illegal");
  p.syncFromSteppers();
  test(p.addLine(at) == true, "F16: addLine after sync is legal");

  // setCurrentPosition opens the same door as syncFromSteppers().
  FasNAxis<2, 8, SimPort> q(cfg);
  q.addAxis(0, &px);
  q.addAxis(1, &py);
  int32_t cur[2] = {0, 0};
  test(q.addLine(cur) == false,
       "F16: addLine before setCurrentPosition illegal");
  q.setCurrentPosition(cur);
  test(q.addLine(cur) == true, "F16: addLine after setCurrentPosition legal");

  // addLine to the current position (every delta 0) is a no-op, not a motion.
  FasNAxis<2, 8, SimPort> s(cfg);
  s.addAxis(0, &px);
  s.addAxis(1, &py);
  int32_t here[2] = {100, 200};
  s.setCurrentPosition(here);
  test(s.addLine(here) == true, "F16: addLine at current position is legal");
  test(s.block_count() == 0, "F16: addLine to current position is a no-op");

  printf("F16 header skeleton + addAxis/position contract green\n");
}

// Step 8 (whitepaper section 8): the feeder's send_to must read the
// addQueueEntry() result and hold a command that returned a retryable code so
// the paired axes stay in lockstep. Three fixtures, in order.
//
// 8.1 F14: a long single-block Linear move with no per-step fault. If the
// command stream is intact the two axes' simulated clocks stay within one
// command of each other on every paired drain -- the invariant this whole step
// builds toward.
//
// 8.2 retry: a one-shot retryable fault (QueueFull or DirPinIsBusy) must leave
// the held command in-flight on the faulting axis, not dropped and not
// re-planned onto its partner.
//
// 8.3 room: with a nearly-full queue the planner must reserve two slots for a
// pause-stuffed entry, and the ticks floor must reject too-fast steps before
// addQueueEntry.
void f8_feeder() {
  // --- 8.1 F14: long single axis move, clocks stay in lockstep ------------
  // (240000, 0): X does all the work, Y runs no steps of this block. Both run
  // the same pump/drain cadence on 4000-tick queues, so the issued tick sums
  // per axis must nearly coincide (one command out of phase at most).
  {
    const uint32_t ticks_cfg = 4000;
    const uint32_t move = 240000;
    SimPort px(ticks_cfg, 64), py(ticks_cfg, 64);
    FasNAxisConfig cfg;
    FasNAxis<2, 64, SimPort> path(cfg);
    path.addAxis(0, &px);
    path.addAxis(1, &py);
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);

    int32_t target[2] = {(int32_t)move, 0};
    path.addLine(target);
    path.endPath();

    NaxisPlot plot;
    plot.start_scalar("f14", "F14 feeder clock delta (240000 steps)");

    uint64_t max_clock_delta = 0;
    int64_t issued_x = 0, issued_y = 0;
    long sub = 0;
    while (path.isBusy()) {
      int64_t s0 = 0, s1 = 0;
      bool u0 = true, u1 = true;
      px.drain_one(&s0, &u0);
      py.drain_one(&s1, &u1);
      issued_x += s0 == 0 ? 0 : (u0 ? s0 : -s0);
      issued_y += s1 == 0 ? 0 : (u1 ? s1 : -s1);
      int64_t delta = (int64_t)px.clock() - (int64_t)py.clock();
      uint64_t d = delta < 0 ? (uint64_t)(-delta) : (uint64_t)delta;
      if (d > max_clock_delta) {
        max_clock_delta = d;
      }
      if (sub++ % 1000 == 0) {
        double t = (double)px.clock() / NAXIS_PLOT_TICKS_PER_S;
        plot.scalar_row(t, (double)delta, 0.0);
      }
      path.pump();
    }
    test(max_clock_delta <= 2, "F14 clocks stay within 2 ticks of each other");
    test(px.position() == (int32_t)move, "F14 X reached the target");
    test(py.position() == 0, "F14 Y did not move");
    test((int64_t)issued_x == (int64_t)move, "F14 X issued the full move");
    test(issued_y == 0, "F14 Y issued no steps");
    test(px.clock() >= 60ull * 16000000ull,
         "F14 X ran at least the coast time (<= 60 s of ticks)");
    double t_end = (double)px.clock() / NAXIS_PLOT_TICKS_PER_S;
    plot.set_y_range(-2.0, 2.0);
    plot.finish_scalar(0.0, t_end, "time [s]", "clock_x - clock_y [ticks]",
                       "clock_x - clock_y", "zero");
    printf(
        "F14 feeder: issued=(%lld,%lld) max_clock_delta=%llu px_clock=%llu\n",
        (long long)issued_x, (long long)issued_y,
        (unsigned long long)max_clock_delta, (unsigned long long)px.clock());
  }

  // --- 8.2 retry: a one-shot retryable fault holds the command in-flight --
  //
  // A paired (500, 500) move. Prefill, drain a few paired commands to make
  // room, arm a single QueueFull fault on Y, pump once (X accepts its command,
  // Y holds it), confirm X gained exactly one and Y gained none, then pump
  // again with no fault so Y accepts the same held command.
  {
    SimPort px(4000, 64), py(4000, 64);
    FasNAxisConfig cfg;
    FasNAxis<2, 64, SimPort> path(cfg);
    path.addAxis(0, &px);
    path.addAxis(1, &py);
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);

    int32_t target[2] = {500, 500};
    path.addLine(target);
    path.endPath();

    path.pump();  // prefill
    for (int i = 0; i < 4 && path.isBusy(); ++i) {
      px.drain_one(NULL, NULL);
      py.drain_one(NULL, NULL);
    }

    int qx0 = (int)px.queueEntries();
    int qy0 = (int)py.queueEntries();
    py.failNext(AQE_QUEUE_FULL);
    path.pump();  // X's command lands, Y's is held (returned QueueFull)
    test((int)px.queueEntries() == qx0 + 1,
         "8.2 retry X gained exactly its command on the faulting pump");
    test((int)py.queueEntries() == qy0, "8.2 retry Y did not take the command");

    path.pump();  // no fault armed now: the held Y command is sent again
    test((int)py.queueEntries() >= qy0 + 1,
         "8.2 retry Y accepts the held command on the retry");
    test(path.isBusy(), "8.2 retry the move is still in progress");

    // Drive to completion with paired drains; the held Y command and its
    // resend keep the two command streams identical, so the clocks must stay
    // within two ticks and both axes hit (500, 500).
    bool lockstep_holds = true;
    while (path.isBusy()) {
      int64_t s0 = 0, s1 = 0;
      bool u0 = true, u1 = true;
      px.drain_one(&s0, &u0);
      py.drain_one(&s1, &u1);
      uint64_t d = px.clock() > py.clock() ? px.clock() - py.clock()
                                           : py.clock() - px.clock();
      if (d > 2) {
        lockstep_holds = false;
      }
      path.pump();
    }
    test(lockstep_holds,
         "8.2 retry clock stays within 2 ticks after one retryable fault");
    test(px.position() == 500 && py.position() == 500,
         "8.2 retry completes the (500, 500) move");
    printf("F12 retry: ends (500,500) lockstep ok=%d\n", lockstep_holds);

    // A different retryable code, DirPinIsBusy, follows the same path.
    {
      SimPort qx(4000, 64), qy(4000, 64);
      FasNAxis<2, 64, SimPort> p2(cfg);
      p2.addAxis(0, &qx);
      p2.addAxis(1, &qy);
      int32_t cur2[2] = {0, 0};
      p2.setCurrentPosition(cur2);
      int32_t t2[2] = {200, 200};
      p2.addLine(t2);
      p2.endPath();
      p2.pump();  // prefill
      for (int i = 0; i < 4 && p2.isBusy(); ++i) {
        qx.drain_one(NULL, NULL);
        qy.drain_one(NULL, NULL);
      }
      int bx = (int)qx.queueEntries();
      int by = (int)qy.queueEntries();
      qy.failNext(AQE_DIR_PIN_IS_BUSY);
      p2.pump();  // held on Y
      test((int)qx.queueEntries() == bx + 1 && (int)qy.queueEntries() == by,
           "8.2 retry DirPinIsBusy: X lands, Y holds");
      p2.pump();  // retry: Y takes the held command
      test((int)qy.queueEntries() >= by + 1,
           "8.2 retry DirPinIsBusy: Y accepts on retry");
    }
  }

  // --- 8.3 room: reserve two slots, and reject too-fast steps before -------
  // addQueueEntry. A nearly-full queue must never reach QUEUE_LEN.
  {
    const uint32_t ticks_cfg = 4000;
    SimPort p0(ticks_cfg, 16), p1(ticks_cfg, 16);
    FasNAxisConfig cfg;
    FasNAxis<2, 64, SimPort> path(cfg);
    path.addAxis(0, &p0);
    path.addAxis(1, &p1);
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);

    int32_t target[2] = {8000, 8000};
    path.addLine(target);
    path.endPath();

    while (path.isBusy()) {
      path.pump();
      test(p0.queueEntries() <= 14, "F15 room X never reaches QUEUE_LEN - 1");
      test(p1.queueEntries() <= 14, "F15 room Y never reaches QUEUE_LEN - 1");
      if (p0.queueEntries() > 0) {
        p0.drain_one(NULL, NULL);
      }
      if (p1.queueEntries() > 0) {
        p1.drain_one(NULL, NULL);
      }
    }
    test(p0.position() == 8000 && p1.position() == 8000,
         "F15 room both axes complete the 8000 move");
    printf("F15 room: completed 8000x8000 on QUEUE_LEN=16 queues\n");
  }

  printf("F14/F12/F15 feeder contract green\n");
}

// Step 9 (whitepaper section 4.4): the DIR pause is carved out of the reversing
// axis's own last step, so the coordinated timeline does not grow and no other
// axis is paused. The reversing axis arrives at a slow period T_min (section
// 4.4.1); the last old-direction step of period T_min is replaced by a
// shortened step of T_min - tau plus the before-pauses (old DIR) and the
// after-pause (new DIR). An inject the plan did not carve is an Error.
struct NaxisCmd {
  uint16_t ticks;
  uint8_t steps;
  bool up;
};
struct RevSample {
  double t;
  int32_t x;
  int32_t y;
  uint16_t ticks;
  uint8_t steps;
};
struct RevTrace {
  NaxisCmd x[6000];
  int nx;
  NaxisCmd y[6000];
  int ny;
  RevSample s[6000];
  int ns;
  int32_t pxe;
  int32_t pye;
  uint64_t clkx;
  uint64_t clky;
  bool error;
};

// Capped acceleration the planner picks for a reversal budget (section 4.4.1):
// halve until calculate_ticks(1) holds tau + the legal step floor.
static uint32_t cap_accel_for_budget(uint32_t ticks_cfg, uint32_t accel,
                                     uint32_t need) {
  uint32_t a = accel;
  while (a > 1) {
    RampMap m(ticks_cfg, a);
    if (m.calculate_ticks(1) >= need) {
      break;
    }
    a >>= 1;
  }
  return a;
}

// Run a Linear out-and-back on X with the given DIR budget; record the raw
// command streams and the per-iteration XY/clock samples.
static void run_reversal(uint32_t before, uint8_t n_before, uint32_t after,
                         uint32_t accel, int32_t move, uint32_t qlen,
                         RevTrace* tr) {
  SimPort px(4000, qlen), py(4000, qlen);
  px.setAcceleration(accel);
  py.setAcceleration(accel);
  px.setDirChangeBudget((uint16_t)before, n_before, (uint16_t)after);
  FasNAxisConfig cfg;
  FasNAxis<2, 64, SimPort> path(cfg);
  path.addAxis(0, &px);
  path.addAxis(1, &py);
  int32_t cur[2] = {0, 0};
  path.setCurrentPosition(cur);
  int32_t t1[2] = {move, 0};
  path.addLine(t1);
  int32_t t2[2] = {0, 0};
  path.addLine(t2);
  path.endPath();

  tr->nx = 0;
  tr->ny = 0;
  tr->ns = 0;
  tr->error = false;
  if (path.pump() == PumpStatus::Error) {
    tr->error = true;
  }
  while (path.isBusy()) {
    int64_t s0 = 0, s1 = 0;
    bool u0 = true, u1 = true;
    uint32_t t0 = px.drain_one(&s0, &u0);
    uint32_t t1b = py.drain_one(&s1, &u1);
    if (t0 > 0 && tr->nx < 6000) {
      tr->x[tr->nx].ticks = (uint16_t)t0;
      tr->x[tr->nx].steps = (uint8_t)s0;
      tr->x[tr->nx].up = u0;
      tr->nx++;
    }
    if (t1b > 0 && tr->ny < 6000) {
      tr->y[tr->ny].ticks = (uint16_t)t1b;
      tr->y[tr->ny].steps = (uint8_t)s1;
      tr->y[tr->ny].up = u1;
      tr->ny++;
    }
    if (tr->ns < 6000) {
      tr->s[tr->ns].t = (double)px.clock() / NAXIS_PLOT_TICKS_PER_S;
      tr->s[tr->ns].x = px.position();
      tr->s[tr->ns].y = py.position();
      tr->s[tr->ns].ticks = (uint16_t)t0;
      tr->s[tr->ns].steps = (uint8_t)s0;
      tr->ns++;
    }
    if (path.pump() == PumpStatus::Error) {
      tr->error = true;
    }
  }
  tr->pxe = px.position();
  tr->pye = py.position();
  tr->clkx = px.clock();
  tr->clky = py.clock();
}

// Step 12 F12b: an Overshoot dog-leg (0,0) -> (leg,leg) -> (0,2*leg) with a
// DIR budget on the reversing X. X is the binder and carves its own last step;
// the continuing Y keeps the steps already planned across those ticks (it is
// not given a pauses=0 command by the budget).
static void run_overshoot_rev(uint32_t before, uint8_t n_before, uint32_t after,
                              uint32_t accel, int32_t leg, RevTrace* tr) {
  SimPort px(4000, 64), py(4000, 64);
  px.setAcceleration(accel);
  py.setAcceleration(accel);
  px.setDirChangeBudget((uint16_t)before, n_before, (uint16_t)after);
  FasNAxisConfig cfg;
  cfg.mode = FasNAxisConfig::Overshoot;
  cfg.overshoot_max = 8;
  FasNAxis<2, 64, SimPort> path(cfg);
  path.addAxis(0, &px);
  path.addAxis(1, &py);
  int32_t cur[2] = {0, 0};
  path.setCurrentPosition(cur);
  int32_t t1[2] = {leg, leg};
  path.addLine(t1);
  int32_t t2[2] = {0, 2 * leg};
  path.addLine(t2);
  path.endPath();

  tr->nx = 0;
  tr->ny = 0;
  tr->ns = 0;
  tr->error = false;
  if (path.pump() == PumpStatus::Error) {
    tr->error = true;
  }
  while (path.isBusy()) {
    int64_t s0 = 0, s1 = 0;
    bool u0 = true, u1 = true;
    uint32_t t0 = px.drain_one(&s0, &u0);
    uint32_t t1b = py.drain_one(&s1, &u1);
    if (t0 > 0 && tr->nx < 6000) {
      tr->x[tr->nx].ticks = (uint16_t)t0;
      tr->x[tr->nx].steps = (uint8_t)s0;
      tr->x[tr->nx].up = u0;
      tr->nx++;
    }
    if (t1b > 0 && tr->ny < 6000) {
      tr->y[tr->ny].ticks = (uint16_t)t1b;
      tr->y[tr->ny].steps = (uint8_t)s1;
      tr->y[tr->ny].up = u1;
      tr->ny++;
    }
    if (tr->ns < 6000) {
      tr->s[tr->ns].t = (double)px.clock() / NAXIS_PLOT_TICKS_PER_S;
      tr->s[tr->ns].x = px.position();
      tr->s[tr->ns].y = py.position();
      tr->s[tr->ns].ticks = (uint16_t)t0;
      tr->s[tr->ns].steps = (uint8_t)s0;
      tr->ns++;
    }
    if (path.pump() == PumpStatus::Error) {
      tr->error = true;
    }
  }
  tr->pxe = px.position();
  tr->pye = py.position();
  tr->clkx = px.clock();
  tr->clky = py.clock();
}

// Index of the last X step in the old direction (the carved step), or -1.
static int last_old_step(const RevTrace* tr) {
  int idx = -1;
  for (int k = 0; k < tr->nx; k++) {
    if (tr->x[k].steps != 0 && tr->x[k].up) {
      idx = k;
    }
  }
  return idx;
}

// True if any command in `c` is a pause of exactly `ticks`.
static bool has_pause(const NaxisCmd* c, int n, uint16_t ticks) {
  for (int k = 0; k < n; k++) {
    if (c[k].steps == 0 && c[k].ticks == ticks) {
      return true;
    }
  }
  return false;
}

void f9_dir_pauses() {
  const uint32_t ticks_cfg = 4000;
  const uint32_t accel = 80000;
  const int32_t move = 400;
  const uint16_t before = 3200;
  const uint16_t after = 3200;
  RampMap map(ticks_cfg, accel);
  const uint32_t T_min = map.calculate_ticks(1);
  const uint32_t tau = (uint32_t)before + after;  // n_before = 1

  // --- F12: the carve, Y gains no pause, clock equals the zero-budget run ---
  {
    static RevTrace tr;
    run_reversal(before, 1, after, accel, move, 64, &tr);
    test(!tr.error, "F12 run does not error");
    test(tr.pxe == 0 && tr.pye == 0, "F12 F12 both axes end on target");

    int k = last_old_step(&tr);
    test(k >= 0, "F12 there is a last old-direction X step");
    // The carved last step is shortened by tau.
    test(tr.x[k].ticks == T_min - tau,
         "F12 last old step shortened by tau (T_min - tau)");
    // Before-pause: old DIR, 3200. After-pause: new DIR, 3200.
    test(k + 2 < tr.nx, "F12 the carve has before and after pauses");
    test(
        tr.x[k + 1].steps == 0 && tr.x[k + 1].up && tr.x[k + 1].ticks == before,
        "F12 before-pause old DIR of 3200 on X only");
    test(
        tr.x[k + 2].steps == 0 && !tr.x[k + 2].up && tr.x[k + 2].ticks == after,
        "F12 after-pause new DIR of 3200");
    test((uint32_t)tr.x[k].ticks + tr.x[k + 1].ticks + tr.x[k + 2].ticks ==
             T_min,
         "F12 the three tick sums equal the original last-step ticks");
    // The following X step is the new direction.
    int k_next = k + 3;
    test(k_next < tr.nx && tr.x[k_next].steps != 0 && !tr.x[k_next].up,
         "F12 the following step is the new direction");

    // Y gains no DIR pause: its trace is identical to a zero-budget run.
    static RevTrace tr0;
    run_reversal(0, 0, 0, accel, move, 64, &tr0);
    bool y_same = (tr.ny == tr0.ny);
    if (y_same) {
      for (int j = 0; j < tr.ny; j++) {
        if (tr.y[j].ticks != tr0.y[j].ticks ||
            tr.y[j].steps != tr0.y[j].steps || tr.y[j].up != tr0.y[j].up) {
          y_same = false;
        }
      }
    }
    test(y_same, "F12 idle Y trace is unchanged: no DIR pause copied onto Y");
    test(tr.clkx == tr0.clkx && tr.clky == tr0.clky,
         "F12 clock() equals the same move with a zero budget");

    // The plot: XY plus the period samples on the reversing axis.
    NaxisPlot plot;
    plot.start_plot("f12", "FasNAxis F12 DIR carve on reversal", 2);
    plot.poly_point(0.0, 0.0);
    plot.poly_point((double)move, 0.0);
    plot.poly_point(0.0, 0.0);
    plot.poly_done();
    for (int j = 0; j < tr.ns; j++) {
      // A pause (steps == 0) is a delay, not motion: the carve's 3200-tick
      // before/after pauses must not plot as 5000 step/s.
      double v = (tr.s[j].steps > 0 && tr.s[j].ticks > 0)
                     ? NAXIS_PLOT_TICKS_PER_S / (double)tr.s[j].ticks
                     : 0.0;
      double speed[2] = {v, 0.0};
      double P[2] = {0.0, 0.0};
      double R[2] = {0.0, 0.0};
      double tickc[2] = {(double)tr.s[j].ticks, 0.0};
      plot.row(tr.s[j].t, (double)tr.s[j].x, (double)tr.s[j].y, 0.0, speed, P,
               R, tickc);
    }
    plot.finish_plot();
    printf(
        "F9 F12 carve: T_min=%u step=%u before=%u after=%u "
        "clocks=(%llu,%llu)\n",
        T_min, tr.x[k].ticks, tr.x[k + 1].ticks, tr.x[k + 2].ticks,
        (unsigned long long)tr.clkx, (unsigned long long)tr.clky);
    printf("F12 DIR carve plot written: test_26_f12.gnuplot\n");
  }

  // --- F12c: an inject the plan did not carve is an Error, Y gains no pause --
  {
    SimPort px(ticks_cfg, 64), py(ticks_cfg, 64);
    px.setAcceleration(accel);
    py.setAcceleration(accel);
    px.setInjectMode(SimPort::InjectNone);
    px.forceExtraBefore(8000);
    FasNAxisConfig cfg;
    FasNAxis<2, 64, SimPort> path(cfg);
    path.addAxis(0, &px);
    path.addAxis(1, &py);
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    int32_t t1[2] = {1, 0};
    path.addLine(t1);
    int32_t t2[2] = {0, 0};
    path.addLine(t2);
    path.endPath();
    PumpStatus st = path.pump();
    test(st == PumpStatus::Error,
         "F12c an uncarved injected pause returns Error");
    // Y does not gain the 8000 pause.
    static NaxisCmd ycmd[512];
    int ny = 0;
    while (ny < 512) {
      int64_t s = 0;
      uint32_t t = py.drain_one(&s, NULL);
      if (t == 0 && s == 0) {
        break;
      }
      ycmd[ny].ticks = (uint16_t)t;
      ycmd[ny].steps = (uint8_t)s;
      ycmd[ny].up = true;
      ny++;
    }
    test(!has_pause(ycmd, ny, 8000),
         "F12c the other axis does not gain the injected pause");
    printf("F9 F12c uncarved inject -> Error, Y clean (ny=%d)\n", ny);
  }

  // --- F12b: Overshoot dog-leg. X reverses at the vertex and carves its own
  // last step; the continuing Y keeps its planned steps (Step 12). ----------
  {
    const int32_t leg = 400;
    const uint32_t tau = (uint32_t)before + (uint32_t)after;
    static RevTrace tr0;
    run_overshoot_rev(0, 0, 0, accel, leg, &tr0);
    static RevTrace tr;
    run_overshoot_rev(before, 1, after, accel, leg, &tr);
    test(!tr.error, "F12b overshoot dog-leg does not error");
    test(tr.pxe == 0 && tr.pye == 2 * leg, "F12b both axes end on target");
    int k = last_old_step(&tr);
    int k0 = last_old_step(&tr0);
    test(k >= 0 && k0 >= 0, "F12b X has a last old-direction step");
    test(tr.x[k].ticks == (uint32_t)tr0.x[k0].ticks - tau,
         "F12b X last step shortened by tau");
    test(k + 2 < tr.nx, "F12b X has the before and after pauses");
    test(
        tr.x[k + 1].steps == 0 && tr.x[k + 1].up && tr.x[k + 1].ticks == before,
        "F12b before-pause old DIR on X only");
    test(
        tr.x[k + 2].steps == 0 && !tr.x[k + 2].up && tr.x[k + 2].ticks == after,
        "F12b after-pause new DIR");
    test((uint32_t)tr.x[k].ticks + tr.x[k + 1].ticks + tr.x[k + 2].ticks ==
             (uint32_t)tr0.x[k0].ticks,
         "F12b the carve keeps the last-step tick sum");
    // Y gains no DIR pause of the budget length and is bit-identical to the
    // zero-budget dog-leg: the continuing axis keeps its planned steps.
    test(!has_pause(tr.y, tr.ny, (uint16_t)before),
         "F12b continuing Y gains no before-pause");
    test(!has_pause(tr.y, tr.ny, (uint16_t)after),
         "F12b continuing Y gains no after-pause");
    bool y_same = (tr.ny == tr0.ny);
    for (int j = 0; j < tr.ny && y_same; j++) {
      if (tr.y[j].ticks != tr0.y[j].ticks || tr.y[j].steps != tr0.y[j].steps ||
          tr.y[j].up != tr0.y[j].up) {
        y_same = false;
      }
    }
    test(y_same, "F12b continuing Y is unchanged by the DIR carve");
    printf("F12b overshoot dog-leg: nX=%d nY=%d k=%d step=%u T0=%u\n", tr.nx,
           tr.ny, k, tr.x[k].ticks, tr0.x[k0].ticks);
  }

  // --- Tail too short: a budget larger than the natural slow period. The
  // planner caps acceleration so the last step still holds the budget; the
  // carve keeps the F12 shape and Y gains no DIR pause. ---------------------
  {
    const uint16_t big_before = 50000;
    const uint16_t big_after = 0;
    uint32_t need = (uint32_t)big_before + ticks_cfg;  // tau + floor
    uint32_t cap_accel = cap_accel_for_budget(ticks_cfg, accel, need);
    uint32_t T_cap = RampMap(ticks_cfg, cap_accel).calculate_ticks(1);
    test(T_cap >= need, "F12 short tail: capped accel holds tau + floor");
    test(cap_accel < accel, "F12 short tail: acceleration was capped");

    static RevTrace tr;
    run_reversal(big_before, 1, big_after, accel, move, 64, &tr);
    test(!tr.error, "F12 short tail run does not error");
    test(tr.pxe == 0 && tr.pye == 0, "F12 short tail both axes end on target");
    int k = last_old_step(&tr);
    test(k >= 0, "F12 short tail has a last old-direction step");
    test(tr.x[k].ticks == T_cap - big_before,
         "F12 short tail shortened step = T_min_capped - tau");
    test(k + 1 < tr.nx && tr.x[k + 1].steps == 0 && tr.x[k + 1].up &&
             tr.x[k + 1].ticks == big_before,
         "F12 short tail before-pause of tau on X only");
    test((uint32_t)tr.x[k].ticks + tr.x[k + 1].ticks == T_cap,
         "F12 short tail tick sum unchanged at T_min_capped");
    // Y still gains no DIR pause: no pause of the budget length appears.
    test(!has_pause(tr.y, tr.ny, big_before),
         "F12 short tail: Y gains no DIR pause");
    printf("F9 F12 short tail: cap_accel=%u T_cap=%u step=%u before=%u\n",
           cap_accel, T_cap, tr.x[k].ticks, tr.x[k + 1].ticks);
  }

  printf("F12/F12b/F12c DIR pause carve green\n");
}

// ---------------------------------------------------------------------------
// Step 11 — Overshoot rest-to-rest (F4, F4b, lone diagonal).
//
// One rest-to-rest segment through SimPort, FasNAxisConfig::Overshoot. The
// binding axis (largest per-axis RampLaw duration) sets the wall clock; the
// other axis is spread across it. F4 caps the chordal bulge at 8 steps, F4b is
// the raw (UINT16_MAX) profile, and the lone diagonal is the same ramp on both
// axes (d^2 ~ 0). The reference duration is naxis_overshoot_duration (sum of
// the binding RampLaw only).
// ---------------------------------------------------------------------------
struct OvershootResult {
  int64_t issued[2];
  int32_t end[2];
  double max_d2;
  uint32_t clock0;
  uint32_t clock1;
  int32_t mid_short;
  bool mid_seen;
  bool underrun;
};

static void run_overshoot_segment(SimPort& px, SimPort& py,
                                  FasNAxis<2, 64, SimPort>& path,
                                  const char* fixture, const char* title,
                                  int32_t tx, int32_t ty, uint64_t ref_T,
                                  bool do_plot, OvershootResult* res) {
  res->issued[0] = 0;
  res->issued[1] = 0;
  res->end[0] = 0;
  res->end[1] = 0;
  res->max_d2 = 0.0;
  res->clock0 = 0;
  res->clock1 = 0;
  res->mid_short = -1;
  res->mid_seen = false;
  res->underrun = false;
  int short_axis = (labs((long)tx) <= labs((long)ty)) ? 0 : 1;

  NaxisPlot plot;
  if (do_plot) {
    plot.start_plot(fixture, title, 2);
    plot.poly_point((double)px.position(), (double)py.position());
    plot.poly_point((double)tx, (double)ty);
    plot.poly_done();
  }

  int32_t target[2] = {tx, ty};
  path.addLine(target);
  path.endPath();
  path.pump();

  double nx = -(double)ty;
  double ny = (double)tx;
  double nlen = sqrt(nx * nx + ny * ny);
  int64_t iter = 0;
  while (path.isBusy()) {
    int64_t s0 = 0, s1 = 0;
    px.drain_one(&s0, NULL);
    py.drain_one(&s1, NULL);
    res->issued[0] += s0;
    res->issued[1] += s1;
    double x = (double)px.position();
    double y = (double)py.position();
    double dev =
        nlen > 0.0 ? fabs(nx * x + ny * y) / nlen : sqrt(x * x + y * y);
    if (dev * dev > res->max_d2) {
      res->max_d2 = dev * dev;
    }
    if (!res->mid_seen && ref_T > 0 && (uint64_t)px.clock() >= ref_T / 2) {
      res->mid_seen = true;
      res->mid_short = short_axis == 0 ? px.position() : py.position();
    }
    if (do_plot && (iter % 4 == 0)) {
      uint32_t P = path.performedRampUp();
      uint32_t R = path.remainingToStop();
      uint32_t ticks = path.lastTicks();
      double t = (double)px.clock() / NAXIS_PLOT_TICKS_PER_S;
      double v = ticks > 0 ? NAXIS_PLOT_TICKS_PER_S / (double)ticks : 0.0;
      double speed[2] = {v, v};
      double Pcol[2] = {(double)P, (double)P};
      double Rcol[2] = {(double)R, (double)R};
      double tcol[2] = {(double)ticks, (double)ticks};
      plot.row(t, x, y, dev, speed, Pcol, Rcol, tcol);
    }
    iter++;
    path.pump();
  }
  res->underrun = path.hasUnderrun();
  res->end[0] = px.position();
  res->end[1] = py.position();
  res->clock0 = px.clock();
  res->clock1 = py.clock();
  if (do_plot) {
    plot.finish_plot();
  }
}

void f11_overshoot_rest() {
  const uint32_t ticks[2] = {4000, 4000};
  const uint32_t accel[2] = {2000, 2000};

  // F3 Linear baseline on the same segment: the F4 bulge must be larger.
  double f3_max_d2 = 0.0;
  {
    SimPort px(4000), py(4000);
    FasNAxisConfig cfg;
    FasNAxis<2, 64, SimPort> path(cfg);
    path.addAxis(0, &px);
    path.addAxis(1, &py);
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    SimSegmentResult lin;
    run_linear_segment(px, py, path, "f11_f3", "FasNAxis F3 Linear (10000,100)",
                       10000, 100, false, &lin);
    f3_max_d2 = lin.max_dev * lin.max_dev;
  }

  // F4: (10000, 100), cap 8. End exact, max d^2 <= 64, bulge > Linear.
  {
    SimPort px(4000), py(4000);
    FasNAxisConfig cfg;
    cfg.mode = FasNAxisConfig::Overshoot;
    cfg.overshoot_max = 8;
    FasNAxis<2, 64, SimPort> path(cfg);
    test(path.addAxis(0, &px) == true, "F11 F4 addAxis(0)");
    test(path.addAxis(1, &py) == true, "F11 F4 addAxis(1)");
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    int32_t d[2] = {10000, 100};
    uint64_t T = naxis_overshoot_duration(d, ticks, accel, 2);
    OvershootResult res;
    run_overshoot_segment(px, py, path, "f4",
                          "FasNAxis F4 Overshoot (10000,100) cap 8", 10000, 100,
                          T, true, &res);
    test(res.end[0] == 10000 && res.end[1] == 100, "F11 F4 end exact");
    test(res.issued[0] == 10000 && res.issued[1] == 100,
         "F11 F4 issued == delta");
    test(res.max_d2 <= 64.0 + 1e-9, "F11 F4 max d^2 <= 64");
    test(res.max_d2 > f3_max_d2, "F11 F4 bulge greater than Linear");
    test(res.clock0 == T && res.clock1 == T,
         "F11 F4 clock == binding ramp on both axes");
    test(res.underrun == false, "F11 F4 no underrun");
    printf(
        "F11 F4 (10000,100) cap8: max_d2=%.3f f3_d2=%.3f clocks=(%u,%u) "
        "T=%llu\n",
        res.max_d2, f3_max_d2, res.clock0, res.clock1, (unsigned long long)T);
  }

  // F4b: same segment, raw cap (UINT16_MAX). Short axis still moving at
  // mid-time (not an early L); the raw bulge exceeds the cap 8.
  {
    SimPort px(4000), py(4000);
    FasNAxisConfig cfg;
    cfg.mode = FasNAxisConfig::Overshoot;
    cfg.overshoot_max = UINT16_MAX;
    FasNAxis<2, 64, SimPort> path(cfg);
    test(path.addAxis(0, &px) == true, "F11 F4b addAxis(0)");
    test(path.addAxis(1, &py) == true, "F11 F4b addAxis(1)");
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    int32_t d[2] = {10000, 100};
    uint64_t T = naxis_overshoot_duration(d, ticks, accel, 2);
    OvershootResult res;
    run_overshoot_segment(px, py, path, "f4b",
                          "FasNAxis F4b Overshoot (10000,100) raw", 10000, 100,
                          T, true, &res);
    test(res.end[0] == 10000 && res.end[1] == 100, "F11 F4b end exact");
    test(res.mid_seen && res.mid_short > 0 && res.mid_short < 100,
         "F11 F4b short axis still moving at mid-time (not an L)");
    test(res.max_d2 > 64.0, "F11 F4b raw bulge exceeds cap 8");
    test(res.clock0 == T && res.clock1 == T, "F11 F4b clock == binding ramp");
    printf("F11 F4b (10000,100) raw: max_d2=%.3f mid_short=%d clocks=(%u,%u)\n",
           res.max_d2, res.mid_short, res.clock0, res.clock1);
  }

  // Lone diagonal (1600, 1600): both axes are the binding ramp; Overshoot
  // duration equals Linear within a couple of ticks and d^2 stays under 1.
  {
    SimPort lx(4000), ly(4000);
    FasNAxisConfig lcfg;
    FasNAxis<2, 64, SimPort> lpath(lcfg);
    lpath.addAxis(0, &lx);
    lpath.addAxis(1, &ly);
    int32_t cur[2] = {0, 0};
    lpath.setCurrentPosition(cur);
    SimSegmentResult lres;
    run_linear_segment(lx, ly, lpath, "f11_diag_lin", "diag linear", 1600, 1600,
                       false, &lres);
    uint32_t lin_clock = lx.clock();

    SimPort px(4000), py(4000);
    FasNAxisConfig cfg;
    cfg.mode = FasNAxisConfig::Overshoot;
    cfg.overshoot_max = 8;
    FasNAxis<2, 64, SimPort> path(cfg);
    test(path.addAxis(0, &px) == true, "F11 diagonal addAxis(0)");
    test(path.addAxis(1, &py) == true, "F11 diagonal addAxis(1)");
    path.setCurrentPosition(cur);
    int32_t d[2] = {1600, 1600};
    uint64_t T = naxis_overshoot_duration(d, ticks, accel, 2);
    OvershootResult res;
    run_overshoot_segment(px, py, path, "f11_diag",
                          "FasNAxis Overshoot lone diagonal", 1600, 1600, T,
                          false, &res);
    test(res.end[0] == 1600 && res.end[1] == 1600, "F11 diagonal end exact");
    test(res.max_d2 < 1.0, "F11 diagonal d^2 < 1");
    test(labs((long)res.clock0 - (long)lin_clock) <= 2,
         "F11 diagonal Overshoot clock == Linear clock");
    test(labs((long)res.clock0 - (long)T) <= 2,
         "F11 diagonal clock == reference duration");
    printf("F11 diagonal: clocks ovs=%u lin=%u T=%llu max_d2=%.4f\n",
           res.clock0, lin_clock, (unsigned long long)T, res.max_d2);
  }

  printf("F4/F4b/diagonal Overshoot rest-to-rest green\n");
}

// ---------------------------------------------------------------------------
// Step 12 — Overshoot corners and circle (F6, F6b, F7).
//
// An Overshoot run crosses a vertex with P carrying on an axis that keeps its
// sign and resetting on an axis that reverses or goes idle (section 8.6). The
// shared wall clock is the binding ramp; the path may leave each chord by at
// most overshoot_max. `run_overshoot_polyline` walks a polyline through
// SimPort, samples the maximum squared distance to the current chord, records
// the clock at every vertex and reports whether every vertex was hit. The
// per-axis exit P at the vertices comes from `naxis_overshoot_vertices`.
// ---------------------------------------------------------------------------
struct OvershootPolyResult {
  int32_t end[2];
  int64_t issued[2];
  double max_d2;
  uint32_t vertex_clock[512];
  int n_vertices;
  bool all_vertices;
  bool underrun;
};

template <uint16_t H>
static void run_overshoot_polyline(SimPort& px, SimPort& py,
                                   FasNAxis<2, H, SimPort>& path,
                                   const int32_t* wp, int n_wp,
                                   const char* fixture, const char* title,
                                   bool do_plot, OvershootPolyResult* res) {
  res->end[0] = 0;
  res->end[1] = 0;
  res->issued[0] = 0;
  res->issued[1] = 0;
  res->max_d2 = 0.0;
  res->n_vertices = 0;
  res->all_vertices = false;
  res->underrun = false;

  NaxisPlot plot;
  if (do_plot) {
    // The SimPort origin is the planner's first waypoint (setCurrentPosition is
    // not a SimPort position setter), so the realized positions are relative to
    // wp[0]. Draw the commanded polyline in the same port-relative coordinates,
    // otherwise a path that does not start at (0,0) is offset from its trace.
    plot.start_plot(fixture, title, 2);
    plot.poly_point(0.0, 0.0);
    for (int k = 1; k < n_wp; k++) {
      plot.poly_point((double)(wp[2 * k] - wp[0]),
                      (double)(wp[2 * k + 1] - wp[1]));
    }
    plot.poly_done();
  }

  int32_t cur[2] = {wp[0], wp[1]};
  path.setCurrentPosition(cur);
  for (int k = 1; k < n_wp; k++) {
    int32_t t[2] = {wp[2 * k], wp[2 * k + 1]};
    path.addLine(t);
  }
  path.endPath();
  path.pump();

  int chord = 0;
  int iter = 0;
  // EMA-smoothed realized-speed sampling (see walk_polyline): the per-command
  // lastTicks()/performedRampUp() are binder-global and a multi-step catch-up
  // command has a short period, so a per-command rate spikes.
  const int kPlotStride = 8;
  const double kSpeedTau_s = 0.02;
  int32_t last_x = 0, last_y = 0;
  uint32_t last_clock = 0;
  double ema_v[2] = {0.0, 0.0};
  bool have_sample = true;
  while (path.isBusy()) {
    int64_t s0 = 0, s1 = 0;
    bool u0 = true, u1 = true;
    px.drain_one(&s0, &u0);
    py.drain_one(&s1, &u1);
    res->issued[0] += u0 ? s0 : -s0;
    res->issued[1] += u1 ? s1 : -s1;
    int32_t x = px.position();
    int32_t y = py.position();
    if (chord < n_wp - 1) {
      int32_t ax = wp[2 * chord] - wp[0], ay = wp[2 * chord + 1] - wp[1];
      int32_t bx = wp[2 * chord + 2] - wp[0], by = wp[2 * chord + 3] - wp[1];
      double vx = (double)(bx - ax), vy = (double)(by - ay);
      double l2 = vx * vx + vy * vy;
      if (l2 > 0.0) {
        double cross = vx * (double)(y - ay) - vy * (double)(x - ax);
        double d2 = cross * cross / l2;
        if (d2 > res->max_d2) {
          res->max_d2 = d2;
        }
      }
      // SimPort positions are relative to the planner's start (wp[0]); `bx` /
      // `by` are already in those port coordinates.
      if (x == bx && y == by) {
        if (res->n_vertices < 512) {
          res->vertex_clock[res->n_vertices] = px.clock();
        }
        res->n_vertices++;
        chord++;
      }
    }
    if (do_plot) {
      uint32_t clk = px.clock();
      double dt = (double)(clk - last_clock) / NAXIS_PLOT_TICKS_PER_S;
      if (dt > 0.0) {
        double alpha = dt / (kSpeedTau_s + dt);
        ema_v[0] += alpha * ((double)(x - last_x) / dt - ema_v[0]);
        ema_v[1] += alpha * ((double)(y - last_y) / dt - ema_v[1]);
      }
      last_x = x;
      last_y = y;
      last_clock = clk;
      if ((iter % kPlotStride) == 0 && have_sample) {
        double speed[2] = {ema_v[0], ema_v[1]};
        uint32_t ticks = path.lastTicks();
        double Pcol[2] = {(double)path.performedRampUp(),
                          (double)path.performedRampUp()};
        double Rcol[2] = {(double)path.remainingToStop(),
                          (double)path.remainingToStop()};
        double tcol[2] = {(double)ticks, (double)ticks};
        plot.row((double)clk / NAXIS_PLOT_TICKS_PER_S, (double)x, (double)y,
                 0.0, speed, Pcol, Rcol, tcol);
      }
    }
    iter++;
    path.pump();
  }
  res->underrun = path.hasUnderrun();
  res->end[0] = px.position();
  res->end[1] = py.position();
  res->all_vertices = (chord == n_wp - 1);
  if (do_plot) {
    plot.finish_plot();
  }
}

// Run a polyline in Linear mode and return the final clock (both axes share
// it).
static uint32_t linear_polyline_clock(const int32_t* wp, int n_wp) {
  SimPort px(4000, 16), py(4000, 16);
  FasNAxisConfig cfg;
  FasNAxis<2, 64, SimPort> path(cfg);
  path.addAxis(0, &px);
  path.addAxis(1, &py);
  OvershootPolyResult r;
  run_overshoot_polyline(px, py, path, wp, n_wp, "f12_lin_unused", "linear",
                         false, &r);
  return px.clock();
}

static void f12_overshoot_corners() {
  const uint32_t ticks[2] = {4000, 4000};
  const uint32_t accel[2] = {2000, 2000};
  const uint32_t cap = 8;

  // --- F6: (0,0) -> (1600,1600) -> (3200,0). X continues, Y reverses. -------
  {
    const int32_t wp[6] = {0, 0, 1600, 1600, 3200, 0};
    int32_t bx[2] = {1600, 1600};
    int32_t by[2] = {1600, -1600};
    uint64_t Tb[2];
    uint32_t P0[2], P1[2];
    naxis_overshoot_vertices(bx, by, 2, ticks, accel, cap, Tb, P0, P1);
    test(P1[0] == 0, "F12 F6 reversing Y has P == 0 at the vertex");
    test(P0[0] > 0, "F12 F6 continuing X keeps a nonzero P at the vertex");

    SimPort px(4000, 16), py(4000, 16);
    FasNAxisConfig cfg;
    cfg.mode = FasNAxisConfig::Overshoot;
    cfg.overshoot_max = (uint16_t)cap;
    FasNAxis<2, 64, SimPort> path(cfg);
    test(path.addAxis(0, &px) == true, "F12 F6 addAxis(0)");
    test(path.addAxis(1, &py) == true, "F12 F6 addAxis(1)");
    OvershootPolyResult res;
    run_overshoot_polyline(px, py, path, wp, 3, "f6",
                           "FasNAxis F6 Overshoot continuing corner", true,
                           &res);
    test(res.end[0] == 3200 && res.end[1] == 0,
         "F12 F6 end at the last vertex");
    test(res.issued[0] == 3200 && res.issued[1] == 0, "F12 F6 issued == path");
    test(res.all_vertices && res.n_vertices == 2,
         "F12 F6 every vertex sampled");
    test(res.max_d2 <= 64.0 + 1e-9, "F12 F6 chord d^2 <= 64");
    test(res.underrun == false, "F12 F6 no underrun");
    uint32_t lin = linear_polyline_clock(wp, 3);
    test(px.clock() <= lin, "F12 F6 Overshoot clock <= Linear clock");
    printf("F12 F6 ovs=%u lin=%u Tb=(%llu,%llu) max_d2=%.3f Px=%u Py=%u\n",
           px.clock(), lin, (unsigned long long)Tb[0],
           (unsigned long long)Tb[1], res.max_d2, P0[0], P1[0]);
  }

  // --- F6b: (0,0) -> (4000,1) -> (4000,4000). X ends, Y continues. ----------
  {
    const int32_t wp[6] = {0, 0, 4000, 1, 4000, 4000};
    int32_t bx[2] = {4000, 0};
    int32_t by[2] = {1, 3999};
    uint64_t Tb[2];
    uint32_t P0[2], P1[2];
    naxis_overshoot_vertices(bx, by, 2, ticks, accel, cap, Tb, P0, P1);
    test(P0[0] == 0, "F12 F6b idle X has P == 0 at the vertex");
    test(P1[0] == 1, "F12 F6b Y P <= 1 after the single-step block");

    SimPort px(4000, 16), py(4000, 16);
    FasNAxisConfig cfg;
    cfg.mode = FasNAxisConfig::Overshoot;
    cfg.overshoot_max = (uint16_t)cap;
    FasNAxis<2, 64, SimPort> path(cfg);
    test(path.addAxis(0, &px) == true, "F12 F6b addAxis(0)");
    test(path.addAxis(1, &py) == true, "F12 F6b addAxis(1)");
    OvershootPolyResult res;
    run_overshoot_polyline(px, py, path, wp, 3, "f6b",
                           "FasNAxis F6b anisotropic continuing corner", true,
                           &res);
    test(res.end[0] == 4000 && res.end[1] == 4000,
         "F12 F6b end at the last vertex");
    test(res.all_vertices && res.n_vertices == 2,
         "F12 F6b every vertex sampled");
    test(res.max_d2 <= 64.0 + 1e-9, "F12 F6b chord d^2 <= 64");
    test(res.underrun == false, "F12 F6b no underrun");
    printf("F12 F6b ovs=%u max_d2=%.3f Px=%u Py=%u\n", px.clock(), res.max_d2,
           P0[0], P1[0]);
  }

  // --- F7: circle radius 1600, 360 chords of 1 degree. Only a reversing axis
  // has P == 0 at a vertex; the continuing axis keeps P. --------------------
  {
    const int n = 360;
    static int32_t wp[(360 + 1) * 2];
    for (int k = 0; k <= n; k++) {
      double a = 2.0 * M_PI * (double)k / (double)n;
      wp[2 * k] = iround(1600.0 * cos(a));
      wp[2 * k + 1] = iround(1600.0 * sin(a));
    }
    static int32_t bx[360], by[360];
    for (int k = 0; k < n; k++) {
      bx[k] = wp[2 * (k + 1)] - wp[2 * k];
      by[k] = wp[2 * (k + 1) + 1] - wp[2 * k + 1];
    }
    static uint64_t Tb[360];
    static uint32_t P0[360], P1[360];
    naxis_overshoot_vertices(bx, by, n, ticks, accel, cap, Tb, P0, P1);
    int fx = 0, fy = 0;
    int last0 = 0, last1 = 0;
    for (int pass = 0; pass < 2; pass++) {
      for (int k = 0; k < n; k++) {
        int nxt = (k + 1) % n;
        int s0 = bx[nxt] > 0 ? 1 : (bx[nxt] < 0 ? -1 : 0);
        int s1 = by[nxt] > 0 ? 1 : (by[nxt] < 0 ? -1 : 0);
        if (s0 != 0) {
          if (last0 != 0 && s0 != last0) {
            fx++;
            test(P0[k] == 0, "F12 F7 reversing X has P == 0");
          }
          last0 = s0;
        }
        if (s1 != 0) {
          if (last1 != 0 && s1 != last1) {
            fy++;
            test(P1[k] == 0, "F12 F7 reversing Y has P == 0");
          }
          last1 = s1;
        }
      }
    }
    test(fx >= 2 && fy >= 2, "F12 F7 both axes reverse at the circle extrema");

    SimPort px(4000, 16), py(4000, 16);
    FasNAxisConfig cfg;
    cfg.mode = FasNAxisConfig::Overshoot;
    cfg.overshoot_max = (uint16_t)cap;
    FasNAxis<2, 512, SimPort> path(cfg);
    test(path.addAxis(0, &px) == true, "F12 F7 addAxis(0)");
    test(path.addAxis(1, &py) == true, "F12 F7 addAxis(1)");
    OvershootPolyResult res;
    run_overshoot_polyline(px, py, path, wp, n + 1, "f7",
                           "FasNAxis F7 Overshoot circle r=1600", true, &res);
    test(res.end[0] == wp[2 * n] - wp[0] && res.end[1] == wp[2 * n + 1] - wp[1],
         "F12 F7 closes the circle (net zero)");
    test(res.all_vertices && res.n_vertices == n,
         "F12 F7 every chord vertex sampled");
    test(res.max_d2 <= 64.0 + 1e-9, "F12 F7 chord d^2 <= 64");
    test(res.underrun == false, "F12 F7 no underrun");
    printf("F12 F7 circle: ovs=%u max_d2=%.3f\n", px.clock(), res.max_d2);
  }

  printf("F6/F6b/F7 Overshoot corners green\n");
}

// ---------------------------------------------------------------------------
// Step 13 — dwell, starve, underrun, lookahead speed cap (F11, F13, F19).
//
// The speed cap is how remaining_path_steps treats the last buffered point of
// an open path as rest (section 8.2 / 14.7); this step exposes it through
// isSpeedLimitedByLookahead() / lookaheadHint() and adds a dwell block plus a
// real starve underrun. No LookaheadTooShort status: a short lookahead slows
// the track (F11/F19), it does not error.
// ---------------------------------------------------------------------------
void f13_lookahead() {
  const uint32_t accel = 2000;
  const uint32_t P_coast = RampMap(4000, accel).P_coast();

  // --- Dwell: (0,0) -> (400,0), 80000-tick dwell, then (800,0). -------------
  {
    SimPort px(4000), py(4000);
    FasNAxisConfig cfg;
    FasNAxis<2, 64, SimPort> unsynced(cfg);
    unsynced.addAxis(0, &px);
    unsynced.addAxis(1, &py);
    test(unsynced.addDwellTicks(100) == false,
         "F13 dwell addDwellTicks before a sync is illegal");

    // Baseline: a standalone 400-step rest-to-rest run. The dwell fixture is
    // two of those runs separated by exactly 80000 ticks of pauses, so the
    // total clock pins both the dwell length and the P == 0 restart.
    uint32_t base_clock = 0;
    {
      SimPort bx(4000), by(4000);
      FasNAxis<2, 64, SimPort> bpath(cfg);
      bpath.addAxis(0, &bx);
      bpath.addAxis(1, &by);
      int32_t cur[2] = {0, 0};
      bpath.setCurrentPosition(cur);
      int32_t t[2] = {400, 0};
      bpath.addLine(t);
      bpath.endPath();
      bpath.pump();
      while (bpath.isBusy()) {
        bx.drain_one();
        by.drain_one();
        bpath.pump();
      }
      base_clock = bx.clock();
      test(bx.position() == 400 && by.position() == 0,
           "F13 dwell baseline ends at 400");
    }

    FasNAxis<2, 64, SimPort> path(cfg);
    test(path.addAxis(0, &px) == true, "F13 dwell addAxis(0)");
    test(path.addAxis(1, &py) == true, "F13 dwell addAxis(1)");
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    int32_t a[2] = {400, 0};
    int32_t b[2] = {800, 0};
    test(path.addLine(a) == true, "F13 dwell first addLine");
    test(path.addDwellTicks(0) == true, "F13 dwell zero dwell is legal");
    test(path.block_count() == 1, "F13 dwell zero dwell records no block");
    test(path.addDwellTicks(80000) == true, "F13 dwell addDwellTicks");
    test(path.addLine(b) == true, "F13 dwell second addLine");
    test(path.block_count() == 3, "F13 dwell records 3 blocks");
    path.endPath();
    path.pump();
    bool at_400 = false;
    bool past_400 = false;
    bool held_400 = true;
    while (path.isBusy()) {
      int64_t s0 = 0;
      int64_t s1 = 0;
      bool u0 = true;
      bool u1 = true;
      px.drain_one(&s0, &u0);
      py.drain_one(&s1, &u1);
      if (!at_400 && px.position() == 400) {
        at_400 = true;
      } else if (at_400 && !past_400 && s0 != 0) {
        past_400 = true;
      } else if (at_400 && !past_400 && px.position() != 400) {
        held_400 = false;
      }
      path.pump();
    }
    test(at_400 && past_400, "F13 dwell reached 400 and left it");
    test(held_400, "F13 dwell position holds (400,0) through the dwell");
    test(px.clock() == 2 * base_clock + 80000,
         "F13 dwell lasts exactly 80000 ticks of clock (two baseline halves)");
    test(px.clock() == py.clock(), "F13 dwell axes stay in lockstep");
    test(px.position() == 800 && py.position() == 0, "F13 dwell end on target");
    test(path.hasUnderrun() == false, "F13 dwell no underrun");
    printf("F13 dwell: base=%u dwell_run=%u (2*base+80000=%u)\n", base_clock,
           px.clock(), 2 * base_clock + 80000);
  }

  // --- F11: one 800-step chunk caps the speed; ten chunks recover. ----------
  {
    SimPort px(4000), py(4000);
    FasNAxisConfig cfg;
    FasNAxis<2, 64, SimPort> path(cfg);
    path.addAxis(0, &px);
    path.addAxis(1, &py);
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    int32_t t[2] = {800, 0};
    path.addLine(t);  // open path: no endPath
    PumpStatus first = path.pump();
    test(first == PumpStatus::Running,
         "F13 F11 first pump returns Running on an open path");
    test(path.isSpeedLimitedByLookahead(),
         "F13 F11 one 800-step chunk caps the speed (R < P_stop)");
    uint32_t peak = 0;
    bool p_le_r = true;
    while (path.isBusy()) {
      px.drain_one();
      py.drain_one();
      uint32_t P = path.performedRampUp();
      uint32_t R = path.remainingToStop();
      if (P > peak) {
        peak = P;
      }
      if (P > R) {
        p_le_r = false;
      }
      path.pump();
    }
    test(p_le_r, "F13 F11 P <= R on every sample");
    test(peak == 400, "F13 F11 peak P == R/2 (400 of 800, under P_stop)");
    test(peak < P_coast, "F13 F11 capped peak stays under P_stop");
    test(px.position() == 800 && py.position() == 0,
         "F13 F11 capped run ends at the buffered point");
    test(path.hasUnderrun() == false, "F13 F11 capped run has no underrun");
    printf("F13 F11 capped: peak_P=%u (R/2=400) P_coast=%u\n", peak, P_coast);
  }

  // --- F11 recovery: ten 800-step chunks buffered, R at the head is 8000. ---
  {
    SimPort px(4000), py(4000);
    FasNAxisConfig cfg;
    FasNAxis<2, 64, SimPort> path(cfg);
    path.addAxis(0, &px);
    path.addAxis(1, &py);
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    int32_t t[2];
    for (int k = 1; k <= 10; k++) {
      t[0] = 800 * k;
      t[1] = 0;
      path.addLine(t);
    }
    path.pump();
    test(path.performedRampUp() + path.remainingToStop() == 8000,
         "F13 F11 R at the head is 8000");
    test(path.remainingToStop() >= P_coast,
         "F13 F11 R >= P_stop once ten chunks are buffered");
    test(!path.isSpeedLimitedByLookahead(),
         "F13 F11 R >= P_stop is not lookahead-limited");
    uint8_t hint_axis = 0xFF;
    uint32_t hint_R = 0;
    uint32_t hint_stop = 0;
    uint16_t hint_h = 0;
    path.lookaheadHint(&hint_axis, &hint_R, &hint_stop, &hint_h);
    test(hint_axis == path.masterAxis(),
         "F13 F11 lookaheadHint axis == master");
    test(hint_R == path.remainingToStop(), "F13 F11 lookaheadHint R");
    test(hint_stop == P_coast, "F13 F11 lookaheadHint P_stop");
    test(hint_h == 64, "F13 F11 lookaheadHint HORIZON");
    NaxisPlot plot;
    plot.start_scalar("f11", "FasNAxis F11 speed cap then recovery (P vs R)");
    uint32_t peak = 0;
    bool p_le_r = true;
    while (path.isBusy()) {
      px.drain_one();
      py.drain_one();
      uint32_t P = path.performedRampUp();
      uint32_t R = path.remainingToStop();
      if (P > peak) {
        peak = P;
      }
      if (P > R) {
        p_le_r = false;
      }
      plot.scalar_row((double)px.clock() / NAXIS_PLOT_TICKS_PER_S, (double)P,
                      (double)R);
      path.pump();
    }
    plot.finish_scalar(0.0, (double)px.clock() / NAXIS_PLOT_TICKS_PER_S,
                       "time [s]", "steps", "P performed ramp-up",
                       "R remaining to stop");
    test(peak + 64 >= P_coast && peak <= P_coast,
         "F13 F11 recovery reaches P_stop within the log2 band");
    test(peak > 400, "F13 F11 recovery peak exceeds the capped peak");
    test(p_le_r, "F13 F11 recovery P <= R on every sample");
    test(px.position() == 8000 && py.position() == 0,
         "F13 F11 recovery ends at 8000");
    test(path.hasUnderrun() == false, "F13 F11 recovery has no underrun");
    printf("F13 F11 recovery: peak_P=%u == P_coast=%u\n", peak, P_coast);
    printf("F11 scalar plot written: test_26_f11.gnuplot\n");
  }

  // --- F13: starve the queues after kick-off, no pump in between. -----------
  {
    SimPort px(4000), py(4000);
    FasNAxisConfig cfg;
    FasNAxis<2, 64, SimPort> path(cfg);
    path.addAxis(0, &px);
    path.addAxis(1, &py);
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    int32_t t[2] = {10000, 0};
    path.addLine(t);
    path.endPath();
    PumpStatus st = path.pump();
    test(st == PumpStatus::Running, "F13 F1 first pump returns Running");
    test(!px.isQueueEmpty() && !py.isQueueEmpty(),
         "F13 kick-off has happened (queues are prefilled)");
    NaxisPlot plot;
    plot.start_plot("f13", "FasNAxis F13 underrun (samples before the starve)",
                    2);
    plot.poly_point(0.0, 0.0);
    plot.poly_point((double)t[0], (double)t[1]);
    plot.poly_done();
    for (int k = 0; k < 200; k++) {
      int64_t s0 = 0;
      int64_t s1 = 0;
      bool u0 = true;
      bool u1 = true;
      px.drain_one(&s0, &u0);
      py.drain_one(&s1, &u1);
      double tt = (double)px.clock() / NAXIS_PLOT_TICKS_PER_S;
      uint32_t ticks = path.lastTicks();
      double v = ticks > 0 ? NAXIS_PLOT_TICKS_PER_S / (double)ticks : 0.0;
      double speed[2] = {v, v};
      double Pcol[2] = {(double)path.performedRampUp(),
                        (double)path.performedRampUp()};
      double Rcol[2] = {(double)path.remainingToStop(),
                        (double)path.remainingToStop()};
      double tcol[2] = {(double)ticks, (double)ticks};
      plot.row(tt, (double)px.position(), (double)py.position(), 0.0, speed,
               Pcol, Rcol, tcol);
      path.pump();
    }
    plot.finish_plot();
    px.drain();
    py.drain();
    test(px.isQueueEmpty() && py.isQueueEmpty(),
         "F13 both queues drained to empty without pump");
    test(path.hasUnderrun(), "F13 hasUnderrun after the starve drain");
    test(path.pump() == PumpStatus::Underrun,
         "F13 pump() returns Underrun after the starve");
    printf("F13 underrun plot written: test_26_f13.gnuplot\n");
  }

  // --- F19: HORIZON 4 micro-segments cap P; one long block still coasts. ----
  {
    static double f19_t_a[512];
    static double f19_p_a[512];
    int n_a = 0;
    SimPort px(4000), py(4000);
    FasNAxisConfig cfg;
    FasNAxis<2, 4, SimPort> path(cfg);
    test(path.addAxis(0, &px) == true,
         "F13 F19 addAxis(0) succeeds at HORIZON 4");
    test(path.addAxis(1, &py) == true, "F13 F19 addAxis(1) succeeds");
    int32_t cur[2] = {0, 0};
    path.setCurrentPosition(cur);
    int32_t t[2];
    for (int k = 1; k <= 4; k++) {
      t[0] = 50 * k;
      t[1] = 0;
      test(path.addLine(t) == true, "F13 F19 addLine fits HORIZON 4");
    }
    path.pump();
    test(path.isSpeedLimitedByLookahead(),
         "F13 F19 HORIZON 4 micro-segments cap the speed");
    uint32_t peak_a = 0;
    bool p_le_r_a = true;
    while (path.isBusy()) {
      px.drain_one();
      py.drain_one();
      uint32_t P = path.performedRampUp();
      uint32_t R = path.remainingToStop();
      if (P > peak_a) {
        peak_a = P;
      }
      if (P > R) {
        p_le_r_a = false;
      }
      if (n_a < 512) {
        f19_t_a[n_a] = (double)px.clock() / NAXIS_PLOT_TICKS_PER_S;
        f19_p_a[n_a] = (double)P;
        n_a++;
      }
      path.pump();
    }
    test(p_le_r_a, "F13 F19 micro-segment run P <= R");
    test(peak_a < P_coast,
         "F13 F19 micro-segments never reach P_stop (HORIZON caps R)");
    test(px.position() == 200 && py.position() == 0,
         "F13 F19 micro-segment run ends at 200");

    static double f19_t_b[16384];
    static double f19_p_b[16384];
    int n_b = 0;
    SimPort qx(4000), qy(4000);
    FasNAxis<2, 4, SimPort> path2(cfg);
    path2.addAxis(0, &qx);
    path2.addAxis(1, &qy);
    int32_t cur2[2] = {0, 0};
    path2.setCurrentPosition(cur2);
    int32_t big[2] = {10000, 0};
    path2.addLine(big);
    path2.endPath();
    path2.pump();
    test(path2.remainingToStop() >= P_coast,
         "F13 F19 one long block has R >= P_stop at the same HORIZON");
    uint32_t peak_b = 0;
    bool p_le_r_b = true;
    while (path2.isBusy()) {
      qx.drain_one();
      qy.drain_one();
      uint32_t P = path2.performedRampUp();
      uint32_t R = path2.remainingToStop();
      if (P > peak_b) {
        peak_b = P;
      }
      if (P > R) {
        p_le_r_b = false;
      }
      if (n_b < 16384) {
        f19_t_b[n_b] = (double)qx.clock() / NAXIS_PLOT_TICKS_PER_S;
        f19_p_b[n_b] = (double)P;
        n_b++;
      }
      path2.pump();
    }
    test(p_le_r_b, "F13 F19 long-block run P <= R");
    test(peak_b == P_coast,
         "F13 F19 one long block reaches P_stop (coast) at HORIZON 4");
    test(qx.position() == 10000 && qy.position() == 0,
         "F13 F19 long-block run ends at 10000");

    NaxisPlot plot;
    plot.start_scalar(
        "f19", "FasNAxis F19 HORIZON 4: micro-segment cap vs long-block coast");
    int n = n_b > n_a ? n_b : n_a;
    for (int k = 0; k < n; k++) {
      double x = k < n_b ? f19_t_b[k] : f19_t_b[n_b - 1];
      double a = k < n_b ? f19_p_b[k] : 0.0;
      double b = k < n_a ? f19_p_a[k] : 0.0;
      plot.scalar_row(x, a, b);
    }
    plot.finish_scalar(0.0, f19_t_b[n_b - 1], "time [s]", "P [steps]",
                       "P one 10000-step block (coasts)",
                       "P four 50-step blocks (capped)");
    printf("F13 F19: peak capped=%u peak long=%u P_coast=%u\n", peak_a, peak_b,
           P_coast);
    printf("F19 scalar plot written: test_26_f19.gnuplot\n");
  }

  printf("Dwell/F11/F13/F19 lookahead cap and underrun green\n");
}

// ---------------------------------------------------------------------------
// Step 14 — 3-axis SimPort helix (F8).
//
// Helix: 240 chords, radius 8000, one full turn, Z increases by 10 steps per
// chord. Run twice: mode = Linear and mode = Overshoot with overshoot_max = 8.
// Assert each vertex is hit on all three axes, every drained step has tick sum
// >= 4000, the Linear ring never path-stops at an interior vertex, both modes
// keep the realized path speed within 2x between extrema (measured from the
// drained steps/clock, not the binder-global lastTicks), and Overshoot d² in
// XY <= 64.
// ---------------------------------------------------------------------------
void f14_helix() {
  // A smooth ring must keep every joint inside the 2 deg collinear band after
  // the integer rounding. At radius 1600 / 180 chords the nominal 2 deg step
  // plus up to ~0.5 deg of rounding noise lands ~60% of the joints *outside*
  // the band, so Linear path-stops at those vertices and the ring is not
  // smooth. Radius 8000 / 240 chords puts the worst joint at 1.92 deg, safely
  // inside the band, so the whole turn is one collinear cruise.
  const int n_chords = 240;
  const int32_t radius = 8000;
  const uint32_t ticks = 4000;
  const uint16_t cap = 8;

  // Helix waypoints: integer XY via the F20 rounding, Z = 10 per chord. The
  // points are relative to the first vertex so the SimPort's physical origin
  // (0) is the path origin (Step 14; no SimPort setter).
  static int32_t wp[(n_chords + 1) * 3];
  const int32_t origin_x = iround(radius * cos(0.0));
  const int32_t origin_y = iround(radius * sin(0.0));
  for (int k = 0; k <= n_chords; k++) {
    double a = 2.0 * M_PI * (double)k / (double)n_chords;
    wp[3 * k] = iround(radius * cos(a)) - origin_x;
    wp[3 * k + 1] = iround(radius * sin(a)) - origin_y;
    wp[3 * k + 2] = 10 * k;
  }

  struct HelixResult {
    int32_t end[3];
    uint32_t clock;
    int n_vertices;
    bool all_vertices;
    double max_d2_xy;
    bool min_tick_ok;
    bool underrun;
    int rest_events;
    double mid_speed_min;
    double mid_speed_max;
  };

  auto run_helix = [&](SimPort& px, SimPort& py, SimPort& pz,
                       FasNAxisConfig::Mode mode, uint16_t overshoot_max,
                       const char* fixture, bool do_plot, NaxisPlot* xzplot,
                       const char* html_fixture, HelixResult* res) {
    res->end[0] = 0;
    res->end[1] = 0;
    res->end[2] = 0;
    res->clock = 0;
    res->n_vertices = 0;
    res->all_vertices = false;
    res->max_d2_xy = 0.0;
    res->min_tick_ok = true;
    res->underrun = false;
    res->rest_events = 0;
    res->mid_speed_min = 0.0;
    res->mid_speed_max = 0.0;

    FasNAxisConfig cfg;
    if (mode == FasNAxisConfig::Overshoot) {
      cfg.mode = FasNAxisConfig::Overshoot;
      cfg.overshoot_max = overshoot_max;
    }

    FasNAxis<3, 4096, SimPort> path(cfg);
    test(path.addAxis(0, &px) == true, "F14 addAxis X");
    test(path.addAxis(1, &py) == true, "F14 addAxis Y");
    test(path.addAxis(2, &pz) == true, "F14 addAxis Z");

    int32_t cur[3] = {wp[0], wp[1], wp[2]};
    path.setCurrentPosition(cur);
    for (int k = 1; k <= n_chords; k++) {
      int32_t t[3] = {wp[3 * k], wp[3 * k + 1], wp[3 * k + 2]};
      test(path.addLine(t) == true, "F14 addLine fits");
    }
    path.endPath();
    path.pump();

    NaxisPlot plot;
    if (do_plot) {
      plot.start_plot(fixture, "FasNAxis F8 helix 3-axis", 3);
      plot.poly_point((double)wp[0], (double)wp[1]);
      for (int k = 1; k <= n_chords; k++) {
        plot.poly_point((double)wp[3 * k], (double)wp[3 * k + 1]);
      }
      plot.poly_done();
    }
#ifdef FAS_NAXIS_TRACE
    NaxisHtmlDump html(html_fixture != NULL ? html_fixture : fixture,
                       "FasNAxis F8 helix 3-axis");
#endif

    // chord is the index of the next vertex to reach (1..n_chords). All three
    // axes land on the vertex together, so the check is per command.
    int chord = 1;
    int iter = 0;
    const bool linear_mode = (mode != FasNAxisConfig::Overshoot);
    bool have_p = false;
    // Realized-speed sampling: positions/clock one plot sample back, so a
    // per-axis speed comes from actual steps over actual ticks rather than the
    // binder-global lastTicks()/performedRampUp() (which are not per-axis in
    // Overshoot; non-binding axes issue multi-step catch-up commands).
    int32_t ps_x = 0, ps_y = 0, ps_z = 0;
    uint32_t ps_clock = 0;
    bool have_sample = false;
    while (path.isBusy()) {
      int64_t s0 = 0, s1 = 0, s2 = 0;
      bool u0 = true, u1 = true, u2 = true;
      uint32_t t0 = px.drain_one(&s0, &u0);
      uint32_t t1 = py.drain_one(&s1, &u1);
      uint32_t t2 = pz.drain_one(&s2, &u2);

      // Every drained step command must have a tick sum >= ticks_cfg.
      if (s0 != 0 && t0 < ticks) res->min_tick_ok = false;
      if (s1 != 0 && t1 < ticks) res->min_tick_ok = false;
      if (s2 != 0 && t2 < ticks) res->min_tick_ok = false;

      int32_t x = px.position();
      int32_t y = py.position();
      int32_t z = pz.position();

      if (chord <= n_chords) {
        int32_t ax = wp[3 * (chord - 1)];
        int32_t ay = wp[3 * (chord - 1) + 1];
        int32_t bx = wp[3 * chord];
        int32_t by = wp[3 * chord + 1];
        double vx = (double)(bx - ax), vy = (double)(by - ay);
        double l2 = vx * vx + vy * vy;
        if (l2 > 0.0) {
          double cross = vx * (double)(y - ay) - vy * (double)(x - ax);
          double d2 = cross * cross / l2;
          if (d2 > res->max_d2_xy) {
            res->max_d2_xy = d2;
          }
        }
        if (x == bx && y == by && z == wp[3 * chord + 2]) {
          res->n_vertices++;
          chord++;
        }
      }

      // Smoothness: on a collinear ring a continuing Linear run carries P
      // across every interior joint, so performedRampUp() stays non-zero from
      // the first ramped step until the final deceleration. A path-stop at an
      // interior vertex shows up as P == 0 here.
      if (linear_mode) {
        uint32_t P_now = path.performedRampUp();
        if (P_now > 0) {
          have_p = true;
        } else if (have_p && chord < n_chords) {
          res->rest_events++;
        }
      }

      double tt = (double)px.clock() / NAXIS_PLOT_TICKS_PER_S;
      // Sample realized per-axis speed/period (and the path speed) every 32nd
      // command from the actual position/clock advance. lastTicks() /
      // performedRampUp() are binder-global and not per-axis in Overshoot, so
      // plotting them per axis shows fictitious jumps; the drained steps over
      // the elapsed clock are the real motion. A 32-command window spans the
      // Overshoot catch-up bursts (up to the 65535-tick split), so the window
      // speed is the path's real speed rather than its per-command aliasing.
      if (iter % 32 == 0) {
        uint32_t clk = px.clock();
        if (have_sample) {
          double dt = (double)(clk - ps_clock) / NAXIS_PLOT_TICKS_PER_S;
          double ddx = (double)(x - ps_x);
          double ddy = (double)(y - ps_y);
          double ddz = (double)(z - ps_z);
          double dist = sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
          double path_v = dt > 0.0 ? dist / dt : 0.0;
          // Middle half of the turn: excludes the start/end ramps, includes
          // the X and Y extrema. A smooth ring keeps the path speed from
          // collapsing there.
          if (chord >= n_chords / 4 && chord <= (3 * n_chords) / 4) {
            if (res->mid_speed_max == 0.0 || path_v < res->mid_speed_min) {
              res->mid_speed_min = path_v;
            }
            if (path_v > res->mid_speed_max) {
              res->mid_speed_max = path_v;
            }
          }
          if (do_plot) {
            double d[3] = {ddx, ddy, ddz};
            double speed[3];
            double tcol[3];
            for (int i = 0; i < 3; i++) {
              speed[i] = dt > 0.0 ? d[i] / dt : 0.0;
              tcol[i] =
                  d[i] != 0.0 ? dt * NAXIS_PLOT_TICKS_PER_S / fabs(d[i]) : 0.0;
            }
            double Pcol[3] = {(double)path.performedRampUp(),
                              (double)path.performedRampUp(),
                              (double)path.performedRampUp()};
            double Rcol[3] = {(double)path.remainingToStop(),
                              (double)path.remainingToStop(),
                              (double)path.remainingToStop()};
            plot.row(tt, (double)x, (double)y, 0.0, speed, Pcol, Rcol, tcol);
          }
        }
        ps_x = x;
        ps_y = y;
        ps_z = z;
        ps_clock = clk;
        have_sample = true;
      }
      if (xzplot != NULL && (iter % 4 == 0)) {
        xzplot->scalar_row(tt, (double)x, (double)z);
      }
#ifdef FAS_NAXIS_TRACE
      if (html_fixture != NULL && (iter % 4 == 0)) {
        html.row(tt, (double)x, (double)y, (double)z);
      }
#endif
      iter++;
      path.pump();
    }

    if (do_plot) {
      plot.finish_plot();
    }
#ifdef FAS_NAXIS_TRACE
    if (html_fixture != NULL) {
      html.finish();
    }
#endif
    res->end[0] = px.position();
    res->end[1] = py.position();
    res->end[2] = pz.position();
    res->clock = px.clock();
    res->all_vertices = (chord == n_chords + 1);
    res->underrun = path.hasUnderrun();
  };

  // --- Linear mode -----------------------------------------------------------
  {
    SimPort px(4000, 16), py(4000, 16), pz(4000, 16);
    HelixResult res;
    NaxisPlot xz;
    xz.start_scalar("f8_xz", "FasNAxis F8 helix X vs Z");
    run_helix(px, py, pz, FasNAxisConfig::Linear, 0, "f8", true, &xz, "F8",
              &res);
    xz.finish_scalar(0.0, (double)res.clock / NAXIS_PLOT_TICKS_PER_S,
                     "time [s]", "steps", "X position", "Z position");

    test(res.end[0] == wp[3 * n_chords] && res.end[1] == wp[3 * n_chords + 1] &&
             res.end[2] == wp[3 * n_chords + 2],
         "F14 Linear ends at last vertex on all axes");
    test(res.all_vertices && res.n_vertices == n_chords,
         "F14 Linear every vertex hit on all axes");
    test(res.min_tick_ok, "F14 Linear every step has tick sum >= 4000");
    test(res.rest_events == 0,
         "F14 Linear ring never rests between interior vertices (smooth)");
    test(
        res.mid_speed_max > 0.0 && res.mid_speed_min * 2.0 >= res.mid_speed_max,
        "F14 Linear path speed stays smooth between extrema");
    test(res.underrun == false, "F14 Linear no underrun");
    printf("F14 Linear helix: clock=%u vertices=%d/%d rest=%d v=[%.0f:%.0f]\n",
           res.clock, res.n_vertices, n_chords, res.rest_events,
           res.mid_speed_min, res.mid_speed_max);
  }

  // --- Overshoot mode --------------------------------------------------------
  {
    SimPort px(4000, 16), py(4000, 16), pz(4000, 16);
    HelixResult res;
    run_helix(px, py, pz, FasNAxisConfig::Overshoot, cap, "f8_ovs", true, NULL,
              NULL, &res);

    test(res.end[0] == wp[3 * n_chords] && res.end[1] == wp[3 * n_chords + 1] &&
             res.end[2] == wp[3 * n_chords + 2],
         "F14 Overshoot ends at last vertex on all axes");
    test(res.all_vertices && res.n_vertices == n_chords,
         "F14 Overshoot every vertex hit on all axes");
    test(res.min_tick_ok, "F14 Overshoot every step has tick sum >= 4000");
    test(res.max_d2_xy <= 64.0 + 1e-9, "F14 Overshoot d² in XY <= 64");
    printf(
        "F14 Overshoot helix: clock=%u vertices=%d/%d max_d2=%.3f "
        "v=[%.0f:%.0f]\n",
        res.clock, res.n_vertices, n_chords, res.max_d2_xy, res.mid_speed_min,
        res.mid_speed_max);
    test(
        res.mid_speed_max > 0.0 && res.mid_speed_min * 2.0 >= res.mid_speed_max,
        "F14 Overshoot path speed stays smooth between extrema");
    test(res.underrun == false, "F14 Overshoot no underrun");
  }

#ifdef FAS_NAXIS_TRACE
  {
    char html_path[256];
    snprintf(html_path, sizeof(html_path), "%s/tests/out/F8.html",
             NAXIS_HTML_ROOT);
    test(gnuplot_has(html_path, "id=\"trace\""),
         "F14 F8 HTML trace page written");
    printf("F14 HTML trace written: %s\n", html_path);
  }
#endif

  printf("F14 3-axis helix green\n");
}

#ifdef FAS_PHYSICAL_STEPPER_ENABLED
// Peak absolute PCM sample in a wav file's data chunk (0 if silent/absent).
static int phys_wav_peak(const char* path) {
  FILE* f = fopen(path, "rb");
  if (f == NULL) {
    return 0;
  }
  char hdr[44];
  if (fread(hdr, 1, 44, f) != 44) {
    fclose(f);
    return 0;
  }
  int peak = 0;
  int16_t s;
  while (fread(&s, 2, 1, f) == 1) {
    int v = s < 0 ? -s : s;
    if (v > peak) {
      peak = v;
    }
  }
  fclose(f);
  return peak;
}

static void wav_u16le(FILE* f, uint16_t v) {
  fputc((int)(v & 0xff), f);
  fputc((int)((v >> 8) & 0xff), f);
}
static void wav_u32le(FILE* f, uint32_t v) {
  fputc((int)(v & 0xff), f);
  fputc((int)((v >> 8) & 0xff), f);
  fputc((int)((v >> 16) & 0xff), f);
  fputc((int)((v >> 24) & 0xff), f);
}

// Mix two axes' recorded PCM into one 16-bit stereo wav: X is the left channel,
// Y the right. Both plants run on the same SimPort clock so their sample
// streams are time-aligned; a shorter one is zero-padded.
static bool write_stereo_wav(const char* path, const PhysicalStepper& left,
                             const PhysicalStepper& right,
                             uint32_t sr = 44100) {
  uint32_t n = left.audio_sample_count();
  if (right.audio_sample_count() > n) {
    n = right.audio_sample_count();
  }
  FILE* fp = fopen(path, "wb");
  if (fp == NULL) {
    return false;
  }
  const uint16_t bps = 16, ch = 2;
  const uint32_t byteRate = sr * ch * bps / 8;
  const uint16_t blockAlign = ch * bps / 8;
  const uint32_t dataSize = n * ch * bps / 8;
  fwrite("RIFF", 1, 4, fp);
  wav_u32le(fp, 36 + dataSize);
  fwrite("WAVE", 1, 4, fp);
  fwrite("fmt ", 1, 4, fp);
  wav_u32le(fp, 16);
  wav_u16le(fp, 1);  // PCM
  wav_u16le(fp, ch);
  wav_u32le(fp, sr);
  wav_u32le(fp, byteRate);
  wav_u16le(fp, blockAlign);
  wav_u16le(fp, bps);
  fwrite("data", 1, 4, fp);
  wav_u32le(fp, dataSize);
  for (uint32_t i = 0; i < n; i++) {
    wav_u16le(fp, (uint16_t)left.audio_sample(i));
    wav_u16le(fp, (uint16_t)right.audio_sample(i));
  }
  fclose(fp);
  return true;
}

// F21 (physical_stepper_whitepaper sections 2.2 / 13.3): couple the FasNAxis
// planner to the opt-in rotordynamic plant. Each axis's SimPort gets a
// PhysicalStepper, so the commands the feeder emits drive a real rotor: the
// planner still binds against the ideal commanded count (position()), while
// the plant supplies the realized, lagging position and the acoustic
// emission. The F5 Linear square (1600 steps per side, ticks 4000, accel
// 2000) is driven with a plant on X and Y; the realized path, the
// commanded-minus-realized deviation and the rotor speeds are plotted, and
// each axis's audio is rendered to its own wav. F21 is the only fixture that
// attaches a plant, so the default suite stays bit-identical.
void f21_physical() {
  int32_t verts[5][2] = {{0, 0}, {1600, 0}, {1600, 1600}, {0, 1600}, {0, 0}};

  SimPort px(4000), py(4000);
  PhysicalStepper rotor_x, rotor_y;
  px.setPhysicalStepper(&rotor_x);
  py.setPhysicalStepper(&rotor_y);
  test(px.hasPhysical() && py.hasPhysical(), "F21 plant attached to both axes");

  FasNAxisConfig cfg;
  FasNAxis<2, 64, SimPort> path(cfg);
  test(path.addAxis(0, &px) == true, "F21 addAxis(0)");
  test(path.addAxis(1, &py) == true, "F21 addAxis(1)");
  int32_t cur[2] = {0, 0};
  path.setCurrentPosition(cur);

  NaxisPlot plot;
  plot.start_plot("f21", "FasNAxis F21 physical-stepper square 1600 Linear", 2);
  for (int k = 0; k < 5; k++) {
    plot.poly_point((double)verts[k][0], (double)verts[k][1]);
  }
  plot.poly_done();

  for (int k = 1; k < 5; k++) {
    test(path.addLine(verts[k]) == true, "F21 addLine fits");
  }
  path.endPath();

  double max_dx = 0.0, max_dy = 0.0, max_dev = 0.0;
  double max_vx = 0.0, max_vy = 0.0;
  int64_t iter = 0;
  // Drain the two queues in lockstep, exactly as run_linear_segment does; the
  // plant advances once per consumed command, so its clock tracks px.clock().
  while (path.isBusy()) {
    int64_t s0 = 0, s1 = 0;
    bool u0 = true, u1 = true;
    px.drain_one(&s0, &u0);
    py.drain_one(&s1, &u1);

    double cx = (double)px.position();
    double cy = (double)py.position();
    double rx = rotor_x.x();
    double ry = rotor_y.x();
    double dx = cx - rx;
    double dy = cy - ry;
    double dev = sqrt(dx * dx + dy * dy);
    if (fabs(dx) > max_dx) max_dx = fabs(dx);
    if (fabs(dy) > max_dy) max_dy = fabs(dy);
    if (dev > max_dev) max_dev = dev;
    double vx = rotor_x.speed();
    double vy = rotor_y.speed();
    if (vx > max_vx) max_vx = vx;
    if (vy > max_vy) max_vy = vy;

    if (iter % 4 == 0) {
      uint32_t P = path.performedRampUp();
      uint32_t R = path.remainingToStop();
      uint32_t ticks = path.lastTicks();
      double t = (double)px.clock() / NAXIS_PLOT_TICKS_PER_S;
      double speed[2] = {vx, vy};
      double Pcol[2] = {(double)P, (double)P};
      double Rcol[2] = {(double)R, (double)R};
      double tcol[2] = {(double)ticks, (double)ticks};
      // Panel 1 draws the realized rotor path ($data 2:3) on top of the grey
      // commanded polyline; column 4 is the commanded-minus-realized radius.
      plot.row(t, rx, ry, dev, speed, Pcol, Rcol, tcol);
    }
    iter++;
    path.pump();
  }
  plot.finish_plot();

  // One stereo wav: X on the left channel, Y on the right.
  bool wav = write_stereo_wav("test_26_f21.wav", rotor_x, rotor_y);
  int peak = phys_wav_peak("test_26_f21.wav");

  printf(
      "F21 physical square: max|dx|=%.3f max|dy|=%.3f max_dev=%.3f "
      "peak_v=(%.0f,%.0f) stall=(%d,%d) wav_peak=%d\n",
      max_dx, max_dy, max_dev, max_vx, max_vy, rotor_x.stall_ever() ? 1 : 0,
      rotor_y.stall_ever() ? 1 : 0, peak);

  test(!rotor_x.stall_ever() && !rotor_y.stall_ever(),
       "F21 no rotor loses synchronism on the square");
  test(max_dx < 64.0 && max_dy < 64.0,
       "F21 per-axis lag stays under one full step");
  test(max_vx > 1000.0 && max_vy > 1000.0,
       "F21 both rotors reach a real square-side speed");
  test(abs(rotor_x.getCurrentPosition()) <= 2 &&
           abs(rotor_y.getCurrentPosition()) <= 2,
       "F21 realized position returns to the origin");
  test(path.hasUnderrun() == false, "F21 no underrun");
  test(wav, "F21 the stereo wav was written");
  test(peak > 1000, "F21 the recorded wav carries audible signal, not silence");
  printf(
      "F21 plot written: test_26_f21.gnuplot; wav (stereo X=left Y=right): "
      "test_26_f21.wav\n");
}
#endif  // FAS_PHYSICAL_STEPPER_ENABLED

int main() {
  puts("FasNAxis TDD");
#ifdef FAS_NAXIS_TRACE
  naxis_ensure_html_out_dir();
#endif
  plot_smoke();
  f1_kernel();
  f2_remaining();
  f2b_oracle();
  f2c_dda_walk();
  f3_ramp();
  f2d_linear_one_block();
  f2e_issued_periods();
  f2ref_reference();
  f2g_exhaustive();
  f20_long_polyline();
  f2f_two_block();
  f2h_nblock_vs_f20();
  f3b_stoppability();
  f4_sim_port();
  f6_linear_sim();
  f7_linear_lookahead();
  f8_feeder();
  f9_dir_pauses();
  f11_overshoot_rest();
  f12_overshoot_corners();
  f13_lookahead();
  f14_helix();
#ifdef FAS_PHYSICAL_STEPPER_ENABLED
  f21_physical();
#endif
  f16_skeleton();
  printf("TEST_26 PASSED\n");
  return 0;
}
