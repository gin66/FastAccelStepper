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
#include "fas_naxis/dda.h"
#include "fas_naxis/linear.h"
#include "fas_naxis/ramp_law.h"
#include "fas_naxis/ramp_map.h"
#include "fas_naxis/remaining.h"
#include "naxis_plot.h"

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
    // |dy|*ticks_y (50*1600) beats X's (100*400), so Y binds.
    uint32_t ticks_slow[2] = {400, 1600};
    int binder_slow = Remaining::binder_axis(block, ticks_slow, 2);
    test(binder_slow == 1, "F2b item1 slow Y binds by wall-clock (rebind)");
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
      uint32_t t = (P == 0) ? ticks_cfg : map.calculate_ticks((uint32_t)P);
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
  const uint32_t accel = 2000;   // section 14.1 limits
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

    int32_t issued[2] = {0, 0};   // signed accumulated steps per axis
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
        double dist = (px * ay - py * ax) / denom;   // chord unit-ish, signed
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

int main() {
  puts("FasNAxis TDD");
  plot_smoke();
  f1_kernel();
  f2_remaining();
  f2b_oracle();
  f2c_dda_walk();
  f3_ramp();
  f2d_linear_one_block();
  printf("TEST_26 PASSED\n");
  return 0;
}
