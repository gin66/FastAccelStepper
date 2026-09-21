#ifndef NAXIS_REF_H
#define NAXIS_REF_H

#include <stdint.h>

#include "fas_naxis/dda.h"
#include "fas_naxis/ramp_map.h"
#include "fas_naxis/remaining.h"

// PC reference track (whitepaper section 12.4.1, todo Step 2ref).
//
// This is the globally fastest constraint-faithful Linear track: G1/G2/G6,
// on the chords, path speed 0 at a non-collinear vertex, DDA on longest
// |delta|, ticks_b lengthened if a slave would exceed v_max, FAS ramp on
// remaining master-steps to the next Linear path-stop. It is the truth the
// interpolator must match (log2 / one-step slack). Not a replay of
// LinearBlock fields.
//
// PC tests only. double is allowed in callers, not here. Not included from
// src/FasNAxis.h. Overshoot T_opt is Step 11.
//
// 1-D law is RampCalculator via RampMap (same P vs R as RampLaw). DDA is
// DdaWalk. R is Remaining::remaining_linear_binder. Infinite HORIZON unless
// the Remaining object sets one.
class NaxisRefLinear {
 public:
  Remaining* rem;
  const uint32_t* ticks_axis;
  uint32_t accel;
  int n_axes;
  int block;
  int master;
  DdaWalk dda;
  RampMap map;
  uint32_t ticks_law;
  uint32_t P;
  uint32_t R;
  uint32_t R_before_cmd;  // R at the start of the last issued command
  uint64_t total_ticks;
  bool finished;

  NaxisRefLinear(Remaining* rem, const uint32_t* ticks_axis, uint32_t accel)
      : rem(rem),
        ticks_axis(ticks_axis),
        accel(accel),
        n_axes(rem->n_axes),
        block(0),
        master(0),
        dda(0, 0),
        map(ticks_axis[0], accel),
        ticks_law(ticks_axis[0]),
        P(0),
        R(0),
        R_before_cmd(0),
        total_ticks(0),
        finished(false) {
    start_block(0, true);
  }

  bool done() const {
    return finished || (block >= rem->n_blocks - 1 && dda.done());
  }

  // One master step. step_out[i] in {-1,0,1}. Period from the FAS map at
  // the updated P, floored to ticks_law.
  uint32_t step(int* step_out) {
    for (int i = 0; i < n_axes; i++) {
      step_out[i] = 0;
    }
    while (!finished && dda.done()) {
      bool last = (block + 1 >= rem->n_blocks);
      bool stop = last || !rem->collinear_same_sense(block, block + 1);
      start_block(block + 1, stop);
    }
    if (finished) {
      return 0;
    }
    R_before_cmd = R;
    uint32_t t = apply_law();
    int out_bind = 0;
    int out_slave = 0;
    dda.step(&out_bind, &out_slave);
    step_out[master] = out_bind;
    if (n_axes > 1) {
      step_out[1 - master] = out_slave;
    }
    return t;
  }

 private:
  static uint32_t abs_u(int32_t d) {
    return d > 0 ? (uint32_t)d : (uint32_t)(-(int64_t)d);
  }

  int next_moving(int from) const {
    for (int b = from; b < rem->n_blocks; b++) {
      bool moving = false;
      for (int i = 0; i < n_axes; i++) {
        if (rem->delta_of(i, b) != 0) {
          moving = true;
          break;
        }
      }
      if (moving) {
        return b;
      }
    }
    return rem->n_blocks;
  }

  void start_block(int b, bool reset_P) {
    b = next_moving(b);
    if (b >= rem->n_blocks) {
      finished = true;
      return;
    }
    int32_t d[2];
    d[0] = rem->delta_of(0, b);
    d[1] = n_axes > 1 ? rem->delta_of(1, b) : 0;
    master = Remaining::longest_axis(d, ticks_axis, n_axes);
    ticks_law = Remaining::ticks_floor(d, ticks_axis, n_axes);
    if (ticks_law == 0) {
      ticks_law = ticks_axis[0];
    }
    map = RampMap(ticks_law, accel);
    int32_t bind = d[master];
    int32_t slave = n_axes > 1 ? d[1 - master] : 0;
    dda = DdaWalk(bind, slave);
    if (reset_P) {
      P = 0;
      R = remaining_path_steps(b);
      if (R == 0) {
        R = abs_u(bind);
      }
    }
    block = b;
    finished = false;
  }

  // Remaining DDA/master steps to the next Linear path-stop. P and R live in
  // these units so a collinear run may rebind the DDA master without
  // changing the ramp-step currency (a 1 deg arc switches longest axis).
  uint32_t remaining_path_steps(int head) const {
    uint32_t s = 0;
    int started = 0;
    for (int b = head; b < rem->n_blocks; b++) {
      int32_t d[2];
      d[0] = rem->delta_of(0, b);
      d[1] = n_axes > 1 ? rem->delta_of(1, b) : 0;
      if (d[0] == 0 && d[1] == 0) {
        if (started) {
          break;
        }
        continue;
      }
      if (started && !rem->collinear_same_sense(b - 1, b)) {
        break;
      }
      int m = Remaining::longest_axis(d, ticks_axis, n_axes);
      s += abs_u(d[m]);
      started = 1;
    }
    return s;
  }

  uint32_t apply_law() {
    uint32_t coast = map.P_coast();
    if (R > P) {
      if (P < coast) {
        P++;
      }
    } else {
      if (P > 0) {
        P--;
      }
    }
    uint32_t t;
    if (P == 0) {
      t = ticks_law;
    } else {
      t = map.calculate_ticks(P);
      if (t < ticks_law) {
        t = ticks_law;
      }
    }
    total_ticks += t;
    if (R > 0) {
      R--;
    }
    return t;
  }
};

#endif /* NAXIS_REF_H */
