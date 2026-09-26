#ifndef FAS_NAXIS_LINEAR_H
#define FAS_NAXIS_LINEAR_H

#include <stdint.h>
#include "fas_arch/common.h"

#include "fas_naxis/dda.h"
#include "fas_naxis/ramp_law.h"
#include "fas_naxis/ramp_map.h"
#include "fas_naxis/remaining.h"

// FasNAxis Linear one-block interpolator (Step 2d, whitepaper section 6.3 /
// 9.2). One committed segment, rest-to-rest, no ring and no addQueueEntry.
//
// The DDA master is the longest |delta| (Remaining::longest_axis, tie-break
// slower ticks_cfg). It runs a RampLaw over |delta_master| steps; each master
// step is one DDA tick (the 2c walker) that also advances 0 or 1 step of the
// slave. The path is the chord because each slave step is locked to a master
// step, and issued |steps_i| equals |delta_i|.
//
// Time-law rebind does not change the DDA master. If a slower short slave
// would exceed v_max under the master's own ticks_cfg, Remaining::ticks_floor
// lengthens the master's period (section 6.3 "lengthen ticks_b"): the long
// axis is scaled down in speed, not in count. Walking DDA on the short axis
// would issue only |delta_slave| steps on the long axis and miss the vertex.
//
// This is the one-block rest-to-rest core of the Linear path. It is a pure
// interpolator: the caller drives it one binder step at a time through
// step(); the trace {ticks, step[NAXES] in {-1,0,1}} is recorded by the
// caller (or the test), not held in this header. Integer arithmetic only, no
// float / double / integer division.
class LinearBlock {
 public:
  int n_axes;        // 2 (the fixtures of Step 2d are 2-axis)
  int binder;        // DDA master axis index (longest |delta|)
  int32_t delta[2];  // signed per-axis displacement
  RampLaw law;       // the master's ramp law over |delta_master| steps
  DdaWalk dda;       // the chord walker, master -> slave

  // `d_in` is the segment displacement (signed per axis); `ticks` is the
  // per-axis configured period (section 6.3). The move length in master
  // steps is |delta_master|, so a zero-length slave never forces a
  // different loop bound. `ticks_cfg` is unused: the time-law period is
  // ticks_floor (max ticks_i of moving axes).
  LinearBlock(uint32_t ticks_cfg, uint32_t accel, int n_axes,
              const int32_t* d_in, const uint32_t* ticks)
      : n_axes(n_axes), binder(0), law(ticks_cfg, accel, 0), dda(0, 0) {
    delta[0] = d_in[0];
    delta[1] = d_in[1];
    binder = Remaining::longest_axis(delta, ticks, n_axes);
    int32_t bind = delta[binder];
    int32_t slave = delta[1 - binder];
    uint32_t t_law = Remaining::ticks_floor(delta, ticks, n_axes);
    if (t_law == 0) {
      t_law = ticks_cfg;
    }
    law = RampLaw(t_law, accel, abs_delta(bind));
    dda = DdaWalk(bind, slave);
  }

  // True when every binder step is consumed (RampLaw done and DDA done).
  bool done() const { return dda.done() && law.done(); }

  // One binder step: apply the ramp law (returns the command period in ticks
  // at the updated P), advance the DDA walker, and fill `step_out` with the
  // per-axis step this binder step issues (-1 / 0 / +1). The binder column is
  // +1/-1, the slave column is -1/0/+1.
  uint32_t step(int step_out[2]) {
    step_out[0] = 0;
    step_out[1] = 0;
    uint32_t ticks = law.step();
    int out_bind = 0, out_slave = 0;
    dda.step(&out_bind, &out_slave);
    step_out[binder] = out_bind;
    step_out[1 - binder] = out_slave;
    return ticks;
  }

 private:
  static uint32_t abs_delta(int32_t d) { return fas_abs(d); }
};

// FasNAxis Linear multi-block interpolator (Step 2f / 2h, whitepaper section
// 8.1 + 8.5 + 9.2). Walks a committed polyline held in a Remaining ring with
// no queues. The DDA master is the longest |delta| of the current block
// (Remaining::longest_axis, tie-break slower ticks_cfg). Each master step is
// one DDA tick (the 2c walker) that also advances 0 or 1 step of the slave.
//
// The ramp law (P vs R) is run per block. R is master steps to the next hard
// stop (section 8.5): master-sense reversal, outgoing master that was idle,
// dwell, or path end. P is live; P <= R at every step. At a hard stop P starts
// from 0. Across every other joint, including a non-collinear bend and a
// master-role change, P carries and R stays in path-step units.
//
// DIR pauses (before/after reversal) are Step 9 (queues). Here a reversal is
// only P -> 0 then the new sign on the next block.
class LinearPoly {
 public:
  Remaining* rem;
  const uint32_t* ticks;
  uint32_t accel;
  int n_axes;
  int block;   // current block index
  int master;  // DDA master axis for the current block
  DdaWalk dda;
  RampMap map;
  uint32_t ticks_law;
  uint32_t P;
  uint32_t R;
  uint32_t R_before_cmd;  // R at the start of the last issued command
  uint32_t total_ticks;
  bool finished;

  LinearPoly(Remaining* rem_, const uint32_t* ticks_, uint32_t accel_)
      : rem(rem_),
        ticks(ticks_),
        accel(accel_),
        n_axes(rem_->n_axes),
        block(0),
        master(0),
        dda(0, 0),
        map(ticks_[0], accel_),
        ticks_law(ticks_[0]),
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

  // One master step. step_out[i] in {-1,0,1}. Period from the FAS map at the
  // updated P, floored to ticks_law. R_before_cmd holds the R that was active
  // at this command so the caller can reconstruct P <= R from issued periods
  // without reading this object's P/R fields.
  uint32_t step(int* step_out) {
    for (int i = 0; i < n_axes; i++) {
      step_out[i] = 0;
    }
    while (!finished && dda.done()) {
      bool stop = true;
      if (block + 1 < rem->n_blocks) {
        int32_t a[2];
        int32_t b[2];
        a[0] = rem->delta_of(0, block);
        a[1] = n_axes > 1 ? rem->delta_of(1, block) : 0;
        b[0] = rem->delta_of(0, block + 1);
        b[1] = n_axes > 1 ? rem->delta_of(1, block + 1) : 0;
        stop = Remaining::linear_joint_stops(a, b, ticks, n_axes);
      }
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
  static uint32_t abs_u(int32_t d) { return fas_abs(d); }

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
    master = Remaining::longest_axis(d, ticks, n_axes);
    ticks_law = Remaining::ticks_floor(d, ticks, n_axes);
    if (ticks_law == 0) {
      ticks_law = ticks[0];
    }
    map = RampMap(ticks_law, accel);
    int32_t bind = d[master];
    int32_t slave = n_axes > 1 ? d[1 - master] : 0;
    dda = DdaWalk(bind, slave);
    R = remaining_path_steps(b);
    if (R == 0) {
      R = abs_u(bind);
    }
    if (reset_P) {
      P = 0;
    } else {
      uint32_t coast = map.P_coast();
      if (P > coast) {
        P = coast;
      }
      if (P > R) {
        P = R;
      }
    }
    block = b;
    finished = false;
  }

  // Master steps to the next hard stop. P and R live in these units so a run
  // may rebind the DDA master without changing the ramp-step currency.
  uint32_t remaining_path_steps(int head) const {
    uint32_t acc[2] = {accel, accel};
    return Remaining::linear_remaining<2>(head, rem->n_blocks, n_axes, ticks,
                                          acc, rem->horizon, *rem);
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
      t = map.calculate_ticks(1);
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

#endif /* FAS_NAXIS_LINEAR_H */
