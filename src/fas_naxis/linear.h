#ifndef FAS_NAXIS_LINEAR_H
#define FAS_NAXIS_LINEAR_H

#include <stdint.h>

#include "fas_naxis/dda.h"
#include "fas_naxis/ramp_law.h"
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
  static uint32_t abs_delta(int32_t d) {
    return d > 0 ? (uint32_t)d : (uint32_t)(-(int64_t)d);
  }
};

#endif /* FAS_NAXIS_LINEAR_H */
