#ifndef FAS_NAXIS_LINEAR_H
#define FAS_NAXIS_LINEAR_H

#include <stdint.h>

#include "fas_naxis/dda.h"
#include "fas_naxis/ramp_law.h"
#include "fas_naxis/remaining.h"

// FasNAxis Linear one-block interpolator (Step 2d, whitepaper section 6.3 /
// 9.2). One committed segment, rest-to-rest, no ring and no addQueueEntry.
//
// The binder (longest |delta|, rebind per the section 6.3 ticks rule in
// Remaining::binder_axis) runs a RampLaw over |delta_bind| steps; each binder
// step is one DDA tick (the 2c walker) that also advances 0 or 1 step of the
// slave. The path is the chord because each slave step is locked to a binder
// step.
//
// This is the one-block rest-to-rest core of the Linear path. It is a pure
// interpolator: the caller drives it one binder step at a time through
// step(); the trace {ticks, step[NAXES] in {-1,0,1}} is recorded by the
// caller (or the test), not held in this header. Integer arithmetic only, no
// float / double / integer division.
class LinearBlock {
 public:
  int n_axes;        // 2 (the fixtures of Step 2d are 2-axis)
  int binder;        // binder axis index (0 or 1)
  int32_t delta[2];  // signed per-axis displacement
  RampLaw law;       // the binder's ramp law over |delta_bind| steps
  DdaWalk dda;       // the chord walker, binder -> slave

  // `d_in` is the segment displacement (signed per axis); `ticks` is the
  // per-axis configured period (section 6.3 binder selection). The move
  // length in binder steps is |delta_bind|, so a zero-length slave never
  // forces a different loop bound.
  LinearBlock(uint32_t ticks_cfg, uint32_t accel, int n_axes,
              const int32_t* d_in, const uint32_t* ticks)
      : n_axes(n_axes), binder(0), law(ticks_cfg, accel, 0), dda(0, 0) {
    delta[0] = d_in[0];
    delta[1] = d_in[1];
    binder = Remaining::binder_axis(delta, ticks, n_axes);
    int32_t bind = delta[binder];
    int32_t slave = delta[1 - binder];
    law = RampLaw(ticks[binder], accel, abs_delta(bind));
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
