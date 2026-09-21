#ifndef FAS_NAXIS_DDA_H
#define FAS_NAXIS_DDA_H

#include <stdint.h>

// FasNAxis Linear DDA walk (whitepaper section 6.3).
//
// The DDA master is the longest |delta| (Remaining::longest_axis). Time-law
// rebind lengthens that master's period; it does not make a shorter axis the
// loop bound. Every slave is scaled down to the master's step count: err_i +=
// |delta_i|; when 2*err_i >= |delta_bind| the slave steps (and err_i -=
// |delta_bind|). The path is the chord because each slave step is locked to a
// master step, and |delta_slave| <= |delta_bind| so the slave takes 0 or 1
// step per master step and issued |steps| equals |delta|.
//
// This header *walks* that error accumulator one binder step at a time (Step
// 2c). Each binder step issues 0 or 1 step per axis (never 2): the binder
// issues 1, an idle slave 0, a moving slave 0 or 1. The walked step count per
// axis must equal Remaining::dda_steps for the same block.
//
// Integer arithmetic only. No float, no double, no integer division. The
// "distance to the chord <= 0.5*sqrt(n)" check of section 12.4 is a test-only
// double check, not part of this walker.
class DdaWalk {
 public:
  // `bind` is the binder displacement, `slave` the slave displacement of one
  // axis (signed). The two magnitudes and the sign of the slave fully define
  // the walk; the binder direction is carried by the binder column below.
  DdaWalk(int32_t bind, int32_t slave)
      : bind(bind),
        slave(slave),
        abs_bind(bind > 0 ? bind : -bind),
        abs_slave(slave > 0 ? slave : -slave),
        bind_dir(bind < 0 ? -1 : 1),
        slave_dir(slave < 0 ? -1 : 1),
        err(0),
        k(0) {}

  int32_t bind;       // signed binder displacement
  int32_t slave;      // signed slave displacement
  int64_t abs_bind;   // |bind|, the loop bound
  int64_t abs_slave;  // |slave|, the error increment
  int bind_dir;       // +1 / -1
  int slave_dir;      // +1 / -1
  int64_t err;        // Bresenham error accumulator
  int k;              // binder steps consumed so far

  bool done() const { return k >= abs_bind; }

  // One binder step. `bind_out` is +1/-1 (the binder always steps),
  // `slave_out` is -1/0/+1 (the slave steps when 2*err >= |bind|). The
  // chord invariant |2*err| <= |bind| holds after every step.
  void step(int* bind_out, int* slave_out) {
    int s = 0;
    err += abs_slave;
    if (2 * err >= abs_bind) {
      s = slave_dir;
      err -= abs_bind;
    }
    if (bind_out) {
      *bind_out = bind_dir;
    }
    if (slave_out) {
      *slave_out = s;
    }
    k++;
  }

  // The chord invariant: |2*err| <= |bind| after a step.
  bool on_chord() const {
    int64_t e = err;
    if (e < 0) {
      e = -e;
    }
    return 2 * e <= abs_bind;
  }
};

#endif /* FAS_NAXIS_DDA_H */
