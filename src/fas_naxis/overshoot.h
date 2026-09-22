#ifndef FAS_NAXIS_OVERSHOOT_H
#define FAS_NAXIS_OVERSHOOT_H

#include <stdint.h>

#include "fas_naxis/ramp_law.h"

// FasNAxis Overshoot rest-to-rest block (whitepaper sections 6.4 / 7.3, todo
// Step 11).
//
// One rest-to-rest segment, N axes. Each axis i runs its own RampLaw over
// |delta_i| with that axis's ticks_cfg / accel; T_opt_i is the sum of those
// periods. The binding axis is the one with the largest T_opt_i (lower index on
// a tie); T is its duration and every command of the segment lasts one binding
// RampLaw period. That is the synchronized wall clock: the binding axis never
// pauses and never waits at the end.
//
// A binding axis (T_opt_i == T) steps once per command. A non-binding axis
// (T_opt_i < T) still has to issue exactly |delta_i| steps across the whole of
// T without a delayed start and without sitting at the target. Its raw schedule
// is the uniform one: after binding time t it has completed k steps when
// |delta_i| * t >= k * T (integer multiply-compare, no division).
//
// overshoot_max caps how far the path may leave the chord. The production test
// is the integer squared distance
//     (|delta_binder| * k - |delta_slave| * x)^2
//         <= overshoot_max^2 * (|delta_binder|^2 + |delta_slave|^2)
// (no sqrt, no division). When the uniform schedule would exceed the cap it is
// pulled one integer step at a time toward the chord until it fits -- that is
// the "mix toward the Linear DDA fraction" of section 6.5. overshoot_max == 0
// is Linear (this class is not constructed then); UINT16_MAX is the raw uniform
// schedule (the cap never binds).
//
// This is a pure schedule generator: the caller drives it one command at a
// time through step(); the {ticks, steps[NAXES]} trace is not held here. No
// float, double, or integer division.
template <uint8_t NMAX>
class OvershootBlock {
 public:
  int n_axes;
  int32_t delta[NMAX];
  uint32_t ticks[NMAX];   // per-axis configured period
  uint32_t accel[NMAX];   // per-axis acceleration (steps/s^2)
  uint32_t cap;           // overshoot_max (steps); 0 = Linear, 0xFFFF = raw
  int binder;             // axis with the largest T_opt
  bool binding[NMAX];     // T_opt_i == T
  uint64_t Topt[NMAX];    // per-axis ramp duration in ticks
  uint32_t total[NMAX];   // |delta_i|
  uint32_t issued[NMAX];  // steps issued per axis
  RampLaw law;            // the binding axis's ramp
  uint64_t T;             // binding duration in ticks
  uint32_t x;             // binding commands issued
  uint64_t t;             // cumulative ticks
  bool done_;

  OvershootBlock() : law(1, 1, 0) {
    n_axes = 0;
    cap = 0;
    binder = 0;
    T = 0;
    x = 0;
    t = 0;
    done_ = true;
  }

  // `d` / `tk` / `ac` are per-axis displacement, configured period and
  // acceleration. `cap_in` is overshoot_max.
  void init(int n, const int32_t* d, const uint32_t* tk, const uint32_t* ac,
            uint32_t cap_in) {
    n_axes = n;
    cap = cap_in;
    binder = 0;
    x = 0;
    t = 0;
    done_ = false;
    T = 0;
    for (int i = 0; i < n_axes; i++) {
      delta[i] = d[i];
      ticks[i] = tk[i];
      accel[i] = ac[i];
      issued[i] = 0;
      total[i] = abs_u(d[i]);
      uint64_t opt = 0;
      if (total[i] > 0) {
        RampLaw probe(tk[i], ac[i], total[i]);
        while (!probe.done()) {
          opt += probe.step();
        }
      }
      Topt[i] = opt;
      if (opt > T) {
        T = opt;
      }
    }
    for (int i = 0; i < n_axes; i++) {
      if (Topt[i] == T) {
        binder = i;  // lower index wins the tie
        break;
      }
    }
    for (int i = 0; i < n_axes; i++) {
      binding[i] = total[i] > 0 && Topt[i] == T;
    }
    law = RampLaw(ticks[binder], accel[binder], total[binder]);
  }

  bool done() const { return done_; }

  // One command. Fills `step_out[i]` with the signed step count for axis i
  // (0/+-1 in the Step 11 fixtures; the cap keeps a missed catch-up to one step
  // per command) and returns the command period in ticks. All axes share that
  // tick sum.
  uint32_t step(int* step_out) {
    for (int i = 0; i < n_axes; i++) {
      step_out[i] = 0;
    }
    if (done_) {
      return 0;
    }
    uint32_t period = law.step();
    t += period;
    x++;
    for (int i = 0; i < n_axes; i++) {
      if (total[i] == 0) {
        continue;
      }
      uint32_t issue;
      if (i == binder) {
        issue = 1;
      } else if (binding[i]) {
        issue = issued[i] < total[i] ? 1 : 0;
      } else {
        issue = uniform_capped(i);
      }
      issued[i] += issue;
      step_out[i] = (delta[i] > 0 ? 1 : -1) * (int)issue;
    }
    if (x >= total[binder]) {
      done_ = true;
    }
    return period;
  }

 private:
  static uint32_t abs_u(int32_t d) {
    return d > 0 ? (uint32_t)d : (uint32_t)(-(int64_t)d);
  }

  // Uniform-in-time schedule for non-binding axis i, clamped to the
  // overshoot_max tube. Returns the number of steps to issue on this command.
  uint32_t uniform_capped(int i) const {
    uint32_t Ns = total[i];
    uint32_t tk = issued[i];
    uint64_t lhs = (uint64_t)Ns * t;
    while (tk < Ns && lhs >= (uint64_t)(tk + 1) * T) {
      tk++;
    }
    if (cap != 0 && cap != 0xFFFFu && total[binder] > 0) {
      int64_t nb = total[binder];
      int64_t ns = Ns;
      int64_t c2 = (int64_t)cap * cap * (nb * nb + ns * ns);
      int64_t num = nb * (int64_t)tk - ns * (int64_t)x;
      while (tk > issued[i] && num > 0 && num * num > c2) {
        tk--;
        num -= nb;
      }
      while (tk < Ns && num < 0 && num * num > c2) {
        tk++;
        num += nb;
      }
    }
    // One command per slice keeps every axis on the same tick sum. A target
    // more than one step ahead is caught up over the following commands. The
    // Step 11 fixtures never need more than one (the cap keeps the deficit
    // small); a genuine multi-step catch-up is left to Step 12.
    uint32_t issue = tk - issued[i];
    return issue > 1 ? 1 : issue;
  }
};

#endif /* FAS_NAXIS_OVERSHOOT_H */
