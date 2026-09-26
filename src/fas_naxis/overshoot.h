#ifndef FAS_NAXIS_OVERSHOOT_H
#define FAS_NAXIS_OVERSHOOT_H

#include <stdint.h>
#include "fas_arch/common.h"

#include "fas_naxis/remaining.h"
#include "fas_ramp/RampCalculator.h"

// FasNAxis Overshoot run (whitepaper sections 6.4 / 6.5 / 7.3 / 8.6, todo
// Steps 11 and 12).
//
// A run is a sequence of committed blocks. Each axis runs its own 1-D FAS ramp
// on its remaining steps in the current direction R_i (the sum of |delta_i|
// until that axis reverses/ends, section 8.6). For the block, T_opt_i is the
// sum of the periods of axis i's |delta_i| steps with its P_i evolving; the
// binding axis is the largest T_opt_i (lower index on a tie) and T is its
// duration. Every command of the block lasts one binding period, so all axes
// share the wall clock and land exactly on the vertex.
//
// A non-binding axis still has to issue exactly |delta_i| steps across T. Its
// raw schedule is uniform in time: after binding time t it has completed k
// steps when |delta_i| * t >= k * T (integer multiply-compare, no division). If
// a command needs more than one of its steps the signed `steps` field carries
// them; a slow short axis is never finished early and then paused.
//
// overshoot_max caps how far the path may leave the chord. The production test
// uses the conservative product bound
//     abs(nb*k - ns*x) <= overshoot_max * max(nb, ns)
// (nb = |delta_binder|, ns = |delta_i|, x = binding steps issued so far),
// which implies the exact squared-distance bound; no sqrt, no division, no
// 64-bit. When the uniform schedule would exceed the cap it is pulled one
// integer step at a time toward the chord -- the "mix toward Linear" of
// section 6.5.
// overshoot_max == 0 is Linear (this class is not used then); UINT16_MAX is the
// raw uniform schedule (the cap never binds).
//
// P carries across a junction when axis i keeps its sign; a sign change or a
// zero-length block resets P_i to 0 at that vertex and R_i is recomputed
// (section 8.6). The caller drives the run one command at a time through
// step(); no float, double, or integer division.
template <uint8_t NMAX>
class OvershootRun {
 public:
  OvershootRun() {
    n_axes = 0;
    cap = 0;
    done_ = true;
    binder = 0;
    ncmd = 0;
    T = 0;
    t = 0;
    x = 0;
    for (uint8_t i = 0; i < NMAX; i++) {
      cfg[i].init();
      tick[i] = 1;
      accel[i] = 1;
      coast[i] = 1;
      P[i] = 0;
      R[i] = 0;
      tot[i] = 0;
      sgn[i] = 0;
      prev[i] = 0;
      issued[i] = 0;
      binding[i] = false;
      Topt[i] = 0;
    }
  }

  // Configure the per-axis FAS maps. `ticks` / `accel` are the per-axis
  // configured period and acceleration; `cap_in` is overshoot_max.
  void configure(int n, const uint32_t* ticks, const uint32_t* accel_in,
                 uint32_t cap_in) {
    n_axes = n;
    cap = cap_in;
    done_ = true;
    for (int i = 0; i < n_axes; i++) {
      tick[i] = ticks[i];
      accel[i] = accel_in[i];
      cfg[i].init();
      cfg[i].parameters.setSpeedInTicks(ticks[i]);
      cfg[i].parameters.setAcceleration((int32_t)accel_in[i]);
      cfg[i].update();
      coast[i] = cfg[i].max_ramp_up_steps;
      if (coast[i] == 0) {
        coast[i] = 1;
      }
      P[i] = 0;
      R[i] = 0;
      prev[i] = 0;
      tot[i] = 0;
      sgn[i] = 0;
      issued[i] = 0;
    }
  }

  // Commit the next block. `d[i]` is the signed displacement of axis i and
  // `Rrem[i]` is that axis's remaining steps in its current direction from this
  // block onward (the section 8.6 scan). A sign change or a zero block resets
  // P_i; a continuation keeps it. `n` must equal the configured axis count.
  void start_block(const int32_t* d, const uint32_t* Rrem) {
    uint32_t Tmax = 0;
    int b = 0;
    bool any = false;
    for (int i = 0; i < n_axes; i++) {
      int32_t di = d[i];
      tot[i] = fas_abs(di);
      sgn[i] = di > 0 ? 1 : (di < 0 ? -1 : 0);
      issued[i] = 0;
      if (tot[i] == 0 || (prev[i] != 0 && sgn[i] != 0 && sgn[i] != prev[i])) {
        P[i] = 0;
      }
      R[i] = Rrem[i];
      uint32_t opt = 0;
      if (tot[i] > 0) {
        opt = simulate(i);
      }
      Topt[i] = opt;
      if (opt > Tmax) {
        Tmax = opt;
        b = i;
        any = true;
      }
      prev[i] = sgn[i];
    }
    binder = b;
    T = Tmax;
    for (int i = 0; i < n_axes; i++) {
      binding[i] = tot[i] > 0 && Topt[i] == Tmax && Tmax != 0;
    }
    t = 0;
    x = 0;
    ncmd = any ? tot[b] : 0;
    done_ = !any;
  }

  bool done() const { return done_; }

  // One command. Fills `step_out[i]` with the signed step count for axis i
  // (0 or a small catch-up count; the cap keeps the deficit tight) and returns
  // the command period in ticks. All axes share that tick sum.
  uint32_t step(int* step_out) {
    for (int i = 0; i < n_axes; i++) {
      step_out[i] = 0;
    }
    if (done_) {
      return 0;
    }
    uint32_t period = law_step(binder);
    t += period;
    x++;
    step_out[binder] = sgn[binder];
    issued[binder]++;
    for (int i = 0; i < n_axes; i++) {
      if (i == binder || tot[i] == 0) {
        continue;
      }
      uint32_t issue;
      if (binding[i]) {
        // T_opt_i == T: the axis rides its own ramp in lockstep, one step per
        // command (section 7.3).
        issue = issued[i] < tot[i] ? 1 : 0;
      } else {
        // Uniform-in-time candidate for a non-binding axis. The feeder turns a
        // multi-step catch-up into one command of `steps` pulses at
        // t_step / steps ticks each (section 9.3), so the shared wall clock is
        // preserved. The quotient (steps issued = tot[i] * t / T) is a log2
        // approximation: sums of logs, no wide multiply or overflow check.
        // At the block's last command (t == T) the last step compares equal
        // in log2 (log2_from(tot[i]) + log2_from(T) vs itself), so issued[i]
        // always reaches tot[i] and the axis lands on the vertex (G1).
        uint32_t k = issued[i];
        int32_t lt_t = (int32_t)log2_from(tot[i]) + log2_from(t);
        int32_t lt_T = log2_from(T);
        while (k < tot[i] && lt_t >= (int32_t)log2_from(k + 1) + lt_T) {
          k++;
        }
        if (cap != 0 && cap != 0xFFFFu && tot[binder] > 0) {
          k = apply_cap(i, k);
        }
        issue = k - issued[i];
      }
      for (uint32_t j = 0; j < issue; j++) {
        law_step(i);
      }
      issued[i] += issue;
      step_out[i] = sgn[i] * (int)issue;
    }
    if (x >= ncmd) {
      done_ = true;
    }
    return period;
  }

  bool last_command() const { return x >= ncmd; }

  int n_axes;
  uint32_t cap;
  uint32_t P[NMAX];  // performed ramp-up steps (persistent across blocks)
  uint32_t R[NMAX];  // remaining steps in the current direction
  int binder;        // axis with the largest T_opt
  uint32_t T;        // binding block duration in ticks
  uint32_t ncmd;     // binding commands in the current block
  uint32_t x;        // binding commands issued in the current block

 private:
  bool binding[NMAX];   // T_opt_i == T for this block
  uint32_t Topt[NMAX];  // per-axis ramp duration for this block
  ramp_config_s cfg[NMAX];
  uint32_t tick[NMAX];
  uint32_t accel[NMAX];
  uint32_t coast[NMAX];
  uint32_t tot[NMAX];
  int8_t sgn[NMAX];
  int8_t prev[NMAX];
  uint32_t issued[NMAX];
  uint32_t t;
  bool done_;

  // Apply the section 7.1 law to axis i, then return the period at the new P.
  uint32_t law_step(int i) {
    if (R[i] > P[i]) {
      if (P[i] < coast[i]) {
        P[i]++;
      }
    } else {
      if (P[i] > 0) {
        P[i]--;
      }
    }
    uint32_t p = period(i);
    if (R[i] > 0) {
      R[i]--;
    }
    return p;
  }

  uint32_t period(int i) const {
    uint32_t p;
    if (P[i] == 0) {
      p = cfg[i].calculate_ticks(1);
    } else {
      p = cfg[i].calculate_ticks(P[i]);
      if (p < tick[i]) {
        p = tick[i];
      }
    }
    return p;
  }

  // Sum of the periods axis i would take for this block's |delta_i| steps,
  // without touching the live P / R.
  uint32_t simulate(int i) const {
    uint32_t p = P[i];
    uint32_t r = R[i];
    uint32_t sum = 0;
    for (uint32_t s = 0; s < tot[i]; s++) {
      if (r > p) {
        if (p < coast[i]) {
          p++;
        }
      } else {
        if (p > 0) {
          p--;
        }
      }
      uint32_t tt;
      if (p == 0) {
        tt = cfg[i].calculate_ticks(1);
      } else {
        tt = cfg[i].calculate_ticks(p);
        if (tt < tick[i]) {
          tt = tick[i];
        }
      }
      uint32_t next = sum + tt;
      sum = next < sum ? 0xffffffffu : next;
      if (r > 0) {
        r--;
      }
    }
    return sum;
  }

  // True when the uniform candidate `k` for axis i lies outside the
  // overshoot_max cap around the chord. No sqrt, no division, no 64-bit:
  // the perpendicular distance is D / sqrt(nb^2+ns^2) with D = |nb*k -
  // ns*x|. The test uses the conservative bound D <= cap * max(nb, ns),
  // which is >= cap * sqrt(nb^2+ns^2), split into two product compares
  // (Remaining::log2_mul_diff) so the realized distance never exceeds the
  // hard cap. The bound is tight when one axis dominates and at most
  // sqrt(2) tighter on a diagonal.
  //
  // log2_mul_diff rounds by up to four log2 units; a comparison within
  // kCapSlack of equality is treated as outside so the guarantee survives
  // the rounding (a one-step move changes D by up to max(nb,ns), so a
  // near-tie would otherwise be decided the wrong way).
  bool outside_cap(uint32_t nb, uint32_t ns, uint32_t k) const {
    const int32_t kCapSlack = 4;
    if (nb == 0 || ns == 0) {
      return false;
    }
    if (nb >= ns) {
      // inside <=> nb*(k-cap) <= ns*x  and  ns*x <= nb*(k+cap)
      if (k >= cap &&
          Remaining::log2_mul_diff(nb, k - cap, ns, x) > -kCapSlack) {
        return true;
      }
      if (Remaining::log2_mul_diff(ns, x, nb, k + cap) > -kCapSlack) {
        return true;
      }
    } else {
      // inside <=> nb*k <= ns*(x+cap)  and  ns*(x-cap) <= nb*k
      if (Remaining::log2_mul_diff(nb, k, ns, x + cap) > -kCapSlack) {
        return true;
      }
      if (x >= cap &&
          Remaining::log2_mul_diff(ns, x - cap, nb, k) > -kCapSlack) {
        return true;
      }
    }
    return false;
  }

  uint32_t apply_cap(int i, uint32_t k) const {
    uint32_t nb = tot[binder];
    uint32_t ns = tot[i];
    // Log2 direction for the walk; the conservative outside_cap() gate
    // stops the walk inside the cap region, so neither the sign slack nor
    // the bound slack can pull the step outside overshoot_max.
    while (k > issued[i] && Remaining::log2_mul_cmp(nb, k, ns, x) > 0 &&
           outside_cap(nb, ns, k)) {
      k--;
    }
    while (k < tot[i] && Remaining::log2_mul_cmp(nb, k, ns, x) < 0 &&
           outside_cap(nb, ns, k)) {
      k++;
    }
    return k;
  }
};

#endif /* FAS_NAXIS_OVERSHOOT_H */
