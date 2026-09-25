#ifndef NAXIS_REF_H
#define NAXIS_REF_H

#include <stdint.h>

#include "fas_naxis/dda.h"
#include "fas_naxis/ramp_law.h"
#include "fas_naxis/ramp_map.h"
#include "fas_naxis/remaining.h"

// PC reference track (whitepaper section 12.4.1, todo Step 2ref).
//
// This is the globally fastest constraint-faithful Linear track: G1/G2/G6,
// on the chords, path speed 0 only at a hard stop (master-sense reversal,
// outgoing master that was idle, dwell, or path end), DDA on longest |delta|,
// ticks_b lengthened if a slave would exceed v_max, FAS ramp on remaining
// master-steps to that stop. P carries across every other joint. It is the
// truth the interpolator must match (log2 / one-step slack). Not a replay of
// LinearBlock fields.
//
// PC tests only. double is allowed in callers, not here. Not included from
// src/FasNAxis.h. Overshoot T_opt is Step 11.
//
// 1-D law is RampCalculator via RampMap (same P vs R as RampLaw). DDA is
// DdaWalk. R is Remaining::linear_remaining. Infinite HORIZON unless the
// Remaining object sets one.
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
      bool stop = true;
      if (block + 1 < rem->n_blocks) {
        int32_t a[2];
        int32_t b[2];
        a[0] = rem->delta_of(0, block);
        a[1] = n_axes > 1 ? rem->delta_of(1, block) : 0;
        b[0] = rem->delta_of(0, block + 1);
        b[1] = n_axes > 1 ? rem->delta_of(1, block + 1) : 0;
        stop = Remaining::linear_joint_stops(a, b, ticks_axis, n_axes);
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
  // FAS_NAXIS_NO_CROSS_BLOCK_R (inside linear_remaining) cuts R to one block.
  uint32_t remaining_path_steps(int head) const {
    uint32_t acc[2] = {accel, accel};
    return Remaining::linear_remaining<2>(head, rem->n_blocks, n_axes,
                                          ticks_axis, acc, rem->horizon,
                                          [this](int b, int ax) -> int32_t {
                                            if (ax >= n_axes) {
                                              return 0;
                                            }
                                            return rem->delta_of(ax, b);
                                          });
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

// Step 11 Overshoot duration oracle (whitepaper section 6.4 / 7.3): each axis
// runs RampLaw over |delta_i| with its own ticks_cfg / accel; T is the largest
// of those per-axis ramp durations (the binding axis). The Overshoot track's
// wall clock must equal this sum -- it is the binding axis's ramp, never
// shortened by the non-binding axes. PC-only, no FasNAxis state.
static uint64_t naxis_overshoot_duration(const int32_t* d,
                                         const uint32_t* ticks,
                                         const uint32_t* accel, int n_axes) {
  uint64_t T = 0;
  for (int i = 0; i < n_axes; i++) {
    int32_t ad = d[i] > 0 ? d[i] : -d[i];
    if (ad == 0) {
      continue;
    }
    RampLaw law(ticks[i], accel[i], (uint32_t)ad);
    uint64_t s = 0;
    while (!law.done()) {
      s += law.step();
    }
    if (s > T) {
      T = s;
    }
  }
  return T;
}

// Step 12 Overshoot vertex oracle (whitepaper sections 6.4 / 7.3 / 8.6). Per
// axis, P evolves by the section 7.1 law over its own R_i (remaining steps in
// the current direction); a sign change or an idle-after-moving block resets
// P_i to 0 at that vertex. The exit P of an axis after a block depends only on
// its own law, so this oracle needs no command walk. `T_block[b]` is the
// binding duration max_i T_opt_i, and the shared wall clock at vertex b+1 is
// the sum of T_block[0..b]. PC-only, no FasNAxis state.
static uint32_t naxis_overshoot_law_step(const RampMap& map, uint32_t* P,
                                         uint32_t* R, uint32_t ticks_cfg) {
  uint32_t coast = map.P_coast();
  if (*R > *P) {
    if (*P < coast) {
      (*P)++;
    }
  } else {
    if (*P > 0) {
      (*P)--;
    }
  }
  uint32_t t;
  if (*P == 0) {
    t = map.calculate_ticks(1);
  } else {
    t = map.calculate_ticks(*P);
    if (t < ticks_cfg) {
      t = ticks_cfg;
    }
  }
  if (*R > 0) {
    (*R)--;
  }
  return t;
}

// Sum of the periods of `steps` law steps for one axis.
static uint64_t naxis_overshoot_sim(const RampMap& map, uint32_t P_in,
                                    uint32_t R_in, uint32_t ticks_cfg,
                                    uint32_t steps) {
  uint32_t p = P_in;
  uint32_t r = R_in;
  uint64_t sum = 0;
  for (uint32_t s = 0; s < steps; s++) {
    sum += naxis_overshoot_law_step(map, &p, &r, ticks_cfg);
  }
  return sum;
}

// `bx` / `by` are the per-block signed deltas of a 2-axis polyline. Fills
// `T_block[b]` with the binding duration and `P0_out[b]` / `P1_out[b]` with
// the per-axis exit P at vertex b+1. `cap` does not affect P, so it is unused.
static void naxis_overshoot_vertices(const int32_t* bx, const int32_t* by,
                                     int n_blocks, const uint32_t* ticks,
                                     const uint32_t* accel, uint32_t cap,
                                     uint64_t* T_block, uint32_t* P0_out,
                                     uint32_t* P1_out) {
  (void)cap;
  RampMap m0(ticks[0], accel[0]);
  RampMap m1(ticks[1], accel[1]);
  uint32_t P0 = 0, P1 = 0;
  int sgn0 = 0, sgn1 = 0;
  for (int b = 0; b < n_blocks; b++) {
    uint32_t R0 = 0, R1 = 0;
    int s0 = 0, s1 = 0;
    for (int k = b; k < n_blocks; k++) {
      int32_t dx = bx[k];
      if (dx == 0) {
        if (s0 != 0) {
          break;
        }
        continue;
      }
      int sg = dx > 0 ? 1 : -1;
      if (s0 == 0) {
        s0 = sg;
      }
      if (sg != s0) {
        break;
      }
      R0 += dx > 0 ? (uint32_t)dx : (uint32_t)-dx;
    }
    for (int k = b; k < n_blocks; k++) {
      int32_t dy = by[k];
      if (dy == 0) {
        if (s1 != 0) {
          break;
        }
        continue;
      }
      int sg = dy > 0 ? 1 : -1;
      if (s1 == 0) {
        s1 = sg;
      }
      if (sg != s1) {
        break;
      }
      R1 += dy > 0 ? (uint32_t)dy : (uint32_t)-dy;
    }
    int32_t dx = bx[b], dy = by[b];
    int ns0 = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
    int ns1 = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
    if (ns0 == 0 || (sgn0 != 0 && ns0 != sgn0)) {
      P0 = 0;
    }
    if (ns1 == 0 || (sgn1 != 0 && ns1 != sgn1)) {
      P1 = 0;
    }
    uint32_t a0 = dx > 0 ? (uint32_t)dx : (uint32_t)-dx;
    uint32_t a1 = dy > 0 ? (uint32_t)dy : (uint32_t)-dy;
    uint64_t t0 = a0 > 0 ? naxis_overshoot_sim(m0, P0, R0, ticks[0], a0) : 0;
    uint64_t t1 = a1 > 0 ? naxis_overshoot_sim(m1, P1, R1, ticks[1], a1) : 0;
    // Advance the exit P (simulate above worked on copies).
    if (a0 > 0) {
      uint32_t p = P0, r = R0;
      for (uint32_t s = 0; s < a0; s++) {
        naxis_overshoot_law_step(m0, &p, &r, ticks[0]);
      }
      P0 = p;
    }
    if (a1 > 0) {
      uint32_t p = P1, r = R1;
      for (uint32_t s = 0; s < a1; s++) {
        naxis_overshoot_law_step(m1, &p, &r, ticks[1]);
      }
      P1 = p;
    }
    T_block[b] = t0 > t1 ? t0 : t1;
    P0_out[b] = P0;
    P1_out[b] = P1;
    sgn0 = ns0;
    sgn1 = ns1;
  }
}

#endif /* NAXIS_REF_H */
