#ifndef FAS_NAXIS_RAMP_LAW_H
#define FAS_NAXIS_RAMP_LAW_H

#include <stdint.h>

#include "fas_naxis/ramp_map.h"

// FasNAxis ramp law (whitepaper section 7.1): the per-axis (Overshoot) and
// per-binder (Linear) control law that RampControl::_getNextCommand applies.
//
// R is remaining steps to stand still in the current direction (lookahead
// scan of section 8). P is performed ramp-up steps. P starts at 0 and is
// not a precomputed min(P_stop, R/2). The two caps are applied live:
//
//   max speed:          P stops increasing at P_coast
//   remaining-to-stop:  when R == P the next steps count P down
//
//   R > P  -> accelerate: P++ (or coast when P already at P_coast)
//   R == P -> decelerate: P--
//   R < P  -> decelerate (overshoot guard, same as FAS)
//
// Period is computed after the P update, matching FAS: first step from rest
// is calculate_ticks(1); last decel step from P == 1 uses ticks_cfg.
// calculate_ticks(0) is never called.
//
// Coasting on a rest-to-rest move of N steps happens when N/2 > P_coast
// (equivalently N > 2 P_coast). A shorter N is a triangle whose peak is
// whatever P has grown to when R catches it -- an outcome, not an input.
//
// FAS_NAXIS_NO_REST_CAP (Step 2b mutation): ignore remaining-to-stop and
// always accel/coast. Short-R probes then fail to brake (peak P is not < R).
class RampLaw {
 public:
  RampMap map;
  uint32_t ticks_cfg;
  uint32_t P;  // performed ramp-up steps
  uint32_t R;  // remaining steps in the current direction
  uint32_t total_ticks;

  // `total` is the move length S (== R at start). The planner drives R down to
  // 0, one command (step) at a time.
  RampLaw(uint32_t ticks_cfg, uint32_t accel, uint32_t total)
      : map(ticks_cfg, accel),
        ticks_cfg(ticks_cfg),
        P(0),
        R(total),
        total_ticks(0) {}

  // Period (ticks) for the command at ramp position P. P == 0 is stopped:
  // calculate_ticks(0) is never called; the only issued command at P == 0 is
  // the last decel step (FAS uses ticks_cfg there). P >= 1 uses the log2
  // period clipped to ticks_cfg (and to ticks_min if a caller supplies one).
  uint32_t period(uint32_t ticks_min = 0) const {
    uint32_t t;
    if (P == 0) {
      t = map.calculate_ticks(1);
    } else {
      t = map.calculate_ticks(P);
      if (t < ticks_cfg) {
        t = ticks_cfg;
      }
    }
    if (t < ticks_min) {
      t = ticks_min;
    }
    return t;
  }

  // One command: apply the section 7.1 law to P, then issue at that P, then
  // count down one remaining step. Accel from rest is 0 -> 1 before the
  // period, so the first step is calculate_ticks(1), not ticks_cfg.
  uint32_t step(uint32_t ticks_min = 0) {
    uint32_t coast = map.P_coast();
#ifdef FAS_NAXIS_NO_REST_CAP
    if (P < coast) {
      P++;
    }
#else
    if (R > P) {
      if (P < coast) {
        P++;  // accelerate (else coast: P held at P_coast)
      }
    } else {
      if (P > 0) {
        P--;  // decelerate (R == P, or overshoot R < P)
      }
    }
#endif
    uint32_t ticks = period(ticks_min);
    total_ticks += ticks;
    if (R > 0) {
      R--;
    }
    return ticks;
  }

  // Issue `n` consecutive steps as one planning chunk. Each step re-applies
  // the law, so the chunk lands on the same P trajectory as n single steps.
  void step_chunk(uint32_t n, uint32_t ticks_min = 0) {
    for (uint32_t i = 0; i < n; i++) {
      step(ticks_min);
    }
  }

  bool done() const { return R == 0; }
};

#endif /* FAS_NAXIS_RAMP_LAW_H */
