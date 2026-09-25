#ifndef FAS_NAXIS_REMAINING_H
#define FAS_NAXIS_REMAINING_H

#include <stdint.h>

// FasNAxis remaining-steps scan R and DDA / time-law oracle (whitepaper
// sections 6.3 and 8).
//
// The DDA / time-law functions (longest_axis, ticks_floor, binder_axis,
// dda_steps) are stateless and are used by the production planner and by the
// PC reference tests.
//
// The stateful remaining-steps scan R and the collinearity test are used only
// by the PC reference track (extras/tests/pc_based). They are compiled only
// when FAS_NAXIS_REFERENCE is defined, so a target build carries no Remaining
// object and, in particular, no backing store for the reference scan.
//
// R is the lookahead kernel: per axis it is the sum of |delta_i| over the
// block ring until the first of (path end, that axis going idle after it
// moved, or a sign flip of that axis). The last buffered point of an open
// path is rest, so the scan simply ends at the last buffered block. A
// non-collinear vertex (section 8.5) additionally ends the *Linear* scan.
//
// This is pure parse: integer add / abs / sign only. No float, double, or
// integer division is formed in the scan. The live speed cap is the ramp
// law in ramp_law.h (P starts at 0, increases, clipped by P_coast and by
// remaining-to-standstill). Remaining.h does not estimate a peak as
// min(P_stop, R/2).
//
// HORIZON (a public field, default unbounded) caps the number of *points*
// the scan may touch: section 8.2 / F19 (a small HORIZON of micro-segments
// caps P below P_stop; the same HORIZON with one long block still coasts
// because R is steps, not points).
//
// Compile-time hook (Step 2b mutation probe) proves the theory fails on a
// wrong model:
//   FAS_NAXIS_NO_REBIND  -> binder_axis is longest-distance only (drops ticks)
// binder_axis is the time-law oracle (wall-clock). The interpolator DDA
// master is longest_axis; ticks_floor lengthens the master's period.
// Remaining-to-standstill is the ramp law (FAS_NAXIS_NO_REST_CAP lives in
// ramp_law.h).
class Remaining {
 public:
#ifdef FAS_NAXIS_REFERENCE
  int n_axes;
  int n_blocks;
  int32_t* deltas;   // n_axes x n_blocks, row-major [axis * n_blocks + block]
  uint32_t horizon;  // max points scanned (0 or INT32_MAX = unbounded)

  // The backing store is sized for the reference caller (the PC test block
  // rings are at most 512 n-dim blocks).
  Remaining(int axes, int blocks)
      : n_axes(axes), n_blocks(blocks), horizon(0xFFFFFFFFU) {
    static int32_t store[1024];
    deltas = store;
    for (int i = 0; i < n_axes * n_blocks; i++) {
      store[i] = 0;
    }
  }

  // Write one n-dim block (signed per-axis displacement).
  void set_block(int block, const int32_t* d) {
    for (int i = 0; i < n_axes; i++) {
      deltas[i * n_blocks + block] = d[i];
    }
  }

  // Signed displacement of axis `axis` in block `block`.
  int32_t delta_of(int axis, int block) const {
    return deltas[axis * n_blocks + block];
  }

  // Section 8.1 per-axis parse from head block `head`. Walks forward,
  // summing |delta| until idle-after-move or a sign flip; HORIZON caps the
  // number of blocks touched. The last buffered point is rest, so an open
  // path stops at the last block naturally.
  int32_t remaining(int axis, int head) const {
    int32_t R = 0;
    int sign = 0;
    int count = 0;
    for (int b = head; b < n_blocks; b++) {
      int32_t d = deltas[axis * n_blocks + b];
      if (d == 0) {
        if (sign != 0) {
          break;  // direction ended: axis went idle after moving
        }
        continue;  // idle on this block; has not started a direction yet
      }
      int s = (d > 0) ? 1 : -1;
      if (sign == 0) {
        sign = s;
      }
      if (s != sign) {
        break;  // reversal: must be at 0 here
      }
      R += (d > 0) ? d : -d;
      if (horizon != 0 && horizon != 0xFFFFFFFFU) {
        count++;
        if (count >= (int)horizon) {
          break;  // HORIZON (points) reached
        }
      }
    }
    return R;
  }

  // Linear binder path-stop (sections 8.1 + 8.5): scan `axis` from `head`,
  // but the first non-collinear vertex ends the scan even when the axis
  // would continue (a square's side is one R-budget, not the perimeter).
  // HORIZON caps the points.
  int32_t remaining_linear_binder(int axis, int head) const {
    if (n_axes == 1 || head + 1 >= n_blocks) {
      return remaining(axis, head);
    }
    int32_t R = 0;
    int sign = 0;
    int count = 0;
    for (int b = head; b < n_blocks; b++) {
      int32_t d = delta_of(axis, b);
      if (d == 0) {
        if (sign != 0) {
          break;
        }
      } else {
        int s = (d > 0) ? 1 : -1;
        if (sign == 0) {
          sign = s;
        }
        if (s != sign) {
          break;  // reversal
        }
      }
      if (b > head && !collinear_same_sense(b - 1, b)) {
        break;  // non-collinear vertex: Linear path-stop
      }
      R += (d > 0) ? d : -d;
      if (horizon != 0 && horizon != 0xFFFFFFFFU) {
        count++;
        if (count >= (int)horizon) {
          break;
        }
      }
    }
    return R;
  }

  // Section 8.5 collinear, same sense between two full path directions.
  //   (dot(d, d'))^2 * 100000    >=   99878 * |d|^2 * |d'|^2
  //   (cos^2(2deg) ~= 0.99878). Integer mul/compare, no division, no sqrt.
  //  The dot sign keeps "same sense": opposite senses fail.
  bool collinear_same_sense(int block_a, int block_b) const {
    int64_t dot = 0;
    int64_t mag_a = 0;
    int64_t mag_b = 0;
    for (int i = 0; i < n_axes; i++) {
      int64_t da = delta_of(i, block_a);
      int64_t db = delta_of(i, block_b);
      dot += da * db;
      mag_a += da * da;
      mag_b += db * db;
    }
    if (dot <= 0 || mag_a == 0 || mag_b == 0) {
      return false;  // opposite sense or a zero vector
    }
    int64_t lhs = dot * dot * 100000;
    int64_t rhs = 99878 * mag_a * mag_b;
    return lhs >= rhs;
  }
#endif /* FAS_NAXIS_REFERENCE */

  // ---- DDA / time-law oracle (whitepaper section 6.3 / 8.3) --------------
  // Pure static functions, no state. The planner must match these on the
  // same inputs. Integer compare only (no integer division in the binder
  // selection).

  // Section 6.3 DDA master: longest |delta|, tie-break on larger ticks_cfg
  // (the slower motor). This is the interpolator loop bound. A shorter axis
  // is never the master: DDA slaves take 0 or 1 step per master step, and
  // issued |steps_i| equals |delta_i| (the vertex is hit). Time-law rebind
  // does not change this axis; it lengthens ticks_b (ticks_floor).
  static int longest_axis(const int32_t* block, const uint32_t* ticks,
                          int n_axes) {
    int b = 0;
    int32_t best_ad = -1;
    uint32_t best_t = 0;
    for (int i = 0; i < n_axes; i++) {
      int32_t ad = block[i] > 0 ? block[i] : -block[i];
      if (ad == 0) {
        continue;
      }
      uint32_t t = ticks ? ticks[i] : 0;
      if (ad > best_ad || (ad == best_ad && t > best_t)) {
        best_ad = ad;
        best_t = t;
        b = i;
      }
    }
    return b;
  }

  // Section 6.3 time-law floor: max ticks_i_cfg over axes with delta_i != 0.
  // Integer max, no division. Every shared command then satisfies every
  // moving axis envelope (ticks >= ticks_i_cfg). When a slow short slave
  // would lose the wall-clock compare, this lengthens the master's period
  // ("lengthen ticks_b") instead of walking DDA on the short axis.
  static uint32_t ticks_floor(const int32_t* block, const uint32_t* ticks,
                              int n_axes) {
    uint32_t t = 0;
    for (int i = 0; i < n_axes; i++) {
      if (block[i] == 0) {
        continue;
      }
      if (ticks[i] > t) {
        t = ticks[i];
      }
    }
    return t;
  }

  // Section 6.3 time-law oracle: the axis that would take the longest
  // wall-clock at its own max speed, i.e. argmax over i of |delta_i| *
  // ticks_i. "Longest |delta| first" when ticks are equal; "rebind if
  // |delta_i|*ticks_i > |delta_b|*ticks_b" when a slow slave dominates.
  // This names who constrains the period, not who walks DDA (that is
  // longest_axis). FAS_NAXIS_NO_REBIND drops the ticks factor
  // (longest-distance only) so the rebind neighbourhood (item 3 of Step
  // 2b) fails when the model is wrong.
  static int binder_axis(const int32_t* block, const uint32_t* ticks,
                         int n_axes) {
    int b = 0;
    int64_t best = -1;
    for (int i = 0; i < n_axes; i++) {
      int64_t ad = block[i] > 0 ? block[i] : -block[i];
      if (ad == 0) {
        continue;
      }
      int64_t score;
#ifdef FAS_NAXIS_NO_REBIND
      score = ad;  // longest-distance only: rebind disabled
#else
      score = ad * (int64_t)ticks[i];  // wall-clock: rebind to slow motor
#endif
      if (score > best) {
        best = score;
        b = i;
      }
    }
    return b;
  }

  // Section 6.3 DDA step count for one block: how many steps slave `slave`
  // issues over the binder's |delta_bind| steps (err += |slave|; if
  // 2*err >= |bind| then step and err -= |bind|). The binder itself (slave
  // magnitude == bind) issues |delta_bind| steps; an idle slave issues 0.
  // Used by the PC reference tests.
  static int dda_steps(int delta_bind, int delta_slave) {
    int64_t abs_bind = delta_bind > 0 ? delta_bind : -delta_bind;
    int64_t abs_slave = delta_slave > 0 ? delta_slave : -delta_slave;
    if (abs_bind == 0) {
      return 0;
    }
    int64_t err = 0;
    int steps = 0;
    for (int i = 0; i < abs_bind; i++) {
      err += abs_slave;
      if (2 * err >= abs_bind) {
        steps++;
        err -= abs_bind;
      }
    }
    return steps;
  }
};

#endif /* FAS_NAXIS_REMAINING_H */
