#ifndef FAS_NAXIS_REMAINING_H
#define FAS_NAXIS_REMAINING_H

#include <stdint.h>
#include "fas_arch/common.h"

#include "fas_naxis/ramp_map.h"

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
// path is rest, so the scan simply ends at the last buffered block.
//
// Linear's binder R (sections 6.3 / 8.5) sums DDA-master steps until a hard
// stop: the master's sense ends, the outgoing master was idle, a dwell, or
// the path end. A non-collinear joint does not end it, and neither does a
// master-role change. collinear_same_sense() is diagnostic only.
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
  // Two uint32 halves of a product. Not a 64-bit type: productive code has
  // none. hi is the upper half.
  struct U32p {
    uint32_t hi;
    uint32_t lo;
  };

  // 2*err >= master, without a widening multiply. err >= 2^31 implies
  // 2*err >= 2^32 > master.
  static bool u32_twice_ge(uint32_t err, uint32_t master) {
    if (err >= 0x80000000u) {
      return true;
    }
    return (err << 1) >= master;
  }

  static U32p u32_mul(uint32_t a, uint32_t b) {
    uint32_t al = a & 0xffffu;
    uint32_t ah = a >> 16;
    uint32_t bl = b & 0xffffu;
    uint32_t bh = b >> 16;
    uint32_t p0 = al * bl;
    uint32_t p1 = al * bh;
    uint32_t p2 = ah * bl;
    uint32_t p3 = ah * bh;
    uint32_t mid = (p0 >> 16) + (p1 & 0xffffu) + (p2 & 0xffffu);
    U32p r;
    r.lo = (p0 & 0xffffu) | (mid << 16);
    r.hi = p3 + (p1 >> 16) + (p2 >> 16) + (mid >> 16);
    return r;
  }

  static int u32p_cmp(U32p a, U32p b) {
    if (a.hi != b.hi) {
      return a.hi > b.hi ? 1 : -1;
    }
    if (a.lo != b.lo) {
      return a.lo > b.lo ? 1 : -1;
    }
    return 0;
  }

  static U32p u32p_add(U32p a, U32p b) {
    U32p r;
    r.lo = a.lo + b.lo;
    r.hi = a.hi + b.hi + (r.lo < a.lo ? 1u : 0u);
    return r;
  }

  static U32p u32p_sub(U32p a, U32p b) {
    U32p r;
    r.lo = a.lo - b.lo;
    r.hi = a.hi - b.hi - (a.lo < b.lo ? 1u : 0u);
    return r;
  }

  static bool u32p_is_zero(U32p a) { return a.hi == 0 && a.lo == 0; }

  // a * m into 64 bits of magnitude. False when a third limb would be set.
  static bool u32p_mul_u32(U32p a, uint32_t m, U32p* out) {
    U32p p0 = u32_mul(a.lo, m);
    U32p p1 = u32_mul(a.hi, m);
    uint32_t hi = p0.hi + p1.lo;
    if (hi < p0.hi || p1.hi != 0) {
      return false;
    }
    out->lo = p0.lo;
    out->hi = hi;
    return true;
  }

  // >0 when a*b > c*d.
  static int u32_mul_cmp(uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
    return u32p_cmp(u32_mul(a, b), u32_mul(c, d));
  }

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

  // Linear binder R from `head`: DDA-master steps to the next hard stop
  // (section 8.5). `axis` is unused; the currency is master steps, not one
  // axis's |delta|. HORIZON caps the points. Ticks/accel are unknown here, so
  // the slower-envelope preparation is not applied (call linear_remaining).
  int32_t remaining_linear_binder(int axis, int head) const {
    (void)axis;
    return (int32_t)linear_remaining<8>(
        head, n_blocks, n_axes, NULL, NULL, horizon,
        [this](int b, int ax) -> int32_t { return delta_of(ax, b); });
  }

  // Section 8.5 collinear, same sense between two full path directions.
  //   (dot(d, d'))^2 * 100000    >=   99878 * |d|^2 * |d'|^2
  //   (cos^2(2deg) ~= 0.99878). Integer mul/compare, no division, no sqrt.
  //  The dot sign keeps "same sense": opposite senses fail.
  bool collinear_same_sense(int block_a, int block_b) const {
    bool dot_neg = false;
    U32p dot = {0, 0};
    U32p mag_a = {0, 0};
    U32p mag_b = {0, 0};
    for (int i = 0; i < n_axes; i++) {
      int32_t da = delta_of(i, block_a);
      int32_t db = delta_of(i, block_b);
      bool neg = (da < 0) != (db < 0);
      uint32_t ua = fas_abs(da);
      uint32_t ub = fas_abs(db);
      U32p prod = u32_mul(ua, ub);
      if (u32p_is_zero(dot)) {
        dot = prod;
        dot_neg = neg && !u32p_is_zero(prod);
      } else if (dot_neg == neg) {
        dot = u32p_add(dot, prod);
      } else if (u32p_cmp(dot, prod) >= 0) {
        dot = u32p_sub(dot, prod);
      } else {
        dot = u32p_sub(prod, dot);
        dot_neg = !dot_neg;
      }
      mag_a = u32p_add(mag_a, u32_mul(ua, ua));
      mag_b = u32p_add(mag_b, u32_mul(ub, ub));
    }
    if (dot_neg || u32p_is_zero(dot) || u32p_is_zero(mag_a) ||
        u32p_is_zero(mag_b)) {
      return false;
    }
    if (dot.hi != 0 || mag_a.hi != 0 || mag_b.hi != 0) {
      return false;
    }
    U32p dot2 = u32_mul(dot.lo, dot.lo);
    U32p mag = u32_mul(mag_a.lo, mag_b.lo);
    U32p lhs;
    U32p rhs;
    if (!u32p_mul_u32(dot2, 100000u, &lhs) ||
        !u32p_mul_u32(mag, 99878u, &rhs)) {
      return false;
    }
    return u32p_cmp(lhs, rhs) >= 0;
  }
#endif /* FAS_NAXIS_REFERENCE */

  // True when every axis of the block is idle (a dwell, or a zero vector).
  static bool block_idle(const int32_t* block, int n_axes) {
    for (int i = 0; i < n_axes; i++) {
      if (block[i] != 0) {
        return false;
      }
    }
    return true;
  }

  // Section 8.5 hard stop between block A and block B. P goes to 0 here.
  // The 2 deg collinear test is not consulted.
  static bool linear_joint_stops(const int32_t* a, const int32_t* b,
                                 const uint32_t* ticks, int n_axes) {
    if (block_idle(a, n_axes) || block_idle(b, n_axes)) {
      return true;
    }
    int ma = longest_axis(a, ticks, n_axes);
    int mb = longest_axis(b, ticks, n_axes);
    int sa = a[ma] > 0 ? 1 : -1;
    int sb = b[ma] > 0 ? 1 : (b[ma] < 0 ? -1 : 0);
    if (sb != sa) {
      return true;  // master reverses or goes idle
    }
    if (a[mb] == 0) {
      return true;  // outgoing master was idle
    }
    // An axis tied with the master (|delta| equal) that reverses or goes
    // idle is a full-speed reversal. The tie-break that named the other
    // axis master does not make that reversal a slave cusp (F6).
    int32_t adm = a[ma] > 0 ? a[ma] : -a[ma];
    for (int i = 0; i < n_axes; i++) {
      if (i == ma) {
        continue;
      }
      int32_t adi = a[i] > 0 ? a[i] : -a[i];
      if (adi != adm || adi == 0) {
        continue;
      }
      int so = b[i] > 0 ? 1 : (b[i] < 0 ? -1 : 0);
      int si = a[i] > 0 ? 1 : -1;
      if (so != si) {
        return true;
      }
    }
    return false;
  }

  // Linear R from `head`: master steps to the next hard stop. When `ticks`
  // and `accel` are both set, a role change that raises ticks_floor shortens
  // R so the incoming ramp reaches the outgoing period before the vertex
  // (P_match = calculate_ramp_steps(ticks_floor_out) on the incoming map).
  // `horizon` 0 or 0xFFFFFFFF means unbounded. `Delta` is (block, axis).
  // N is the maximum axis count the caller will pass.
  template <int N, typename Delta>
  static uint32_t linear_remaining(int head, int n_blocks, int n_axes,
                                   const uint32_t* ticks, const uint32_t* accel,
                                   uint32_t horizon, Delta delta) {
    if (head >= n_blocks || n_axes <= 0 || n_axes > N) {
      return 0;
    }
    int32_t blk[N];
    int32_t nxt[N];
    for (int i = 0; i < N; i++) {
      blk[i] = 0;
      nxt[i] = 0;
    }
    for (int i = 0; i < n_axes; i++) {
      blk[i] = delta(head, i);
    }
    if (block_idle(blk, n_axes)) {
      return 0;
    }
    uint32_t sum = 0;
    uint32_t best = 0xFFFFFFFFU;
    int included = 0;
    bool limit_h = horizon != 0 && horizon != 0xFFFFFFFFU;
    for (int b = head; b < n_blocks; b++) {
      if (b != head) {
        for (int i = 0; i < n_axes; i++) {
          blk[i] = delta(b, i);
        }
        if (block_idle(blk, n_axes)) {
          break;
        }
      }
      int m = longest_axis(blk, ticks, n_axes);
      int32_t ad = blk[m] > 0 ? blk[m] : -blk[m];
      sum += (uint32_t)ad;
      included++;
#ifdef FAS_NAXIS_NO_CROSS_BLOCK_R
      best = sum;
      break;
#endif
      if (limit_h && included >= (int)horizon) {
        break;
      }
      if (b + 1 >= n_blocks) {
        break;
      }
      for (int i = 0; i < n_axes; i++) {
        nxt[i] = delta(b + 1, i);
      }
      if (linear_joint_stops(blk, nxt, ticks, n_axes)) {
        break;
      }
      if (ticks != NULL && accel != NULL) {
        int mb = longest_axis(nxt, ticks, n_axes);
        if (m != mb) {
          uint32_t tin = ticks_floor(blk, ticks, n_axes);
          uint32_t tout = ticks_floor(nxt, ticks, n_axes);
          if (tout > tin && tin > 0) {
            int binder = binder_axis(blk, ticks, n_axes);
            RampMap map(tin, accel[binder]);
            uint32_t capped = sum + map.calculate_ramp_steps(tout);
            if (capped < best) {
              best = capped;
            }
          }
        }
      }
    }
    if (best < sum) {
      return best;
    }
    return sum;
  }

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
    uint32_t best_ad = 0;
    uint32_t best_t = 0;
    bool any = false;
    for (int i = 0; i < n_axes; i++) {
      if (block[i] == 0) {
        continue;
      }
      uint32_t ad = fas_abs(block[i]);
#ifdef FAS_NAXIS_NO_REBIND
      uint32_t t = 0;
#else
      uint32_t t = ticks[i];
#endif
      if (!any) {
        any = true;
        best_ad = ad;
        best_t = t;
        b = i;
        continue;
      }
#ifdef FAS_NAXIS_NO_REBIND
      if (ad > best_ad) {
#else
      if (u32_mul_cmp(ad, t, best_ad, best_t) > 0) {
#endif
        best_ad = ad;
        best_t = t;
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
    uint32_t abs_bind = fas_abs(delta_bind);
    uint32_t abs_slave = fas_abs(delta_slave);
    if (abs_bind == 0) {
      return 0;
    }
    int32_t err = 0;
    int steps = 0;
    for (uint32_t i = 0; i < abs_bind; i++) {
      err += (int32_t)abs_slave;
      if (err > 0 && u32_twice_ge((uint32_t)err, abs_bind)) {
        steps++;
        err -= (int32_t)abs_bind;
      }
    }
    return steps;
  }
};

#endif /* FAS_NAXIS_REMAINING_H */
