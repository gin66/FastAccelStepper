# naxes: log2 instead of 16/32-bit multiplication

Priority: **P4** — code size and the hot path, after the 32-bit
rewrite.

Status: not started.

## Problem

Productive code no longer uses 64-bit integers. The replacement in
`src/fas_naxis/remaining.h` is a 16×16 schoolbook multiply
(`Remaining::u32_mul`) and compares of those products
(`u32_mul_cmp`). Overshoot calls that compare on every command
(`tot[i] * t` against `(k + 1) * T` in `overshoot.h`). The binder
tie-break and the chord-cap check do the same.

That is still a general multiply. The ramp map already answers the
same kind of question with `log2_multiply` / `log2_divide` in
`src/fas_ramp/RampCalculator.h` and `src/log2/`.

## Fix

Replace those 16-bit and 32-bit multiplies in the naxes planner with
log2 arithmetic. A compare of two products is a compare of sums of
logs. A quotient that is already allowed to be the log2 approximation
(`ticks * steps` fitting a command) should be `log2_divide`, not a
wide multiply followed by an exactness check.

Keep a multiply only where the result must be bit-exact and log2
slack would change a step. Say which call sites those are.

## References

- `src/fas_naxis/remaining.h` — `u32_mul`, `u32_mul_cmp`.
- `src/fas_naxis/overshoot.h` — per-command product compare, cap.
- `src/fas_ramp/RampCalculator.h` — existing log2 usage.
