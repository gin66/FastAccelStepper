# Linear junction carry

Priority: **P1** — blocks the committed `test_naxes` `path-stops` check
and is the core smoothness fix.

Status: **implemented**. `R` is master steps to the next hard stop.
`P` carries across every other joint. Judged by `naxis_ref`
(F2, F2f, F20, the naxes 7.5° helix, F14).

## Problem

The Linear planner currently resets the path ramp at every joint that
fails the 2 deg `collinear_same_sense()` test (`src/FasNAxis.h`
`remaining_path_steps()` / `advance_block()`), so a sampled arc comes to
rest at every chord. That is wrong: a matched X/Y circle can run at a
continuous path speed, and the master reaches `v_max` exactly when the
other axis reverses (whitepaper §6.3 worked model).

## Model

- `R` (the binder's remaining master-steps) ends only at a
  **master-sense reversal** or the **path end** (last buffered point /
  `endPath()`). It does **not** end at a non-collinear joint, and it does
  not end when the DDA master role moves to another axis.
- The **master role is tied to the ramp**: "largest `|delta|`" is the
  larger path-projected speed, so on a circle the role switches where
  the two projected ramp levels cross (the 45 deg points). The switch is
  a ramp-level event; `P` carries across it and no stop is implied.
- A role switch that the outgoing axis cannot sustain given the incoming
  `R` **prepares** a deceleration to the allowed `P` over the incoming
  steps (the general joint-speed step: allowed joint speed =
  `min(incoming, outgoing implied)`, with `|delta P|` fitting the
  incoming `R`). It is not necessarily to 0.
- A **slave** axis reversal is a DDA cusp: the DDA takes it through 0
  and back; the path need not stop.
- The 2 deg `collinear_same_sense()` test becomes **diagnostic only**.

The axis-aligned square corner still stops because the outgoing master
was idle (the allowed joint speed is 0). The sampled circle cruises.

## Implementation surface

- `src/fas_naxis/remaining.h`: `remaining_linear_binder()` (the
  `collinear_same_sense()` diagnostic role lives in the PC test harness
  `test_26.cpp`).
- `src/FasNAxis.h`: `remaining_path_steps()`, `start_block()`,
  `advance_block()`.
- `extras/tests/pc_based/naxis_ref.h`: the reference track moves to the
  carried model.
- `extras/tests/pc_based/test_26.cpp`: F5 (square, still stops), F7/F8
  (circle/helix cruise), F20 (arc cruise), F2/F2b/F2f/F2g/F2h (junction
  probes), F14 helix.
- `extras/tests/simavr_based/test_naxes/detect_geometry.py`:
  `MAX_PATH_STOPS` tuned to the legitimate stops once the example
  cruises.

## Joint rule (pinned)

A joint is a hard stop (`R` ends, `P → 0`) when:

- the incoming master reverses or goes idle, or
- the outgoing master was idle on the incoming block, or
- an axis tied with the master (`|Δ|` equal) reverses or goes idle
  (F6; the tie-break must not turn a full-speed reversal into a cusp), or
- the next block is a dwell or the path ends.

Otherwise `P` carries, including a non-collinear bend and a role switch
where both axes are already moving (sampled circle / naxes helix). A
role switch that raises `ticks_floor` shortens incoming `R` to
`steps_through_the_block + P_match`, with `P_match =
calculate_ramp_steps(ticks_floor_out)` on the incoming map (F2
role-change: `P_match = 250`, `R = 8250`). A strictly shorter slave
reversal does not end `R`.

Whitepaper §8.5.

## References

- `extras/doc/n_axes_whitepaper.md` §3.1 G3, §3.2, §6.2, §6.3, §7.2,
  §8.1, §8.3, §8.4, §8.5, §8.8, §12.4.1, §14, §17
