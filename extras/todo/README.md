# FasNAxis — open items

Source of truth: `extras/doc/n_axes_whitepaper.md`.

The test-driven implementation plan (Steps 0–14) is complete; its
tests live in `extras/tests/pc_based/test_26.cpp`. Remaining work is
tracked here, one file per item.

## Tracked entries

- [naxes hardware/simavr validation](naxes_hw_validation.md)
- [Cubic start (`s_h`) overlay](cubic_start.md)
- [AFAP vs timed — two implementations](planner_modes.md)
- [Faithful timed trajectory](timed_trajectory.md)
- [Pluggable motion sources — engine generalization](engine_sources.md)

## Out-of-scope log

Decisions taken item by item. Items without an entry are documented as
non-goals in the whitepaper §3.2 and are not tracked separately.

- **Raising `pd_test` `MAX_STEPPER` — dropped.** Not debt but a design
  decision: n > 2 PC tests use `FasNAxis<N, HORIZON, SimPort>`; 1- and
  2-axis golden paths use real FAS queues (whitepaper §4.7).
- **simavr / hardware / PlatformIO jobs for FasNAxis — tracked**
  (see `naxes_hw_validation.md`).
- **Per-block feedrate `F` — not a separate item.** It is a requested
  speed, hence timed-world input; it belongs to `timed_trajectory.md`
  (no AFAP `F`-as-cap variant).
- **Inverse kinematics — not tracked.** Application concern, not a
  library feature. Keep the whitepaper §3.2 non-goal, but clarify that
  the affine motor-map is caller-side too (the caller transforms
  waypoints; FasNAxis stays in step space).
- **Running `pump()` from `manageSteppers()` and generalizing the ramp
  generator / naxes — merged and tracked** (see `engine_sources.md`).
- **A Linear oracle that is faster by leaving the chord / cutting a
  corner / skipping a vertex — not tracked.** Not a backlog item: such a
  track is not constraint-faithful, so it is not a faster Linear
  reference at all (§12.4.1). The sanctioned mode that may leave the
  chord is Overshoot; a deliberate corner-blend geometry stays a §3.2
  later overlay (G1/G2), not an oracle.

## Under discussion

(none)
