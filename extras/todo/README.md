# FasNAxis — open items

Source of truth: `extras/doc/n_axes_whitepaper.md`.

The test-driven implementation plan (Steps 0–14) is complete; its
tests live in `extras/tests/pc_based/test_26.cpp`. Remaining work is
tracked here, one file per item.

## Tracked entries (priority order)

| Priority | Item | Why now |
|----------|------|---------|
| **P1** | [Linear junction carry](linear_junction_carry.md) | Core smoothness fix; unblocks the committed `test_naxes` `path-stops` check. |
| **P2** | [Feeder command batching](feeder_command_batching.md) | AVR/ESP32 queue-drain safety; prerequisite for the example on hardware. |
| **P3** | [naxes example smoothness](naxes_example_smoothness.md) | End-to-end simavr/hardware check; depends on P1 + P2. |
| **P4** | [Cubic start (`s_h`) overlay](cubic_start.md) | Later feature, not v1. |
| **P5** | [Faithful timed trajectory](timed_trajectory.md) | Later implementation, not v1. |

## Out-of-scope log

Decisions taken item by item. Items without an entry are documented as
non-goals in the whitepaper §3.2 and are not tracked separately.

- **Raising `pd_test` `MAX_STEPPER` — dropped.** Not debt but a design
  decision: n > 2 PC tests use `FasNAxis<N, HORIZON, SimPort>`; 1- and
  2-axis golden paths use real FAS queues (whitepaper §4.7).
- **simavr / hardware / PlatformIO jobs for FasNAxis — implemented.**
  `examples/naxes/` (helix → hexagon → square → origin, one
  `FastAccelStepper` per axis) builds for every CI architecture;
  `extras/tests/simavr_based/test_naxes/` runs it on the ATmega328p and
  `detect_geometry.py` judges the reconstructed curve. The 16 KB
  ATmega168 and the ATmega32u4 are skipped by `build-platformio.sh` for
  space.
- **Per-block feedrate `F` — not a separate item.** It is a requested
  speed, hence timed-world input; it belongs to `timed_trajectory.md`
  (no AFAP `F`-as-cap variant).
- **Inverse kinematics — not tracked.** Application concern, not a
  library feature. Keep the whitepaper §3.2 non-goal, but clarify that
  the affine motor-map is caller-side too (the caller transforms
  waypoints; FasNAxis stays in step space).
- **Running `pump()` from `manageSteppers()` and generalizing the ramp
  generator / naxes — implemented.** Design record moved to
  `extras/doc/engine_sources.md` (Path A single driver + stop hook).
- **AFAP vs timed — decided.** `FasNAxis` stays AFAP-only; the faithful
  timed trajectory is a separate implementation (tracked in
  `timed_trajectory.md`). Design record moved to
  `extras/doc/planner_modes.md`.
- **A Linear oracle that is faster by leaving the chord / cutting a
  corner / skipping a vertex — not tracked.** Not a backlog item: such a
  track is not constraint-faithful, so it is not a faster Linear
  reference at all (§12.4.1). The sanctioned mode that may leave the
  chord is Overshoot; a deliberate corner-blend geometry stays a §3.2
  later overlay (G1/G2), not an oracle.

## Under discussion

(none)
