# FasNAxis — open items

Source of truth: `extras/doc/n_axes_whitepaper.md`.

The test-driven implementation plan (Steps 0–14) is complete; its
tests live in `extras/tests/pc_based/test_26.cpp`. Remaining work is
tracked here, one file per item.

## Tracked entries (priority order)

| Priority | Item | Why now |
|----------|------|---------|
| **P5** | [Engine synchronized start](engine_synchronized_start.md) | Kick-off is still one `addQueueEntry(NULL, true)` per axis. |
| **P6** | [Cubic start (`s_h`) overlay](cubic_start.md) | Later feature, not v1. |
| **P7** | [Faithful timed trajectory](timed_trajectory.md) | Later implementation, not v1. |

## Done

- **naxes example smoothness — implemented.** simavr `test_naxes` passes
  with 11 legitimate stops (`MAX_PATH_STOPS = 11`); no helix chord stops.
  P3 also found and fixed the block ring not sliding past `HORIZON`
  (`FasNAxis::compact_ring()`), which had chunked any path longer than
  `HORIZON` into per-ring ramp-to-rest segments. See
  [naxes_example_smoothness.md](naxes_example_smoothness.md).
- **naxes log2 product compares — implemented.** The production naxes
  planner no longer schoolbook-multiplies: the `binder_axis` tie-break and
  the Overshoot uniform schedule / cap-side sign checks compare products
  as log2 sums (`Remaining::log2_mul_cmp`; sums of `log2_from`, the
  sanctioned slack per section 6.3), and `u32_mul_cmp` is gone. The exact
  `U32p` product stays where a quantum would flip a step: `outside_cap`'s
  chord-cap geometry against the hard `overshoot_max` bound (6.5 / 12.4.1
  G7) and the reference `collinear_same_sense` probe; `u32_twice_ge`
  is a shift, not a product. The F2b rebind neighbourhood, all
  F11/F12/F14 cap bounds, and the `FAS_NAXIS_NO_REBIND` /
  `FAS_NAXIS_NO_REST_CAP` mutation proofs still hold. See
  whitepaper §6.3 / §6.6.
- **Linear junction carry — implemented.** `R` ends at a master-sense
  reversal, an idle or tied outgoing axis, a dwell, or the path end.
  `P` carries across every other joint, so a sampled helix cruises and
  the axis-aligned square still stops. See
  [linear_junction_carry.md](linear_junction_carry.md) and whitepaper
  §8.5.
- **Feeder command batching — implemented.** The ramp generator's
  command size: one step when the period is already at least 1 ms, and
  about 2 ms of equal-period steps when it is shorter. Productive code
  uses 32-bit integers only; `naxes` on the ATmega328 is 27384 bytes
  (limit 30720). See
  [feeder_command_batching.md](feeder_command_batching.md) and
  whitepaper §4.3.1.

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
