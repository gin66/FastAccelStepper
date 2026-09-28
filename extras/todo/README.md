# Open items (TODO)

Library-wide open items, one file per item. The list is not
FasNAxis-specific: the ramp-generator arrival (080) is single-axis,
and the rest is n-axis work.

The filename prefix is the priority, three digits wide
(`050_name.md`). Numbers step by 10, so a new item takes a free
number between two existing ones (`055_` sits between `050_` and
`060_`). Items that share a priority share the prefix.

Source of truth for the n-axis items: `extras/doc/n_axes_whitepaper.md`.
Their test-driven implementation plan (Steps 0–14) is complete; its
tests live in `extras/tests/pc_based/test_26.cpp`.

## Tracked entries (priority order)

| Priority | Item | Why now |
|----------|------|---------|
| **040** | [IDF 6 RMT runs sequence 02 slow](040_idf6_rmt_slow.md) | Pin trace is 120.8 s vs 91.6 s, same step count. A ~10 ms hole every 20 ms of motion. |
| **050** | [ESP32 synchronized start](050_esp32_synchronized_start.md) | Native per-driver release (I2S group, RMT group start, MCPWM/PCNT) pending. |
| **050** | [Pico synchronized start](050_pico_synchronized_start.md) | PIO block-start HW sync for multiple steppers to be verified. |
| **050** | [AVR synchronized start](050_avr_synchronized_start.md) | Shared-timer start likely final; verify and close. |
| **050** | [SAM synchronized start](050_sam_synchronized_start.md) | PWM/TC common release point to be identified. |
| **050** | [SAMD51 synchronized start](050_samd51_synchronized_start.md) | TCC cross-instance release to be identified. |
| **050** | [Teensy synchronized start](050_teensy_synchronized_start.md) | TMR within/cross-module release to be decided. |
| **060** | [Cubic start (`s_h`) overlay](060_cubic_start.md) | Later feature, not v1. |
| **070** | [Common head speed](070_common_head_speed.md) | Later planner. One acceleration and one max path speed for an x/y/z/… head. Waypoints are `dx, dy, dz, …, v`. |
| **080** | [Delta steps](080_delta_steps.md) | AFAP input variation: `int16_t` chunks instead of absolute waypoints. |
| **080** | [Ramp time and moveTo eta](080_move_to_eta.md) | Record ramp time next to performed ramp steps; `moveTo(position, eta_ticks)` caps speed so the move finishes by that tick. |
| **090** | [Smooth stop at end of path](090_end_path_decel.md) | Open: append a decel tail on `endPath()`, or hand the stop to the ramp generator. |
| **100** | [Pipeline position estimate](100_position_pipeline_estimate.md) | Low priority. Estimate how far a pipelined driver has played out, so `getCurrentPosition()` can lead the pin by less. A pulse counter remains the real position. |
| **110** | [RMT V1/V2 file split](110_rmt_v1_v2_split.md) | Low hanging fruit. `SUPPORT_RMT_V1` / `SUPPORT_RMT_V2` instead of one flag for both and `V2` only for IDF5/6. |

## Done

- **ESP32 RMT extra step — implemented.** IDF5/6 translates queue
  commands in `StepperISR_idf5_esp32_rmt_encode.cpp` instead of filling
  fixed RMT halves. 20× `seq_02` and 20× `seq_03` passed on IDF5 RMT.
  See [esp32_rmt_extra_step.md](../doc/implemented/esp32_rmt_extra_step.md).
- **Faithful timed trajectory — implemented.** Separate from
  `FasNAxis`: `FasTimed::addDelta` takes per-axis delta steps in
  [-128, 128] and one shared duration in [MIN_CMD_TICKS, 65535]
  ticks. One call is one rate. A rate the motors cannot reach is
  `TimingNotAchievable`. PC test `extras/tests/pc_based/test_29.cpp`.
  See [timed_trajectory.md](../doc/implemented/timed_trajectory.md).
- **Engine synchronized start (generic layer) — implemented.** The engine
  exposes a plain non-static `synchronizedStart()` member, `FasNAxis`
  receives the engine in its constructor (defaulted `Engine` template
  parameter), and the kick-off releases all active queues in one engine
  operation. The per-platform native mechanisms remain tracked above. See
  [engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md).
- **NaxesAFAP example smoothness — implemented.** simavr `test_NaxesAFAP` passes
  with 11 legitimate stops (`MAX_PATH_STOPS = 11`); no helix chord stops.
  P3 also found and fixed the block ring not sliding past `HORIZON`
  (`FasNAxis::compact_ring()`), which had chunked any path longer than
  `HORIZON` into per-ring ramp-to-rest segments. See
  [NaxesAFAP_example_smoothness.md](../doc/implemented/NaxesAFAP_example_smoothness.md).
- **naxes log2 product compares — implemented.** The production naxes
  planner has no 64-bit type and no fake 64-bit emulation: the `binder_axis`
  tie-break, the Overshoot uniform schedule, and the Overshoot cap compare
  products as log2 sums (`Remaining::log2_mul_cmp` / `log2_mul_diff`, sums
  of `log2_from`, the sanctioned slack per section 6.3). `outside_cap` uses
  the conservative bound `abs(nb*k - ns*x) <= cap*max(nb,ns)` (>= the exact
  cap) with a four-unit log2 margin, so the realized distance stays under
  the hard `overshoot_max` bound (6.5 / 12.4.1 G7). The 2 deg
  `collinear_same_sense` diagnostic moved to the PC test harness
  (`test_26.cpp`, double); `u32_twice_ge` is a shift, not a product. The F2b
  rebind neighbourhood, all F11/F12/F14 cap bounds, and the
  `FAS_NAXIS_NO_REBIND` / `FAS_NAXIS_NO_REST_CAP` mutation proofs still
  hold. See whitepaper §6.3 / §6.6.
- **Linear junction carry — implemented.** `R` ends at a master-sense
  reversal, an idle or tied outgoing axis, a dwell, or the path end.
  `P` carries across every other joint, so a sampled helix cruises and
  the axis-aligned square still stops. See
  [linear_junction_carry.md](../doc/implemented/linear_junction_carry.md) and whitepaper
  §8.5.
- **Feeder command batching — implemented.** The ramp generator's
  command size: one step when the period is already at least 1 ms, and
  about 2 ms of equal-period steps when it is shorter. Productive code
  uses 32-bit integers only; `NaxesAFAP` on the ATmega328 is 27384 bytes
  (limit 30720). See
  [feeder_command_batching.md](../doc/implemented/feeder_command_batching.md) and
  whitepaper §4.3.1.

## Out-of-scope log

Decisions taken item by item. Items without an entry are documented as
non-goals in the whitepaper §3.2 and are not tracked separately.

- **Raising `pd_test` `MAX_STEPPER` — dropped.** Not debt but a design
  decision: n > 2 PC tests use `FasNAxis<N, HORIZON, SimPort>`;
  1- and 2-axis golden paths use real FAS queues (whitepaper §4.7).
- **simavr / hardware / PlatformIO jobs for FasNAxis — implemented.**
  `examples/NaxesAFAP/` (helix → hexagon → square → origin, one
  `FastAccelStepper` per axis) builds for every CI architecture;
  `extras/tests/simavr_based/test_NaxesAFAP/` runs it on the ATmega328p and
  `detect_geometry.py` judges the reconstructed curve. The 16 KB
  ATmega168 and the ATmega32u4 are skipped by `build-platformio.sh` for
  space.
- **Per-block feedrate `F` — not a separate item.** It is a requested
  speed, hence timed-world input; it belongs to
  `extras/doc/implemented/timed_trajectory.md`
  (no AFAP `F`-as-cap variant).
- **Inverse kinematics — not tracked.** Application concern, not a
  library feature. Keep the whitepaper §3.2 non-goal, but clarify that
  the affine motor-map is caller-side too (the caller transforms
  waypoints; FasNAxis stays in step space).
- **Running `pump()` from `manageSteppers()` and generalizing the ramp
  generator / naxes — implemented.** Design record moved to
  `extras/doc/engine_sources.md` (Path A single driver + stop hook).
- **AFAP vs timed — decided.** `FasNAxis` stays AFAP-only; the faithful
  timed trajectory is a separate implementation. Design record
  moved to `extras/doc/implemented/timed_trajectory.md` and
  `extras/doc/planner_modes.md`.
- **A Linear oracle that is faster by leaving the chord / cutting a
  corner / skipping a vertex — not tracked.** Not a backlog item: such a
  track is not constraint-faithful, so it is not a faster Linear
  reference at all (§12.4.1). The sanctioned mode that may leave the
  chord is Overshoot; a deliberate corner-blend geometry stays a §3.2
  later overlay (G1/G2), not an oracle.

## Under discussion

- **How a short path comes to rest.** `endPath()` today can freeze
  fewer steps than the current ramp, and the last steps then go out
  at speed. The coordinated fix is a decel tail after the last
  waypoint; handing each axis to the ramp generator leaves the chord.
  See [090_end_path_decel.md](090_end_path_decel.md).
