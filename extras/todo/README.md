# Open items (TODO)

Library-wide open items, one file per item. The list is not
FasNAxis-specific: the ramp-generator arrival (110) is single-axis,
and the rest is n-axis work.

The filename prefix is the priority, three digits wide
(`010_name.md`). Numbers step by 10, so a new item takes a free
number between two existing ones (`015_` sits between `010_` and
`020_`). Items that share a priority share the prefix.

Source of truth for the n-axis items: `extras/doc/n_axes_whitepaper.md`.
Their test-driven implementation plan (Steps 0–14) is complete; its
tests live in `extras/tests/pc_based/test_26.cpp`.

## Tracked entries (priority order)

Tokens = estimated LLM token cost to develop and test each item (input +
output, including iterative refinement).  Effort = human time for a
**professional embedded C++ developer** already familiar with the codebase.
A hobbyist should multiply the effort column by **2–3×** (new items by
**3–5×**) to account for ramp-up on platform-specific driver architecture,
timer/PWM/PIO registers, and the ramp generator's log2 fixed-point math.

| Priority | Item | Tokens | Effort | Why now |
|----------|------|--------|--------|---------|
| **010** | [MCPWM/PCNT defect — two queues on ESP32](010_mcpwm_pcnt_defect.md) | ~100 k | 1–2 d | **Critical.** Configuring two MCPWM/PCNT queues causes the second stepper to emit continuously and never stop. Runaway motion. |
| **020** | [stopMove() does not stop a queued move](020_stopMove_behavior.md) | ~100 k | 1–2 d | **Critical.** `stopMove()` only sets a flag; queued commands run to completion. Unexpected motion. |
| **030** | [Interrupt slow steps](030_interrupt_slow_steps.md) | ~500 k | 1–2 w | Bug: slow steps (e.g. 1 step/s) are not interruptible — `abort()` / `reset()` effectively non-functional. |
| **040** | [ESP32 synchronized start](040_esp32_synchronized_start.md) | ~20 k | 1–2 d | Native per-driver release (I2S group, RMT group start, MCPWM/PCNT) pending. |
| **040** | [Pico synchronized start](040_pico_synchronized_start.md) | ~20 k | 1–2 d | PIO block-start HW sync for multiple steppers to be verified. |
| **040** | [AVR synchronized start](040_avr_synchronized_start.md) | ~10 k | 0.5 d | Shared-timer start likely final; verify and close. |
| **040** | [SAM synchronized start](040_sam_synchronized_start.md) | ~20 k | 1–2 d | PWM/TC common release point to be identified. |
| **040** | [SAMD51 synchronized start](040_samd51_synchronized_start.md) | ~20 k | 1–2 d | TCC cross-instance release to be identified. |
| **040** | [Teensy synchronized start](040_teensy_synchronized_start.md) | ~20 k | 1–2 d | TMR within/cross-module release to be decided. |
| **050** | [Cross-driver start skew ~66% worse than same-driver](050_cross_driver_skew.md) | ~100 k | 0.5 d | Medium: critical characterization result previously hidden by a firmware bug. |
| **060** | [AVR RAM was 51% string literals](060_avr_ram_strings.md) | ~100 k | 0.5 d | Medium: resource constraint (1040 B of 2048 B `.rodata` in SRAM). Fixed, documented. |
| **070** | [i2s_direct has 2 channels on ESP32, not 3](070_i2s_direct_channels.md) | ~100 k | 0.5 d | Medium: constant overstates channel count by one. Graceful failure. |
| **080** | [Cubic start (`s_h`) overlay](080_cubic_start.md) | ~200 k | 1–2 w | Later feature, not v1. |
| **090** | [Common head speed](090_common_head_speed.md) | ~300 k | 1–2 w | Later planner. One acceleration and one max path speed for an x/y/z/… head. Waypoints are `dx, dy, dz, …, v`. |
| **100** | [Delta steps](100_delta_steps.md) | ~400 k | 1–2 w | AFAP input variation: `int16_t` chunks instead of absolute waypoints. |
| **110** | [Ramp time and moveTo eta](110_move_to_eta.md) | ~400 k | 1–2 w | Record ramp time next to performed ramp steps; `moveTo(position, eta_ticks)` caps speed so the move finishes by that tick. |
| **120** | [Smooth stop at end of path](120_end_path_decel.md) | ~200 k | 1 w | Open: append a decel tail on `endPath()`, or hand the stop to the ramp generator. |
| **130** | [Pipeline position estimate](130_position_pipeline_estimate.md) | ~100 k | 2–3 d | Low priority. Estimate how far a pipelined driver has played out, so `getCurrentPosition()` can lead the pin by less. A pulse counter remains the real position. |
| **140** | [RMT V1/V2 file split](140_rmt_v1_v2_split.md) | ~100 k | 0.5 d | Low hanging fruit. `SUPPORT_RMT_V1` / `SUPPORT_RMT_V2` instead of one flag for both and `V2` only for IDF5/6. |
| **140** | [Modular ramp generator](140_modular_ramp_generator.md) | ~2 M | 3–4 w | Major refactor: extract 4 modules, write PC tests, documentation, regression suite. |
| **150** | [GPIO set support (#316)](150_gpio_set_support.md) | ~300 k | 1 w | Audit toggle vs. set per platform, add `SUPPORT_GPIO_SET` flag, benchmark, test. |
| **160** | [16-bit GPIO encoding](160_16bit_gpio_encoding.md) | ~800 k | 2–3 w | Cross-cutting type change: `pin_t` in every API, queue struct, platform init; 8-bit retained for AVR. |
| **170** | [i2s_direct characterization — 23/25 pass, 2 skipped](170_i2s_direct_characterization.md) | ~100 k | 0.5 d | Low: documentation of a characterization result, not a defect. |
| **175** | [stopMove() / forceStop() / forceStopAndNewPosition() — three APIs, one harness conflated two](175_stop_api_conflation.md) | ~100 k | 0.5 d | Low: harness bug that was found, fixed, and documented. |
| **total** | 23 items (16 existing + 7 new) | ~6.5 M | 18–26 w | All priorities 010–175. |

## Done

- **R7 — Virtual I2S mux: 37-channel test harness — implemented.**
  `scripts/i2s_mux_decoder.py` turns an 8-channel capture into a 37-channel one
  (5 passthrough + 32 mux slots, the 3 bus wires consumed) and the existing
  evaluators read it with no mux-specific code. The firmware can now connect
  `i2s_mux` steppers (`PIN_I2S_FLAG` slots) and the bus is the last three
  analyzer channels, so D0..D4 keep their names on both sides of a decode.
  `--mode scale --driver i2s_mux --pin-mode nodir` sweeps 1…32 multiplexed
  steppers; **29 of 32 pass**. Three design assumptions were wrong and are
  corrected with measurements: a slot is high for one bclk period (125 ns), not
  for the frame; the bus needs ≥ 3 samples per bit period so 24 MS/s and not
  8 — while 48 MS/s truncates the capture below a scenario's duration; and the
  word goes out MSB first. Also found and fixed: an intermittent dropped step at
  20+ slots (the remaining 3), a `uint8_t` serial line length that corrupted any
  command over 255 characters, and a `sorted()` channel map that handed stepper
  27 another stepper's channel. See
  [180_r7_virtual_i2s_mux.md](../doc/implemented/180_r7_virtual_i2s_mux.md).

- **IDF 6 RMT sequence 02 slow — implemented.** F2 caps every RMT sub-entry
  (`rmt_encode_fill()`), so the RMT buffer spans less time than the ramp
  lookahead and the queue is no longer drained mid-move. HW (IDF 5.3.1, M1 RMT)
  `seq_03_02` dropped 123 s -> 94 s. See
  [idf6_rmt_slow.md](../doc/implemented/idf6_rmt_slow.md).
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
  See [120_end_path_decel.md](120_end_path_decel.md).
