# Open items (TODO)

Library-wide open items, one file per item. The list is not
FasNAxis-specific: the ramp-generator arrival (110) is single-axis,
and the rest is n-axis work.

The filename prefix is the priority, three digits wide
(`010_name.md`). Numbers step by 10, so a new item takes a free
number between two existing ones (`025_` would sit between `020_` and
`030_`). Items that share a priority share the prefix.

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
| **020** | [The queue admission latch has no lifecycle](020_queue_admission_latch.md) | ~80 k | 1–2 d | **Critical.** `ignore_commands`: 4 writers in 2 layers, no reader, no public clear, refusal returns `AQE_OK`. Low-level callers cannot queue again after a stop. |
| **025** | [Pico `forceStop()` discards an exact step count](025_pico_force_stop_loses_step_count.md) | ~30 k | 0.5 d | Medium: `pio_sm_clear_fifos` drops RX, then `pos_offset = 0`. Also a read-before-test in `getCurrentStepCount()`. Not started; an unverified attempt was dropped. Split out of [020](020_queue_admission_latch.md). |
| **030** | [Interrupt slow steps](030_interrupt_slow_steps.md) | ~500 k | 1–2 w | Bug: slow steps (e.g. 1 step/s) are not interruptible — `abort()` / `reset()` effectively non-functional. |
| **040** | [ESP32 synchronized start](040_esp32_synchronized_start.md) | ~20 k | 1–2 d | Native per-driver release (I2S group, RMT group start, MCPWM/PCNT) pending. |
| **040** | [Pico synchronized start](040_pico_synchronized_start.md) | ~20 k | 1–2 d | PIO block-start HW sync for multiple steppers to be verified. |
| **040** | [AVR synchronized start](040_avr_synchronized_start.md) | ~10 k | 0.5 d | Shared-timer start likely final; verify and close. |
| **040** | [SAM synchronized start](040_sam_synchronized_start.md) | ~20 k | 1–2 d | PWM/TC common release point to be identified. |
| **040** | [SAMD51 synchronized start](040_samd51_synchronized_start.md) | ~20 k | 1–2 d | TCC cross-instance release to be identified. |
| **040** | [Teensy synchronized start](040_teensy_synchronized_start.md) | ~20 k | 1–2 d | TMR within/cross-module release to be decided. |
| **050** | [Cross-driver start skew — I2S dominates](050_cross_driver_skew.md) | ~100 k | 0.5 d | Medium: characterization. Up to **109 step periods** with an I2S driver; the old "cross-driver is 66% worse" ratio is withdrawn — the same-driver and cross-driver ranges overlap. |
| **070** | [i2s_direct has 2 channels on ESP32, not 3](070_i2s_direct_channels.md) | ~100 k | 0.5 d | Medium: constant overstates channel count by one. Graceful failure. |
| **072** | [MAP does not report a multiplexed stepper's direction slot](072_map_does_not_report_mux_direction_slot.md) | ~150 k | 1 d | Medium: the host derives a mux stepper's dir slot as `step_slot + 1`; correct only while allocation stays gapless. |
| **076** | [i2s_mux in `dir`: the second stepper's slot is intermittently not decoded](076_i2s_mux_dir_second_slot_not_decoded.md) | ~200 k | 1–2 d | Medium: `CONFIG 2 i2s_mux,i2s_mux dir` loses S2 on both I2S SDKs; `nodir` is unaffected and green. |
| **080** | [Cubic start (`s_h`) overlay](080_cubic_start.md) | ~200 k | 1–2 w | Later feature, not v1. |
| **090** | [Common head speed](090_common_head_speed.md) | ~300 k | 1–2 w | Later planner. One acceleration and one max path speed for an x/y/z/… head. Waypoints are `dx, dy, dz, …, v`. |
| **100** | [Delta steps](100_delta_steps.md) | ~400 k | 1–2 w | AFAP input variation: `int16_t` chunks instead of absolute waypoints. |
| **110** | [Ramp time and moveTo eta](110_move_to_eta.md) | ~400 k | 1–2 w | Record ramp time next to performed ramp steps; `moveTo(position, eta_ticks)` caps speed so the move finishes by that tick. |
| **120** | [Smooth stop at end of path](120_end_path_decel.md) | ~200 k | 1 w | Open: append a decel tail on `endPath()`, or hand the stop to the ramp generator. |
| **130** | [Pipeline position estimate](130_position_pipeline_estimate.md) | ~100 k | 2–3 d | Low priority. Estimate how far a pipelined driver has played out, so `getCurrentPosition()` can lead the pin by less. A pulse counter remains the real position. |
| **140** | [Modular ramp generator](140_modular_ramp_generator.md) | ~2 M | 3–4 w | Major refactor: extract 4 modules, write PC tests, documentation, regression suite. |
| **150** | [GPIO set support (#316)](150_gpio_set_support.md) | ~300 k | 1 w | Audit toggle vs. set per platform, add `SUPPORT_GPIO_SET` flag, benchmark, test. |
| **160** | [16-bit GPIO encoding](160_16bit_gpio_encoding.md) | ~800 k | 2–3 w | Cross-cutting type change: `pin_t` in every API, queue struct, platform init; 8-bit retained for AVR. |
| **170** | [i2s_direct characterization — 23/25 pass, 2 skipped](170_i2s_direct_characterization.md) | ~100 k | 0.5 d | Low: documentation of a characterization result, not a defect. |
| **181** | [mcpwm_pcnt emits more steps than were commanded, in `sync`](181_mcpwm_pcnt_sync_extra_steps.md) | ~100 k | 1–2 d | Medium: 67 steps where 64 were commanded, IDF 5.5.3 only, ~1 in 3, period exact. Found while closing 015/016. |
| **182** | [`i2s_mux` mangles any command from n ≥ 16](182_i2s_mux_command_mangled_from_16_steppers.md) | ~150 k | 1–2 d | High: the mux's 32-stepper claim cannot be tested — the host's own parser refuses `CONFIG 16 …`. Pre-existing. |
| **total** | 25 items | ~6.5 M | 18–26 w | Priorities 020–182. |

## Done

- **`stopMove()` does not stop a queued move — closed as a defect, by
  design.** It sets a flag the ramp generator reads for its *next*
  command; nothing already queued is touched, so the truncation point is
  the prefill depth and is *not stable across runs*. Now asserted rather
  than discovered: `SR_25` and `SR_30` send an identical program and
  assert opposite outcomes, both passing on all six matrix rows. Three
  claims this item once made were withdrawn — `forceStop()` does not
  substitute for it, the behaviour is not documented in the library's
  public API, and the emergency stop a caller wants is
  `forceStopAndNewPosition()`, which works. Re-evaluating it is what
  surfaced [020](020_queue_admission_latch.md), the real defect behind
  it. Record:
  `extras/doc/implemented/saleae_based_test_harness.md` (§ *stopMove
  does not truncate queued motion*).

- **Platform-release matrix, first hardware run — 6 firmwares, 350
  measurements, and six harness bugs that had been hiding results.**
  `extras/tests/saleae_based/scripts/run_matrix.py` flashes each
  `RELEASE_MATRIX` row once (arduino 4.4.0/5.3.0/6.13.0, idf
  5.3.0/6.13.0/7.1.2) and measures the SR catalogue, one `scale` sweep per
  driver the board admits, and all ten `sync` combinations against that one
  flash. First execution found the two open library items above, plus:

  - **`read_map()` read `slots=` as one entry per channel; the firmware sends
    one per stepper.** Agrees with itself in `nodir`, runs off the end of the
    list in `dir`, and handed every multiplexed stepper past the first a GPIO
    channel — so `sync --imux` reported 0 of 64 steps for a stepper the
    capture shows stepping 64 times. Verified against the board
    (`CONFIG 2 i2s_mux,i2s_mux dir` → `slots=0,2`); the two unit tests that
    encoded the wrong rule were corrected to the measured reply.
    Tracked as [072](072_map_does_not_report_mux_direction_slot.md) for the
    direction-slot half of the same gap.
  - **The I2S mux's 24 MS/s floor keyed off `--driver`, and `sync` has none** —
    so `--imux` sampled the 8 MHz bus at 4 MS/s (0.5 samples/bit) and the
    decode was not a wrong answer but not an answer. The floor now keys off
    `--imux`, i.e. off whether the capture carries the bus.
  - **The sample-rate log line mixed two numbers**: the ratio came from the
    resolved rate, the printed rate from `args.sample_rate`, which is `0` for
    "auto". It always read "0 MS/s would have read 0.5" when the truth was
    4 MS/s.
  - **A refused `IMUX` was fatal**, but the USB reset does not always fire; when
    it does not, the mux from the previous point is still up and
    `initI2sMux()` is correctly refused. Now resolved by asking the firmware
    (`mux_init=` in `DRIVERS`).
  - **One point's exception aborted the rest of a sweep**, so a transient at
    n=3 cost n=4…8.
  - **A narrowed `--targets` discarded the other rows**, so recovering one row
    whose upload hit "wrong boot mode" meant re-flashing all six. Rows now
    merge, and a report built across invocations carries a per-row timestamp.

  And in the report generator: the scale and sync tables printed **empty**
  (mode points key as `{run_tag}_{point}`; the report compared for equality),
  and no capture link ever rendered (`capture_link` appended `.vcd` to a `.sr`
  path, so it looked for `foo.sr.vcd`).

- **140 — RMT V1/V2 split — resolved, the two RMT paths had one flag and
  the umbrella was doing the work the version test should do.**
  `SUPPORT_ESP32_RMT` is replaced by `SUPPORT_ESP32_RMT_V1` (IDF4 half
  filler) and `SUPPORT_ESP32_RMT_V2` (IDF5/6 encoder translator), the six
  sources are renamed to say which path they are, and no RMT source or RMT
  branch tests `ESP_IDF_VERSION` any more. Verified by compiling the AVR,
  Arduino (IDF 4.4.7), IDF 4.4.3 and IDF 5.5.3 builds and by the four
  PC tests that include those sources directly.
- **015 + 016 — IDF 5.5.3 panicked on RMT and `i2s_direct` was unstable —
  resolved, and both were the harness overflowing the main task's stack.** One
  defect, not two: `CONFIG_ESP_MAIN_TASK_STACK_SIZE` is 3584 B and every `CONFIG`
  constructs its drivers on that task, so `stepperConnectToPin()` ran out of
  budget before it started. Measured with `uxTaskGetStackHighWaterMark()` on the
  board: **216 B** left on 5.5.3, **312 B** on 6.1 — both over budget, 6.1
  merely shallow enough to escape. The overflow runs off the *top* of the stack,
  which on ESP32 is where the DRAM tlsf pool begins, so it presented as a
  corrupted free list (`LoadProhibited` in `block_locate_free`) rather than as a
  stack fault. Confirmed by `CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192` changing
  nothing in `src/` and making `CONFIG 1 rmt dir` answer `OK`.

  The stack was spent on **reply buffers held as stack locals** (~1900 B — a
  local array holds its slot for the whole function, so a 1152 B reply buffer was
  resident while `rmt_new_tx_channel()` allocated) and on **libc `sscanf` (1496 B)
  and `snprintf` (384 B)** for a protocol that only formats %s/%u/%d. Worth
  recording that the libc fix alone bought nothing here: the buffers dominate.
  Peak 4272 B → **2336 B** of the 3584 B default, so the cliff is gone rather
  than moved and no Kconfig change was needed. `SAL_REPLY_BUF` decides static
  off AVR and empty on AVR in one place, because on AVR the stack *is* SRAM and
  `static` would commit ~640 B of a 2048 B part for the program's life.

  Closed by `run_matrix.py --targets idf-6.13.0 --force`: catalogue **26/26**,
  and in the sync table all four RMT combinations plus both `i2s_direct` pairs
  went from panic/refused to measured passes. Two findings worth keeping:

  - **The four RMT combinations used to be reported as `refused (bound)` with
    the Guru Meditation text quoted as the reason** — a panic and a board
    limit are the same refusal to the classifier, which is why this read as a
    driver-capability question for as long as it did. Same gap as an all-skip
    run still reporting `failed=0`.
  - **`SR_00` ran twice on every catalogue run**, because
    `if any(t != "SR_00" for t in tests)` is true for the default `--tests`, which
    already starts with SR_00. The second run FAILED on one rerun, which skipped
    all 26 scenarios while the run still exited 0 — so the matrix printed the
    row `ok` having measured nothing.

  Two measurements that did **not** become fixes, kept because the obvious fix is
  wrong: blocking the idle loop for one tick does not fix the IDF 4.4.3 idle
  watchdog (it fires with no command outstanding — `QINFO` alone reproduces
  it, so it is IDF 4.4's own), and it grew trailing pulses on mcpwm_pcnt, so it
  was reverted. And `MIN_CMD_TICKS` is a *duration* floor (`ticks × steps`),
  not a floor on `ticks` — `maxspeed*` in `QINFO` is the fastest *period*, usable
  only on a command long enough to clear it, which is what made the documented
  `QSEG` example build commands the firmware refuses.

  Full analysis, measurements and guards:
  [idf55_main_task_stack_overflow.md](../doc/implemented/idf55_main_task_stack_overflow.md).
  Two new items came out of the acceptance run and are **not** covered by it:
  [181](181_mcpwm_pcnt_sync_extra_steps.md) and
  [182](182_i2s_mux_command_mangled_from_16_steppers.md).

- **010 — MCPWM/PCNT emitted continuously on every queue after the first —
  resolved, `pcnt_new_unit()` was clearing the interrupt-enable bit.**
  Measured `PCNT.int_ena == 0x39` with three queues connected (bits 1 and 2
  cleared) and `0x3F` after the fix; `--mode scale --driver mcpwm_pcnt` now
  passes n = 1…6 in `nodir` and n = 1…4 in `dir`, with n = 7/8 correctly
  refused. The original lead — the `channel_num` / `pcnt_unit_id` indexing —
  was *not* the cause but is a real latent defect of its own, so both
  invariants are now written up in
  [`extras/doc/platforms/esp32.md`](../doc/platforms/esp32.md).

- **180 — i2s_mux short move "no output" — resolved, decoder fixed.**
  The "64-step move puts nothing on the wire" reading was a decoder bug plus a
  capture/QRUN sync misread: the decoder framed the word at the ws *rising*
  edge and read it whole-word MSB-first, while the hardware sends two 16-bit
  halves, low half (slots 0-15) first, each half MSB-first, starting at the ws
  *fall* — so every word decoded half-swapped and one step looked like two
  phase groups. `extract_frames()` is now a bclk-edge shift register anchored
  at the first ws fall with the halves swapped back, and the test fixture
  renders the measured wire order. The short move decodes 64 x `0x000FFFFF`
  and the long control 327 675 words; the descriptor-seeding hypothesis was
  refuted by measurement. The remaining real defect — an intermittent
  single-step drop, reproduced once at n=28, slot 16, with the corrected
  decoder — is recorded in the saleae `AGENTS.md` known finding and in
  [r7_virtual_i2s_mux.md](../doc/implemented/r7_virtual_i2s_mux.md)
  §5.

- **R7 — Virtual I2S mux: 37-channel test harness — implemented.**
  `scripts/i2s_mux_decoder.py` turns an 8-channel capture into a 37-channel one
  (5 passthrough + 32 mux slots, the 3 bus wires consumed) and the existing
  evaluators read it with no mux-specific code. The firmware can now connect
  `i2s_mux` steppers (`PIN_I2S_FLAG` slots) and the bus is the last three
  analyzer channels, so D0..D4 keep their names on both sides of a decode.
  `--mode scale --driver i2s_mux --pin-mode nodir` sweeps 1…32 multiplexed
  steppers; with the corrected decoder **31 of 32 pass**, the one failure an
  intermittent dropped step (see the 180 entry above). Three design assumptions
  were wrong and are corrected with measurements: a slot is high for one bclk
  period (125 ns), not for the frame; the bus needs ≥ 3 samples per bit period
  so 24 MS/s and not 8 — while 48 MS/s truncates the capture below a scenario's
  duration; and the word goes out as two 16-bit halves, low half first, not
  MSB-first. Also found and fixed: a `uint8_t` serial line length that
  corrupted any command over 255 characters, and a `sorted()` channel map that
  handed stepper 27 another stepper's channel. See
  [r7_virtual_i2s_mux.md](../doc/implemented/r7_virtual_i2s_mux.md).

- **IDF 6 RMT sequence 02 slow — implemented.** F2 caps every RMT sub-entry
  (`rmt_encode_fill()`), so the RMT buffer spans less time than the ramp
  lookahead and the queue is no longer drained mid-move. HW (IDF 5.3.1, M1 RMT)
  `seq_03_02` dropped 123 s -> 94 s. See
  [idf6_rmt_slow.md](../doc/implemented/idf6_rmt_slow.md).
- **ESP32 RMT extra step — implemented.** IDF5/6 translates queue
  commands in `StepperISR_rmt_v2_encode.cpp` instead of filling
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
