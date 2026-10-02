# 120 Saleae-based test harness

## Goal

Build a hardware-in-the-loop harness that uses a **Saleae Logic Analyzer** (or
any sigrok-compatible analyzer) to **characterize `addQueueEntry()` at the pin
level**, on every supported architecture.

All eight channels are used, but **how** depends on the configuration under
test: two per stepper with a direction pin (4 steppers), or one per stepper in
step-only mode (8 steppers). See *Channel assignment*.

Results are keyed by **architecture, SDK version and driver**, and every result
names its driver explicitly. There is no automatic driver selection: a result
that does not say which driver produced it characterizes nothing.

## Scope

### The one thing under test

`FastAccelStepper::addQueueEntry()`, the ring queue behind it, and the pulse
driver. The question is always: *given these exact `stepper_command_s` values,
what step/dir waveform comes out of the pin?*

### Explicitly **not** in scope

| Not here | Why | Lives in |
|----------|-----|----------|
| Ramp generator (`move()`, `setAcceleration()`) | Pure integer math. The analyzer only measures the arithmetic back. | `pc_based` test_02/05/09/10 |
| `moveTimed()` | Nothing but an `addQueueEntry()` loop. | `pc_based` test_20/24/25 |
| n-axis planner (`FasNAxis`) | Planner logic; its pin output is only interesting once the per-stepper queue is characterized. | `pc_based` test_23–26 |

A test that only drives `move()` or `moveTimed()` and counts pulses belongs in
`pc_based`. The only way to earn a Saleae test is to show that the pin waveform
carries information the arithmetic does not already determine.

### Characterization targets

Two **generic** test modes cover the matrix; neither names an architecture,
because the architecture is a tag on a run, not a mode.

| Mode | Question | Varies over |
|------|----------|--------------|
| **`scale`** | How does one driver behave from 1 up to its maximum steppers in parallel? | driver, stepper count 1…driver-max |
| **`sync`** | On architectures with more than one driver, do they start together, and does each keep its own speed? | every driver-list combination |

`scale` is the one run an AVR board needs. `sync` is for architectures that
have several drivers — ESP32 today, but the mode itself is not ESP32-specific,
and a driver that is not connected yet (`i2s_mux`) must be runnable by naming
it, with no new code path. Details and the work breakdown are in *The redesign
now in progress*.

Underneath both modes sit the original targets:

1. **Step pulse high time / duty vs speed** — how wide is the pulse the driver
   emits, and how does it scale with the commanded `ticks`?
2. **Step timing vs speed and vs stepper count** — is the inter-step period
   exactly `ticks`, at 1…255 steps per command, at `ticks` = 1 and 65535? Does a
   second stepper perturb the first one's timing?
3. **Dir change → first step** — how long after the dir edge does the first
   step of the reversed phase appear?
4. **Driver edge behaviour** — MCPWM/PCNT counter-limit overrun, pause commands,
   synchronized start.

### Channel assignment

The analyzer has **8 channels** and each stepper needs a step pin, so the
channel budget decides how many steppers can run:

| Mode | Mapping | Max steppers | Why |
|------|---------|--------------|-----|
| `dir` (default) | stepper *i* → Step `D(2i)`, Dir `D(2i+1)` | **4** | two channels per stepper |
| `nodir` (step-only) | stepper *i* → Step `D(i)` | **8** | one channel per stepper |

```
dir:    CH0 StepA  CH1 DirA  CH2 StepB  CH3 DirB
                 CH4 StepC  CH5 DirC  CH6 StepD  CH7 DirD

nodir:  CH0 StepA  CH1 StepB  CH2 StepC  CH3 StepD
        CH4 StepE  CH5 StepF  CH6 StepG  CH7 StepH
```

Step-only mode doubles the parallel count, which matters because the driver
queue limits (see *Architecture notes*) reach 6–8 while the `dir` mapping caps
at 4. The firmware reports the mode and stride via `MAP`, so the host and the
firmware cannot disagree about which channel carries which stepper.

## Rules that apply to everything below

- **`ticks` is a raw 16-bit queue period, not microseconds.** That is what
  makes the interesting boundaries addressable (`ticks` = 1 and 65535,
  `steps` = 1…255). The host reads `TICKS_PER_S` from the firmware with `QINFO`
  and converts. **Never hardcode 16 MHz** — it differs per platform (Teensy
  prescales), and the AVR speed floor additionally depends on the number of
  connected steppers.
- **A spurious or swallowed pulse is a defect, not a statistic.** There is no
  tolerance and no "glitch count" to trade off: it is a test failure. This is
  why the analyzer's metrics are `missing_steps` / `extra_steps` /
  `short_periods` / `long_periods`, each with an explicit threshold.
- **The firmware owns the command program.** 115200 baud needs ~0.3 s for a few
  hundred entries, and the stepper would move long before the last one arrives,
  so streaming a plan is not viable; a large static plan does not fit in AVR RAM
  either. The host sends one short line per segment (`QSEG`), the firmware keeps
  ≤ 8 segments (48 B, shared) plus an 8 B cursor per stepper.
- **Every scenario is a handful of commands.** No per-test firmware.

## Implementation plan

Position: **steps 1–4 done, step 5 in progress.** The analyzer has never yet
been run against a known-bad waveform, which is the only way to know it can
fail.

1. **Saleae bridge** — `scripts/capture.py`: sigrok-cli wrapper, device
   auto-detect, configurable rate/time, output + duration verification. Records
   `.sr` (compact, one packed byte per sample) and derives a change-only **VCD**
   for evaluation (`--vcd`); sigrok picks `$timescale` from the capture rate.
   **Done.**
2. **Signal parser** — `scripts/signal_parser.py`. **Done.**
   - [x] edges, pulse widths, inter-step period, duty, step count,
         dir→step delay, cross-channel skew
   - [x] `load_sr` (dependency-free srzip), `load_vcd`, `load_capture`;
         `load_vcd` takes the rate from sigrok's `$comment`
   - [x] defect checks: `period_defects`, `step_count_defects`,
         `rate_adherence` (sag / jitter / per-step deviation -- catches the ISR
         overhead that a step count and a gross period check both miss)
   - [x] `analyze_csv.py` SR_00 expectations
3. **addQueueEntry() feeder** — `common/saleae_app.cpp`. **Done.**
   - [x] `QSEG` / `QRUN` / `QINFO` / `QCLR`, DIR-pause retry
   - [x] prefill to half depth, `synchronizedStart()` kick-off
   - [x] `1ch` / `2ch` / `4ch_rmt` / `4ch_mcpwm` / `mixed` configs
   - [x] AVR step pins via `stepPinStepperA/B`; `SALEAE_MAX_STEPPERS` from
         `MAX_STEPPER`
4. **Golden VCD fixtures** — `scripts/tests/vcd_fixtures.py` +
   `make_fixtures.py`. **22 fixtures committed.** **Done.**
4b. **Global pin invariants** — `run_tests.check_pin_invariants()`, applied by
   `evaluate()` to every capture. **Done.**
   - [x] DIR must never change while STEP is high (a driver latches direction on
         the STEP edge). Checked on every stepper, on every test, not only in the
         direction-change scenario
   - [x] a DIR change on the rise or fall sample is a boundary, not a violation
   - [x] `bad_dir_during_step_high` fixture is clean on step count, period and
         rate adherence, and fails only on the invariant — so a per-scenario
         verdict could not catch it

5. **Analyzer negative testing** — `scripts/tests/test_analyzer_fixtures.py`.
   **Done.** The evaluators are now proven able to fail.
   - [x] the real `run_tests.py` evaluators run over the real VCDs, parsed by
         the real `load_vcd` -- nothing stubbed
   - [x] good fixtures must pass; bad fixtures must be rejected with the named
         defect present in the result
   - [x] anti-rot: every rule has ≥ 1 failing fixture, every fixture is
         reachable from a scenario, and each fixture still matches the segment
         list its scenario actually sends
   - [x] mutation-checked: forcing each defect check to always pass turns the
         suite red, one rule at a time
6. **Scenario wiring** — `scripts/run_tests.py`: scenario table + evaluators.
   - [x] scenario table for SR_01–SR_14, `QINFO` plumbing
   - [x] every implemented evaluator proven against fixtures (SR_01–SR_10,
         SR_14, SR_27)
   - [x] every wired scenario covered, by a committed fixture or a generated
         test — SR_03, SR_06, SR_07 and SR_08 were wired with no coverage at all
   - [x] no two scenarios of the same config send an identical segment list;
         this is what found SR_03 sending 640 ticks, the same as SR_01, so
         "at the speed floor" never tested the floor (`sc_ticks_min` used
         `max_speed_ticks`, the *fastest* legal speed, not `min_cmd_ticks`)
   - [x] per-period checks apply only *within* a command. The gap between two
         commands is the queue's trailing wait and is two periods wide by
         design, so feeding it to `period_defects` made every multi-command
         scenario fail on a correct waveform — SR_06 failed its own good
         fixture until this was fixed (`intra_command_periods()`)
   - [x] SR_07 (2000 steps) and SR_08 (4000 steps) generated at full size
         rather than committed: a 4000-step VCD is ~300 KB for one assertion.
         Covers step count, that every intra-command period is examined rather
         than a bounded window, and a step dropped at the *end* of the run
   - [x] SR_27 `single_step` — `steps == 1` takes the other ISR branch and is
         the only command with no inter-step period to measure
   - [x] **on hardware** — ESP32 + Saleae clone (fx2lafw) at 24 MS/s. SR_01,
         SR_03, SR_06, SR_09 and SR_27 all accepted by their real evaluators.
         SR_09's pause measures 839 us, matching (640 + 12800) ticks.
   - [x] every wired scenario run on hardware — **12/12 accepted** by their own
         evaluators: SR_01 (8 @ 640), SR_02 (255, uint8_t max), SR_03 (speed
         floor), SR_04 (ticks=65535, 4092 us periods), SR_05, SR_06 (the
         inter-command gap), SR_07 (2000 steps), SR_08 (4000 steps), SR_09
         (pause, 839 us), SR_10 (direction change), SR_14 (2 steppers,
         2000 each, aligned), SR_27 (single step). No DIR change during STEP
         high in any capture.
   - [x] two measurement traps found while doing this, both worth remembering:
         **`QRUN` takes a channel mask** — sending `QRUN 1` for a 2ch scenario
         silently excludes stepper B, so its pin stays idle while its queue
         still reports the full step count. And **the fx2lafw loses a channel
         on a sparse selection**: `-C D0,D2` reports nothing on D2, while
         `-C D0,D1,D2` is correct. Always capture a contiguous channel run.
   - [x] triggering on the first STEP edge costs exactly one step from the
         count (that edge becomes sample 0). SR_02 reads 254/255 triggered and
         255/255 untriggered. Use an untriggered window sized to contain the
         run when the step count is what is being checked.
   - [x] **measured, not assumed: the inter-command gap is ONE period.** Two
         2-step commands at ticks=1600 measured 99.917 us across the boundary
         against 99.917 us within a command — ratio 1.000. There is no trailing
         wait. `render()` used to append one extra `ticks` per command, which
         described a 2x gap the hardware never produces; that wrong fixture is
         what made SR_06 "fail", and the evaluator change made to accommodate
         it (`intra_command_periods`) was also wrong and is reverted.
   - [x] `MIN_CMD_TICKS` bounds the **whole command**, not the period:
         `ticks * steps` for steps > 1, so 2 steps at 640 ticks is refused and
         2 at 1600 accepted. `legal_ticks()` clamps every scenario builder, so
         no fixture describes a command the firmware rejects.
   - [x] SR_00 no longer auto-starts at boot. It toggles eight pins at 1 Hz, so
         it filled every pre-CONFIG capture with square waves over the pulses
         under test. The host starts it with `SR00` when it wants the
         identification pre-check.
   - [x] **SR_11 / SR_12 / SR_13 implemented and run on hardware.** SR_11
         (reverse then forward) measured 40/40 steps with per-phase counts
         [20, 20]; SR_12 (forward, reverse, forward) 30/30 with [10, 10, 10].
         SR_13 is the suite's only negative test on the firmware: 8 steps at
         399 ticks is 3192 ticks of motion against a floor of 3200, so
         addQueueEntry() refuses it, and the measurement confirms
         **zero pulses emitted and POS 0**. A rejection that still stepped
         would be a real defect; it does not happen.
   - [x] `eval_direction_phases` asserts each phase contributes its own step
         count and that the dir pin ends at the commanded level. Two things had
         to be got right, both found by the fixtures: split at *every* dir
         change, not just a rise (reverse-then-forward has one rise and two
         phases), and drop regions containing no steps -- on hardware the dir
         pin settles to its starting level before the first step, which adds a
         leading empty region that is not a defect.
   - [x] the white paper's SR_13 error text (`ERR QE ticks … < maxspeed …`)
         does not match the firmware, which emits `ERR QE step0 rc=-1`
         (ErrorTicksTooLow). Worth correcting in the paper.
   - [x] SR_14 / SR_16 / SR_17 all implemented and run on hardware; see the
         entries further down. (These two boxes were stale: the work landed in
         `d564b204` but the checklist was never ticked.)
   - [x] **SR_18–SR_20 MCPWM/PCNT overrun — no defect found.** Wired the three
         scenarios plus `eval_counts_and_gap`, and ran them on the ESP32 on the
         MCPWM/PCNT driver. SR_18 measured 256/256 steps with the phase
         structure the test asserts: 255 steps at 39.96 us, then one 439.67 us
         gap (expected 440), then exactly one step. The PCNT high-limit re-arm
         is correct at the full 255 boundary. The band SR_19 cares about was
         then swept on hardware at n = 200, 240, 250, 251, 252, 253, 254, 255,
         each followed by a single step: every one produced exactly n+1 steps,
         no lost or duplicated pulse anywhere. SR_20 (255, pause, 255) measured
         510/510.
   - [x] a spec bug found while writing SR_18: the white paper specifies
         `QSEG 1 <max> 1`, but a `steps == 1` command is bounded by its ticks
         alone, so `<max>` = 640 is below `MIN_CMD_TICKS` = 3200 and the
         firmware refuses it with ErrorTicksTooLow. The trailing single step has
         to use at least 3200 ticks (200 us), so the scenario's last phase runs
         at a different period from the run before it.
   - [x] **SR_21 / SR_23 pass on hardware.** SR_21 (200 RMT steps, hunting an
         irregular gap at a hardware buffer split): 200/200, zero gaps outside
         tolerance. SR_23 (I2S, whose timing comes from a DMA-fed sample stream
         rather than a compare register): 64/64 at 39.96–40.0 us.
   - [x] **SR_15 implemented by extending the protocol, not by hardcoding it
         into the firmware.** It needs two steppers at *different* periods, and
         `QSEG <steps> <ticks> <dir>` appended to one shared `program[]` that
         `qe_feed` walked for every slot -- so both steppers necessarily got
         identical steps, ticks and direction. A firmware-resident test sequence
         would have fixed SR_15's parameters in silicon and lost the ability to
         sweep them, so the protocol grew instead:
         `QSEG <idx> <steps> <ticks> <dir>` targets one stepper's own program.
         The two forms differ in argument count, so no existing command can be
         misread as the other, and a stepper with its own program ignores the
         shared one. **Measured: A at 640 ticks = 39.9661 us, B at 1280 ticks =
         79.9326 us, 200/200 each, first-step skew 27.0417 us** -- the arm
         stayed aligned while each stepper kept its own speed, which is the
         whole point. The skew is reported, not gated, as with SR_14 and SR_17.
         **Cost: +110 bytes of RAM on AVR** (1481 -> 1591 of 2048, 77.7% used)
         for the per-stepper segment matrix. The only cost that mattered, and it
         fits. **All 25 scenarios re-run after the change: 25/25 pass**, so the
         shared-program path is unregressed.
   - [x] **SR_22 not applicable here.** RMT V2 fill-encoder output needs a chip
         with RMT V2; this ESP32 has V1. Nothing to measure, so it is left
         unwired rather than wired to a config that cannot exist.
   - [x] **SR_24 not applicable here.** AVR timer OC pins, and no AVR board is
         connected. Not inferable from ESP32 results.
   - [x] **SR_26 passes: the 16-bit pause field works.** 65535 ticks of silence
         between two single steps, measured gap 8185 us against 8191.875 us
         expected. Deviates from the white paper by adding a step *after* the
         pause: silence can only be measured between two pulses, so with one step
         before and none after the gap is unobservable rather than wrong.
         **Corrected a units error of my own along the way** -- 65535 ticks at
         16 MHz is 4.0965 *ms*, not seconds.
   - [x] **SR_25 found a real library behaviour: `stopMove()` does not stop a
         move that is already queued.** `stopMove()` only sets a flag that the
         ramp generator reads when asked for its *next* command, so a move
         sitting in the pulse queue runs to completion. Measured:
         - 2000 steps @ 4000 ticks, `STOP` at POS 510 -> finished at **2000**,
           no effect at all.
         - 20000 steps @ 640 ticks, `STOP` at POS 5825 -> stopped at **14240**,
           truncating only once the queue had to refill.
         The scenario uses a move far larger than the queue, so it tests
         stopping rather than queue drain. **Verified on a waveform:** stopped at
         11475 of 20000, no partial pulse, and the pin held still for the
         remaining **2.823 s** of the capture. The stop is clean once it lands;
         the surprise is only how long it takes to land.
7. **Parameter sweeps** — **done.** `scripts/sweep.py` plans and runs them:
   - **SR_02 over `steps` = 1…255, 19 points, all pass.** Exact step counts
     everywhere, and `legal_ticks` pulls the small-step points up to the floor
     so each runs as fast as it legally can: steps=1 at 3200 ticks (200 us),
     steps=2 at 1600 (100 us), steps=3 at 1067 (66.69 us), steps=4 at 800
     (50 us), steps>=8 at 640 (40 us). **No lost or duplicated step anywhere in
     the range, including at the 16-bit and uint8_t boundaries.**
   - **SR_05 over `ticks` = 3200…65535, 6 points, all pass.** The useful result
     is the one that does *not* vary: **pulse high time is a constant 15.625 us
     (250 ticks) at every period from 200 us to 4096 us.** The driver emits a
     fixed-width pulse and varies the gap, so high time is a property of the
     driver alone and can be checked once instead of per speed.
     (The duty column reads 100% at these points because each uses a single
     step, so there is no low interval to divide by -- an artifact of the sweep
     shape, not a defect.)
   Every point is asserted legal before any hardware runs, so no run was spent
   measuring `ErrorTicksTooLow`.
8. **Reporting** — done, and now actually to the white paper's section 8. The
   box was ticked earlier against a console table, which did not meet the spec;
   that was premature.
   - `scripts/generate_report.py` builds the artefacts §8.1 lists: `index.md`
     (dashboard, pass rate per tag, full results table), one `test_SR_XX.md` per
     test, `spec_compliance.md`, `regression.md` against a baseline,
     `tag_summary/` per configuration, and `all_results.csv`. 28 tests.
   - `run_hardware.py --results DIR` writes one JSON per scenario, and the
     generator only formats: it never re-parses a capture, so it cannot disagree
     with the run that measured it. A 24 MS/s capture is ~96M samples of
     pure-Python waveform, and a report that re-measured would be free to
     disagree.
   - **A committed baseline exists**: `reports/esp32/` holds the result JSONs
     and the generated report for the full 25/25 run, so `index.md` is
     reviewable in the diff and `--baseline reports/esp32/results` is a
     working regression comparison.
   - **Timings are reported as distributions, not averages.** min/max/spread and
     a median for every period and pulse width, because a mean hides the finding:
     SR_05's constant 15.625 us pulse width shows as min == max, and one short
     pulse in ten thousand moves only the minimum.
   - **No glitch counter.** The paper's CSV names a `glitch_count` column and it
     was tempting to fill the schema in, but a glitch count needs a threshold
     invented for it and collapses a distribution into one number. The statistics
     are worth more and have a source. Pulse width is recorded, not judged: the
     driver sets it, so its value is a property of the silicon and becomes the
   baseline a regression is measured against.
   - *Not done, and not claimed:* `design_specs.json` (section 7) is not emitted;
     `spec_compliance.md` derives its expectations from the QINFO values the DUT
     reports instead. That covers the commanded period and the step count, which
     is what the library promises, but the paper's per-driver spec table is not
     reproduced. Cross-configuration comparison is implemented and exercised, but
     with one architecture measured it has nothing to compare yet.

## Analyzer negative testing — why the fixtures exist

**The tests are written by LLM agents, so a garbage test suite can look green.**
Concretely: if an analyzer assertion is vacuous, if a fixture is never loaded,
or if the evaluator simply returns "pass" for everything, the suite stays green
while measuring nothing. There is no reviewer in the loop to notice.

The defence is **negative testing**: every rule the analyzer enforces must have
at least one fixture that *must fail*, so that a broken analyzer makes the suite
red. `scripts/tests/vcd_fixtures.py` holds one spec per golden waveform — the
change list, the DUT limits from `QINFO`, and the verdict the analyzer must
reach — and `scripts/tests/test_analyzer_fixtures.py` asserts **both**
directions:

- a good waveform must pass,
- a corrupted waveform must be rejected, and the named defect must appear in
  the result (`extra_steps`, `n_short`, `n_long`, `pause_found`, …),
- every fixture in the manifest must be exercised by a test, and every rule
  must have at least one failing fixture.

That last check is the anti-rot one: it fails if someone adds an evaluator
rule without a negative case, or adds a fixture no test loads.

The fixtures are change-only VCDs in exactly the format `sigrok-cli` emits, at
a realistic 4 MS/s capture rate, so they exercise the same parser that runs on
real captures. Regenerate with:

```bash
python3 scripts/tests/make_fixtures.py
```

Current fixtures — good: `good_period_8_steps`, `good_steps_255`,
`good_ticks_max`, `good_rate_adherence`, `good_pause`, `good_dir_change`;
bad: `bad_merged_pulse`, `bad_dropped_step`, `bad_extra_pulse`,
`bad_rate_sag`, `bad_short_pause`, `bad_dir_change`, `bad_single_step`,
`bad_sync_missing_step`, `bad_dir_during_step_high`; plus `skew_three_periods`, which must **pass** while
reporting its known skew — the measured-but-not-gated category.

Good fixtures are *rendered from* the live `sc_*()` scenario builders rather
than recorded, so a scenario change makes the fixtures stale and the suite
fails. Only the injected faults are hand-written.

Two evaluator bugs this surfaced, both of which had been silently passing:

- `eval_pause` searched pulse *high* widths for a pause. A pause is a stretch
  of *silence*, so it appears as one long inter-step period, not a wide pulse.
- `eval_sync_start` measured first-step skew and then ignored it: the pass
  verdict depended on step counts alone, so a `synchronizedStart()` that lined
  nothing up passed. It now requires the first steps to land within one
  commanded period of each other.

`eval_sync_start` was the opposite mistake: it enforced that the steppers start
together. The general rule is now white paper §1.3, "Measured vs asserted":
where a value is limited by what the hardware can do rather than by whether the
queue is correct, it is recorded and compared across architectures instead of
deciding pass/fail. Skew is a **measurement, not a defect** — RMT and MCPWM arm a hardware
compare, while PCNT and the AVR ISR step the pin from an interrupt, so the
offset depends on the driver *and* on what the uC is doing at that moment. A
few microseconds of skew is a platform characteristic, and gating on it would
fail every PCNT and AVR target by construction. It now reports the skew, each
stepper's first-step time, and the skew in step periods; the step counts stay a
hard requirement. A `skew_three_periods` fixture with a *known* 3-period offset
must still **pass** while reporting exactly 3 periods — that pins the metric
down without making the value a pass criterion, which is what stops it silently
reading 0.0 forever.

A third, subtler one: the rate-sag fixture was originally 25 % off, which
`period_defects` caught on its own. A fixture that a cruder check already
rejects proves nothing about the finer check, so the sag is now sized between
the two tolerances (24 ticks = 3.75 %, above rate adherence's 2 % and below the
gross period check's 5 %). Forcing `rate_adherence` to always pass now turns
the suite red.

## Architecture notes that shape the plan

- **AVR speed floor depends on the stepper count.**
  `StepperQueue::adjustSpeedToStepperCount()` sets `max_speed_in_ticks` to
  `TICKS_PER_S/50000` with one stepper but **426** with two, because the ISR
  needs ~14 µs. So `1ch` and `2ch` must both be characterized on the same board,
  and a sweep done only at `1ch` reports a speed the board cannot sustain with
  two steppers connected.
- **AVR step pin is not a free choice.** It must be the pin the library maps to
  the timer compare output (`stepPinStepperA` / `stepPinStepperB` in
  `src/AVRStepperPins.h`), and which physical pin that is depends on
  `FAS_TIMER_MODULE`. `MAX_STEPPER` is 2 on a 328P, so `CONFIG 4ch_*` cannot
  work there.
- **"Driver max" is per driver, not a constant.** On this ESP32 the queue
  limits are RMT 8, MCPWM/PCNT 6, I2S mux 32, I2S direct 3
  (`src/pd_esp32/pd_config_idf5.h`, with `SUPPORT_DYNAMIC_ALLOCATION`). The
  analyzer caps a `dir` run at 4 and a `nodir` run at 8, so the binding limit
  is the smaller of the two. Mode `scale` must stop at the driver's own limit
  and say so, not at whatever the analyzer happens to allow.
- **I2S drivers are ESP32-only** (`SUPPORT_ESP32_I2S`), so those tests are
  simply not run elsewhere. Everything else is architecture-independent and
  must be run on every target — that comparison is where the value is.

## Orchestration — what actually exists

The design below is what shipped, not what the earlier draft proposed.

| Script | Role |
|--------|------|
| `scripts/capture.py` | sigrok-cli wrapper. `--vcd` also writes a `.meta` sidecar with the true sample count. |
| `scripts/signal_parser.py` | VCD/srzip/CSV reader plus edge, metric and **distribution** statistics. |
| `scripts/run_tests.py` | Scenario table (`SCENARIOS`), evaluators (`EVALUATORS`), `STOP_AFTER`, per-stepper programs. |
| `scripts/run_hardware.py` | Runs scenarios against a real board, one **cold boot** each, and writes one JSON result per scenario. |
| `scripts/sweep.py` | Plans a parameter sweep and **asserts every point is legal before any hardware runs**; `--run` drives the board. |
| `scripts/generate_report.py` | Builds the §8 artefacts from the JSON results. Formats only — never re-parses a capture. |
| `scripts/report.py` | Quick console view of a run. |

One scenario per cold boot: the question every run asks is what the board does
from a fresh start, and a board carrying state from the previous scenario would
answer a different one.

## Status

**25 of the white paper's 28 scenarios are implemented and verified on
hardware (ESP32, 25/25 pass).** Three are documented as not applicable to this
hardware: SR_22 needs RMT V2, SR_24 an AVR board, SR_00 is the opt-in pin
self-test. Committed baseline: `reports/esp32/`.

Implementation plan items 1–8 are done. See *Decisions and findings* for what
the hardware actually showed.

### The redesign now in progress

**Why:** the harness drove most scenarios with `CONFIG 1ch` / `CONFIG 2ch`,
which resolved to the library's *automatic* driver choice. Seventeen of the
twenty-five recorded results were tagged `auto`. That is not a
characterization — it records whatever the firmware picked, so it cannot say
anything about a driver, and it makes the results untagged by driver in the
report. **Every result must name the driver it ran on.**

The redesign is tracked item by item below so it can be picked up by more than
one agent. Each item is independently checkable and states how to verify it.

- [ ] **R1 — firmware: `auto` removed.** `parse_driver()` must not return
      `SA_AUTO`; an unknown or absent driver is an error, not a fallback.
      `CONFIG` takes an explicit count plus a per-stepper driver list, and
      refuses anything it cannot provide rather than clamping.
      *Verify:* `CONFIG 2` / `CONFIG 2 auto` is refused; no `SA_AUTO` reaches
      `connect_stepper()`; both builds clean.
- [ ] **R2 — firmware: 1…8 steppers, both channel modes.** `SALEAE_MAX_STEPPERS`
      rises from 4 to 8, `CONFIG` accepts a `nodir` mode, and a new `MAP`
      command reports mode and stride so host and firmware cannot disagree on
      which channel is which stepper.
      *Verify:* `MAP` matches the capture's channel use; 8 steppers in `nodir`
      and 4 in `dir`; AVR RAM still fits (it is at 1591/2048 with the
      per-stepper matrix — re-measure, and keep the matrix at a small
      segment count since only SR_15 needs it).
- [ ] **R3 — host: one generic entry point, two modes.** One call covers both:
      - **mode `scale`** — 1…driver-max steppers in parallel on a single named
        driver. For AVR this is one run; for Pico, SAMD and the rest, the same.
      - **mode `sync`** — for architectures with more than one driver: every
        driver-list combination, measuring **sync start** (first-step skew) and
        **adherence** (each stepper keeps the period it was given).
      Neither mode names an architecture. `--arch` / `--sdk` are metadata tags,
      not modes. A driver the board has not got connected — `i2s_mux` today —
      is one flag away and needs no new code path.
      *Verify:* `--mode scale` and `--mode sync` both run on the ESP32 and
      produce the tables in R5.
- [ ] **R4 — host: channel map is configuration, not a constant.** Today
      `STEP_CHANNELS`/`DIR_CHANNELS` hardcode A=`D0`, B=`D2`… which is only
      right in `dir` mode with four steppers. Evaluators, fixtures and the
      report must take the map from the result record.
      *Verify:* an `nodir` 8-stepper result maps A…H to `D0`…`D7` and an SR_15
      result still maps A=`D0`, B=`D2`.
- [ ] **R5 — report: two new tables, keyed by arch / sdk / driver list.**
      A parallel-count table (per driver: 1…max steppers, each stepper's period
      and count) and a sync-permutation table (per driver list: first-step skew
      in µs **and in step periods**, plus per-stepper adherence).
      *Verify:* both tables appear for the ESP32 run; a driver-list with no
      measured skew says so rather than printing an empty table.
- [ ] **R6 — whitepaper: complete revision.** It predates all of this and is
      wrong in several places that matter: it describes an automatic driver
      choice, the old console-only reporting, and channel configs the harness
      never implemented. Rewrite the driver model, the channel-config table
      (including `8ch_step_only`), the two test modes, and §8 to match what
      shipped. Do this **after** R1–R3 so it documents the code rather than
      anticipating it.
      *Verify:* every claim in the paper maps to a test that exists, or is
      marked as not applicable with a reason.
- [ ] **R7 — `i2s_mux` measured.** Never run. The build supports it
      (`SUPPORT_ESP32_I2S` is defined for IDF 5/6, `QUEUES_I2S_MUX` = 32 under
      dynamic allocation) and the mux is internal to the library
      (`i2s_manager` `_is_mux`/`_mux_state`), so no extender board is implied —
      but it is untested, and the user has not connected it. It is a single
      `--driver i2s_mux` run once it is available.
      *Verify:* a recorded result tagged `i2s_mux`; until then the todo must
      say *untested*, not *absent*.
- [ ] **R8 — re-run and re-baseline.** The existing 25 results are tagged
      `auto` for the most part and stop being meaningful the moment R1 lands.
      Re-run the full set with explicit drivers and regenerate
      `reports/esp32/`.
      *Verify:* no result file contains the tag `auto`.

### Not started, and deliberately so

- **Cross-architecture runs.** Only the ESP32 has been measured. AVR, Pico,
  SAMD and the other ESP32 variants are unmeasured, so the paper's central
  comparison does not exist yet. This needs boards, not code — but it is the
  largest remaining gap in value, and mode `scale` is what makes it one command
  per board.
- **AVR-specific predictions** (the speed floor rising from `TICKS_PER_S/50000`
  to 426 with a second stepper) remain unconfirmed.

## Decisions and findings

Recorded because they change what the tests mean.

- [x] **The driver never promises a pulse width.** Measured a constant
      **15.625 µs (250 ticks) at every period from 200 µs to 4096 µs**. So pulse
      width is *recorded*, never asserted: the driver sets it, its value is a
      property of the silicon, and it is the baseline a regression is measured
      against. A glitch counter was considered and dropped — it needs an
      invented threshold and collapses a distribution into one number.
- [x] **`stopMove()` does not stop a move that is already queued.** It only
      sets a flag the ramp generator reads when asked for its *next* command.
      A 2000-step move stopped at 510 ran to **2000** — no effect. A 20000-step
      move stopped at 5825 finished at **14240**, truncating only once the
      queue refilled. Verified on a waveform: truncated at 11475 of 20000, no
      partial pulse, pin still for the remaining 2.823 s. Worth knowing before
      relying on `stopMove()` as an emergency stop.
- [x] **Cross-driver start skew is not worse than same-driver skew.**
      29.5417 µs for RMT+PCNT and the same 29.5417 µs for two steppers on one
      driver — about 0.74 of a step period. So the paper's premise that the two
      drivers arm through unrelated hardware and must therefore diverge is not
      what the hardware does. Reported, never gated.
- [x] **MCPWM/PCNT counter-limit overrun: no defect.** 256/256, 510/510, and a
      200…255 sweep where every count was exact.
- [x] **The inter-command gap is one period**, not a trailing wait — the earlier
      premise behind `intra_command_periods()` was wrong and was reverted.
- [x] **A VCD cannot show you a flat tail.** VCDs record value changes only, so
      a line that stays flat after its last edge leaves the file's last
      timestamp short of the real capture end. `capture.py` writes a `.meta`
      sidecar with the true sample count and `load_vcd` pads to it; without
      that, "the run stopped" and "the recording ran out" are indistinguishable.
- [x] **The clone's buffer holds 64 MSamples** — 2.66 s at 24 MS/s. A capture
      that hits the limit ends mid-run.
- [x] **Sparse channel selections drop channels** on this clone: `-C D0,D2`
      yields nothing on `D2` while `-C D0,D1,D2` works. Contiguous only.
- [x] **`QSEG` grew a stepper index** (`QSEG <idx> <steps> <ticks> <dir>`) so
      SR_15 could give two steppers different periods. The 3-argument form still
      means the shared program, so nothing else changed meaning; all 25
      scenarios were re-run afterwards and passed.
