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
hardware (ESP32, 25/25 pass, every one naming its driver).** Three are
documented as not applicable to this hardware: SR_22 needs RMT V2, SR_24 an AVR
board, SR_00 is the opt-in pin self-test. Committed baseline: `reports/esp32/`,
regenerated with explicit drivers (R8).

Implementation plan items 1–8 are done, and the redesign's **R1** (firmware:
`auto` removed) and **R8** (re-run and re-baseline) are done with hardware
behind them. R2 (firmware: 1…8 steppers, both pin modes, `MAP`) is done too.
R3 (host: the two generic modes `scale` and `sync`) and R4 (the channel map
becoming configuration rather than a global) are done too. R3's first hardware
run found a live defect in the MCPWM/PCNT driver; R4's found that SR_15's
ratio collapsed to 1:1 on the real chip, so it had been measuring nothing.
R5 (the two mode report tables) is done as well. R7 is rewritten rather than
done: `i2s_mux` turned out not to be measurable by this rig at all, and chasing
it found that the host had been guessing what each chip can do. See *Decisions and findings* for what the hardware actually showed — including the one recorded finding that R1's re-run
invalidated.

### The redesign now in progress

**Why:** the harness drove most scenarios with `CONFIG 1ch` / `CONFIG 2ch`,
which resolved to the library's *automatic* driver choice. Seventeen of the
twenty-five recorded results were tagged `auto`. That is not a
characterization — it records whatever the firmware picked, so it cannot say
anything about a driver, and it makes the results untagged by driver in the
report. **Every result must name the driver it ran on.**

The redesign is tracked item by item below so it can be picked up by more than
one agent. Each item is independently checkable and states how to verify it.

- [x] **R1 — firmware: `auto` removed. Done.** `parse_driver()` returns a
      `bool` and never `SA_AUTO`; an unknown name *and* a real driver this build
      has no queues for are both refused, so `CONFIG 2 rmt,rmt` on a 328P does
      not quietly hand back two timer queues. `enum saleae_driver` has no
      automatic member and `connect_stepper()` no longer has a `default: break`
      that leaves `DRIVER_DONT_CARE` in `fd` — the only way out of the switch is
      `return false`.
      `CONFIG <count> <driver>[,<driver>...] [dir|nodir]` replaces the eight
      named presets. It **refuses rather than clamps** on all four counts: a
      count above `SALEAE_MAX_STEPPERS`, a driver the build cannot provide, a
      list whose length is not the count, and a pin mode that is not `dir`
      (which is the only one implemented; `nodir` arrives with R2). The count
      token is validated *whole* (`*end != '\0'`), not with `atol`: `CONFIG 1ch`
      would otherwise have `atol` read the leading `1` and quietly connect one
      stepper on an unspecified driver — the exact failure mode being removed.
      The eight preset names are gone from the firmware entirely; a grep for
      `SA_AUTO`, `DRIVER_DONT_CARE` or `strcmp(name, "1ch")` finds nothing but
      the comments that explain why they are gone.
      Driver names stay explicit on every architecture, including the ones with
      a single native driver, where the list simply repeats it: `timer` on
      AVR/SAM/SAMD, `pio` on Pico (white paper §3.1). Naming one costs nothing
      and keeps every result tagged with the driver that made it.
      The `OK CONFIG` reply now names the drivers the board actually connected
      (`OK CONFIG n=2 mode=dir drivers=rmt,mcpwm_pcnt maxspeed0=…`), so a run can
      be checked against what the hardware did rather than what the host asked
      for. `rmt` is reported, not `rmt_v2`: the RMT generation is a property of
      the SDK, which is already a tag on the run.
      *Host side, because the firmware no longer speaks the old vocabulary:*
      `run_tests.CONFIGS` owns the logical-config → `(count, driver list)`
      mapping, `config_wire()` renders the line and `driver_tag()` names the
      drivers for the result record — so `driver_of()`'s hardcoded `"1ch": "auto"`
      table is deleted rather than left lying. `harness.py` drops
      `--channel-config` for `--count` / `--pin-mode` / `--drivers`, and
      `run_hardware.py` / `sweep.py` gained `--dut-driver` (`--driver` was
      already the sigrok device, and renaming it would have broken `sweep.py`).
      Every scenario now sends a named driver: `SR_17` `CONFIG 2 rmt,mcpwm_pcnt
      dir`, `SR_18` `CONFIG 1 mcpwm_pcnt dir`, and on AVR `SR_14` `CONFIG 2
      timer,timer dir`.
      *A cost worth recording.* String literals live in `.data` on AVR, and the
      first draft of the messages cost **+212 bytes** of a 2048-byte budget
      (1591 → 1803, i.e. 77.7% → 88.0%). Collapsing four near-duplicate error
      strings into one and dropping the driver-name table from the AVR build
      brought it to **+84 bytes (1591 → 1675, 81.8%)**, which is what the
      uniform `drivers=` reply above costs. Worth remembering before adding a
      sixth `ERR` variant.
      *A second cost, found by the test that checks the first.* The generic
      grammar makes argument 2 — the driver list — grow with the stepper count,
      and it was still parsed with `%31s` into a `char[32]`. Four
      `mcpwm_pcnt`/`i2s_direct` names is 43 characters, so `CONFIG 4
      mcpwm_pcnt,mcpwm_pcnt,mcpwm_pcnt,mcpwm_pcnt dir` was truncated to
      `mcpwm_pcnt,mcpwm_pcnt,mcpwm_pc` and refused with `ERR CONFIG no such
      driver` — a **legal request refused with a misleading reason**, which is
      the failure mode this whole item exists to remove. `arg2` is now `char[48]`
      read with `%47s`. The old `mixed` form had the same 31-char cap, so the
      bug predates R1; the grammar is only what made it reachable at 4 steppers.
      `SALEAE_LINE_MAX` (64) still has room at 4 steppers — the longest legal
      line is 57 characters — but **R2 will have to raise it**: 8 steppers is
      `CONFIG 8 i2s_direct,… dir` at 100 characters.
      *Verify:* `CONFIG 2` / `CONFIG 2 auto` / `CONFIG 1ch` / `CONFIG 2 rmt
      timer` / `CONFIG 3 timer,timer dir` are all refused with an `ERR` and no
      stepper connects; `CONFIG 2 rmt,mcpwm_pcnt dir` connects both; the `SA_AUTO`
      and preset-name greps above are clean; `saleae_avr` **1675/2048**,
      `saleae_esp32` and `esp32_idf_V5_3_0` all build with no new warning; 97
      unit tests pass. Seven new hardware-free tests in
      `scripts/tests/test_saleae.py::TestConfigGrammar`, each **mutation-checked**
      — reintroducing `SA_AUTO`, dropping the whole-token count check, padding a
      config to fewer drivers than its count, shrinking `arg2`, and narrowing
      the `sscanf` format each turn the suite red. That last two exposed a
      vacuous first version of the line-budget check, which read the width from
      the format string and so could not see a buffer that disagreed with it.
      **Not verified on hardware** — no board was connected, so R8 has to re-run
      the set before any of these numbers mean anything.
- [x] **R2 — firmware: 1…8 steppers, both channel modes. Done and measured on
      hardware.** `SALEAE_MAX_STEPPERS` rises 4 → 8, `CONFIG` takes a `nodir`
      mode, and `MAP` reports count + mode + stride + the GPIO behind each
      reachable channel.

      *One pin table serves both modes, and the stride is what selects between
      them.* `kChanPin[8]` maps analyzer channel to GPIO; in `dir` stepper *j*
      owns channels 2*j* (step) and 2*j*+1 (dir), in `nodir` it owns channel *j*.
      So the mode cannot drift from the map — there is nothing to keep in step
      but the stride. It is also the same pins in the same order as SR_00's
      eight (`common/saleae_test.cpp`), so a channel the self-test proved is a
      channel a scenario measures. The two old tables (`kStepPins[4]`,
      `kDirPins[4]`) could only ever express the `dir` shape; eight entries
      that a stride indexes express both.

      *The count cap is `min(stepper queues, channels/stride)`,* and a refusal
      names **both** bounds — `ERR CONFIG n=5 max=4/8/8/2` — because "too many
      steppers" cannot say which one bit, and they are different facts (MCPWM/PCNT
      has 6 queues on IDF 5; the channel budget runs out at 4 with `dir`).

      *`nodir` is a real pin configuration, not a shorthand.* No dir pin is
      connected at all, so `setDirectionPin()` is not called; the QSEG direction
      argument still **parses** (a scenario's program is unchanged between modes)
      but is **forced true**, because there is no pin to toggle for a false and
      the queue would otherwise refuse with `ErrorNoDirPinToToggle`. That is the
      one semantic difference, and it is why the direction-observing scenarios
      are `dir`-mode by construction rather than by convention.

      *Verified on the connected ESP32* (RMT, 4 MS/s, Saleae clone):

      | CONFIG | MAP | capture |
      |--------|-----|---------|
      | `4 rmt,rmt,rmt,rmt dir` | `count=4 mode=dir stride=2` | steps on **D0,D2,D4,D6**; D1,D3,D5,D7 quiet (dir) |
      | `8 rmt×8 nodir` | `count=8 mode=nodir stride=1` | **8 steps on every one of D0…D7**, all 40.00 µs, all 15.50 µs high |
      | `3 rmt×3 nodir`, program `dir=0` | `count=3 mode=nodir stride=1` | 12 steps each — the forced-true path runs |

      `POS` agrees in every case (8×8, 8×4, 12×3), so the counts are the board's
      and not a capture artifact. **And `MAP` matches the capture's channel use
      in both modes**, which is the check that matters: it is the firmware's
      claim about itself, corroborated by the wires.

      *AVR RAM: 1741 / 2048 (was 1675), so it fits with 307 bytes spare.* The
      +66 was not free, and finding out where it went was most of the work:

      - **`SALEAE_ARG2_MAX` is a `#if` ladder, not `12 * SALEAE_MAX_STEPPERS`,**
        because it doubles as an `sscanf` field width and a format string cannot
        hold an expression. AVR gets 24 bytes where ESP32 gets 96. The
        `static_assert(SALEAE_ARG2_MAX >= 12 * SALEAE_MAX_STEPPERS)` is what
        makes the ladder safe for a platform that outgrows its top rung.
      - **avr-gcc puts the stack in `.data`, so every reply buffer is RAM.** A
        flat 256-byte CONFIG reply cost **112 bytes** and took the build to
        87 %; a single shared `reply_scratch` took it to 93 % — worse, because
        the static buffer is *added* to what the frame already held. The buffers
        are sized per platform (`SALEAE_CFG_REPLY_MAX`, `SALEAE_SHORT_REPLY_MAX`).
      - **The `ERR CONFIG` reply is 46 characters, not 88.** Prose on a 2 KB part
        is 64 bytes of stack; `n=5 max=4/8/8/2` carries the same information and
        cost 58 bytes back on its own.

      *Host side.* `run_tests.py` derives the channel map from `MAP` instead of
      the hardcoded A=D0/B=D2 table, and `channel_map` + `pin_map` travel in every
      result record. This was **not cosmetic** — measured on the 2-stepper `nodir`
      run, the old map reads stepper B as **0 steps** because B is on D1, not
      D2, i.e. it reports a working driver as dead. On the 8-stepper `nodir` run
      it cannot even name E–H. `harness.py` refuses a count the channels cannot
      carry before opening a capture. *Full threading of the map through the
      evaluators is still R4 — `evaluate()` takes `chan_map` and installs it, and
      every current scenario connects the 4-stepper `dir` shape, which is
      unchanged.*

      *Tests: 110 pass, up from 98.* New `TestChannelMap` (9) covers both
      shapes, the specific case the hardcoded map got wrong, the channel budget,
      the `MAP` reply parse, and the firmware's `nodir` semantics. The
      line-buffer test now resolves the `#if` ladder the way the preprocessor
      would — **including the trailing `#else`**, which is easy to miss: it
      answered with the last `#elif`'s value (72 where 96 was meant) and the
      8-stepper driver list then failed a check it should have passed.
      *Mutation-checked, all caught:* ladder rung too small, `arg2` losing its
      `+1` terminator, `sscanf` width off by one, `LINE_MAX` slack too small,
      the `#else` rung deleted, firmware dropping `count_up` forcing,
      `setDirectionPin` called unconditionally, `nodir` made unreachable,
      host stride/cap/map ignoring the stride, and `harness` dropping the
      budget check. Two were missed on the first pass — an unreachable `nodir`
      (`if (false && …)` still contains the strcmp) and an unconditional
      `setDirectionPin` — so those assertions are now anchored to the `if`.
- [x] **R3 — host: one generic entry point, two modes. Done, and it found a
      library defect on its first run.**
      `harness.py --mode scale|sync`. Neither mode names an architecture, which
      is the property worth testing: `scale` is the whole cross-architecture
      matrix (one run on a 328P, eight on an ESP32 in `nodir`), and `sync`
      applies to any board with more than one driver.

      **The two modes are deliberately not the same measurement, and the
      difference is the whole point.** `scale` gives every stepper **one shared
      program**, so each run answers "does this driver still emit the commanded
      period with N attached, and does every one get every step". `sync` gives
      each stepper **its own period** (a 1:2:3 ratio ladder), because
      adherence is only checkable if the steppers were given different
      expectations — a synchronized start that dragged them all onto one speed
      satisfies a first-step test *and* a shared-period test, and would be
      reported as a perfect sync. Each stepper is then checked against **its
      own** commanded period, so a collapse is *seen* rather than inferred from
      its absence. Skew is still reported, never gated, in µs **and** in step
      periods.

      **`scale` stops at min(driver, channels) and says which bound it was.**
      `DRIVER_MAXS` carries the library's own `QUEUES_*` counts and the test
      cross-checks the ESP entries against `src/pd_esp32/pd_config_idf5.h`, so
      the table cannot drift away from the build it plans for. A missing entry
      is **refused**, not guessed: a guessed bound either stops the sweep early
      (reporting a driver limit that is not one) or runs past what the board can
      connect. A `0` entry means "this chip has no such driver" and is distinct
      from an absent one. So "RMT reaches 8 steppers in `nodir`" and "RMT
      reaches 4 in `dir`" are different findings, and only the second is about
      the analyzer rather than RMT.

      **`sync` enumerates over driver *identities*, not names.** `rmt` and
      `rmt_v2` are one driver — the firmware maps both to `SA_RMT` and reports
      both as `rmt` — so enumerating spellings would put `rmt_v2+rmt` in the
      table as a *cross-driver* combination, under a name claiming they differ.
      **That is precisely how R1's wrong finding was produced**: the old `mixed`
      config discarded its driver list for the automatic choice, so the
      "cross-driver" run *was* RMT+RMT — and the tell was that two supposedly
      different configurations agreed to four decimal places. Identical numbers
      across different configurations is the signature to look for, so the plan
      has to contain both cases to make it meaningful.

      **Measured on the connected ESP32** (RMT, 5 µs step, 4 MS/s, Saleae
      clone). `--mode scale --driver rmt_v2 --pin-mode nodir`: **8/8 passed**,
      every stepper 64/64 steps at 160 ticks, and the period spread across
      steppers was **0.004 µs** at its widest (9.996 vs 10.000 µs, i.e. one
      4 MS/s sample) — so RMT holds its period from 1 stepper to 8 with nothing
      to report but "it scales".

      `--mode sync --arch esp32` — all 10 combinations attempted:

      | driver list | result | skew µs | in periods | adherence |
      |---|---|---|---|---|
      | `rmt+rmt` | passed | 37.5 | 3.75 | both kept 160 t / 320 t |
      | `rmt+mcpwm_pcnt` | passed | 62.25 | 6.23 | both kept theirs |
      | `mcpwm_pcnt+i2s_direct` | passed | 858.75 | 85.88 | both kept theirs |
      | `rmt+i2s_direct` | passed | 990.0 | **99.0** | both kept theirs |
      | `i2s_direct+i2s_direct` | passed | 52.25 | 5.23 | both kept theirs |
      | `mcpwm_pcnt+mcpwm_pcnt` | **failed** | 6.5 | 0.65 | **B: 11053 steps, not 64** |
      | `*+i2s_mux` (4) | refused | — | — | `ERR connect step 0/1 drv=i2s_mux` |
      | `i2s_mux+i2s_mux` | refused | — | — | as above |

      **A refused point is recorded, not skipped, and the plan continues** — for
      `scale` the point where the board says no *is* the answer, and for `sync`
      a driver this build cannot connect is a fact worth recording next to the
      combinations that did measure. That is what makes `i2s_mux` (R7) one flag
      away with no new code path: it is already in the ESP32 driver list, so the
      run attempts it and records the refusal.

      **The I2S skew is the interesting number, and it is 30× the
      cross-driver figure R1 recorded.** `rmt+i2s_direct` comes in at ~990 µs
      (99 step periods) against `rmt+mcpwm_pcnt`'s 62 µs, and it **reproduces to
      a sample across three runs** (990.0 / 990.25 / 1068.25 µs; `rmt+rmt` was
      37.5 / 37.5 / 37.75 over the same three). I2S is the one driver here that
      emits from a DMA callback rather than from an armed timer, so "arm
      together" cannot include it — a millisecond is the time for the DMA
      pipeline to produce anything at all, not a scheduling jitter. R1's
      cross-driver conclusion ("skew tracks driver heterogeneity, ~66 % worse
      than same-driver") **holds and understates**: it was drawn from RMT and
      MCPWM, which are both armed timers, and the I2S case is a different
      mechanism entirely. Skew in *periods* is what makes that visible — 6.23
      against 99.0, which one number in µs would not convey.

      #### Two real bugs the first `--mode scale` run exposed

      Both are in code R3 merely *reached*, not code R3 wrote, and both had been
      invisible because **the path that uses them had never run.**

      **1. `QINFO` concatenated its per-stepper floors, so it was silently wrong
      for any run with more than one stepper.** `handle_qinfo` printed
      `maxspeed=` followed by one value per stepper with **no separator**, so a
      3-stepper board replied `maxspeed=808080` — which the host's regex read as
      the single number 808080. Every `QSEG` built from that exceeded the 16-bit
      ticks field, so the run failed with `ERR QSEG ticks=1..65535`: *accurate*,
      and about a number nothing had asked for. The fix separates them
      (`maxspeedN=<ticks>`) and adds `maxall=`, the **largest** floor, which is
      what a shared program must be planned against — reading stepper A's own
      floor would plan too fast whenever a later stepper is slower, which is
      exactly the RMT+MCPWM case. `maxall` is printed **first** so a buffer
      overrun cannot truncate the one field the host cannot reconstruct.
      `SALEAE_QINFO_REPLY_MAX` is a separate per-rung buffer (QINFO grows ~13 B
      per stepper and could not share `SALEAE_SHORT_REPLY_MAX`; an 8-stepper
      reply truncated its own tail, after which the host reported "no QINFO
      reply" with no hint that the firmware had run out of buffer). AVR cost
      +14 B → **1755/2048**. `legal_ticks()` now also clamps to 65535, so that
      class of mistake fails as a slow-but-legal run rather than as a confusing
      refusal.

      **2. `harness.py` could not run at all.** It never defined
      `--capture-dir` or `--sr00-sample-rate`, both of which `run_tests.py` reads,
      so *every* non-`--dry-run` invocation died with `AttributeError: 'Namespace'
      object has no attribute 'capture_dir'` on the first capture — including
      the documented `--flash` example in AGENTS.md. Verified pre-existing by
      stashing. Also `build-pio-dirs.sh` still pointed at
      `apps/arduino/saleae_main.ino`, which moved to `apps/arduino/src/` in
      fa3e419a, so the `pio_dirs/saleae` project could not build either.

      #### The library defect R3 exists to find, found on the first sweep

      **`scale` on `mcpwm_pcnt` fails at 2 steppers and above: stepper B emits
      continuously and never stops.** `POS` reads **non-monotonic** — `64 31`,
      then `64 26`, then `64 32` — all below 64, which is the signature of a
      position counter being re-read mid-run rather than of steps being lost.
      The capture shows why: D2 carries **22 143 rising edges at exactly the
      commanded 10 µs period** (5.0 µs high, 5.0 µs low, continuously, to the end
      of the window) where 64 were asked for.

      **Localized to two MCPWM/PCNT queues, not to MCPWM and not to the mode:**

      | configuration | `POS` over 1.5 s |
      |---|---|
      | `rmt+rmt` | `64 64` every time — stable |
      | `rmt+mcpwm_pcnt` | `64 64` every time — stable |
      | `mcpwm_pcnt+rmt` (swapped) | `64 64` every time — stable |
      | `mcpwm_pcnt+mcpwm_pcnt` | `64 31 / 64 26 / 64 32 / 64 50 / 64 34` |
      | `mcpwm_pcnt+mcpwm_pcnt` at 3200 ticks (200 µs) | also fails — not a speed limit |

      It reproduces on the **unmodified firmware** (stashed and re-flashed), so
      it is a library defect, not an artefact of this mode. Most likely
      `channel2mapping[]`/`pcnt_unit_to_queue[]` indexed by `channel_num` with
      `pcnt_unit_id = timer_num`, while the ESP32 has **4 MCPWM timers**
      (2 groups × 2) against **`QUEUES_MCPWM_PCNT` = 6** — the second queue's
      timer/PCNT mapping is the thing to look at first. Left as a finding rather
      than fixed here: the library is out of R3's scope, and R3's job was to
      surface it. **`DRIVER_MAXS` says MCPWM/PCNT has 6 queues on IDF 5, and
      that number is now known to be wrong** — the driver cannot run 2 queues
      correctly on this chip.

      *Tests: 144 pass, from 110.* New `TestModes` (23) plus 11 regression
      tests covers both plan shapes,
      the mask selecting every connected stepper, the wire matching the label,
      `scale` using a shared program, `sync`'s periods being *distinct* and its
      commands legal, identity folding, the bound arithmetic, and what each
      evaluator actually catches — a collapsed rate, a lost step, a silent
      stepper, an off-period stepper, and skew that is *reported but not gated*.
      Eleven more cover the two bugs above and the `scale`/`sync` runners: the
      QINFO reply is **rendered from the firmware's own format strings** and then
      parsed, so a disagreement between the two sides fails rather than being
      papered over by a hand-written example (a hand-written one agrees with
      whichever side the author was looking at); the largest floor is read, not
      the first; `QINFO` keeps its own buffer and checks what `snprintf` wanted to
      write; and a **fully faked** catalogue run is driven end to end, so
      `harness.py`'s missing `capture_dir` — which made *every* real invocation
      die on its first capture — cannot come back. *Mutation-checked, all caught:* the
      driver bound ignored, the channel budget off by one, an unknown driver
      guessed, identity folding dropped, permutations instead of combinations,
      a dropped combination, the sweep skipping n=1, a mask leaving steppers
      idle, shared periods in `sync`, skew pinned to zero, and either adherence
      check made vacuous. Two misses were equivalent mutants — `dmax <
      chan_cap` → `dmax < chan_cap + 1` changes no output when `dmax ==
      chan_cap` — and one "mutation" only edited a docstring; the behavioural
      version of it was caught twice.
- [x] **R4 — host: the channel map is configuration, not a constant. Done, and
      it exposed three defects on the way.**
      `STEP_CHANNELS`/`DIR_CHANNELS` were two module-level dicts that
      `evaluate()` overwrote per run, hardcoding A=`D0`, B=`D2`… — right only in
      `dir` mode with four steppers. They are gone, replaced by a **`Pins`
      object passed to every evaluator**. There is no channel table at module
      scope any more, so a stale one cannot be read by accident; a test greps
      for its return.

      **Why the global was worse than a wrong default.** It made a wrong map a
      *silent substitution* rather than an error: in `nodir`, stepper B is on
      `D1`, so reading it on `D2` measures a quiet pin and reports a driver that
      emits nothing. And it could only ever hold one map at a time, so two
      results with different shapes could not be compared in one process —
      which is what a fixture and a mode run need. `test_two_maps_can_be_judged
      _in_one_process` pins that property; the globals could not satisfy it.

      **The load-bearing assertion is that the verdict *changes* with the map.**
      `test_a_nodir_map_cannot_be_judged_as_a_dir_map` puts a working pair of
      steppers on `D0`/`D1` and evaluates them twice: with the `nodir` map it
      passes and reports B at 40/40 on `D1`; with the `dir` map it fails. A test
      that only checked the right map passed would pass just as well against a
      decorative one.

      #### Three defects found while doing it

      **1. A stepper the board connected but the capture lacked was silently
      skipped — and the run *passed*.** `step_wave()` returns `None` for a
      channel that is not in the capture, and the evaluators skipped it. With a
      map naming three steppers and a capture carrying two, SR_16 reported only
      A, found nothing wrong, and returned **passed**: a result for a stepper
      nothing was measured about. Reading an absent channel as a quiet one
      invents a defect; *passing* it invents a result. `Pins.missing()` now
      detects it and `evaluate()` fails the run with the missing steppers and
      channels named, before any evaluator runs — centrally, because every one
      of the thirteen would otherwise have to get it right.

      **2. SR_15 could never have run.** The runner asked "does this scenario
      need per-stepper programs?" by calling `per_stepper_programs(scenario,
      None)`, which dereferences the QINFO dict: `TypeError`. The question is
      now answered from an explicit `PER_STEPPER_SCENARIOS` set, because the
      answer is needed *before* the board is wired and the old way of asking
      needed a value that does not exist yet.

      **3. SR_15's 2:1 ratio collapsed to 1.0 on the real chip — so the scenario
      measured nothing and reported success.** `legal_ticks()` has a 160-tick
      floor, and the ESP32's real RMT floor is **80**. SR_15 derived its slow
      stepper from `max_speed_ticks * 2`, so it asked for 80 and for 160 — which
      both clamp to **160**. Two identical periods went to the board, and the
      evaluator, checking each stepper against *its own* command, passed.

      **It went unnoticed because the fixture DUT's floor is 640**, where the
      ratio survives by arithmetic accident (640 and 1280 both clear 160). The
      fixture docstring brags that it is the only source of time constants so an
      evaluator cannot hardcode 16 MHz — and that discipline missed the one
      number that mattered, because the fixture was **10× more generous than the
      silicon**. The slow stepper is now derived from the *clamped* fast period,
      so the ratio is a ratio: 160/320 on the real RMT floor, 640/1280 on the
      fixture, 426/852 on an AVR timer.

      *Verified on hardware.* `--mode scale --driver rmt_v2 --pin-mode nodir`:
      **8/8 passed, 64/64 steps each**, and the recorded map at every count
      matches the capture — 1→`A=D0` … 8→`A=D0…H=D7`. Catalogue `dir` scenarios
      re-run: SR_01, SR_14, SR_15, SR_16, SR_17 all pass with the map **A=`D0`,
      B=`D2`** preserved. SR_15 now measures something it never measured before:
      A commanded 160 t → **9.9987 µs**, B commanded 320 t → **19.9987 µs**,
      200/200 steps each, skew 37.5 µs = 3.75 periods.

      Also: `run_hardware.wire_plan()` captured a fixed `D0..D3` regardless of
      stepper count and a fixed `QRUN 3` mask, so a four-stepper scenario would
      have scored C and D as silent drivers. Both are derived from the config
      now. The test that covers it widens the scenario table to four steppers
      deliberately, because no SR id reaches four and a test that only walked
      the catalogue would not have caught the cap.

      *Tests: 161 pass, from 144.* New `TestPins` (12) and
      `TestPerStepperPrograms` (5). *Mutation-checked, all caught:* the map
      ignored by `evaluate()`, a module-level table returning, `report.py`
      ignoring the recorded map, `for_scenario` always claiming four steppers,
      `Pins.items()` truncated, `dir_of` returning the step pin, the invariant
      comparing a pin to itself in `nodir`, a missing channel no longer noticed,
      the SR_15 ratio collapsing, SR_15 no longer flagged, the runner probing
      with `None` again, and the channel cap back to a fixed 4.
- [x] **R5 — report: the two mode tables, keyed by arch / sdk / driver list.**
      A parallel-count table (per driver: 1…max steppers, each stepper's period
      and count) and a sync-permutation table (per driver list: first-step skew
      in µs **and in step periods**, plus per-stepper adherence).

      Both ship in `report.py`. They exist because **a mode run produces a
      result and no capture**: there is no VCD for the catalogue pipeline to
      evaluate, so before these a `--mode scale` run had nothing to report at
      all. `--results-dir` names where the mode JSON lives (defaulting to the
      run directory), and the markdown gains both tables alongside whatever
      catalogue rows there are.

      #### Skew is not a score, and the table proved it

      Writing the sync table immediately paid for itself. `mcpwm_pcnt+mcpwm_pcnt`
      posts the **smallest first-step skew of any combination measured — 6.25 µs,
      0.625 periods** — while being the driver that never stops. Ranked by skew,
      the defect is the best row in the table. Its *period* is flawless:
      **10 717 steps at exactly the commanded 19.9991 µs** where 64 were
      commanded, so a step count is the only thing that shows it. This is the R3
      runaway, and the reason adherence is a column rather than a footnote.

      A deviation is therefore **named, not flagged**. `DEVIATED` cannot tell a
      swallowed step from an extra one — a driver that stops early and one that
      never stops are opposite bugs — so the cell reads
      `B 19.9991us x10717/64 (+10653 extra steps)` and, where it applies,
      `-13 missing` or `7 long/2 short periods`. A stepper that keeps its count
      and drifts its rate has nothing wrong with its count, so the period is
      named in the same cell.

      The skew cell for a driver list that could not be measured states **why**,
      including the refusal text. A blank is indistinguishable from a report
      bug, and for a refused driver list the refusal *is* the measurement.

      #### Reading the target out of the record, not the tag key

      The tag key encodes arch and SDK, but as a naming convention: `esp32_arduino_…`
      splits on two underscores and `esp32_idf5_3_0_…` on one. A table keyed by
      architecture that recovers the target by string surgery is keyed by a
      convention, so `arch`, `framework` and `sdk_version` are now recorded in
      every mode result. Pre-existing records show `? / ?` rather than a guess —
      which is the correct thing for them, and made the gap visible: every sync
      result from R3 was in a directory that never recorded its own target.

      *Verified on hardware.* A fresh ESP32 run of both modes:
      `scale rmt_v2/nodir` **8/8 passed** (parallel-count table, 8 rows),
      `sync` **10 combinations** (sync table, 6 measured / 4 refused), both
      under `Target: esp32 / arduino / sdk latest`. The fresh run reproduced
      R3's findings independently — MCPWM runaway again (10 717 steps, 6.25 µs
      skew) and all four `i2s_mux` combinations refused.

      Two smaller things the tests forced out: a MODE record whose `mode` key is
      missing fits neither table, and filtering on that key alone dropped it
      from the report with nothing said — so it is listed under *Mode records
      with no recognised mode* rather than vanishing, because a run that
      disappeared reads as a run that was never made. And the results
      directory is now keyed by file name, not by the record's own `tag_key`;
      keying on the field meant a collision silently reported one run of two.

      *Tests: 178 pass, from 161.* New `TestModeTables` (14). *Mutation-checked,
      all caught:* catalogue `SR_*` records swept into the mode tables, a refused
      driver list claiming its steppers were fine, an unknown-mode record
      dropped again, `collect_modes` returning nothing, a missing skew printing
      `None`, the target read from a constant, the measured count replaced by
      the expected count, the extra-step count unreported, a lost step
      unreported, a wrong period unreported, and a refusal's reason dropped.
- [x] **R6 — whitepaper: complete revision. Done, and done *first*.** Done
      before R1–R3 on purpose: the paper is the spec, so R1–R5 now implement a
      written design rather than the code inventing one the paper then
      documents. What changed:
      - **§1.2 two generic modes** (`scale`, `sync`), neither naming an
        architecture; architecture/sdk/driver are tags on a run.
      - **§1.4 the pulse-width measurement**, which moved that row of the
        measured/asserted table from *asserted* to *measured* — the hardware
        disagreed with the original reasoning, so the reasoning changed.
      - **§3.1–3.2 the driver model**: no automatic selection anywhere, and the
        eight named channel presets replaced by the single generic
        `CONFIG <count> <driver>[,…] [dir|nodir]`, with the old table kept in a
        collapsed `<details>` for the record.
      - **§3.3–3.4 the channel budget and why there is no start marker.** All
        eight channels carry step/dir pins, so no marker channel exists on a
        4-stepper run; the library's own probes cannot substitute because they
        exist only in the RMT drivers and only on two chips. Also removed the
        fictitious "CH 8"/"CH 9" the draft reserved — the clone has 8 channels.
      - **§4.5 the protocol**: `CONFIG` grammar, `MAP`, and both `QSEG` forms
        with the +110-byte AVR cost stated.
      - **§5.5 the sync permutations**, including the result that
        cross-driver skew is *identical* to same-driver skew (29.5417 µs,
        0.7385 periods), which contradicts what §5.2 previously asserted.
      - **§5.6 `stopMove()` and the pulse queue**, with the two measured cases.
      - **§8 what actually ships**: no `tag_index.json`, no `design_specs.json`,
        and why; no `glitch_count` column and no single `period_us`; real sample
        output; LF-only CSV.
      - **§9 the real directory listing**, and the instrument/harness split.
      - **§10 the 4-vs-8 channel budget** and the 64 MSample buffer limit.
      - **Two catalogue entries corrected against the firmware**: SR_13's error
        text is `ERR QE step0 rc=-1` (`ErrorTicksTooLow`), not the predicted
        `ERR QE ticks … < maxspeed …`; and SR_18's trailing single step cannot
        use `<max>` ticks because a `steps=1` command is bounded by its ticks
        alone and `<max>` = 640 is under `MIN_CMD_TICKS`.
      *Verify:* all 15 internal `§` references resolve; the 28 scenario ids in
      the paper match the harness exactly (25 wired, 3 documented
      not-applicable); every file the paper names exists.
- [ ] **R7 — `i2s_mux`: not connected yet, and the harness was the problem.**
      *Remaining:* wire the multiplexer and give it a pin assignment, then run
      `--driver i2s_mux`. The harness is ready: `--imux data,bclk,ws` brings it
      up over serial. Until a pin assignment exists this must keep saying
      *not connected*, not *absent* and not *broken*.

      **What it actually is, from the library.** `i2s_mux` is not a driver this
      rig can probe, and the reason is structural rather than electrical. In mux
      mode a "step pin" is **not a pin**: it is a *slot index* on one shared I2S
      data line — `esp32_set_enable_pin_state()` masks `PIN_I2S_FLAG` and uses
      `pin & 0x1F` as the slot (`src/pd_esp32/esp32_queue.h:186`). Up to 32
      steppers share that one line. The harness model — one analyzer channel per
      step pin, `MAP count/mode/stride`, `Pins` — assumes distinct pins, so a
      32-slot mux cannot be measured with an 8-channel probe. Characterizing it
      means probing `data/bclk/ws` and selecting one slot at a time, which is a
      different measurement and a different tool.

      **The refusal was correct, and the harness caused it.** `initI2sMux()` must
      be called before any `stepperConnectToPin(DRIVER_I2S_MUX)` and cannot run
      twice (`FastAccelStepperEngine.h:92`). The harness never called it, so
      `stepperConnectToPin` returned NULL and CONFIG reported
      `ERR connect step 0 … drv=i2s_mux` — a *correct* refusal for a pin
      assignment nobody had made. R3 recorded that as `refused` and moved on,
      which was the right behaviour, but left the reason as an open question.

#### The real finding: the host had no business guessing

      R7 was framed as "measure `i2s_mux`". Asking why it would not connect led
      somewhere better: **`harness.py` carried a hand-maintained table of what
      every chip has, and used it to decide how far to sweep.** `DRIVER_MAXS` said
      `mcpwm_pcnt: 6` and `i2s_mux: 32`.

      **The 6 was accurate, and that is exactly the problem.** The board really
      does allocate six MCPWM/PCNT queues — CONFIG accepts six and refuses the
      seventh with `ERR connect step 6`. What the constant cannot say is how
      many of those six *run*, and on this board **one does**: steppers 2–6 all
      connect and then emit ~21 000 steps where 64 were commanded. A queue count
      is an allocation figure, not a health check, and no host table can hold
      the second number because only the hardware knows it.

      So the sweep ran to `min(channels, DRIVER_MAXS)` = 6 and its summary read
      **"MCPWM reaches 6"** — printed directly above five runaway steppers. Not
      a wrong number in the table; **the wrong question**, bounded by a constant
      that cannot answer it.

      #### What changed

      - **`scale` no longer predicts.** The loop limit is the analyzer's channel
        budget and nothing else; the board decides where it stops, and a CONFIG
        refusal *is* the measured bound. `DRIVER_MAXS` survives only as a
        cross-check that is printed, never obeyed.
      - **A new `DRIVERS` firmware command** reports what the build accepts,
        derived from the *same* conditions `parse_driver()` uses, so a capability
        report cannot disagree with what CONFIG will do. A second copy of those
        `#if`s would be a second thing to keep in sync.
      - **A new `IMUX data,bclk,ws` command** brings the multiplexer up at
        runtime. `initI2sMux()` is once-only, so making it a serial command
        means wiring a mux up is three pins on *existing* firmware rather than a
        recompile — which is what makes the pending measurement a matter of
        naming three pins.
      - **`present` and `up` are reported separately.** `i2s_mux=1 mux_init=0` is
        a wiring gap; `i2s_mux` compiled out is a different build; a driver that
        refuses at connect is a third thing. A reader shown only "i2s_mux
        refused" cannot tell them apart, and would file a wiring gap as a driver
        defect.
      - **The report gained a capability table**, so "what else can this target
        do" is an answer attached to the run rather than something to rediscover
        with the hardware attached.
      - **A board that cannot be asked is an error, never a fallback to the
        table.** Silently reverting to the old belief is how the wrong number
        got believed in the first place.

      *Verified on hardware.* `DRIVERS` on the connected ESP32:
      `rmt=1 rmt_v2=1 mcpwm_pcnt=1 i2s_direct=1 i2s_mux=1 mux_init=0` — five
      drivers compiled in, multiplexer not brought up, exactly the distinction
      that was missing. `mcpwm_pcnt`/`nodir` swept 1..8 and now reports three
      separate facts where it previously reported one:

      | n | outcome |
      |---|---|
      | 1 | **passed** |
      | 2–6 | **failed** — connect, then ~21 000 steps where 64 were commanded |
      | 7–8 | **refused** at CONFIG (`ERR connect step 6`) |

      `rmt_v2`/`nodir` re-measured 1..8: **8/8 passed**, no regression.

      *Tests: 190 pass, from 178.* New coverage for the bound, the `DRIVERS`
      parse and the capability table. *Mutation-checked, all caught:* the host
      table bounding the sweep again, the bound reported as the driver rather
      than the channels, only the first two drivers parsed, `mux_init` always
      reported up, an unanswerable board falling back to an empty set, an `IMUX`
      failure reported as success, a compiled-in-but-down mux not
      distinguished, and a fabricated accepted-driver list.

      Two of those were missed the first time round and both were real: the
      `DRIVERS` tests had been asserting on the **regex** rather than on
      `read_drivers()`, so its own parse of the fields was never exercised. The
      boot-log test is not hypothetical either — the first `DRIVERS` after a
      reset really does arrive behind the ESP32's banner, several hundred bytes
      of unrelated text.

      *Cost:* AVR RAM 1755 → **1845 of 2048** (90.1 %, 203 B free) for `DRIVERS`
      and `IMUX`. The reply buffer is sized from each build's own reply rather
      than borrowed from the CONFIG constant, but the `.rodata` for the new
      strings is the larger part and avr-gcc puts that in RAM. `saleae_avr` is
      the tightest target in the matrix and still fits; noted rather than papered
      over, because the alternative is a driver no AVR build can report on.

- [x] **R8 — re-run and re-baseline. Done.** All 25 wired scenarios re-run on the
      connected ESP32 with the R1 firmware, one cold boot each, Saleae clone at
      24 MS/s: **25/25 accepted by their own evaluators**. `reports/esp32/`
      regenerated from those result JSONs.
      `--dut-driver rmt_v2` was chosen for the `1ch`/`2ch` scenarios
      deliberately: under `SUPPORT_DYNAMIC_ALLOCATION` the old `auto` resolved to
      RMT first, so this is a like-for-like comparison and any regression would
      be attributable rather than confused with a driver change.
      *The re-run is a clean like-for-like:* every one of the 25 step counts is
      **identical** to the previous baseline (SR_02 255, SR_07 2000, SR_08 4000,
      SR_25 stopping at 11475 of 20000, …), all verdicts unchanged, and every
      period within one 24 MS/s sample of the old value. So R1 changed the
      *labelling* of the runs, as intended, and nothing else.
      **What naming the drivers bought, immediately:** the report's pulse-width
      column now separates the three drivers instead of showing one number.
      RMT emits a constant **15.54–15.625 µs** high time, MCPWM/PCNT
      **19.875–99.83 µs**, I2S **1.958–2 µs**. Under `auto` all of the RMT and
      MCPWM rows had been filed under one tag and could not be compared.
      *And it exposed a wrong measurement that had been recorded as a finding* —
      see *Cross-driver start skew* in *Decisions and findings*, which was
      invalidated by this re-run and has been corrected in white paper §5.5.
      *Verify:* no result file contains the tag `auto` — confirmed, 25/25 files
      carry a named driver (`rmt_v2` ×20, `mcpwm_pcnt` ×3, `rmt+mcpwm_pcnt` ×1,
      `i2s_direct` ×1). The three stale `tag_summary/esp32_auto_*.md` pages were
      deleted, since `generate_report.py` writes but does not prune.

      > **Known gap in the re-baseline, left for R5.** `regression.md` compares
      > verdicts and periods only, so it reports "no changes" — while the one
      > number that materially changed in this whole re-run (SR_17's skew,
      > 29.54 → 48.92 µs) is invisible in it. The skew column R5 adds to the
      > sync-permutation table would close this; it is not added here.

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

- **`QUEUES_I2S_DIRECT` is 3; the ESP32 has 2 I2S channels.** Measured
  `--mode scale --driver i2s_direct --pin-mode nodir` on the connected board:
  **n=1 and n=2 pass, n=3..8 are refused**, and the refusal is the IDF's own --
  `E (129) i2s_common: i2s_new_channel(902)`, i.e. `ESP_ERR_NO_MEM` from the
  peripheral rather than a policy limit. `SOC_I2S_NUM` is 2 on the ESP32 and
  `I2sManager::create()` allocates one TX channel per manager
  (`i2s_new_channel(..., &chan, NULL)`, `i2s_manager.cpp:31`), so two is the
  ceiling and the constant overstates it by one.

  This is a *different kind* of wrong from the MCPWM defect below. There the
  constant counted allocations correctly and only the health was bad. Here the
  allocation figure itself is wrong, so anything sizing a queue array from
  `NUM_QUEUES` over-allocates by one. It was found only because R7 stopped
  trusting the constant: `DRIVER_MAXS` had carried `i2s_direct: 3` through every
  previous run, unchanged and untested, because `i2s_direct` had only ever been
  measured as a *partner* in a `sync` combination with a stepper on another
  driver. The failure mode is graceful -- `I2sManager::create()` returns nullptr
  and the harness reports `refused` with the peripheral's own error -- so this is
  a capacity and documentation bug, not a crash. Left as a library change; the
  harness's job was to find and record it.

- **`i2s_direct` on the full catalogue: 23 passed, 2 skipped. Two of the three
  original failures were harness bugs; the third was withdrawn.**
  The 25 wired scenarios had only ever been run on `rmt_v2`, so this is the
  first characterization of the I2S step/dir waveform. **Two** of the three
  original failures were harness bugs (below); the two that remain are driver
  behaviour no scenario on RMT could have surfaced.

  **1. ~~The driver silently discards short entries.~~ Wrong: the harness did.**
  This was first written up as a driver defect and that was incorrect. The
  driver is right and the firmware is right:

  - `addQueueEntry()` bounds the whole command -- `ticks * steps >= MIN_CMD_TICKS`
    -- and returns `AQE_ERROR_TICKS_TOO_LOW` (`queue_add_entry.cpp:46`).
  - The firmware surfaces it: `ERR QE step0 rc=-1`.

  Three harness bugs, all real:

  1. **`sc_pulse_high_time`, `sc_pause`, `sc_long_run` bypassed `legal_ticks()`**,
     using `max(max_speed_ticks, 160)` instead. `addQueueEntry` bounds the
     *command*, so 16 steps need `ticks*16 >= 3200`. `rmt_v2`'s floor is 640, so
     the expression gave 10240 and the scenario passed; `i2s_direct`'s floor is
     80, so it gave 2560 and the queue rejected it. **One expression, two
     drivers, and the difference was invisible until a driver with a lower floor
     was tried.** All three now use `legal_ticks(info, steps, ...)`.
  2. **The rejection is asynchronous, so `program()`'s check could not see it.**
     `QSEG` replies `OK QSEG` after *parsing* only; the real `addQueueEntry()`
     runs in `qe_feed()` from `qe_pump()`, in the main loop, *after* `QRUN`. The
     error therefore lands in the post-run drain, and the harness went on to
     measure.
  3. **The post-run `ERR QE` was read as a measurement.** So SR_05 was recorded
     as "the pin emitted 0 of 16 steps" -- a statement about the hardware that
     was simply false. The pin carries every legal move perfectly.

  `measure()` now treats a post-run `ERR QE` as a **setup failure**, and
  `program()` refuses an entry the queue must reject *before* a capture is
  spent. SR_05 and SR_09 re-run: **both pass.** The general guard is a test that
  every scenario is programmable against three QINFO shapes whose
  `max_speed_ticks` differ by 8x, plus one that asserts the shape set has not
  collapsed to a single floor.

  `REJECTION_SCENARIOS` names the one scenario whose subject *is* a rejection
  (SR_13), so exempting it from both checks is a recorded decision rather than a
  carve-out that could quietly widen. It also broke twice on the way: the
  exemption read `test_id`, which is not a parameter of `measure()`, and all 199
  tests passed because they drove the helpers rather than `measure()` itself. It
  is now exercised directly.

  **2. A step appears on the wrong side of a direction change** (SR_12) --
  **still open, and now itself suspect.**
  30/30 steps, 2 dir edges and a correct final dir, but the steps land
  **10 / 9 / 11** across the three phases instead of 10/10/10. One step of a
  phase is attributed to its neighbour -- precisely the dir-to-step ordering
  this harness exists to check. The total count and the per-phase count
  disagree, and only the second one notices.

  **3. ~~`STOP` does not stop an I2S queue.~~ Withdrawn, and the scenario behind
  it was measuring a thing that does not exist.**

  The harness's `STOP` was `stopMove()` **plus zeroing the feeder cursor** -- a
  hybrid matching neither documented behaviour. The library has three, and the
  harness was conflating the first two:

  | API | contract |
  |---|---|
  | `stopMove()` | a flag for the ramp's **next** command. Must **not** truncate queued motion. |
  | `forceStop()` | `ignore_commands = true`; nothing further *added*, queue drains (~20 ms). |
  | `forceStopAndNewPosition()` | aborts everything queued -- no further step issued. |

  So the number reported earlier -- 7655 steps left on `i2s_direct`, 7608 on
  `rmt_v2`, both just under the 8160 a 32-deep queue of 255-step commands holds
  -- was **the harness's own arithmetic, not a library guarantee.** Nothing in
  the library promises it.

  `STOP` is now `stopMove()` alone, and `ESTOP` is `forceStop()`. Because the
  expected outcomes are *opposite*, they are two scenarios rather than one with a
  flag; `CONTRASTING_PAIRS` declares them so the anti-duplication test accepts the
  shared waveform. SR_25 asserts `stopMove()` did **not** truncate; SR_29
  asserts `forceStop()` drained within the queue bound.

  ### What made any of it measurable: a marker channel

  None of the above could be established before, because the stop instant was
  inferred from "the pulses ceased" and the capture this rig *delivers* is not the
  capture it *requests* (24 MHz truncates). Measured: both drivers had their last
  step landing exactly on the capture edge, 0.0 ms of quiet tail, so
  `truncated = step_count < requested` was satisfied by the recording ending.

  A new `MARK <ch>` command designates an analyzer channel **no stepper owns**,
  and the firmware flips its level when it processes a stop, so the instant is on
  the waveform. `MAP` reports `marker=`; it is refused rather than silently
  overwriting a step pin, and at 8 steppers in `nodir` there is no free channel --
  a real limit of the approach. The level alternates per event, so no
  sub-millisecond delay primitive is needed (an ESP-IDF busy-wait would block the
  very loop that drains the queue).

  *Placement was load-bearing.* Sent after `QRUN` it cost two serial round-trips
  (~0.25 s each) before the stop, so on any driver whose move was shorter than
  that the stop arrived after the move had finished and the marker edge fell past
  the end of the delivered capture -- reported as "STOP was never processed".
  `MARK` is configuration and belongs in the setup phase.

  *Verified on hardware*, rmt_v2, same program, opposite assertions:

  | | steps emitted | after the stop marker | verdict |
  |---|---|---|---|
  | SR_25 `stopMove()` | **20000 / 20000** | 8099 | **passed** -- did not truncate, as required |
  | SR_29 `forceStop()` | 18615 / 20000 | 7464 (bound 8160) | **passed** -- stopped adding |

  *Cost:* AVR RAM 1845 -> **1912 of 2048** (93.4 %). Two literals in `MARK` rather
  than formatted errors, because avr-gcc copies `.rodata` into RAM and the first
  version of that function took the build to 1992 of 2048.

- **`QUEUES_I2S_DIRECT` is 3; the ESP32 has 2 I2S channels.** Measured
  `--mode scale --driver i2s_direct --pin-mode nodir` on the connected board:
  **n=1 and n=2 pass, n=3..8 are refused**, and the refusal is the IDF's own --
  `E (129) i2s_common: i2s_new_channel(902)`, i.e. `ESP_ERR_NO_MEM` from the
  peripheral rather than a policy limit. `SOC_I2S_NUM` is 2 on the ESP32 and
  `I2sManager::create()` allocates one TX channel per manager
  (`i2s_new_channel(..., &chan, NULL)`, `i2s_manager.cpp:31`), so two is the
  ceiling and the constant overstates it by one.

  This is a *different kind* of wrong from the MCPWM defect below. There the
  constant counted allocations correctly and only the health was bad. Here the
  allocation figure itself is wrong, so anything sizing a queue array from
  `NUM_QUEUES` over-allocates by one. It was found only because R7 stopped
  trusting the constant: `DRIVER_MAXS` had carried `i2s_direct: 3` through every
  previous run, unchanged and untested, because `i2s_direct` had only ever been
  measured as a *partner* in a `sync` combination with a stepper on another
  driver. The failure mode is graceful -- `I2sManager::create()` returns nullptr
  and the harness reports `refused` with the peripheral's own error -- so this is
  a capacity and documentation bug, not a crash. Left as a library change; the
  harness's job was to find and record it.

- **`i2s_direct` on the full catalogue: 23 passed, 2 skipped. Two of the three
  original failures were harness bugs; one further finding is withdrawn.**
  The 25 wired scenarios had only ever been run on `rmt_v2`, so this is the
  first characterization of the I2S step/dir waveform. **Two** of the three
  original failures were harness bugs (below); the two that remain are driver
  behaviour no scenario on RMT could have surfaced.

  **1. ~~The driver silently discards short entries.~~ Wrong: the harness did.**
  This was first written up as a driver defect and that was incorrect. The
  driver is right and the firmware is right:

  - `addQueueEntry()` bounds the whole command -- `ticks * steps >= MIN_CMD_TICKS`
    -- and returns `AQE_ERROR_TICKS_TOO_LOW` (`queue_add_entry.cpp:46`).
  - The firmware surfaces it: `ERR QE step0 rc=-1`.

  Three harness bugs, all real:

  1. **`sc_pulse_high_time`, `sc_pause`, `sc_long_run` bypassed `legal_ticks()`**,
     using `max(max_speed_ticks, 160)` instead. `addQueueEntry` bounds the
     *command*, so 16 steps need `ticks*16 >= 3200`. `rmt_v2`'s floor is 640, so
     the expression gave 10240 and the scenario passed; `i2s_direct`'s floor is
     80, so it gave 2560 and the queue rejected it. **One expression, two
     drivers, and the difference was invisible until a driver with a lower floor
     was tried.** All three now use `legal_ticks(info, steps, ...)`.
  2. **The rejection is asynchronous, so `program()`'s check could not see it.**
     `QSEG` replies `OK QSEG` after *parsing* only; the real `addQueueEntry()`
     runs in `qe_feed()` from `qe_pump()`, in the main loop, *after* `QRUN`. The
     error therefore lands in the post-run drain, and the harness went on to
     measure.
  3. **The post-run `ERR QE` was read as a measurement.** So SR_05 was recorded
     as "the pin emitted 0 of 16 steps" -- a statement about the hardware that
     was simply false. The pin carries every legal move perfectly.

  `measure()` now treats a post-run `ERR QE` as a **setup failure**, and
  `program()` refuses an entry the queue must reject *before* a capture is
  spent. SR_05 and SR_09 re-run: **both pass.** The general guard is a test that
  every scenario is programmable against three QINFO shapes whose
  `max_speed_ticks` differ by 8x, plus one that asserts the shape set has not
  collapsed to a single floor.

  `REJECTION_SCENARIOS` names the one scenario whose subject *is* a rejection
  (SR_13), so exempting it from both checks is a recorded decision rather than a
  carve-out that could quietly widen. It also broke twice on the way: the
  exemption read `test_id`, which is not a parameter of `measure()`, and all 199
  tests passed because they drove the helpers rather than `measure()` itself. It
  is now exercised directly.

  **2. A step appears on the wrong side of a direction change** (SR_12) --
  **still open, and now itself suspect.**
  30/30 steps, 2 dir edges and a correct final dir, but the steps land
  **10 / 9 / 11** across the three phases instead of 10/10/10. One step of a
  phase is attributed to its neighbour -- precisely the dir-to-step ordering
  this harness exists to check. The total count and the per-phase count
  disagree, and only the second one notices.

  **3. ~~`STOP` does not stop an I2S queue.~~ Withdrawn: never established.**
  Written up from a `run_tests.py` catalogue run in which all 20 000 steps came
  out. Three separate harness problems stacked up, and the conclusion was not
  available from that run:

  - **`measure()` never issued STOP at all.** The only `STOP` in `run_tests.py`
    was SR_00's cleanup; `STOP_AFTER` was honoured solely by `run_hardware.py`.
    So a catalogue run through the main orchestrator never sent one, on any
    driver — SR_25 asserts "pulses cease when STOP is issued" while no STOP was
    issued. Now fixed: `measure()` issues it.
  - **The stop time was a fixed 0.15 s, not derived from the program.** Replaced
    with `stop_after_for()`: a quarter of the run, capped by `STOP_AFTER` and
    floored so a stop cannot precede the start.
  - **Even with STOP issued, the test cannot tell a stopped move from a capture
    that ended first.** Measured on *both* drivers, identically:

    | driver | capture | first pulse | last pulse | quiet tail |
    |---|---|---|---|---|
    | `i2s_direct` | 458.9 ms | 280.6 ms | 458.9 ms | **0.0 ms** |
    | `rmt_v2` | 458.2 ms | 282.4 ms | 458.2 ms | **0.0 ms** |

    The pulses run to the capture edge on both, because 24 MHz truncates the
    requested 0.7 s to ~458 ms while the move needs ~281 ms of startup plus its
    own duration. So `truncated = step_count < requested` was satisfied by the
    capture ending, not by anything stopping. It was evidence of nothing.

  **On the mechanism, which is answerable from the code rather than a run:**
  `stop_all()` calls `stepper->stopMove()` and then `memset(&slots[i].cur, 0)`
  and `clear_programs()`. So queue *filling* is cancelled immediately — the
  feeder cursor is zeroed and the segment program dropped, so `qe_feed()` stops
  calling `addQueueEntry()`. What is **not** cancelled is what is already queued:
  the library's `stopMove()` only sets a flag consulted when the queue asks for
  its *next* command, so queued commands still emit. The truncation point is
  therefore however much was prefilled, not when STOP arrived.

  And the feeder runs *far* ahead of the driver — it is pumped from the main
  loop — so on a driver that takes ~281 ms to start emitting, the whole program
  can be queued before the first pulse. Then STOP has nothing left to cancel and
  the full program runs, which is the documented library behaviour and not a
  defect.

  **What SR_25 needs before it can answer the question:** a capture that is
  *measured* to outlast the move rather than requested to (`capture.py` already
  warns that 24 MHz truncates, and `--strict` fails on it); a STOP issued after
  the driver's startup latency rather than at a fixed offset from QRUN; and a
  `stopped_early` test in absolute time rather than as a fraction of a capture
  whose length is not under the harness's control. Until then it reports a
  number and calls it truncation.


- [x] **Two MCPWM/PCNT queues on one ESP32 do not work: the second stepper
      emits continuously and never stops.** Found by `--mode scale` on its first
      hardware run, which is the argument for having the mode at all — SR_16
      asks whether a *second* stepper perturbs the first and would report
      "perturbed", not "runaway".

      `CONFIG 2 mcpwm_pcnt,mcpwm_pcnt dir`, `QSEG 64 160 1`, `QRUN 3`: stepper A
      emits exactly 64 steps. Stepper B emits **22 143** rising edges at exactly
      the commanded 10 µs period (5.0 µs high, 5.0 µs low) and never stops. The
      board's own `POS` reads **non-monotonic** — `64 31`, `64 26`, `64 32`,
      `64 50` — all below 64, which says the *position counter* is being re-read
      while the pin runs on rather than that steps were lost. So the capture and
      the firmware disagree about what happened, and the capture is right.

      **It is two MCPWM/PCNT queues, not MCPWM, and not `nodir`:**

      | configuration | `POS` sampled over 1.5 s |
      |---|---|
      | `rmt+rmt` | `64 64` every time |
      | `rmt+mcpwm_pcnt` | `64 64` every time |
      | `mcpwm_pcnt+rmt` (order swapped) | `64 64` every time |
      | `mcpwm_pcnt+mcpwm_pcnt` | `64 31 / 64 26 / 64 32 / 64 50 / 64 34` |
      | `mcpwm_pcnt+mcpwm_pcnt` at 3200 ticks (200 µs) | fails too — not a speed limit |

      Reproduced on the **unmodified** firmware (stashed and re-flashed), so it
      is a library defect and not an artefact of the mode or of the `nodir`
      change. Also fails with only stepper B selected (`QRUN 2`), which rules
      out the synchronized kick-off as the trigger — **configuring a second
      MCPWM/PCNT queue is enough**, no `QRUN` needed.

      *Where to look:* `StepperISR_idf5_esp32_mcpwm_pcnt.cpp` indexes
      `channel2mapping[NUM_QUEUES]` and `pcnt_unit_to_queue[QUEUES_MCPWM_PCNT]`
      by `channel_num` and sets `pcnt_unit_id = timer_num`, while the ESP32 has
      **4 MCPWM timers** (2 groups × 2) and `QUEUES_MCPWM_PCNT` is **6**. The
      second queue's timer/PCNT assignment is the first thing to check.

      **Consequence for the harness's own tables:** `DRIVER_MAXS` records
      MCPWM/PCNT as 6 queues on IDF 5, from `pd_config_idf5.h`, and **that
      number is now known to be wrong** — not in the count but in what the
      driver can actually do. Left unfixed here: the library is outside R3's
      scope and R3's job was to surface it. Until it is fixed, any MCPWM/PCNT
      row in the R5 tables is a measurement of a defect, not of the driver.
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
- [x] **Cross-driver start skew *is* worse than same-driver — and the earlier
      finding that said otherwise was an artefact of the firmware's `mixed`
      config, not a property of the hardware.** Two `rmt` steppers start
      **29.5417 µs** apart (0.7385 step periods); one RMT and one MCPWM/PCNT
      start **49.0 µs** apart (**1.2250** periods), i.e. ~66 % worse, and
      reproducibly so (three consecutive runs each, to within one sample at
      24 MS/s). The paper's §5.2 premise — two drivers arming through unrelated
      hardware must diverge — is what the hardware does.
      **How the wrong number was produced, because it is the most interesting
      thing here.** The old firmware's `mixed` channel config parsed its
      per-stepper driver list into `drivers[]` and then, three lines later,
      overwrote every entry with the automatic driver choice:

      ```c
      if (fill == SA_AUTO && n > 1) {
        for (uint8_t i = 0; i < n; i++) drivers[i] = SA_AUTO;   // <- discarded the list
      }
      ```

      `fill` was only non-`SA_AUTO` for `4ch_rmt`/`4ch_mcpwm`, so `CONFIG mixed
      rmt,mcpwm` was byte-for-byte equivalent to `CONFIG 2ch`: both steppers on
      the automatic choice, which under `SUPPORT_DYNAMIC_ALLOCATION` is RMT.
      **SR_17 never crossed drivers at all.** It reported 29.5417 µs — the
      same-driver number, because it *was* the same-driver run, agreeing to a
      sample at 24 MS/s. That absurd precision is what gave it away: two
      independent hardware paths do not agree to four decimal places.
      Verified by flashing the pre-R1 firmware and re-running: SR_17 came back
      at 29.5417 µs again, and SR_14 at 29.5417 µs, on the old build.
      **Consequence for the baseline:** the committed `reports/esp32/` SR_17
      entry is labelled `rmt+mcpwm_pcnt` but measured RMT+RMT. R8 replaces it.
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
