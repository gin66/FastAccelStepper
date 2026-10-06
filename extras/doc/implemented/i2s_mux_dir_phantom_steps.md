# 023 — `i2s_mux` in `dir`: the harness counted the capture, not the move

Closes todo **023** (`extras/todo/023_i2s_mux_dir_phantom_steps_at_24ms.md`).

The finding the todo was filed on is unchanged and was never in doubt: 24 MS/s
over an 8 MHz bclk is three samples per bit period, `dir` holds the direction
bits high so the data line toggles in every frame, and the decode races with
those transitions. Its three reasons are still the reasons — no code path in
`src/` can set the bit that appeared, the pulses start 285 ms before the move,
and the same wire decodes differently one sample earlier.

What changed is what the harness does about it. The decode is still not
trustworthy frame by frame, and it still cannot be made so on this analyzer
(48 MS/s truncates an eight-channel capture to 0.18 ms). So the harness stopped
asking the whole capture whether the driver emitted the right number of steps,
and started asking **the commanded move**, which is the thing it knows exactly.

## What was wrong

Every evaluator counted `channel_metrics(...)` over the *entire* capture. A
capture is not the move: it starts before the test is triggered over serial and
outlives it, so it holds however long the host took to send the command and
however long the board idled afterwards. Nothing the driver did is in that
stretch, and counting it reported a driver emitting steps it did not emit —
the one failure this harness exists to catch.

The recorded IDF 5.5.3 capture, `--mode sync --imux --pin-mode dir`, two
`i2s_mux` steppers:

```
A ch=S0 ticks=400  steps 67/64  extra 3   off-grid 178229 / 64106 / 112553 us
B ch=S2 ticks=800  steps 64/64
```

Three pulses where the plan has none: one 242 ms *before* the move, one 64 ms
before it, one 114 ms after it — the off-grid numbers are the gaps to them, and
the first is also what made `first_step_skew_us` read **242 334 us** (44.9 step
periods), because stepper A's "first step" was that phantom. Both steppers'
moves in fact start on the same sample, which is what the run should have said.

The edge-1 sample point (todo 184's fix, `da66a6bd`) had already cut the
phantom count from 51 to 3, so what was left was the harness's arithmetic
rather than the decoder's phase.

## The fix: a move window

`run_tests.move_window()` and the two helpers it rests on. The plan fixes the
move's timeline exactly — `addQueueEntry()` is driven directly, so no ramp
stretches it — which means the program says how many steps come out and how far
apart they are:

- `commanded_timeline(segments, info, limit)` derives the offset of every
  commanded step in ticks. A segment of `steps` at `ticks` occupies
  `steps * ticks`, a pause (`steps == 0`) occupies `ticks`, and steps land at the
  **start** of their own tick — which is what makes a scenario's duration
  `steps * ticks` and not `(steps - 1) * ticks`. The gaps between consecutive
  steps therefore come out per-gap, not as one constant: the step after a pause
  is one pause plus one period away. A window placed by a single period would
  not find any of the four scenarios that program a pause (SR_09, SR_18, SR_20,
  SR_26).
- `gap_is_legal()` decides whether a measured gap is one the command could have
  produced: on a grid (the mux) a whole number of frames within a quarter frame,
  derived exactly as `grid_period_defects()` derives it; off a grid a quarter of
  the commanded gap. It is deliberately **looser** than the rule that judges
  periods — it locates, that one judges — because it has to tell a gap the
  command could have produced from the gap to a pulse that was never commanded,
  which are orders of magnitude apart, and it must not refuse to anchor a driver
  whose steps arrive late.
- `move_window(edges, …)` takes the **first contiguous run of `expected` steps
  whose gaps all match the command**, and reports every step outside it. Three
  gaps decide the search — first, middle, last — so a window beginning on an
  unexplained pulse is rejected in constant time; only the one candidate that
  can be the move is checked gap by gap.

`moved_metrics()` routes the evaluators through it: the step count and the
inter-step periods come from the window, while the pin's own numbers (pulse
widths, duty, idle level) stay properties of the whole capture, because they
are — a step is as wide before the move as during it. The first-step skew is
now `skew_of_first_steps()` over the windowed first steps.

### The rules that keep it from being a way to pass a bad run

Each of these is a test in `TestMoveWindow`, and each is the failure mode the
window could otherwise have introduced:

| | |
|---|---|
| **A dropped step is still a defect** | 39 edges where 40 were commanded: no window of 40 exists, so nothing anchors and the capture is measured whole. |
| **An extra step next to the move is still a defect** | The window spans to the end of the last commanded *pulse*, so a step inside that span is counted. |
| **A driver at the wrong rate cannot place its move** | No run of gaps matches, so nothing anchors, the whole capture is measured and the rate error is reported. A window loose enough to place this would be a window that can hide a rate error. |
| **A clean capture measures exactly as before** | Nothing to trim, so the window is the whole capture and every number is what it always was. |
| **`nodir`'s zero-step assertion is untouched** | SR_13 is the one scenario about absence; a pulse anywhere in that capture is still the failure. Same for the marker-relative SR_25/SR_30. |

The one still-open item this could have masked is
the MCPWM/PCNT overrun (`../platforms/esp32.md#mcpwm-pcnt-overrun`) — `mcpwm_pcnt`
emitting more steps than were commanded in `sync`, at the commanded period. Those
extra steps
land inside the move's span (the span ends one commanded tick after the last
step's edge, and one more period is further) and are counted, which is the case
`test_an_extra_step_next_to_the_move_is_still_a_defect` pins.

### The one case it cannot decide

A pulse exactly one commanded period ahead of the move's first step has the same
gaps behind it as the move, so nothing in the waveform tells the two apart. The
window takes the earliest candidate, absorbs it, and the move then holds one step
more than the plan commands — **which fails the run**. That is the intended
outcome: the capture holds a pulse the harness cannot account for, and a run
that has seen one does not report the driver as clean on the strength of which of
two equally-plausible readings it happened to prefer. A pulse further out, whose
gap to the move is not one the command could produce, is outside the window and
is recorded.

### Nothing is discarded

Every result carries the window:

```json
"window": {"expected_steps": 64, "measured_steps": 67, "anchored": true,
           "first_step_sample": 7019745, "window_start_us": 292489.375,
           "window_end_us": 294085.0417, "commanded_span_us": 1575.0,
           "steps_in_window": 64, "steps_outside": 3,
           "outside_before_us": [-242334.9167, -64105.75],
           "outside_after_us": [114123.4167]}
```

The trimmed pulses keep their offsets from the move's own start, so a pre-move
pulse reads as how far before the move it landed. That is the evidence which
distinguishes a sampling race from a driver defect, and it is the number a later
run is compared against.

## Measured on hardware

Re-measured with the board and analyzer attached, `--mode sync --arch esp32
--framework idf --pin-mode dir --imux`, one sweep per I2S SDK:

| SDK | `i2s_direct+i2s_mux` | `mcpwm_pcnt+i2s_mux` | `rmt+i2s_mux` | `i2s_mux+i2s_mux` |
|---|---|---|---|---|
| IDF 5.5.3 (`esp32_idf_V6_13_0`) | **passed** | **passed** | **passed** | **passed** |
| IDF 6.1.0 (`esp32_idf_V7_1_2`) | **passed** | **passed** | **passed** | **passed** |

All eight were red before. Every one of them is now `passed` with each stepper at
64/64, the mean periods on their own commands (24.9319 / 49.9266 µs on
`i2s_mux+i2s_mux`, 24.9788 / 49.9259 µs where stepper A is a GPIO channel), and
`first_step_skew_us` **0.0** on both `i2s_mux+i2s_mux` rows against 242 334 µs.
The non-mux combinations pass in the same sweeps, so the fix did not move them.

The pulses that were set aside, one to three per capture, and how far from the
move each landed:

| SDK | row | frame faults | trimmed (µs from the move's start) |
|---|---|---|---|
| IDF 5.5.3 | `i2s_direct+i2s_mux` | 100 | B −167 950 |
| IDF 5.5.3 | `mcpwm_pcnt+i2s_mux` | 102 | B −4 552 |
| IDF 5.5.3 | `rmt+i2s_mux` | 103 | B −99 432 |
| IDF 5.5.3 | `i2s_mux+i2s_mux` | 98 | A −253 665, A +65 189, B +205 710 |
| IDF 6.1.0 | `i2s_direct+i2s_mux` | 102 | B +103 588 |
| IDF 6.1.0 | `mcpwm_pcnt+i2s_mux` | 101 | B +100 311 |
| IDF 6.1.0 | `rmt+i2s_mux` | 100 | B +150 613 |
| IDF 6.1.0 | `i2s_mux+i2s_mux` | 101 | A −160 076, B −73 514, B +75 288 |

The smallest margin between a trimmed pulse and the move is 4.5 ms (B, IDF 5.5.3
`mcpwm_pcnt+i2s_mux`), against a move 1.575 ms long — three orders of magnitude
of capture, so nothing here is a judgement call about where the move ends.

### `i2s_mux` at 32 steppers, measured

`SR_31` on `i2s_mux`, ESP-IDF 5.5.3, `nodir`: the board accepted **32** on the
**first** probe attempt (no refusals above it — 32 is the slot budget), and all
32 steppers measured **64/64 steps at a mean period of 24.9306 µs, spread
0.0000 µs, zero off-grid**, with `POS 64 64 … ` (32 values) agreeing.
`frame_faults` 102, recorded. The intermittent dropped pulse
(`r7_virtual_i2s_mux.md` §5) did not appear in this run, so this is one clean
measurement rather than a rate — but it is the measurement the row was missing,
and it is no longer blocked by the sampling race.

### One row is red on IDF 5.5.3, and it is not this item

`sync mcpwm_pcnt+i2s_direct` failed **3 sweeps in 3** on IDF 5.5.3 with stepper
A (`mcpwm_pcnt`, D0, 160 ticks) emitting **65 steps where 64 were commanded**:
the extra pulse is the last one, one commanded period after the run, every
inter-step period legal, `POS 64 64`. That is
the [MCPWM/PCNT overrun](../platforms/esp32.md#mcpwm-pcnt-overrun), which was open
before this item was started and which the hardware run reproduced three times,
and which was later verified to be the driver deducting the overrun from the next
command (a system limitation, not a defect). It is recorded there.

It also found a knife-edge in this item's own window, which the offline
re-evaluation could not: at 24 MS/s the extra pulse lands on the tick boundary
to within a sample, so a window cut at "the last commanded pulse ends" would
have set it aside half the time and reported the driver as clean. **The window
now extends through any pulse whose gap from the one before it is one the command
could have produced** — the anchor's own rule, applied forwards — so a driver
that keeps stepping after the run is counted, while a pulse 12 ms later still is
not. Two tests, and the capture above is the reason they exist.

## Measured offline: every recorded capture re-evaluated

Before the hardware was attached, every result in `results/` with a capture on
disk was re-run through the current `evaluate()` and compared with the recorded
verdict. This is what established that the change was sound before anything was
re-measured, and it is kept because the evidence is still what it was:

- **212 SR catalogue results across the six matrix firmwares: 212 unchanged, 0
  changed.** No scenario's verdict moved, including all 26 per-row catalogues
  and every SR that programs a pause, two steppers or 4000 steps.
- **16 `scale` `i2s_mux` results: 16 unchanged, 0 changed.** The intermittent
  dropped pulse is not masked by this: a swallowed step has no window to anchor
  on.
- **28 `sync` results across the other driver combinations: unchanged**, and the
  seven mux `dir` rows that were red now evaluated as passed — which is what the
  hardware runs above then confirmed.

## The decoder's own diagnostics now reach the result

The other half of the todo: `extract_frames()` has always computed the
frame-alignment faults (`report_faults=True`) and `decode()` **discarded** them.
So a run could not say the capture was at fault, and reported a driver
defect instead.

`decode()` and `decode_channels()` take a `faults` sink, `decode_mux_capture()`
passes one, and the run's result carries `decoded_from.frame_faults` plus
examples. **98 to 103 on every 24 MS/s `dir` capture measured**, of which 99 %
are ws edges that do not land on a frame boundary (a ws fall off a word
boundary, a ws rise off the half-frame) and the rest one trailing partial frame.
One capture in 1250 decodes wrong, and the run says so next to the step count
instead of the step count being the only evidence and reading as a driver.

The `nodir` `SR_31` capture at 32 steppers is the interesting counter-example:
**102 faults** there too, with the data line idle-low and no transitions to race
with. So the faults belong to 24 MS/s over an 8 MHz bclk and not to `dir` —
which is a second reason they are recorded and never gated: a threshold high
enough to be meaningful is a threshold every mux run at this rate fails.

**Recorded, not gated**, for the reason above. The zero case is a test too — a
clean bus must still report zero, or the count would say nothing.

The todo's third option — reject a mux frame that contradicts its neighbours — was
**not** taken, and the reason is the same as for the sample point: it would hide
the rate problem rather than fix it, and now that the count travels with the
result there is nothing left for it to buy.

## What is still open

- **The intermittent dropped pulse** (`r7_virtual_i2s_mux.md` §5). It did not
  appear in the `SR_31` measurement at n = 32, so the 32-stepper row is now green
  once; a dropped step is still caught by the window, because a swallowed step
  leaves no run of gaps to anchor on. So it bounds how often that row goes red,
  not whether a real defect would be seen.
- **The 24 MS/s race is not fixed, and cannot be on this analyzer.** What is
  fixed is the harness's ability to be wrong about it. 48 MS/s gives six samples
  per bit period and truncates an eight-channel capture to 0.18 ms, which is
  shorter than any scenario.
- **The matrix report rows** (`reports/esp32_platform_matrix.md`) still carry the
  pre-fix numbers for these two I2S rows; the *results* behind them are current
  now, on both SDKs. `run_matrix.py --targets idf-6.13.0 --force` regenerates
  the table.

## References

- Harness guide: `extras/tests/saleae_based/AGENTS.md` (§ *The mux in `dir`
  mode*, § *The I2S mux*)
- Original finding, reproduced verbatim at the top:
  `extras/doc/implemented/r7_virtual_i2s_mux.md`
- Design: `white_paper_saleae_test_harness.md` §2.2, §7.4