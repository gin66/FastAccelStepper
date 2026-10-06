# 181 mcpwm_pcnt emits more steps than were commanded, in `sync`

## Priority

**MEDIUM** — an intermittent extra-step defect on one driver of one SDK, in a
scenario that otherwise passes. It does not stop the firmware and it is not
visible to any other test, so it will be re-found rather than fixed.

## Finding

`--mode sync --arch esp32 --framework idf --version 6.13.0 --pin-mode dir
--sync-count 2 --imux`, ESP-IDF **5.5.3**, stepper A on `mcpwm_pcnt`:

```
A ch= D0 ticks= 160
   steps: {'steps_expected': 64, 'steps_measured': 67, 'missing_steps': 0,
           'extra_steps': 3, 'ok': False}
   period: {'expected_period_us': 10.0, 'tolerance_us': 0.5,
            'periods_measured': 66, 'short_periods_us': [],
            'long_periods_us': [], 'n_short': 0, 'n_long': 0, 'ok': True}
   mean_period_us: 10.0
```

67 steps where 64 were commanded. **Zero missing steps, and the period is
exact** — 66 inter-step periods, none short, none long, mean exactly 10.0 us.
So this is not a stepper running fast and not a lost pulse: it is three extra
pulses at precisely the right spacing, which is the signature of hardware that
kept emitting after the queue should have been done.

Stepper B (`i2s_direct` on D2, 320 ticks) was exact, 64/64 at 20.0 us, in the
same capture. Nothing else in the run failed.

## What has been ruled out

- **Not the IDF version.** IDF 6.1.0 (`esp32_idf_V7_1_2`), same combination,
  same firmware logic: A is 64/64, period 9.9907 us. Only 5.5.3 shows it.
- **Not a decoding artefact.** The period statistics come from the same
  evaluator and are clean; 3 is not a plausible miscount of a 64-step grid at
  24 MS/s on a GPIO pin.
- **Not the feeder double-issuing.** `qe_feed()` decrements `left` only on
  `rc == OK`, so the retryable DIR-drain path re-pumps rather than re-counts a
  segment (`common/saleae_app.cpp`). A retry cannot inflate the count.
- **Not the idle loop.** This was chased: `saleae_hal_idle()` was `vTaskDelay(1)`
  and 2 runs in 2 failed; it was reverted to the sub-tick spin and 3 further runs
  gave 2 pass / 1 fail. Intermittent either way, so the idle implementation is
  not the cause — see `doc/implemented/idf55_main_task_stack_overflow.md`. The
  spin is what stayed, which is also what the whole matrix was measured with.

## Rate

Roughly **1 failure in 3** on `sync mcpwm_pcnt+i2s_direct` at IDF 5.5.3. Enough
to be worth chasing, rare enough that a single passing run proves nothing — the
same trap the mux dropped-step finding fell into.

## What to find

- **Does it need `sync`?** Every observation is a two-stepper `dir` run at
  160/320 ticks. `--mode scale --driver mcpwm_pcnt --pin-mode nodir` has been
  green n = 1…6 repeatedly on this SDK. Whether a single stepper, or `nodir`,
  or a longer program reproduces it narrows this from "sync" to "mcpwm_pcnt at
  ~10 us". 10 us is the fastest period in the catalogue, so a trailing-pulse
  hypothesis predicts it should get *worse* at higher speed, not better.
- **Where are the 3 steps?** Relative to the marker edge, to stepper B's last
  step, and to the end of the capture. This distinguishes "hardware kept
  emitting" from "the queue was fed more than the program says", and SR_25 /
  SR_30 already measure exactly that quantity for a stopped run — reuse their
  evaluator rather than writing another one.
- **MCPWM compare units and PCNT are independent of the queue.** RMT and
  MCPWM/PCNT each take one queue entry at a time with nothing in flight, which
  is why SR_30 measures 0 leaked steps for them. If that invariant does not
  quite hold for a multi-compare MCPWM timer at the fastest period, that is the
  defect, and it belongs in the MCPWM/PCNT section of
  `extras/doc/platforms/esp32.md` next to the two existing invariants.
- Note the contrast with the I2S leak, which is understood and documented:
  I2S streams from a DMA buffer and leaks 67 of 4080 steps after `XSTOP`.
  If mcpwm_pcnt's extra steps turn out to be the same "already handed to
  hardware" effect, it is a much smaller absolute number and belongs with that
  finding rather than as a separate mechanism.

## Note

Found while closing [015](../doc/implemented/idf55_main_task_stack_overflow.md)
and 016. The stack overflow that was the subject of those items was real, in
the harness, and fixed; this is a separate defect that the same acceptance run
walked past, and it is recorded here rather than left in a paragraph of a
resolved item where it would have looked settled.
## 2026-10-05 — did not reproduce in the full matrix

One clean run of every combination across all six rows
(`reports/esp32_platform_matrix.md`), including `mcpwm_pcnt+i2s_direct` on both
I2S SDKs: **16 measurements, all exact.** No extra steps, no missing steps,
every stepper's period on grid.

So the rate is somewhere below 1-in-16, against roughly 1-in-3 measured during
the acceptance work that found it. That is a real reduction in weight but not a
fix, and it is worth being precise about why: nothing was changed in the driver
between those measurements, so the difference is more likely the sample than the
code. A defect that appears in a third of runs and in none of sixteen is the
same defect as before, described with a worse denominator.

The useful new datum is that the *good* case is exactly right — 64/64 on both
SDKs, periods on grid, no drift — so this is not a boundary condition that
sometimes resolves. Something occasionally emits three extra pulses at precisely
the commanded spacing, and otherwise the driver is exact.

## 2026-10-06 — 3 failures in 3 runs, one extra step, always the last one

Re-measured while closing [023](../doc/implemented/i2s_mux_dir_phantom_steps.md),
because the run that verifies 023's fix is the run that lands on this row. Three
back-to-back `--mode sync … --pin-mode dir --imux` sweeps on the same board,
ESP-IDF **5.5.3**: `mcpwm_pcnt+i2s_direct` failed **3 times in 3**. Both other
`mcpwm_pcnt` combinations in each of those sweeps (`rmt+mcpwm_pcnt`,
`mcpwm_pcnt+mcpwm_pcnt`, `mcpwm_pcnt+i2s_mux`) passed every time.

So the "roughly 1 in 3" above and the "0 of 16" recorded after it are both
understated for this combination: three for three, with nothing changed in the
driver in between. The honest reading is that the rate is combination-dependent
and this measurement says nothing about the others — a defect that appears in a
third of runs, in none of sixteen, and then in three of three is the same defect
described with three different denominators.

**One extra step, not three, and it is the last one.** Every occurrence:

```
A ch=D0 ticks=160  steps 65/64  extra 1
   period: 64 periods, n_short 0, n_long 0, ok=True, mean 9.9915 us
   window: anchored, steps_in_window 65, steps_outside 0
   step offsets from the first: 0, 10, 20, 29.958, … 629.5, 639.5
   commanded span 630 us, measured span 639.5 us -- exactly one period more
B ch=D2 ticks=320  steps 64/64, exact, in the same capture
reply: OK QRUN / POS 64 64 64 64
```

That answers **"Where are the 3 steps?"** for these observations, and it is the
most useful thing in this entry: the extra pulse is not scattered through the run,
it is *after* it, at the commanded spacing, and the queue's own tally says 64 —
so nothing over-fed the queue, and the extra pulse is emitted after the command
the queue believes it has finished. That is the "already handed to hardware"
shape the item hypothesises, at one step rather than 67.

It also settles a harness question that was open until today: an extra step that
continues the move's rhythm past its last commanded step must be **counted**, not
set aside as pre-move idle. At 24 MS/s the pulse lands on the tick boundary to
within a sample, so a move window cut at "the last commanded pulse ends" would
have hidden this defect half the time. The window now extends through any pulse
whose gap is one the command could have produced
(`run_tests.move_window()`, and `TestMoveWindow`'s two continuation tests).

**Still to do** — unchanged by this: whether it needs `sync` (every observation
is two steppers, `dir`, 160/320 ticks), and the MCPWM/PCNT invariants in
`extras/doc/platforms/esp32.md`. The `POS` disagreement above narrows the search
to the driver, not the feeder.
