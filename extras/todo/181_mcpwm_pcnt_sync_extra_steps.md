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