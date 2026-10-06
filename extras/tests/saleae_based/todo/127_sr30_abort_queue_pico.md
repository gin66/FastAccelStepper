# SR_30 — `forceStopAndNewPosition()` scenario fails on RP2350 (Pico / PIO)

**Status:** implemented; passes on ESP32 (RMT, MCPWM/PCNT, I2S), **fails on
RP2350**. Found 2026-10-06 while bringing the harness up on a Pico 2
(`--arch rpipico2 --driver pio`).

**Context.** SR_30 is SR_25's mirror: the same fill, stopped a quarter of the
way in, but with `XSTOP` — `forceStopAndNewPosition()`, which empties the queue
so the queued commands never run. The measurement is `steps_after_stop`: it must
be ~nothing (only whatever a driver had already handed to its hardware can still
step), against SR_25's whole remainder.

**Measured on RP2350** (`rpipico2_arduino_pio_pio2_dir_SR_30.json`):

| field | value | expected |
|---|---|---|
| `queue_filled_entries` | 16 | 16 |
| `filled_steps` | 4080 | 4080 |
| `steps_before_stop` | 6519 | ~1000 (25 % of the fill) |
| `steps_after_stop` | **0** | ~nothing — this part is right |
| `abort_tail_bound_steps` | 510 | — |
| `stop_interrupted_the_run` | false | true |
| `reply` | `OK QRUN` / `OK XSTOP abortqueue` / `DONE 0` / `POS 0` | — |

`XSTOP` did empty the queue (`steps_after_stop=0`, `POS 0`), so the thing SR_30
exists to measure is correct on Pico. The failure is the same as SR_25's: the
stop landed **after the whole fill** (6519 > 4080), so
`stop_interrupted_the_run` is false and the run is not a measurement of the
abort at all. The queue was topped up past the `QFILL` depth before `XSTOP`
arrived.

**Relationship to todo 025.** Separate from this: `extras/todo/
025_pico_force_stop_loses_step_count.md` covers the Pico `forceStop()` throwing
away the exact performed-step count (`pio_sm_clear_fifos` drops RX, then
`pos_offset = 0`), so `POS` after an abort is not the position actually reached.
SR_30 is the right scenario to measure that once this item is fixed: it already
compares `POS` against the steps on the wire. The `POS 0` above is consistent
with 025 but cannot be distinguished from a correct empty-queue until the run is
a real abort.

**Hypotheses to test:** identical to SR_25 (todo 126) — the fill depth is
unstable (`q=11` by hand, `q=16` in the harness) and the run exceeds the fill
before the stop. Fixing the `no_topup`/prefill behaviour should fix both
scenarios; then SR_30 additionally validates the Pico abort path.

**Where to look:** `common/saleae_app.cpp` (`arm_cursors()`, `qe_pump()`,
`handle_qfill()`), `scripts/run_tests.py` (`SCENARIO_FILL`, `fill_queue()`,
`eval_abort_queue()`, `ABORT_TAIL_ENTRIES`), `src/pd_pico/pico_queue.cpp`
(`forceStop()`, `getCurrentStepCount()`), `extras/todo/025_*.md`.
