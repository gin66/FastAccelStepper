# SR_25 — `stopMove()` scenario fails on RP2350 (Pico / PIO)

**Status:** implemented; passes on ESP32 (RMT, MCPWM/PCNT, I2S), **fails on
RP2350**. Found 2026-10-06 while bringing the harness up on a Pico 2
(`--arch rpipico2 --driver pio`).

**Context.** SR_25 sends the same program as SR_30 — four `QUEUE_FILL_STEPS`
(=16 × 255 = 4080-step) segments — then `QFILL 1 16` before `QRUN 1`, and stops
a quarter of the way into the *fill* with `STOP` (`stopMove()`). `stopMove()`
must not truncate already-queued motion, so the whole remainder of the fill must
still come out and the marker is what witnesses it.

`QRUN` on a `QFILL`ed queue sets `no_topup`, so the run is exactly what `QFILL`
reported and nothing is added after the start. SR_25 evaluates
`eval_stop_move_contract` and requires `len(steps) <= filled*255`.

**Measured on RP2350** (`rpipico2_arduino_pio_pio2_dir_SR_25.json`):

| field | value | expected |
|---|---|---|
| `queue_filled_entries` | 16 | 16 |
| `filled_steps` | 4080 | 4080 |
| `steps_before_stop` | 6479 | ~1000 (25 % of the fill) |
| `steps_after_stop` | 7801 | ~3080 |
| `steps_measured` | 14280 | 4080 |
| `nothing_added_after_start` | false | true |
| `stop_interrupted_the_run` | false | true |

The stop landed **after the whole fill had already been drained** (6479 >
4080), i.e. the run was longer than the queue `QFILL` put there. That is the
`no_topup` invariant that holds on ESP32 not holding on Pico.

**What has already been checked:**

- The marker channel works (`marker=7`), so this is not the earlier
  `ch=- marker=` parsing gap.
- The mechanism exists: a manual `QFILL 1 16` / `QRUN 1` on the same firmware
  shows `no_topup=1 fill_only=0`.
- `QFILL` reaches an **unstable depth**: `q=11` by hand, `q=16` in the harness,
  on the same flash. The queue depth on PIO is the first thing to explain,
  because `QE_PREFILL` and every expected count are derived from it.
- The manual `QRUN 1` also answered `ERR QE start rc=-2` (a
  `synchronizedStart()` failure). rc=-2's meaning in `AqeResultCode` should be
  established; if `synchronizedStart()` on Pico leaves the cursors re-armed,
  `no_topup` may not be honoured.

**Hypotheses to test:**

- `queueEntries()` on PIO under-reports (or `QE_ROOM_RESERVE`/`QUEUE_LEN`
  overstate capacity), so the prefill loop in `qe_pump()` adds entries after
  `QRUN` even though `no_topup` is set — the prefill loop checks `fill_only`
  but not `no_topup`.
- `synchronizedStart()` on Pico re-arms/clears the per-cursor state that carries
  `no_topup`.
- Something in the PIO queue keeps consuming after the fill, so the depth the
  host asserts against is not the depth that runs.

**Where to look:** `common/saleae_app.cpp` — `arm_cursors()` (`no_topup`),
`qe_pump()` (prefill at `queueEntries() < QE_PREFILL`, top-up loop),
`handle_qfill()`; `scripts/run_tests.py` — `SCENARIO_FILL`, `fill_queue()`,
`stop_after_for()`, `eval_stop_move_contract()`. The PIO queue itself lives in
`src/pd_pico/pico_queue.cpp`; see also todo 127 (SR_30) — both scenarios share
this root cause.
