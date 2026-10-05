# 176 i2s_direct characterization — 23/25 pass, 2 skipped

## Priority

**LOW** — documentation of a characterization result, not a defect.  Useful
for comparing drivers.

## Finding

The 25 wired scenarios had only ever been run on `rmt`, so this is the
first characterization of the I2S step/dir waveform.  **Two** of the three
original failures were harness bugs (below); the two that remain are driver
behaviour no scenario on RMT could have surfaced.

### Harness bugs (fixed)

1. **`sc_pulse_high_time`, `sc_pause`, `sc_long_run` bypassed `legal_ticks()`**,
   using `max(max_speed_ticks, 160)` instead.  `addQueueEntry` bounds the
   *command*, so 16 steps need `ticks*16 >= 3200`.  `rmt`'s floor is 640,
   so the expression gave 10240 and the scenario passed; `i2s_direct`'s floor
   is 80, so it gave 2560 and the queue rejected it.  **One expression, two
   drivers, and the difference was invisible until a driver with a lower floor
   was tried.**  All three now use `legal_ticks(info, steps, ...)`.

2. **The rejection is asynchronous, so `program()`'s check could not see it.**
   `QSEG` replies `OK QSEG` after *parsing* only; the real `addQueueEntry()`
   runs in `qe_feed()` from `qe_pump()`, in the main loop, *after* `QRUN`.
   The error therefore lands in the post-run drain, and the harness went on
   to measure.

3. **The post-run `ERR QE` was read as a measurement.**  So SR_05 was
   recorded as "the pin emitted 0 of 16 steps" — a statement about the
   hardware that was simply false.  The pin carries every legal move perfectly.

`measure()` now treats a post-run `ERR QE` as a **setup failure**, and
`program()` refuses an entry the queue must reject *before* a capture is
spent.  SR_05 and SR_09 re-run: **both pass.**

### Remaining (driver behaviour, not harness)

- SR_12 (direction change): 30/30 steps, 2 dir edges and a correct final dir,
  but the steps land **10 / 9 / 11** across the three phases instead of
  10/10/10.  One step of a phase is attributed to its neighbour — precisely
  the dir-to-step ordering this harness exists to check.

- One scenario withdrawn (see 177).

## Result

**23 passed, 2 skipped** (one harness bug fixed, one harness bug fixed, one
withdrawn, one still open — SR_12 direction-change ordering).

## 2026-10-05 — the open scenario is still uncovered, and that is the finding

The full release matrix does **not** close this item, and the reason is
structural rather than accidental: the catalogue runs `SR_00…SR_30` on **one**
tag per row, `CONFIG 1 rmt dir`. Every scenario in it is therefore an RMT
measurement, on all six rows.

So `SR_12` — the direction-change ordering that is this item's last open
scenario — passes on all six rows, but six rows of RMT is not the measurement
that was missing. The open case is `SR_12` **on `i2s_direct`**, and
`i2s_direct` has no wired scenario in the release matrix at all: of the 28 test
ids recorded across the run, the only `i2s_direct` entries are `--mode scale`
points (`n=1…8`) and `--mode sync` combinations, both of which command a single
direction.

That is worth stating plainly because it is the same trap this item was written
to catch. An earlier version of this file reported "23 passed, 2 skipped" for
`i2s_direct`, and a green matrix that never runs a wired scenario on
`i2s_direct` reads the same way. The characterization gap is one level up from
the scenario list: it is a **driver coverage** gap in the matrix definition, not
a scenario gap.

Closing this needs the release matrix to run the catalogue per driver, not only
per row — a change to `release_runs()` in `scripts/harness.py`, not a new
scenario.
