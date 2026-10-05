# 025 Pico `forceStop()` discards a step count it could keep exactly

Priority: **025**. Effort: ~30 k tokens, 0.5 d.

Status: not started. An attempt was made and **deliberately dropped**: no
Pico toolchain was available, so the change was unbuilt and unmeasured,
and unverified PIO register work does not belong in the tree. The A/B
analysis below is kept because it is the part that does not need
hardware — but the choice between A and B should be made against the
board, not in the abstract. Found while working
[020](020_queue_admission_latch.md), which records it as *"not this
item's subject"* — a separate defect, on one platform, with its own
trigger and its own verification.

## Verdict

A real defect, and the only place in the library where it is
*fixable rather than approximable*. `StepperQueue::forceStop()` on Pico
throws away the exact performed-step count, then sets `pos_offset = 0`,
so `getCurrentPosition()` after an abort does not return the position
the stepper is actually at.

## The two lines

`src/pd_pico/pico_queue.cpp`, as found:

```cpp
pio_sm_set_enabled(pio, sm, false);
pio_sm_clear_fifos(pio, sm);   // discards RX as well as TX
...
read_idx = next_write_idx;

pos_offset = 0;
```

RX is where the performed-step count lives. The clear is what makes the
loss, and `pos_offset = 0` then denies the caller any way to compensate.

## A second, independent read bug

`getCurrentStepCount()`, as it was (now fixed at `:213-218`):

```cpp
for (uint8_t i = 0; i <= 4; i++) {
  pos = pio_sm_get(pio, sm);            // reads
  if (pio_sm_is_rx_fifo_empty(pio, sm)) {
    break;                              // then tests
  }
}
```

The read precedes the emptiness test, so on an empty FIFO `pos` is
whatever the read returns — the count is read *after* deciding it may
not exist. Contrast the drain loop above it, which tests first and then
reads. Both loops mean to do the same thing and disagree about the
order.

This is reachable independently of `forceStop()`: any call to
`getCurrentPosition()` on a stopped, drained queue hits it.

## Why Pico is the only platform where this is closable

`pio_sm_set_enabled(pio, sm, false)` at `:158` halts the state machine
synchronously. RX already holds the count of what was emitted, so the
read window between *stop* and *read* is genuinely closable — there is
no in-flight hardware work to race.

Everywhere else it is not. RMT keeps playing already-transmitted
symbols; I2S keeps playing DMA blocks; MCPWM/PCNT has a counter but no
comparable synchronous boundary. On those platforms "abort now, know
exactly where you stopped" is approximate by construction.

That asymmetry is itself part of why the stop API needs documenting per
platform — see the closing argument in
[020](020_queue_admission_latch.md) § *Platform note*, and
`extras/doc/platforms/pico.md`.

## The fix — two self-consistent options, and a trap

The doc's original snippet in [020](020_queue_admission_latch.md)
mixes two mutually exclusive designs: it drains RX into `performed`,
*then* calls `clear_fifos` (which discards RX), *then* computes
`pos_offset = true_pos - performed`. But `getCurrentStepCount()` reads
and drains RX itself at `:213-218`, so after a clear it reads ~0 and
the subtraction is applied to a count that no longer exists.

Two self-consistent options:

**A — keep RX.** Drain TX manually while halted and skip
`pio_sm_clear_fifos` for RX. Then `getCurrentStepCount()` keeps working
and `pos_offset = true_pos - performed` is right. More exact, and the
form `queue_add_entry.cpp:60` already relies on.

**B — keep `clear_fifos`.** Read RX inside the halt, discard it, and
put the surviving position in `pos_offset` outright.

**A is the better design** — the exact answer rather than a bounded one,
and the one that leaves `queue_add_entry.cpp:60` composing unchanged. It
is also the riskier edit: it needs a TX-only drain (drain TX while
halted, since it cannot refill), which is a different PIO call from the
one the file already makes.

**B** keeps `clear_fifos()` and puts the surviving position in
`pos_offset` outright. Simpler and it introduces no new SDK symbol, but
it costs the per-step resolution: with RX cleared, `pos_offset` holds a
constant until the restarted SM counts again, so
`getCurrentPosition()` does not advance *within* an abort — which is
arguably what a stopped stepper should do. It still composes with
`queue_add_entry.cpp:60`, which recomputes `pos_offset` whenever the
queue next goes empty and idle.

Three things to settle on the board, not in the abstract:

- **Do not read the count through `getCurrentStepCount()`.** Its
  `!running` branch *drains* RX and pushes a dummy FIFO entry to force a
  fresh count. The SM is halted, so the dummy would never be processed;
  and if the halt landed on PC 0 the drain would discard the very count
  being saved. Read RX inline instead.
- **The read-before-test reorder is a separate fix** and is wrong on its
  own: it consumes a value on an empty RX FIFO, which is the normal
  state after `clear_fifos()` and any time the queue goes quiet. `pos`
  also needs initialising, because the test can break on the first
  iteration. Neither depends on the abort path, so this half is provable
  without hardware.
- **The loop bound `<= 4`** is inherited from the two loops already in
  this function, not derived from the RX FIFO depth. Confirm the depth
  is what it is assumed to be before trusting a 5-iteration read.

## Interaction with 020

None in code: [020](020_queue_admission_latch.md) changes the queue
*admission* path and does not touch `forceStop()` or `pos_offset`.

One directional note. Today a low-level caller that aborts cannot
queue again — the latch blocks it — so this path is reachable mainly
via `forwardStep()`, which clears the latch
(`FastAccelStepper.cpp:647`). Once 020 lands, any low-level caller can
re-arm and queue on the same connection, so a wrong position after an
abort becomes easier to hit. Already reachable, more so afterwards.

Sequence this **after** 020, not before.

## The test

`extras/tests/pc_based` cannot reach it: `StepperISR_test.cpp:18`
stubs `StepperQueue::forceStop()` to an empty body, and the PIO
registers have no meaning off-hardware.

Two options:

- **Saleae**, where a Pico build already runs
  (`--arch rpipico --driver pio`). A scenario that fills, runs,
  `XSTOP`s mid-fill and compares `POS` against the steps on the wire
  would measure the error directly. This is the stronger test: it
  compares the reported position against the pins.
- **A unit test of the read order** at `:213-218`, on the principle
  that a loop which tests-then-reads in one place and read-then-tests
  in another is a defect regardless of platform. Cheap, and it is the
  half that is provable without hardware.

Do both. The second catches the regression; the first measures the
magnitude.

## References

- [020](020_queue_admission_latch.md) § *Platform note: Pico destroys a
  count it could keep exactly* — where this was found, and why it was
  split out
- `src/pd_pico/pico_queue.cpp:154-169` — `forceStop()`, both halves
- `src/pd_pico/pico_queue.cpp:179-205` — `getCurrentStepCount()`, read order
- `src/fas_queue/queue_add_entry.cpp:60` — the `pos_offset` convention
- `src/FastAccelStepper.cpp:604,621` — the other two `pos_offset` writers
  (`setCurrentPosition`, `setPositionAfterCommandsCompleted`)
