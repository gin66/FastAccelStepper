# 026 Pico `getCurrentPosition()` intermittently returns 0 during a run

Status: **CLOSED** — fixed and measured on RP2350. Record:
[pico_position_read_returns_zero.md](../doc/implemented/pico_position_read_returns_zero.md).

The defect was the RX-FIFO read treating a **stale** sample as current: the PIO
pushes the position with a non-blocking push, so once the 4-entry FIFO is full it
stops updating and holds samples from the run's beginning. Measured `POS 0` in
15 of 40 SR_30 runs against ~1270 steps on the wire; 0 of 70 after the fix. The
running read now discards the stale samples and waits for a current push, and the
stopped path returns `queue_end.pos` (the SM restart clears the position in the
shift register and `pos_offset`). SR_30 now asserts `POS` against the wire
(`check_commanded_position`), where before it was recorded and never read.

Priority it had: **026**. Effort: ~40 k tokens, 1 d.

## The defect

On Pico, `getCurrentPosition()` reads the position out of the PIO's **RX FIFO**:

```c
int32_t StepperQueue::getCurrentStepCount() const {
  bool running = isRunning();
  uint32_t pos = 0;
  if (!running) { /* drain RX, push a dummy entry, wait for its pushed
                     position */ }
  for (uint8_t i = 0; i <= 4; i++) {
    if (pio_sm_is_rx_fifo_empty(pio, sm)) break;
    pos = pio_sm_get(pio, sm);
  }
  return (int32_t)pos;
}
```

The PIO program pushes the running position into RX periodically (`push` in the
period loop, `pico_pio.cpp`). On a **running** queue the `!running` branch is
skipped, so the function just reads whatever happens to be in the FIFO at that
instant — and when the FIFO is momentarily empty (the SM has not pushed since
the last consumer, or it was just drained) it returns the initialised `0`.

`getCurrentPosition()` is then `0 + pos_offset`; for the abort path
`forceStopAndNewPosition(getCurrentPosition())` latches the `0`
(`FastAccelStepper.cpp:467-484`), so `DONE`/`POS` read 0 after a stop that in
fact emitted the whole run.

The read-order fix in `pico_queue.cpp` (test RX-empty before reading, initialise
`pos`) makes the empty case **deterministic** — `0` rather than a stale read — but
it does not make the position available when the FIFO is empty. The position has
to be carried somewhere other than the FIFO while the queue runs.

## Measured

RP2350, `--arch rpipico2 --driver pio`, SR_30 (`XSTOP` ~25 % into a 4080-step
fill), eight consecutive runs:

```
DONE 0 / POS 0        3 of 8   <- wire carried the full ~1200-step run
DONE 1273 / POS 1273  5 of 8
```

No catalogue scenario asserts `POS`, so nothing goes red. SR_30 judges the pulses
on the wire (`eval_abort_queue`) precisely because `POS` is the queue's own
bookkeeping; that design is right for the stop contract, and it is also why this
stayed invisible. `test_36`-style coverage of `getCurrentPosition()` does not
exist on Pico.

## Direction

- The library already carries `pos_offset` for exactly this "compensate the
  hardware count" purpose (`queue_add_entry.cpp`). On Pico it is set when the
  queue next goes empty and idle; it is not maintained while the queue runs.
- The honest sources of "how far has the SM got" are the PIO's **`X`/`ISR`
  position**, which the program already computes, or a step **pulse counter**.
  Reading the FIFO is a sample of the former, not the value.
- Do not simply drain-and-dummy on a running queue: the SM is mid-program, so a
  dummy entry would be interpreted as a command.

Settle on the board, not in the abstract: whether the position can be read
without perturbing the SM (an `exec`/`sm_get` of ISR, or a PIO IRQ that keeps a
software copy), and what it should read between the last push and the next.

## The test

- A Saleae scenario that, like SR_30, stops mid-fill and asserts the firmware's
  `POS` against the steps on the wire — currently `POS` is recorded in `reply`
  but not judged. This is the measurement that showed the 3-in-8.
- Or a source-level/unit check that `getCurrentStepCount()` does not return 0
  while `isRunning()`.

## References

- [pico_start_false.md](../doc/implemented/pico_start_false.md) — the item this
  was found under (SR_25/SR_30 on RP2350), and its "Still open" section
- `src/pd_pico/pico_queue.cpp` — `getCurrentStepCount()`, `isRunning()`,
  `getCurrentPosition()`
- `src/pd_pico/pico_pio.cpp` — the `push` of the position in the period loop
- `src/pd_pico/pico_queue.cpp` `forceStop()` — `pos_offset = 0`
- `src/fas_queue/queue_add_entry.cpp` — the `pos_offset` convention
- `extras/tests/saleae_based/scripts/run_tests.py` — `eval_abort_queue`
