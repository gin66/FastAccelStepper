# Pico `getCurrentPosition()` returned 0 during a run — SR_30 on RP2350

Closes the item tracked as `extras/todo/026_pico_position_read_returns_zero.md`.
Measured on an RP2350 (`--arch rpipico2 --driver pio`, GPIO2..GPIO9 = D0..D7,
Saleae clone), the same rig as
[pico_start_false.md](pico_start_false.md).

## Summary

On Pico, `getCurrentPosition()` reads the position out of the PIO's **RX FIFO**.
The PIO pushes the running position with a *non-blocking* push, so once the FIFO
(4 entries) is full the push stops updating it: a queue nobody has read since it
started holds samples from the run's beginning, including 0 before the first
step. Reading one of those reports a position the stepper left long ago.

`forceStopAndNewPosition(getCurrentPosition())` — the harness's `XSTOP` — then
latches that stale sample, so `DONE`/`POS` read 0 after a stop that in fact
emitted the whole run. Measured on RP2350, SR_30: **`POS 0` in 15 of 40** runs
while the wire carried ~1270 steps, and **7 of 15** on the confirmation run.

SR_30 judges the pulses on the wire, deliberately, so it passed either way; the
position was recorded and never read. The scenario now asserts it
(`check_commanded_position`), and the firmware reads a current sample.

## The defect

`StepperQueue::getCurrentStepCount()` on a **running** queue skipped its
drain-and-obtain branch and read whatever was in the RX FIFO:

```c
bool running = isRunning();
uint32_t pos = 0;
if (!running) { /* drain, push a dummy entry, wait for its pushed position */ }
for (uint8_t i = 0; i <= 4; i++) {
  if (pio_sm_is_rx_fifo_empty(pio, sm)) break;
  pos = pio_sm_get(pio, sm);
}
return (int32_t)pos;
```

Instrumented on the board at the `XSTOP` instant, the FIFO was **full**
(`rxl=4`), the SM running (`run=1`), and all four entries stale — the value read
was 0. The read-order fix already in `pico_start_false.md` (test before read)
only made the *empty* case deterministic; it did not make a stale sample
current.

The PIO program pushes in its period loop:

```
label_period_loop:
  push block=false autopush=false   ; the running position (ISR)
  mov isr, x
  jmp y-- label_period_loop
```

`push(block=false)` drops the push when the FIFO is full. Since the loop runs
many times per half-step, the FIFO fills with the first four pushes and never
updates until a consumer frees a slot. The position is thus a **sample that goes
stale**, which is what the FIFO is for — the defect is the read treating a stale
sample as current.

## The fix

`src/pd_pico/pico_queue.cpp`:

- **Running** (`getCurrentStepCount`): discard the samples already in the FIFO,
  then wait for the SM to push a current one. The wait is short — the period
  loop reaches its push every 3 cycles and the longest gap before it (the step
  and position-update section) is tens of cycles. If the SM is instead stalled on
  a `pull`, there is no live position and the move's own target is the fallback
  (`queue_end.pos - pos_offset`, which `getCurrentPosition()` turns back into
  `queue_end.pos`).

  ```c
  while (!pio_sm_is_rx_fifo_empty(pio, sm)) pio_sm_get(pio, sm);
  for (uint16_t spin = 0; spin < 256; spin++) {
    if (!pio_sm_is_rx_fifo_empty(pio, sm)) return (int32_t)pio_sm_get(pio, sm);
  }
  return queue_end.pos - pos_offset;
  ```

- **Stopped** (`getCurrentPosition`): return the queue's own bookkeeping rather
  than a hardware sample. `forceStop()` restarts the SM, which clears the input
  shift register the PIO keeps the position in, and sets `pos_offset = 0` — so
  the sample path reads 0 even though `forceStopAndNewPosition()` just stored the
  real position in `queue_end.pos`. For a stopped queue `queue_end.pos` *is* the
  position, which is what the generic `getCurrentPosition()` returns for an empty
  queue:

  ```c
  if (!isRunning()) return queue_end.pos;
  return getCurrentStepCount() + pos_offset;
  ```

Both halves are needed. The running read provides the value the abort latches;
the stopped path keeps it observable afterwards.

## The test

`check_commanded_position()` in `scripts/run_tests.py` joins the two halves of
the evidence: the evaluator's `steps_before_stop` (the wire) and the firmware's
`POS` reply. It is ANDed into the verdict, so `POS 0` turns a would-be `passed`
run red. It is keyed by `POSITION_SCENARIOS = {"SR_30": ("rpipico", "rpipico2")}`
— the one scenario that can, because it empties the queue at the marker — and
scoped to Pico, because that is the only place the two describe the same number.

Scoping is not a convenience. On a driver with a pipeline they do **not**, by
design: measured on ESP32, `rmt` reports a position ~17 steps *ahead* of the pin
(its committed symbols) and `i2s_direct` ~162 *behind* it (its DMA buffer), while
`mcpwm_pcnt`, which takes one entry at a time, matches to 2. A judgement that
assumed they were equal would turn a passing RMT row red; the PIO has no such
pipeline between its counter and the pin.

The tolerance (`POSITION_TOLERANCE_STEPS = 16`) is the stop-call latency, not
the position's accuracy: the frozen position is taken just before the queue is
emptied and the marker just after, so the SM emits a few steps in between.
Measured on RP2350 the delta is 0–3 in almost every run, with the odd scheduling
hiccup near 10. The defect it gates is off by the whole run, so the tolerance is
far from it.

`TestPositionCheck` (eight tests) pins the parser, the zero regression, the
tolerance edges, a missing `POS` line (a failure, not a silent pass), that a
pipelined (non-Pico) driver is not judged, and that scenarios not in
`POSITION_SCENARIOS` are untouched.

## Verification

Firmware builds for `rpipico2` and `rpipico` (RP2040 uses the same source);
`python3 -m unittest discover -s scripts/tests` is 370 tests, OK.

Measured on the RP2350, SR_30, the same capture path as the recorded matrix:

| firmware | runs | `POS 0` replies | gate verdict |
|---|---|---|---|
| unmodified (HEAD) | 40 + 15 | **15 + 7** | fails on the zero runs, passes otherwise |
| fixed | 30 + 25 + 15 = 70 | **0** | 70/70 passed |

The gate was validated end-to-end by flashing the unmodified firmware and
running the new harness: seven of fifteen runs were caught as
`position_check.ok = false` with `firmware_pos = 0` and `delta_steps` ~1200–1540
— which is the defect, not a flake. On the fixed firmware the delta is 0–3.

A pre-existing, unrelated flake remains: about 4 of 40 SR_30 runs record a single
`11.75 µs` inter-step gap against the commanded `10 µs`, at the last step before
the stop. It reproduces on the unmodified firmware (measured 4 of 40 there), so
it is not this change, and it is not the position. It is left as a finding rather
than fixed here.

## References

- [pico_start_false.md](pico_start_false.md) — where this was found, and its
  "Still open" section
- `src/pd_pico/pico_queue.cpp` — `getCurrentStepCount()`, `getCurrentPosition()`
- `src/pd_pico/pico_pio.cpp` — the `push` of the position in the period loop
- `scripts/run_tests.py` — `check_commanded_position()` / `eval_abort_queue`
- `scripts/tests/test_saleae.py` — `TestPositionCheck`
