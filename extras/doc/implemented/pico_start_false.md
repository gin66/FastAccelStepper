# Pico ignored `addQueueEntry(cmd, start=false)` — SR_25/SR_30 on RP2350

Closes the RP2350 item tracked as `extras/todo/126_sr25_stopmove_pico.md` and
`127_sr30_abort_queue_pico.md`. Measured on an RP2350 (`--arch rpipico2
--driver pio`, GPIO2..GPIO9 = D0..D7), Saleae clone.

## Summary

The two stop scenarios failed on Pico because the run started **before `QRUN`**,
so the stop landed after the queue had drained and measured nothing. The cause
was not the stop path and not the drivers: the Pico port started its PIO feeder
on **every** `addQueueEntry()`, including a `start=false` add, so the harness's
`QFILL` (which queues with `start = false` by design) began stepping the motor.
This also broke the documented "fill the queue, then start it" contract that
`FasNAxis::pump()` → `FastAccelStepperEngine::synchronizedStart()` relies on.

Fixed in `src/fas_queue/queue_add_entry.cpp`; a compounding harness bug in
`common/saleae_app.cpp` was fixed with it; and a read-order defect in
`src/pd_pico/pico_queue.cpp` found while writing this up. SR_25 and SR_30 pass.

## The defect

`StepperQueue::addQueueEntry(cmd, start)` ended with:

```c
  if (!isRunning() && start) {
    startQueue();
  } else {
#if defined(SUPPORT_RP_PICO)
    startQueue();          // <-- reached for start=false too
#endif
  }
```

The `else` is taken whenever `isRunning() || !start`, so for `start=false` the
Pico branch called `startQueue()` unconditionally. `startQueue()` pushes the
first entry into the PIO TX FIFO and enables the feeder interrupt, and the state
machine — already enabled in `setupSM()` — begins emitting steps. There is no
"queued but not started" state on Pico.

The documented contract (`FastAccelStepper.h`):

> If the queue is not running, then the start parameter defines starting it or
> not. The latter case is of interest to first fill the queue and then start it.

`FasNAxis` prefills every axis with `start=false` and then releases them with one
`synchronizedStart()` (`FasNAxis.h`). On Pico the first axis queued would already
be running, so the coordinated start was gone; the manual
`ERR QE start rc=-2` (`ErrorEmptyQueueToStart`) noted in the todo is the same
defect seen from the other side.

The re-arm is now:

```c
    if (isRunning()) {
      startQueue();
    }
```

which keeps the one thing the Pico branch actually needs — re-enabling the
feeder after the ring drains — without starting an idle queue.

## Why SR_25/SR_30 failed

Both scenarios `QFILL 1 16` the queue (start = false), then `QRUN 1`, then stop
25 % into the fill. On Pico the `QFILL` started the stepper, so by the time the
`QRUN`/`STOP` serial round-trips completed the fill had drained and the stop
landed after the run.

Recorded `rpipico2_arduino_pio_pio2_dir_SR_25.json`:

| field | measured | expected |
|---|---|---|
| `queue_filled_entries` | 16 | 16 |
| `filled_steps` | 4080 | 4080 |
| `steps_before_stop` | 6626 | ~1000 |
| `steps_after_stop` | 7654 | ~3080 |
| `steps_measured` | 14280 | 4080 |
| `stop_interrupted_the_run` | false | true |
| `period.ok` | false | true |

The VCD says it directly: one isolated pulse at 0.281 ms, then a **208.8 ms
gap**, then the real burst 0.490→0.633 s. The stray pulse is `QFILL` starting the
stepper; the 208.8 ms gap is the "long period" that also failed `period.ok`.

A second bug compounded it. `qe_pump()`'s **prefill** loop checked `fill_only`
but not `no_topup`, and `qe_feed(cap = 0)` fills to capacity. `QRUN` clears
`fill_only` and sets `no_topup`, so the loop refilled a queue `QFILL` had just put
at a known depth. It was invisible on ESP32 — the QFILLed queue is still at
`QE_PREFILL` when QRUN arrives, so the loop body never runs — and surfaced on
Pico, where the queue had drained. Both loops that can add to a QFILLed cursor
now honour `no_topup`.

## The read-order defect

`getCurrentStepCount()` read the RX FIFO **before** testing it empty:

```c
  for (uint8_t i = 0; i <= 4; i++) {
    pos = pio_sm_get(pio, sm);            // reads
    if (pio_sm_is_rx_fifo_empty(pio, sm)) {
      break;                              // then tests
    }
  }
```

so on an empty FIFO `pos` was whatever the read returned. Fixed to test first and
initialise `pos = 0`, matching the drain loop above it.

## Verification

Firmware builds for `rpipico2` (`pio run -e rpipico2`) and the AVR harness build
pass; `python3 -m unittest discover -s scripts/tests` is 362 tests, OK.

Hardware re-run, both the count-1 tag (`rpipico2_arduino_pio1_dir`) and the
count-2 tag the old red rows used (`rpipico2_arduino_pio_pio2_dir`; SR_25/30 are
`1ch` scenarios, so both run one stepper):

| test | measured | before stop | after stop | fill | interrupted |
|---|---|---|---|---|---|
| SR_25 | 4080 | 1278 / 1037 | 2802 / 3043 | 4080 | true |
| SR_30 | ~1100–1290 | = measured | **0** | 4080 | true, queue discarded |

`steps_after_stop = 0` and `queue_discarded` are the abort finally measured on
Pico; SR_25's full-fill drain is the opposite contract.

## Still open — `POS` after an abort is intermittent

Not fixed here and **not asserted by SR_30**, which is why the scenario passes
either way. On a mid-run `XSTOP`, the firmware replies

```
OK XSTOP abortqueue
DONE 1281        <- often correct
POS 1281
```

but in 3 of 8 consecutive runs it replied `DONE 0` / `POS 0` while the wire still
carried the full run. `getCurrentPosition()` returned 0 **during the run**: the
PIO pushes the position into the RX FIFO periodically, and `getCurrentStepCount()`
on a *running* queue reads whatever happens to be in the FIFO — 0 when it is
momentarily empty. `forceStopAndNewPosition(getCurrentPosition())` then latches
that 0. The read-order fix above makes the empty case deterministic (`0`) rather
than a stale read; it does not make the position available when the FIFO is
empty. A real fix has to carry the position outside the FIFO while running (a
pulse counter, or software bookkeeping), which is tracked as
[`extras/todo/026_pico_position_read_returns_zero.md`](../todo/026_pico_position_read_returns_zero.md).
