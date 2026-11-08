# Pico synchronized start

Priority: **040** — platform-specific part of the engine synchronized start.
This is the one Pico item. The other platforms keep their own files.

Status: generic fallback active on the Pico build
(`fasDisableInterrupts()` / `fasEnableInterrupts()` around a per-stepper
`addQueueEntry(NULL, true)` loop, see
[engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md)).
A single-write PIO enable is the candidate replacement.

## Problem

`synchronizedStart()` releases the steppers the caller names. On Pico
that loop pushes each stepper's first command in turn, so the first
steps are separated by the pushes. The PIO CTRL register can enable
several state machines in one store, which puts those machines on the
same clock edge.

can2040 is the example of that store. In `pio_setup()` it writes
`pio_hw->ctrl = 0x07 << PIO_CTRL_SM_ENABLE_LSB`, which enables its
sync, rx, and match machines together
(`src/can2040.c`, SM_SYNC = 0, SM_RX = 1, SM_MATCH = 2). That set is
fixed: those three machines always exist, they always start together,
and they live on one PIO block.

The FastAccelStepper collection is the pointer list. Already-running
steppers are skipped. An empty queue does not cancel the others. The
selected steppers may be a subset of one block, and a later stepper
claims the next block, so one call can name machines on PIO0 and PIO1.
One CTRL write covers one block. The release is one mask write per
block that has a member in this call, and the mask contains only the
state machines in the call.

Those machines are enabled in `setupSM()` at connect and then wait on
`pull`. A later enable bit does nothing while the machine is already
running. The can2040 pattern applies when the participating machines
are held disabled, the first command is already in the TX FIFO, and the
mask write enables them. Machines already running stay out of the mask.

## Work items

- Hold each newly connected state machine disabled. `connect()` /
  prefill still pushes the first FIFO word.
- In `synchronizedStart()`, group the named steppers by PIO. Skip a
  stepper that is already running. Build one enable mask per block and
  write it once (`pio_enable_sm_mask_in_sync()`, which also restarts
  the clock dividers named in the mask).
- Two blocks in one call start one write apart. Measure that gap on
  the Saleae rig (two steppers on the second PIO, which is stepper 5
  once a block holds four).
- Keep the interrupt disable around the mask writes. An empty member
  still does not cancel the others.
- A block that already holds the cycle-counter program
  ([032](032_pico_cycle_counter_pio.md)) is one FastAccelStepper does
  not claim. The mask is over FastAccelStepper's own blocks.

## References

- `src/pd_pico/pico_queue.cpp` — fallback `synchronizedStart()`,
  `setupSM()` enables the machine at connect.
- can2040 `src/can2040.c` — `pio_setup()`, the CTRL mask write.
- `extras/doc/pico_pio.md` — PIO usage notes.
- [engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md) —
  running steppers skipped, empty queue does not stop the others.
