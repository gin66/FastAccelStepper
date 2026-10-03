# 171 stopMove() does not stop a queued move

## Priority

**HIGH** — library behaviour that can cause unexpected motion.  A user who
calls `stopMove()` expecting an emergency stop will get the full queued move.

## Finding

`stopMove()` only sets a flag the ramp generator reads when asked for its
*next* command.  A move that is already in the pulse queue runs to completion.

Measured:

- 2000-step move stopped at 510 → ran to **2000** — no effect.
- 20000-step move stopped at 5825 → finished at **14240**, truncating only
  once the queue refilled.
- Verified on a waveform: truncated at 11475 of 20000, no partial pulse, pin
  still for the remaining 2.823 s.

## Mechanism

`stop_all()` calls `stepper->stopMove()` and then `memset(&slots[i].cur, 0)`
and `clear_programs()`.  So queue *filling* is cancelled immediately — the
feeder cursor is zeroed and the segment program dropped, so `qe_feed()` stops
calling `addQueueEntry()`.  What is **not** cancelled is what is already
queued: the library's `stopMove()` only sets a flag consulted when the queue
asks for its *next* command, so queued commands still emit.  The truncation
point is therefore however much was prefilled, not when STOP arrived.

The feeder runs *far* ahead of the driver — it is pumped from the main loop
— so on a driver that takes ~281 ms to start emitting, the whole program can
be queued before the first pulse.  Then STOP has nothing left to cancel and
the full program runs, which is the documented library behaviour and not a
defect.

## Workaround

Use `forceStop()` (cancels nothing already queued, queue drains ~20 ms) or
`forceStopAndNewPosition()` (aborts everything queued, no further step issued)
instead of `stopMove()` for emergency stops.
