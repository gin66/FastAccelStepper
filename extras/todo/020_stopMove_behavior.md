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

## 2026-10-05 — unchanged, and now asserted on six rows

The full release matrix runs `SR_25` (`STOP` = `stopMove()`) and `SR_30`
(`XSTOP` = `forceStopAndNewPosition()`) on all six rows. Both pass everywhere,
which does **not** mean this is fixed and is the point of recording it:

- `SR_25` passing means the queued move ran to completion — the behaviour
  described above, now asserted rather than discovered. `stopMove()` still sets
  a flag and still does not truncate already-queued motion.
- `SR_30` passing means `forceStopAndNewPosition()` did empty the queue.

The distinction matters for anyone reading the matrix as a health report: the
contract these two scenarios verify is that `STOP` *does not* stop. A user who
wants an emergency stop wants `XSTOP`, and the library gives them no third
option that is both immediate and documented in one place. That conflation was
a harness bug, found and fixed while closing this item; the API naming it
exposed is what is still open here.

Behaviour here is by design in the library and the defect is documentation and
API naming, not the queue.
