# SAMD51 synchronized start

Priority: **P5** — platform-specific part of the engine synchronized start
(one item per open platform).

Status: generic fallback active on SAMD51
(`fasDisableInterrupts()` / `fasEnableInterrupts()` around a per-stepper
`addQueueEntry(NULL, true)` loop, see
[engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md)).
Native TCC release unverified.

## Problem

SAMD51 drives each queue from a TCC instance (`queue_for_tcc[]`), and
`startQueue()` primes period 1 and issues a RETRIGGER to release the first
compare. A single critical section does not guarantee one hardware event
across different TCC instances. Whether the TCC sync bus or a shared
software trigger can release several armed TCCs together is not decided yet.

## Work items

- Determine the TCC instance / channel mapping per step pin and how many
  TCCs a typical multi-axis setup spans.
- Check whether a TCC sync-bus prescription or one common software retrigger
  can start the armed TCCs on the same edge.
- If yes, implement the group release in the
  `synchronizedStart()` implementation and drop the critical
  section.
- If no, keep the critical section, document it as final, and close this
  item.

## References

- `src/pd_samd/samd_queue.cpp` — `startQueue()`, the TCC ISRs,
  `queue_for_tcc[]`, and the `synchronizedStart()`
  implementation.
- `extras/doc/samd51/` — SAMD51 hardware notes.
- [engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md)
  — generic layer and the fallback behaviour contract (running steppers
  skipped, empty queue does not stop the others).
