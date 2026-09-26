# Pico synchronized start

Priority: **P5** — platform-specific part of the engine synchronized start
(one item per open platform).

Status: generic fallback active on the Pico build
(`fasDisableInterrupts()` / `fasEnableInterrupts()` around a per-stepper
`addQueueEntry(NULL, true)` loop, see
[engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md)). PIO HW sync
under verification.

## Problem

Multiple steppers can share one PIO block, where the PIO block start may
be a single hardware start point for all steppers on that block. If so,
starting one block synchronizes all its steppers without a critical section;
otherwise the critical section must remain the release point.

## Work items

- Check PIO block-start behaviour: does starting a PIO block start all its
  steppers' first steps on the same edge, including steppers added with
  `addQueueEntry(NULL, true)`?
- If HW-supported: release by one block start per block (one call per block
  inside `synchronizedStart()`), drop the critical section.
- If not: keep the current critical section (the same fallback contract as
  every other platform).

## References

- `src/pd_pico/pico_queue.cpp` — fallback `synchronizedStart()`, PIO
  block/stepper layout.
- `extras/doc/pico_pio.md` — PIO usage notes.
- [engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md) — generic
  layer and the fallback behaviour contract (running steppers skipped,
  empty queue does not stop the others).
