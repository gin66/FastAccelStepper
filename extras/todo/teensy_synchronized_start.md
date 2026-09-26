# Teensy synchronized start

Priority: **P5** — platform-specific part of the engine synchronized start
(one item per open platform).

Status: generic fallback active on Teensy 4
(`fasDisableInterrupts()` / `fasEnableInterrupts()` around a per-stepper
`addQueueEntry(NULL, true)` loop, see
[engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md)).
Native TMR release unverified.

## Problem

Teensy 4 uses the four QuadTimer (TMR) modules, four channels each.
`queue_for_channel[module][channel]` shows several steppers can share one
module, whose channels count the same counter, so arming all channel compare
interrupts under one critical section should already align them on that
module. Different modules have independent counters, so a single critical
section cannot make modules agree to better than the enable latency.

## Work items

- Confirm that channels on one TMR module share the counter and that
  `startQueue()` only arms per-channel compare state (no per-channel timer
  restart).
- Check whether all channels of a module can be armed and released with one
  `CTRL` write; use it for the within-module case if cheaper.
- Decide the cross-module case: accept the critical section, or find a
  common trigger across TMR modules.
- If the critical section stays, document it as final and close this item.

## References

- `src/pd_teensy/teensy_queue.cpp` — `startQueue()`, `queue_for_channel[][]`,
  the TMR ISRs, and the `synchronizedStart()` implementation.
- i.MX RT QuadTimer reference and luni64/TeensyStep4 `TMR.h`.
- [engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md)
  — generic layer and the fallback behaviour contract (running steppers
  skipped, empty queue does not stop the others).
