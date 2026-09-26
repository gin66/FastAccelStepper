# AVR synchronized start

Priority: **P5** — platform-specific part of the engine synchronized start
(one item per open platform).

Status: generic fallback active on AVR
(`fasDisableInterrupts()` / `fasEnableInterrupts()` around a per-stepper
`addQueueEntry(NULL, true)` loop, see
[engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md)).
Likely already final; verify and close.

## Problem

On AVR the two/three step channels share one timer (Timer1, Timer3 or
Timer5). The timer runs continuously and `startQueue()` only arms the
channel's compare register and enables its compare interrupt. Because all
channels count the same counter, enabling the compare interrupts under one
critical section should already align the first compare events to within the
interrupt-enable latency. There is no per-channel timer restart to
synchronize, so the critical section may be the final mechanism.

## Work items

- Confirm the shared-counter argument: all participating channels use one
  `FAS_TIMER_MODULE` and only the compare interrupt enable differs per
  channel.
- If confirmed, keep the critical section, document it as final, and close
  this item.
- If a cheaper group arm exists (write all `OCRnx` first, then set all
  compare-enable bits with one masked write), use it and drop the critical
  section.

## References

- `src/pd_avr/avr_queue.cpp` — `startQueue()`, the four `AVR_START_QUEUE`
  paths, and the `synchronizedStart()` implementation.
- `src/fas_arch/arduino_avr.h` — `fasDisableInterrupts()` /
  `fasEnableInterrupts()`.
- [engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md)
  — generic layer and the fallback behaviour contract (running steppers
  skipped, empty queue does not stop the others).
