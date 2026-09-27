# AVR synchronized start

Priority: **050** — platform-specific part of the engine synchronized start
(one item per open platform).

Status: **implemented** (EXPERIMENTAL).

A native AVR release is implemented for all platforms:

- 328P / 168P (no channel C): two-phase arm + trigger — write 20-tick
  offset to both `OCRnA` and `OCRnB`, then clear both flags and enable
  both compare interrupts in one critical section (328P has exactly two
  steppers, no bitmask needed).
- 2560 / 32U4 (with channel C): bitmask approach — any 2 or all 3
  channels, same two-phase arm + trigger.

The generic fallback remains the `test` platform.

See [engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md) for the
generic contract.

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
