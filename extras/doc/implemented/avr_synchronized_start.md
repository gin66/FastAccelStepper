# AVR synchronized start

Priority: **050** — platform-specific part of the engine synchronized start
(one item per open platform).

Status: **implemented**.

A native AVR release is implemented for all platforms:

- 328P / 168P (no channel C): two-phase arm + trigger — arm both `OCRnA`
  and `OCRnB` with one shared compare value, then clear both flags and
  enable both compare interrupts in one critical section (328P has exactly
  two steppers, no bitmask needed).
- 2560 / 32U4 (with channel C): bitmask approach — any 2 or all 3
  channels, same two-phase arm + trigger.

All channels share one `FAS_TIMER_MODULE`. The arm loop writes one target
captured from the counter (`AVR_SYNC_TARGET(T) + AVR_SYNC_ARM_DELAY`, both
in `src/pd_avr/avr_queue.cpp`) into every participating `OCRnX`, so the
compare matches fall on the same timer tick. The delay only has to exceed
the arm loop and moves the start by that many ticks.

The generic fallback remains the `test` platform.

See [engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md) for the
generic contract.

## Problem

On AVR the two/three step channels share one timer (Timer1, Timer3 or
Timer5). The timer runs continuously and `startQueue()` only arms the
channel's compare register and enables its compare interrupt. Enabling all
compare interrupts under one critical section is not sufficient by itself:
arming each channel with `TCNT + offset` leaves the first compare events
separated by the time the arm loop needs between the channels (measured as
~47 ticks on the 328P and ~112 on the 2560). Arming all channels with one
captured target removes that skew.

## References

- `src/pd_avr/avr_queue.cpp` — `startQueue()`, the four `AVR_START_QUEUE`
  paths, and the `synchronizedStart()` implementation.
- `src/fas_arch/arduino_avr.h` — `fasDisableInterrupts()` /
  `fasEnableInterrupts()`.
- [engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md)
  — generic layer and the fallback behaviour contract (running steppers
  skipped, empty queue does not stop the others).
