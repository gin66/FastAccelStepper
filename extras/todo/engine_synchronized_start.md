# Engine synchronized start

Priority: **P5** — several steppers should begin on one start, not
one after another.

Status: not started.

## Problem

`addQueueEntry(NULL, true)` only starts that stepper's queue. The
comment in `FastAccelStepper.h` calls a series of those calls, with
interrupts off, a near synchronous start. `FasNAxis::pump()` does
exactly that loop after the prefill: one `addQueueEntry(NULL, true)`
per axis.

The first step of each axis is then separated by whatever that loop
takes. There is no engine call that arms the queues and releases them
together.

## Fix

`FastAccelStepperEngine` should start a chosen set of steppers on one
event, so their first commands begin together. Prefill stays
`start=false`. The release is one engine operation, not N separate
`startQueue()` calls.

The mechanism is per platform (AVR timer, ESP32 RMT, Pico PIO, SAM).
The API is on the engine. FasNAxis should use it for the kick-off.

## References

- `src/FastAccelStepper.h` — `addQueueEntry(NULL, true)` near-sync note.
- `src/FasNAxis.h` — `pump()` kick-off loop.
- `src/FastAccelStepperEngine.cpp` — `manageSteppers()`.
