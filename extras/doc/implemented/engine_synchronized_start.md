# Engine synchronized start

Priority: **P5** — several steppers should begin on one start, not
one after another.

Status: **implemented**. `FasNAxis::pump()` kick-off goes through
`FastAccelStepperEngine::synchronizedStart()`, which the planner receives
as an engine reference in its constructor. Every platform implements it
today as one critical section around a per-stepper
`addQueueEntry(NULL, true)` loop. The per-platform native mechanisms are
tracked in `extras/todo/`:
[AVR](../../todo/avr_synchronized_start.md),
[SAM (Due)](../../todo/sam_synchronized_start.md),
[SAMD51](../../todo/samd51_synchronized_start.md),
[Teensy](../../todo/teensy_synchronized_start.md),
[ESP32](../../todo/esp32_synchronized_start.md),
[Pico](../../todo/pico_synchronized_start.md).

## Problem

`addQueueEntry(NULL, true)` only starts that stepper's queue. The
comment in `FastAccelStepper.h` calls a series of those calls, with
interrupts off, a near synchronous start. `FasNAxis::pump()` does
exactly that loop after the prefill: one `addQueueEntry(NULL, true)`
per axis.

The first step of each axis is then separated by whatever that loop
takes. There is no engine call that arms the queues and releases them
together.

## Done (generic layer)

- Engine API: `FastAccelStepperEngine::synchronizedStart(FastAccelStepper**
  const, uint8_t)` — a plain non-static engine member (the engine only ever
  manages `FastAccelStepper`). One engine operation releases the listed
  steppers' queues; returns `AqeResultCode::OK` or the first per-stepper
  error.
- `FasNAxis` receives the engine as a constructor argument
  (`FasNAxis<NAXES, HORIZON, Stepper, Engine> planner(cfg, engine)`). The
  `Engine` template parameter defaults to `FastAccelStepperEngine`, so user
  code writes `FasNAxis<2, 64>`; the test world names its own
  `TestFastAccelStepperEngine` (in `naxis_sim_port.h`) for the duck-typed
  `SimPort`.
- `FasNAxis::pump()` kick-off calls `_engine->synchronizedStart()` on the
  active queues, replacing the per-axis `addQueueEntry(NULL, true)` loop.
- The implementation (one critical section around the per-stepper starts)
  lives in the platform queue file of every build:
  `pd_avr/avr_queue.cpp`, `pd_esp32/esp32_queue.cpp`, `pd_pico/pico_queue.cpp`,
  `pd_sam/sam_queue.cpp`, `pd_samd/samd_queue.cpp`,
  `pd_teensy/teensy_queue.cpp`, and the PC test world
  (`extras/tests/pc_based/StepperISR_test.cpp`). A stepper already running is
  skipped; an empty queue does not stop the others (see
  `synchronizedStart()` doc).
- The naxes example constructs the planner with the engine:
  `FasNAxis<NAXES_HW, NAXES_HORIZON> naxes_planner(FasNAxisConfig{}, engine)`.

## Platform-specific follow-ups

The critical-section fallback is in place for every platform. Whether it
is the final mechanism, or a native start primitive should replace it, is
tracked one file per platform in `extras/todo/`:

- [AVR](../../todo/avr_synchronized_start.md)
- [SAM (Due)](../../todo/sam_synchronized_start.md)
- [SAMD51](../../todo/samd51_synchronized_start.md)
- [Teensy](../../todo/teensy_synchronized_start.md)
- [ESP32](../../todo/esp32_synchronized_start.md)
- [Pico](../../todo/pico_synchronized_start.md)

## References

- `src/FastAccelStepper.h` — `addQueueEntry(NULL, true)` near-sync note.
- `src/FasNAxis.h` — `pump()` kick-off, engine constructor argument.
- `src/FastAccelStepperEngine.h` — `synchronizedStart()`.
- `examples/naxes/naxes.ino` — planner constructed with the engine.
- `extras/tests/pc_based/StepperISR_test.cpp` — PC implementation.
- `extras/tests/pc_based/naxis_sim_port.h` —
  `TestFastAccelStepperEngine`.