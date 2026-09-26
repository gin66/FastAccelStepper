# ESP32 synchronized start

Priority: **P5** — platform-specific part of the engine synchronized start
(one item per open platform).

Status: generic fallback active on all ESP32 builds
(`fasDisableInterrupts()` / `fasEnableInterrupts()` around a per-stepper
`addQueueEntry(NULL, true)` loop, see
[engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md)). Native per-driver
release pending.

## Problem

ESP32 has three stepper drivers in one build, each with a different start
point. A single critical section does not guarantee that all steppers' first
steps share one hardware start event, especially across drivers (I2S DMA,
RMT groups, MCPWM/PCNT timers).

## Work items

- **I2S** — steppers on an I2S channel share clock/DMA; implement a group
  release: arm the first step of every stepper on the channel, start the
  channel (DMA) once, so all I2S axes' first steps leave together.
- **RMT** — RMT supports synchronized group start; pick the right start
  sequence (group start vs. per-channel burst start) so all RMT steppers'
  first step edges coincide.
- **MCPWM/PCNT** — to see: MCPWM/PCNT steppers run from PCNT timer events;
  decide whether their starts are already phase-locked to one timer, or need
  an explicit one-time trigger.
- **Mixed drivers in one `synchronizedStart()` call** — one release point per
  driver, coordinated in
  `FastAccelStepperEngine::synchronizedStart()` so I2S + RMT + MCPWM/PCNT
  steppers still start on one event (same as the current fallback: one
  critical section, now extended to per-driver group release).

## References

- `src/pd_esp32/esp32_queue.cpp` — fallback `synchronizedStart()`.
- `src/pd_esp32/i2s_manager.h` — I2S channel/DMA start points.
- `src/FasNAxis.h` — kick-off through the engine reference.
- [engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md) — generic
  layer and the fallback behaviour contract (running steppers skipped,
  empty queue does not stop the others).
