# 130 Interrupt slow steps (e.g. 1 step/s)

## Problem

At very low step rates (e.g. 1 step per second), the current ramp
generator is **not interruptible** — a queued command cannot be aborted
or preempted until the long-duration step completes.  This makes
`abort()` / `reset()` effectively non-functional at slow speeds.

## Root cause

The pulse driver waits for the full step period (which may be tens of
thousands of timer ticks) before checking whether a higher-priority
command has arrived.  The interrupt check happens only at queue-drain
boundaries, which are spaced by the current step period.

## Desired behaviour

- A slow step (any period) should be **preemptible** as soon as a new
  queue command arrives.
- The interrupt should complete the current step (or abort it cleanly)
  and honour the new command without violating signal-integrity
  constraints.

## Implementation approach

1. **Periodic interrupt check** — In the timer ISR (or a dedicated
   watchdog timer), check for new queue entries at a fixed interval
   (e.g. every 100 µs) regardless of the current step period.
2. **Safe abort point** — When a slow step is interrupted, ensure the
   current step pulse completes (or is aborted at a defined boundary)
   before switching to the new command.  No partial/glitched pulses.
3. **Platform differences** — AVR (timer 1/3/5), ESP32 (RMT/MCPWM),
   Pico (PIO), SAM (TC/PWM) each have different timer capabilities.
   A common abstraction layer should hide these differences.

## Acceptance criteria

- 1 step/s is interruptible within a bounded time (< 1 ms after new
  command).
- No signal glitches on interrupt.
- Existing tests (`test_01`–`test_17`) still pass.

## Status

_idea — not started_
