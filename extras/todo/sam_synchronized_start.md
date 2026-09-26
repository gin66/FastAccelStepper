# SAM synchronized start

Priority: **P5** — platform-specific part of the engine synchronized start
(one item per open platform).

Status: generic fallback active on SAM (Due)
(`fasDisableInterrupts()` / `fasEnableInterrupts()` around a per-stepper
`addQueueEntry(NULL, true)` loop, see
[engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md)).
Native release unverified.

## Problem

SAM steppers are driven from PWM channels plus a TC handler. Each queue arms
its own PWM compare/period state in `startQueue()`, so a single critical
section around the per-stepper starts does not necessarily make the first
PWM compare events coincide. Whether a common release point exists (e.g. one
`TC_CCR_SWTRG` / PWM enable covering all armed channels) is not decided yet.

## Work items

- Inspect `startQueue()` and the PWM/TC arming sequence: which registers are
  written per queue, and which one actually releases the first compare.
- Decide whether all armed channels can be released with one TC trigger or
  one PWM interface enable.
- If yes, implement the group release in the
  `synchronizedStart()` implementation and drop the critical
  section.
- If no, keep the critical section, document it as final, and close this
  item.

## References

- `src/pd_sam/sam_queue.cpp` — `startQueue()`, `TC5_Handler()`,
  `PWM_Handler()`, and the `synchronizedStart()`
  implementation.
- `src/fas_arch/arduino_sam.h` — `fasDisableInterrupts()` /
  `fasEnableInterrupts()`.
- [engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md)
  — generic layer and the fallback behaviour contract (running steppers
  skipped, empty queue does not stop the others).
