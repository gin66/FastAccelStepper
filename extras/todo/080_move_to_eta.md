# Ramp time, and moveTo with an arrival tick

Priority: **080** — single-axis ramp generator. Behind the current
050–070 queue. Independent of FasNAxis.

Status: not started.

## State

`ramp_rw_s` stores `performed_ramp_up_steps`. `stepsToStop()` returns
that count: under the linear ramp, decelerating from here takes the
same number of steps.

Add `ramp_time_ticks` (`uint32_t`) beside it: the tick sum of the
commands that built the current speed.

- Accelerate: add this command's tick sum (`ticks * steps`, including
  a pause split).
- Coast: hold the value. It is the time the acceleration took.
- Decelerate: subtract this command's tick sum.

On the linear ramp the period is `calculate_ticks(P)`, and
deceleration replays those periods, so the time left to stop equals
the stored ramp time. The PC check is: after a pure acceleration to
`P`, `ramp_time_ticks` equals the sum of `calculate_ticks(p)` for
`p = 1 .. P`.

`ticksToStop()` is the sibling of `stepsToStop()` and returns that
stored time. Arrival from here is:

- decelerating: `ramp_time_ticks`
- coasting: remaining coast ticks + `ramp_time_ticks`
- still accelerating: the forecast at the chosen peak (time still to
  accelerate, plus a coast, plus a decel of that same future ramp
  time)

Both the stored time and the public eta are `uint32_t` driver ticks.
A ramp whose tick sum passes 2^32 − 1 (~268 s at 16 MHz) is outside
this feature; `src/` stays free of 64-bit counters. The counter
saturates, and an eta past the saturate point is not planned.

Cubic `s_h` (060) does not reverse into the same tick sum. This item
is the linear ramp. A cubic move keeps today's as-fast-as-possible
`moveTo` until the cubic profile has its own reverse.

## Call

The ramp generator gains an arrival:

```c
MoveResultCode moveTo(int32_t position, uint32_t eta_ticks);
```

`eta_ticks` is the duration from this call, in driver ticks, not
seconds and not a wall-clock stamp. The existing
`moveTo(int32_t position, bool blocking = false)` stays the
as-fast-as-possible move and keeps the blocking flag.

An untyped integer literal is ambiguous between `bool` and
`uint32_t` in C++. Callers pass a `uint32_t`. If that overload is
too sharp in practice, the same behavior ships as a distinct name
(`moveToInTicks`) so `moveTo(position, blocking)` cannot change
meaning.

The move uses the configured acceleration. Only the speed ceiling
changes: lower the cruise (larger travel ticks for this move only,
not a sticky `setSpeed`) until the predicted duration meets `eta`.
The as-fast-as-possible duration is the forecast at the configured
max speed. When `eta` is shorter than that, the call returns a new
`MoveResultCode` and does not start. When `eta` is longer, the
ceiling is the fastest cruise whose forecast still finishes by
`eta` (quantized to the ramp map, so it may land a few ticks
early).

A second call while the motor is running replans from the stored
ramp time, which is the decel already committed, and from the tick
sum already issued on this move. A reversal is ignored, as it is
for `moveTo` today. If the remaining budget is shorter than the
time to stop, the call returns the same error and the ramp runs
down as already planned.

## Variations

- **`move(int32_t delta, uint32_t eta_ticks)`** — the relative twin.
- **Leading dwell.** Spend the spare ticks as a pause first, then
  run the ordinary ramp, so the last step lands on `eta` without
  lowering the cruise. The primary behavior is the speed ceiling;
  the dwell is the alternative when the matched cruise would be
  slower than the motion should creep, or when the profile shape
  must stay the as-fast-as-possible ramp.
- **FasNAxis forecast.** The coordinated planner can expose the
  same "ticks still to go" from its binder ramp (issued tick sum
  plus the symmetric decel). A deadline on `endPath` belongs to
  `070_timed_trajectory.md`, not to a speed cap inside `FasNAxis`.

## References

- `src/fas_ramp/RampControl.h` — `ramp_rw_s::performed_ramp_up_steps`
- `src/fas_ramp/RampGenerator.h` — `stepsToStop()`
- `src/fas_ramp/RampControl.cpp` — `_getNextCommand` updates the
  performed step count after each command
- `src/FastAccelStepper.h` — `moveTo`, `MoveResultCode`
- `extras/todo/060_cubic_start.md` — cubic reverse is a different sum
- `extras/todo/070_timed_trajectory.md` — path-level deadline
