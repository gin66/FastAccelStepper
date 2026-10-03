# Smooth stop when the path ends

Priority: **090** — open design on the working FasNAxis planner.
Behind the current 050–070 queue.

Status: undecided. Two mechanisms; the coordinated one fits the
planner that exists.

## What happens today

`endPath()` marks the last committed point as rest and freezes `R`.
The ramp law in `src/fas_naxis/ramp_law.h` then counts `P` down one
step at a time while `R` counts down. When the remaining steps were
already at least `P`, both hit zero together and the stop is the
usual linear decel along the chord.

When the frozen remainder is shorter than `P` (`R < P`), each issued
step still only decrements `P` by one, and motion ends when `R` hits
zero. `P` is still high, so the last steps go out at nearly the
current period and the axis stops there. That is an abrupt stop on
the waypoint.

The single-axis ramp generator does the other thing. In
`_getNextCommand`, `remaining_steps < performed_ramp_up_steps` selects
`RAMP_STATE_REVERSE` and sets `remaining_steps` to the performed ramp
count. The motor runs the full decel and comes to rest past the
original target.

## Preferred: extend the path

On `endPath()`, if the unexecuted remainder is shorter than the
stop distance, append a tail along the last block's direction whose
length covers the missing steps. The existing ramp law then reaches
`P == 0` at the configured acceleration, on the same chord (Linear)
or the same shared time (Overshoot).

Committed waypoints stay where they were. The tail is after the last
waypoint, and the rest position is that far past it. `endPath()` with
enough remaining steps appends nothing.

A separate `stopPath()` drops the unexecuted suffix and appends the
decel from here along the current direction. `endPath()` completes
the committed polyline; `stopPath()` abandons it. Both are
coordinated: one tail, every axis, one clock.

## Alternative: hand the stop to the ramp generator

Seed each stepper's ramp with the live `performed_ramp_up_steps`,
`curr_ticks`, and `ramp_time_ticks` (`080_move_to_eta.md`) and let
`_getNextCommand` run the overshoot stop above. This is a poor fit
for a coordinated path:

- Each axis stops on its own ramp and leaves the chord, unless the
  handover also invents a per-axis step budget that shares one decel
  time. That budget is the tail already described.
- The queue must not already hold later FasNAxis commands, and the
  ramp state has to be seeded because the ramp generator did not
  issue the steps that got the motor up to speed.

Handing over only once `P == 0` and the queues are idle returns the
axes to ordinary `moveTo`. It does not perform the deceleration; the
tail does. After a finished path the steppers are idle and a later
`moveTo` already addresses them, so a standstill handover adds no
motion.

## Variations

- **Tail length from ramp time.** Once `ramp_time_ticks` exists, the
  tail's duration is that time and the step count is `P`. The two
  have to agree with `calculate_ticks` on the linear map; a mismatch
  is a bug in the counter.
- **Keep the waypoint exact.** Refuse `endPath()` when the remainder
  is shorter than `P`, and require the caller to have streamed the
  decel. That preserves "the last point is the rest position" and
  leaves the abrupt-stop case to the caller.
- **Seeded handover for one axis.** After `stopPath()` has the
  machine at rest, or for a path that was always a single axis, the
  ramp generator can own the stop. Multi-axis coordinated stops stay
  on the appended tail.

## References

- `src/FasNAxis.h` — `endPath()`
- `src/fas_naxis/ramp_law.h` — `R < P` decrements `P` once per step
- `src/fas_ramp/RampControl.cpp` — `remaining_steps < performed`
  takes `RAMP_STATE_REVERSE`
- `extras/todo/080_move_to_eta.md` — ramp time used to seed a handover
