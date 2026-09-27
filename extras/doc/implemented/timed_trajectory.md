# Faithful timed trajectory

Status: implemented, as `FasTimed` in `src/FasTimed.h`.
PC test: `extras/tests/pc_based/test_29.cpp`.

This is planner problem (2) from `extras/doc/n_axes_whitepaper.md`
§3.3. It is not a `FasNAxis` mode. See `extras/doc/planner_modes.md`.

## Waypoint

One call is one constant rate. The planner does not draw a curve
between calls.

```c
TimedAdd addDelta(const int16_t steps[NAXES], uint16_t ticks);
```

- `steps[i]` is in **[-128, 128]**. A larger magnitude is
  `Rejected`. A longer run is several calls.
- `ticks` is the shared duration of those steps, in driver ticks
  (`TICKS_PER_S`). It must lie in **[MIN_CMD_TICKS, 65535]**.
  The type caps the top; a smaller value is `Rejected`.
- All-zero steps are a dwell on that common clock.
- One call has one sign per axis. A reversal is two calls, and
  the axis must already be at ramp-step 0.

The chunk ring defaults to 8. A chunk is already one queue command,
so a longer move streams: `addDelta` until the ring is full, `pump`,
and `addDelta` again.

`128` is the largest magnitude that still fits one queue
command's step count with room for a second command when the
duration does not divide the step count. `65535` is the largest
`stepper_command_s.ticks` value, so one waypoint's wall time fits
in one command field.

## What gets queued

Each axis emits one or two `addQueueEntry` commands. The tick sums
are equal across axes. A pause (`steps == 0`) keeps the last
direction.

When the requested duration is already a legal split (one period,
or a `q` / `q+1` pair whose commands each span at least
`MIN_CMD_TICKS`), that duration is issued. Otherwise the shared
sum moves to the nearest legal duration, searching shorter first,
and no farther than the longest `|delta|` of the call. A wider
gap is `TimingNotAchievable` rather than a different rate.
`actualTicks()` is the sum of the durations actually committed.

## Feasibility

`TimingNotAchievable` (not `Rejected`, not a feed-hold):

- A moving axis would step faster than `getMaxSpeedInTicks()`.
- The ramp-step change from the previous chunk, from
  `RampMap::calculate_ramp_steps`, is larger than this chunk's
  step count.
- The sign flips, or an axis is given zero steps, while its
  previous ramp-step is not 0.

A request that is too fast is not stretched until it fits. A
request that is only a few ticks off the command grid may be
quantized as above. Limits are the values read at `addAxis`.

An injected direction-change pause the plan did not emit makes
`pump()` return `Error`. An external stop latches `Stopped` until
`syncFromSteppers()` or `setCurrentPosition()`. An empty queue
after kick-off, while chunks remain, latches `Underrun`; the
feeder still delivers the rest.

## Left out

- No comparison against the `naxis_ref` duration of an AFAP track.
  The ramp-step check above is the motor limit for one constant
  rate against the previous rate.
- A ~2× period jump between chunks is not a separate error. The
  feeder's own split changes a period by one tick. A caller who
  wants a smooth ramp submits chunks whose ramp-steps fit.
- Direction-change pauses are not carved. A driver that injects
  one is `Error`.
