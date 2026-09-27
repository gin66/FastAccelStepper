# FasTimed

Experimental. The calls below may change.

`FasTimed` runs a waypoint at one constant speed for a duration the caller
supplies. It does not draw a ramp between waypoints. A polyline with no
requested time belongs to [FasNAxis](FasNAxis.md).

The header is not included by `FastAccelStepper.h`.

```cpp
#include "FastAccelStepper.h"
#include "FasTimed.h"

FasTimed<NAXES, HORIZON, Stepper, Engine>
```

`NAXES` is the axis count. `HORIZON` is how many chunks may wait to be
queued (default 8). `Stepper` and `Engine` default to `FastAccelStepper`
and `FastAccelStepperEngine`.

A hardware sketch that plays a one-second rise and a one-second fall on
each side of a diamond is [examples/NaxesTimed](../../examples/NaxesTimed/README.md).

Each chunk is already one queue command, so the ring stays small. A longer
move adds chunks, calls `pump` when `addDelta` returns `Rejected`, and
adds again.

## Setup

```cpp
explicit FasTimed(Engine& engine);
bool addAxis(uint8_t i, Stepper* s);
void syncFromSteppers();
void setCurrentPosition(const int32_t p[NAXES]);
```

`addAxis` attaches stepper `s` as axis `i` and reads that stepper's speed
and acceleration. Those limits stay until the next `addAxis` for that
index. The call returns false when `i` is out of range, `s` is null, or
the stepper is already running.

`syncFromSteppers` copies each stepper's position and allows `addDelta`.
`setCurrentPosition` does the same from a caller-supplied array. Both drop
any chunks not yet issued.

## Waypoint

```cpp
TimedAdd addDelta(const int16_t steps[NAXES], uint16_t ticks);
int32_t plannedPosition(uint8_t i) const;
uint32_t actualTicks() const;
```

One call is one rate. `steps[i]` is the delta of axis `i`, in [-128, 128].
`ticks` is the shared duration of those steps, in driver ticks, from
`MIN_CMD_TICKS` through 65535. All-zero steps are a dwell on that clock.
One call has one sign per axis. A reversal is a later call, and that axis
must already be stopped.

On `Ok` the planned position advances by `steps`, and `actualTicks()` grows
by the duration that will be issued. That duration is the requested
`ticks`, or the nearest value the queues can emit, and it never moves by
more than the longest `|delta|` of the call. `plannedPosition` is the
planned coordinate, not the stepper's live position.

| `TimedAdd` | Meaning |
| --- | --- |
| `Ok` | The chunk was accepted. |
| `Rejected` | Not synced, an axis is missing, the ring is full, a step is outside ±128, or `ticks` is below `MIN_CMD_TICKS`. |
| `TimingNotAchievable` | The chunk is faster than an axis allows, or the speed change from the previous chunk does not fit in this chunk's steps. The call is not rewritten into a slower one. |

A smooth start or stop is a sequence of chunks whose speeds the motors can
reach from one chunk to the next.

## Pump

```cpp
TimedStatus pump();
bool isBusy() const;
```

Call `pump` from the main loop. The first call fills the queues and starts
them together. A retryable queue-full result holds that command; the next
`pump` sends it again.

| `TimedStatus` | Meaning |
| --- | --- |
| `Idle` | Nothing left to feed, and the queues are empty. |
| `Running` | Chunks are being fed. |
| `Underrun` | A queue ran dry after the start while chunks remain. Stays set until `syncFromSteppers` or `setCurrentPosition`. |
| `Error` | The driver inserted a direction pause this planner did not emit. |
| `Stopped` | An axis was stopped outside the planner. Stays set until `syncFromSteppers` or `setCurrentPosition`. |

`isBusy` is true while chunks, a held command, or queue entries remain.
