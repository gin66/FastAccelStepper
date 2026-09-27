# FasNAxis

Experimental. The calls below may change.

`FasNAxis` runs a polyline as fast as the motors and the geometry allow.
There is no requested speed and no requested time. A waypoint that carries
its own duration belongs to [FasTimed](FasTimed.md).

The header is not included by `FastAccelStepper.h`.

```cpp
#include "FastAccelStepper.h"
#include "FasNAxis.h"

FasNAxis<NAXES, HORIZON, Stepper, Engine>
```

`NAXES` is the axis count. `HORIZON` is how many committed blocks are kept
ahead of the motion (default 64). `Stepper` and `Engine` default to
`FastAccelStepper` and `FastAccelStepperEngine`.

A full sketch is in [examples/NaxesAFAP](../../examples/NaxesAFAP/README.md). The
motion rules are in the [whitepaper](n_axes_whitepaper.md).

## Setup

```cpp
explicit FasNAxis(const FasNAxisConfig& cfg, Engine& engine);
bool addAxis(uint8_t i, Stepper* s);
void setLimitsFromSteppers();
void syncFromSteppers();
void setCurrentPosition(const int32_t p[NAXES]);
```

Construct with `FasNAxisConfig{}` for the defaults. `addAxis` attaches
stepper `s` as axis `i` and reads that stepper's speed and acceleration.
It returns false when `i` is out of range, `s` is null, or the stepper is
already running. After a later `setSpeedInHz` or `setAcceleration`, call
`setLimitsFromSteppers` so the planner sees the new limits.

`syncFromSteppers` copies each stepper's position and opens the path.
`setCurrentPosition` does the same from a caller-supplied array. Waypoints
are illegal until one of these has been called.

### FasNAxisConfig

| Field | Default | Meaning |
| --- | --- | --- |
| `mode` | `Linear` | `Linear` stops at a corner. `Overshoot` may leave the chord by up to `overshoot_max` steps. |
| `dt_ticks` | 32000 | Planning slice, in driver ticks. 0 selects this default. |
| `kappa_stop_q8` | 320 | Corner threshold (1.25 in Q8). 0 selects this default. |
| `overshoot_max` | 8 | Steps the path may leave the chord in `Overshoot`. Ignored in `Linear`. |
| `dir_before_ticks` | 0 | Pause on the old direction before a change. 0 uses the stepper's own delay. |
| `dir_after_ticks` | 0 | Pause on the new direction after a change. 0 uses the stepper's own delay. |

## Path

```cpp
bool addWaypoint(const int32_t p[NAXES]);
bool addDwellTicks(uint32_t ticks);
void endPath();
uint32_t block_count() const;
```

`addWaypoint` appends an absolute position in steps. It returns false
before the position has been synced, and when `HORIZON` blocks are already
waiting. A position equal to the current one records nothing and returns
true. Call `pump` and try the same point again when the ring is full.

`addDwellTicks` waits `ticks` driver ticks on every axis, from a stop to a
stop. Zero ticks records nothing and returns true. The same false cases as
`addWaypoint` apply.

`endPath` marks the last committed point as a stop. `block_count` is how
many motion blocks `addWaypoint` has recorded since the last sync.

A small `HORIZON` makes the planner slow down so it can still stop inside
the buffered points. That slowdown is not an error.
`isSpeedLimitedByLookahead()` reports it.

## Pump

```cpp
PumpStatus pump();
bool isBusy() const;
bool hasUnderrun() const;
```

Call `pump` from the main loop. The first call fills the queues and starts
them together. Later calls keep them fed.

| `PumpStatus` | Meaning |
| --- | --- |
| `Idle` | Nothing left to feed, and the queues are empty. |
| `Running` | The path is being fed. |
| `Underrun` | A queue ran dry after the start while the path still has motion. Stays set until `syncFromSteppers`, `setCurrentPosition`, or `clearFault`. |
| `Error` | A queue rejected a command. Fix the cause before continuing. |
| `Stopped` | A member axis was stopped outside the planner. Positions are untrusted until `syncFromSteppers` or `setCurrentPosition`. |

`isBusy` is true while blocks or queue entries remain. `hasUnderrun` is
true once a queue has run dry after the start.

## Stop

```cpp
bool isFaulted() const;
void clearFault();
void emergencyStop();
```

`emergencyStop` calls `forceStop` on every attached axis and aborts the
plan. `isFaulted` stays true until `syncFromSteppers`,
`setCurrentPosition`, or `clearFault`. `clearFault` drops the fault flag
and leaves the positions as they are.
