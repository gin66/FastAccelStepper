# Common head speed

Priority: **070** — later planner, not v1.

Status: not started.

## What

A tool head with axes x, y, z, … moves under **one acceleration** and
**one maximum path speed**. The waypoint is the relative move plus
the speed of that segment:

```text
(dx, dy, dz, …, v)
```

`v` is how fast the head should travel along that segment. The
common maximum is the ceiling: a segment never runs faster than
that, and never faster than `v`. The common acceleration ramps the
head speed between segments. Each motor's step rate is the share of
that path speed, so the axes stay on the segment.

This is the planner that draws the ramp. `FasNAxis` has no requested
speed (it goes as fast as the motors allow). `FasTimed` takes a
duration in ticks and holds one rate for the whole waypoint.

## Open

The unit of `v`. An axis speed in this library is ticks per step. A
head speed is a rate along the path. The call has to pick one and
keep the axis limits (`getMaxSpeedInTicks`, `getAcceleration`) as
hard ceilings on top of the common head limits.

## References

- `src/FasNAxis.h` — as-fast-as-possible polyline, no `v`
- `src/FasTimed.h` — timed chunks, no ramp
- `extras/doc/planner_modes.md` — the two existing planners
