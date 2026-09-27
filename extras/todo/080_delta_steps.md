# Delta steps instead of waypoints

Priority: **080** — input variation on the working FasNAxis planner.
Behind the current 050–070 queue.

Status: not started.

## Call

`addWaypoint(const int32_t p[NAXES])` takes an absolute position and
stores `p[i] - _p[i]` in the `int32_t` block ring. The variation takes
the relative chunk directly:

```c
bool addDelta(const int16_t d[NAXES]);
```

Each component is a signed step count, at most 32767 steps. A longer
line is several calls. The running position stays the `int32_t` sum
the planner already keeps (`_p`). `setCurrentPosition` and
`syncFromSteppers` stay the absolute entry points; both clear the
path, so a re-home still rebases the sum.

`int16_t` matches `moveTimed`'s step argument. The block ring can stay
`int32_t`: the limit is the size of one call, not the length of a
merged block.

All-zero is a no-op, as a repeated waypoint is today. A pause stays
`addDwellTicks(uint32_t)`. One call has one sign per axis; a reversal
is two calls.

## Why

The caller streams motor steps. One call cannot carry a segment of
hundreds of thousands of steps, so lookahead sees the path at the
grain the application produced. The timed implementation uses the
same step vector plus a tick count
(`extras/doc/implemented/timed_trajectory.md`). The timed call
caps each component at ±128 and the shared duration at
[MIN_CMD_TICKS, 65535], which is tighter than this AFAP chunk.

## Variations

- **Coalesce collinear chunks.** Consecutive deltas that are not a
  P1 hard stop merge into one block, so `HORIZON` counts joints.
  Without a merge, a straight line split every 32767 steps takes one
  ring slot per chunk, and a slot boundary can become a rest.
- **Absolute calls with the same grain.** `addWaypoint` subtracts the
  planned position and accepts the call only when every axis fits in
  `int16_t`. A farther target is the caller's job to split.
- **Keep both uncapped.** Absolute waypoints stay full `int32_t`
  ("go here"); deltas are only the streaming form.
- **Position check.** `expectPosition(const int32_t p[NAXES])`
  compares the planned sum with an absolute checkpoint and returns
  false on a missed chunk. It does not move.

## References

- `src/FasNAxis.h` — `addWaypoint`, `_blk`, `_p`
- `src/FastAccelStepper.h` — `moveTimed(int16_t steps, ...)`
- `extras/doc/implemented/timed_trajectory.md` — same step vector plus ticks
