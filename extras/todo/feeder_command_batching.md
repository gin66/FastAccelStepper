# Feeder command batching (command length vs pump interval)

Priority: **P2** — AVR/ESP32 queue-drain safety; needed for the naxes
example to run on hardware without underrun.

Status: design agreed, not implemented.

## Problem

The feeder emits one command per master step (the natural Linear slice).
Each command must satisfy `ticks * steps >= MIN_CMD_TICKS`, but a
one-step command is as short as `MIN_CMD_TICKS` (40 us on AVR, 200 us on
ESP32/Pico/SAM). With `QUEUE_LEN - 2` usable slots the queue then only
guarantees

| Platform | `MIN_CMD_TICKS` | `QUEUE_LEN - 2` | Worst-case queue |
|----------|----------------|-----------------|------------------|
| AVR | 640 (40 us) | 14 | 560 us |
| ESP32 / Pico / SAM | 3200 (200 us) | 30 | 6 ms |

If the application calls `pump()` only every `pump_interval`, the queue
drains unless the **average accepted command lasts at least**

```
command_ticks >= pump_interval / (QUEUE_LEN - 2)
```

On AVR at a 4 ms `pump_interval` that is ~286 us, about 7 master steps
at 40 us. ESP32's 200 us floor already covers 4 ms at `QUEUE_LEN = 32`.

## Fix

Batch several master steps into one command (`steps > 1`, constant
period across the batch) so each accepted command spans
`command_ticks`, while never letting a batch cross a master-role switch
or a path-stop unprepared (see `linear_junction_carry.md`).

## Cost

A batch holds one period while the ramp would have varied it over the
batch. For a batch of `n` master steps the period changes by
`delta tau ~ tau' * n`, so the path lags/leads by up to `~n * delta tau`
master-step times. The chord is still hit at the batch boundary, but the
in-batch DDA spacing uses the batch period. Keep `n` small enough that
`n * delta tau` is a small fraction of a slave step, and cap the batch at
the next prepared boundary.

This is the same granularity trade FAS makes with its 2 ms planning
chunks; it must be a stated feeder rule, not hidden in a `pump()`
frequency assumption.

## Implementation surface

- `src/FasNAxis.h`: `feed_one()` / `feed_loop()` batching by master step.
- Tests: F14 (feeder drift), F15 (queue room), F2d/F20 (command stream
  period noise), a new fixture that runs with a 4 ms `pump()`.

## References

- `extras/doc/n_axes_whitepaper.md` §4.2, §4.3, §4.3.1, §9.3
