# Faithful timed trajectory

Priority: **070** — later implementation, not v1.

Status: later implementation, not v1.

## Separation

Per `extras/doc/planner_modes.md` this is a **complete distinct implementation**,
not a `FasNAxis` mode: its own lookahead/block planner, interpolator and
feeder. Architecturally separate, but code reuse is intended where it
makes sense — shared conventions (`addQueueEntry` contract, pin setup),
`SimPort`, the PC test rig, and common helpers factored out rather than
copied.

## Input / output

- **Input**: a sequence of chunks. Each chunk is `int16_t` delta
  steps per axis and one shared `uint32_t` delta ticks (driver
  ticks, `TICKS_PER_S`, the duration of those steps).

  ```c
  bool addDelta(const int16_t steps[N], uint32_t delta_ticks);
  ```

  There is no absolute waypoint and no time in seconds. A caller
  who sampled a curve a second apart has to turn that into the
  steps that actually occur and the tick count of each piece
  before calling in. One call is one rate: `delta_ticks` is the
  wall time of these steps on every axis, and a speed change is
  the next call. The planner does not invent a ramp across a
  sparse pair of positions.

  `int16_t` is the same step width as `moveTimed`. It caps one
  chunk at 32767 steps on an axis; a longer constant-rate run is
  several calls. `delta_ticks` is `uint32_t`, so one chunk can
  name at most 2^32 − 1 ticks (~268 s at 16 MHz). A chunk that
  long is still one constant rate, not a curve the planner fills
  in.

  All-zero steps and a non-zero `delta_ticks` is a dwell on the
  common clock. A reversal is two chunks; one chunk has one sign
  per axis.

  The 1 ms frame (`moveTimed(Δ, 1 ms)`) is the failure in
  whitepaper §3.3.1 / GitHub #363: 1.5 steps per box alternates
  1 kHz and 2 kHz. A chunk carries the real duration of its
  steps, and the feeder still emits step-separated periods.

- **Output**: the same geometry at the requested timing with a
  near-exact step period, using **step-separated** commands:
  - period `τ` from the speed at the point (ticks/step)
  - `τ` short → one command with `steps > 1` at `τ`
    (`steps/duration` **is** the speed)
  - `τ` long → one step plus pauses placed at the right time in the
    interval (not a step at every frame start)
  - track `actual_duration`; the next command adapts (drift)
  - planning covers more than the next millisecond (queue depth)

## Errors

A feasibility error can occur **only** with a time-constrained
trajectory (see `extras/doc/planner_modes.md`):

- Faster than the AFAP track, or needs a/v the motors cannot do
  smoothly → `TimingNotAchievable` (not `LookaheadTooShort`, not a
  feed-hold).
- Too slow: stretch by lengthening periods, still hit vertices, still
  G2.
- Feasibility (§3.3.1): requested duration ≥ `naxis_ref` (a few Δt
  slack); consecutive speeds reachable under `a_max`/`ticks_cfg`;
  issued periods must not hunt by ~2× every millisecond.

`naxis_ref` is the **duration bound**; smoothness is a second check.

## Per-block feedrate `F`

Per-block `F` is a **requested speed**, i.e. timed-world input, and
therefore belongs to this implementation. In this encoding `F` is the
chunk itself: the master period is `delta_ticks` over the master's
`|delta steps|`, in ticks per step, never a value in seconds. The
block is executed at that rate, and a request the motors cannot
achieve (faster than the AFAP/`naxis_ref` bound, or needing `a`/`v`
beyond `ticks_cfg`) is a `TimingNotAchievable` error. There is no
separate `F`-as-cap variant in the AFAP planner — without a requested
speed there is simply no `F`.

Slaves share the chunk's tick sum (G6). A slave is feasible when its
step count times its period equals `delta_ticks`; the feeder reports
the quantized `actual_duration` and the next chunk absorbs the drift.
A too-slow chunk (few steps, large `delta_ticks`) stays legal: one
step plus pauses, vertices still hit. That is a slow constant rate,
and it is distinct from a seconds-apart waypoint asking the planner
to draw the curve in between.

## Variations

- **Hard ceiling on `delta_ticks`.** The type still allows a
  one-second constant-rate chunk. A ceiling (queue horizon, a few
  tens of ms) would force resampling even for constant speed. The
  default leaves constant-rate chunks alone and relies on "one call,
  one rate" to keep curves out of the planner.
- **Reject a ~2× period jump between chunks** as
  `TimingNotAchievable` (the #363 hunt), in addition to the
  AFAP duration bound.
- **Same `int16_t` step vector as AFAP `addDelta`**
  (`080_delta_steps.md`), so one geometry buffer can feed either
  implementation. Timed adds the tick count; AFAP does not.
- **Path-level deadline** ("be at `endPath` at this tick") is this
  item. The single-axis form is `080_move_to_eta.md`.

## References

- `extras/doc/n_axes_whitepaper.md` §3.2, §3.3, §3.3.1
- GitHub issue #363
