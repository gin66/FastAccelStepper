# Faithful timed trajectory

Status: later implementation, not v1.

## Separation

Per `extras/doc/planner_modes.md` this is a **complete distinct implementation**,
not a `FasNAxis` mode: its own lookahead/block planner, interpolator and
feeder. Architecturally separate, but code reuse is intended where it
makes sense — shared conventions (`addQueueEntry` contract, pin setup),
`SimPort`, the PC test rig, and common helpers factored out rather than
copied.

## Input / output

- **Input**: polyline plus **speed at each point** (and time). Not
  Δpos per 1 ms frame — `moveTimed(Δ, 1 ms)` hunts 1 kHz/2 kHz
  (whitepaper §3.3.1).
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
therefore belongs to this implementation. It is the G-code form of the
time constraint: the block is executed at the requested feedrate, and a
request the motors cannot achieve (faster than the AFAP/`naxis_ref`
bound, or needing `a`/`v` beyond `ticks_cfg`) is a `TimingNotAchievable`
error. There is no separate `F`-as-cap variant in the AFAP planner —
without a requested speed there is simply no `F`.

## References

- `extras/doc/n_axes_whitepaper.md` §3.2, §3.3, §3.3.1
- GitHub issue #363
