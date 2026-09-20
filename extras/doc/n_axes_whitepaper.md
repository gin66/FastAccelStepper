# FasNAxis — n-axis coordinated motion on FastAccelStepper queues

White paper / concept. Not an implementation.

Status: draft concept, 2026-09-20.
Working name: **FasNAxis**.

---

## 1. Abstract

FastAccelStepper (FAS) generates a high-quality single-axis ramp and
executes it from a tick-exact command queue. Coordinated n-axis motion
is a different problem: the *path* and the *arrival times* couple the
axes, so each axis’s built-in ramp generator is the wrong *planner*.
Its *kinematics* are still the right ones.

The execution primitive is a queue command `{ticks, steps, count_up}`
via `addQueueEntry()` — the same call the ramp generator uses.
**Decision (§4.1):** FasNAxis does not use `moveTimed()`.

FasNAxis sits *on top of* the queues. It accepts a stream of
n-dimensional waypoints, plans under the **actual per-stepper speed
and acceleration limits**, and feeds synchronized commands fast
enough that the queues never run empty.

Planning reuses the FAS ramp map, in **log2** (`RampCalculator`):
period ↔ ramp-steps, no float and no integer division on the hot path.
Lookahead is parsed until **end of path or a direction change**.
That remaining-step count `R_i` is the upper limit on ramp-steps
(`P_i ≤ R_i`) and therefore on speed. The **path direction** then
implies the other axes’ speeds (Linear DDA / Overshoot common `T`).
A short lookahead **reduces speed**; it is not an error. Angle
changes of the path still need motor accel/decel and therefore
**preparation** (§8.4).

Two geometry modes are first-class:

- **Linear** — stay on the straight chords between trajectory points.
- **Overshoot** — each axis runs its own ramp. Waypoints are still
  hit on a common clock, but unmatched accel/coast/decel phases pull
  the path slightly off the chord. The deviation is capped so it
  stays slight.

The library is specified to be **tested on PC only**. Two- and
three-axis runs emit gnuplot (same pattern as `test_02`) and, when
`FAS_NAXIS_TRACE` is set, a **single static HTML file** that embeds
every sample.

---

## 2. Problem

### 2.1 What FAS already does well

Per stepper, FAS offers:

- trapezoidal (and optionally cubic-start) ramps under `v_max`, `a_max`
- a command queue of length 16 (AVR / PC-test) or 32 (ESP32 / Pico / SAM)
- tick-exact execution (`TICKS_PER_S` is 16 MHz on the platforms that
  matter here)
- `addQueueEntry()` for applications that bring their own timing
- near-synchronous start of several queues via `start=false` prefill
  and a later `addQueueEntry(NULL, true)`
- log2 period-from-ramp-steps (`RampCalculator::calculate_ticks` /
  `calculate_ramp_steps`) so the AVR never does `v = sqrt(2 a s)` in
  float

The README already states the multi-axis gap:

> For coordinated movement of two or more axis, the current ramp
> generation will not provide good results. The planning of steps needs
> to take into consideration max.speed/acceleration of all steppers
> […] If this kind of multi-dimensional planning is used, then
> FastAccelStepper is a good solution to execute the raw commands
> (without ramp generation) with near-synchronous start […]

The missing piece is that planner. It should not invent a second
kinematics. It should run the existing ramp map on a remaining-steps
horizon per axis, then issue queue commands.

### 2.2 Why independent ramps fail

A 2-axis move from `(0,0)` to `(1000,100)` with identical `v_max` and
`a_max` on both motors:

- Independent `moveTo()`: both axes start together and the short axis
  finishes first. The path in the plane is **not** the straight line.
- Independent ramps scaled only at the endpoints: the long axis is
  still accelerating while the short axis is already decelerating. The
  path bows.
- Stop at every waypoint: geometrically correct if each segment is
  linear and starts/ends at rest, but a circle approximated by 1°
  chords (the `examples/MoveTimed` pattern) becomes hundreds of
  stop–start ramps.

Time-optimal coordinated motion has to:

1. visit the trajectory points in order, on a common clock,
2. use as much of each axis’s `v_max` / `a_max` as the path and the
   **already buffered** future path allow,
3. start decelerating early enough for a corner, a reversal, the
   last buffered waypoint (treated as rest), or `endPath()`,
4. choose, per run, whether the *geometry between* the points is an
   exact line or is allowed to leave that line because the acc/speed
   profiles of the axes do not share one time-law.

(3) is the lookahead–speed relation. Parse the buffered n-dim
points until the path **ends** or some axis **changes direction**.
The remaining steps `R` in that run are the upper bound on
ramp-steps `P` (same comparison `_getNextCommand` already makes
between `remaining_steps` and `performed_ramp_up_steps`). The
path’s direction vector then scales that bound onto every axis.
If `R` is short, speed is low — motion continues. A
`LookaheadTooShort` error / feed-hold is the wrong concept.

Angle changes of the path (a new `Δ'` that is not collinear with
`Δ`) are the remaining hard part: the implied per-axis speeds
jump, so motors must accel/decel, and that needs distance
**before** the vertex. v1’s preparation is conservative and is
specified in §8.4; it is not an error code.

(4) is the mode switch. Independent FAS `moveTo()` on each axis is
the uncontrolled version of overshoot: the short axis finishes first
and the path is an L. FasNAxis overshoot still **synchronizes arrival
at every trajectory point**, so the L cannot happen; the leftover
freedom is only the shape of each axis’s ramp inside the common
interval, which yields a **slight bulge / overshoot of the chord**.

### 2.3 What `examples/MoveTimed` already proves — and what it does not

`MoveTimed.ino` interpolates a circle with a 4 ms time step, a sine
table, and per-axis drift compensation. It demonstrates:

- common duration across axes
- `steps = 0` to keep an idle axis on the same clock
- `start=false` prefill and a kick-off call

It does **not**:

- respect per-axis `v_max` / `a_max` (the step count per 4 ms is
  whatever the sine table says)
- look ahead (the next 1° is issued blindly)
- handle direction-change pauses as a coordinated event
- test the result except by watching serial output on hardware

FasNAxis is that example turned into a planner with constraints,
horizon, and a PC-checkable oracle.

---

## 3. Goals and non-goals

### 3.1 Goals

| ID | Goal |
|----|------|
| G1 | n-axis motion through a sequence of trajectory points, n ≥ 1, native space = stepper steps |
| G2 | Hard constraints are the **currently configured** per-stepper period (`getSpeedInTicks()`) and acceleration (`getAcceleration()`), plus the device min-period `getMaxSpeedInTicks()` |
| G3 | Two geometry modes, both time-optimal under G2: **Linear** (exact chords, shared time-law) and **Overshoot** (per-axis ramps, slight chordal deviation, waypoints still hit) |
| G4 | Parse lookahead until end or direction change → `R_i` is the cap on ramp-steps (`P_i ≤ R_i`). Path direction implies the other axes’ speeds. Short `R` **reduces speed**, it is not an error. Angle changes need accel/decel **preparation** (§8.4) |
| G5 | Execution through `addQueueEntry()`. Timekeeping pauses never flip DIR. The planner **issues** the driver’s before/after DIR pauses on a reversal (all axes share that time). Leftover injection is globalized, not ignored |
| G6 | Tick-level timebase shared by all axes; lost sync is a hard error |
| G7 | Tests run on the existing PC harness only — no simavr, no hardware, no PlatformIO job for this library |
| G8 | 2D / 3D tests dump gnuplot (as `test_02` / `test_08` / `test_15` do) and may dump a self-contained HTML page |
| G9 | **Header-only.** One public include, pulled in only by sketches that need it. No extra `.cpp` in `src/`, not referenced from `FastAccelStepper.h` |
| G10 | Production header: **no `float`**, **no integer `/` in kinematics**. Period, speed, and accel use `log2_value_t` and `RampCalculator` (same as the single-axis generator). PC oracles may use double under `FAS_NAXIS_TRACE` |

### 3.2 Non-goals (v1)

- Replacing the FAS single-axis ramp generator, or calling it
  concurrently with FasNAxis on the same stepper.
- Running the planner in the FAS `~4 ms` manage-steppers interrupt.
- Hardware-in-the-loop or simavr coverage for FasNAxis.
- Inverse kinematics (CoreXY, SCARA, …) as a built-in. An affine
  motor-map can be added later; v1 plans in step space.
- True circular/NURBS interpolation. Arcs are the caller’s polyline.
- S-curve / jerk limits. v1 uses the existing FAS trapezoid (optional
  cubic start is a later overlay of `s_h`).
- Missing trajectory points. Overshoot leaves the *chord*, it does
  not skip or round past a waypoint. Corner-cutting junction
  deviation (`δ`) is a different mechanism and is not a v1 mode.
- A per-block feedrate `F`. v1 is axis-limit only.
- A `LookaheadTooShort` / feed-hold error when `R < P_stop`.
  Short lookahead is a speed cap (G4), not a fault.
- Feed holds, jogging, or on-the-fly waypoint edits other than
  “append more blocks” / “end path” / “dwell at rest”.
- A blended non-zero junction speed at a finite path-angle
  change (GRBL-style). v1 Linear path-stops; Overshoot prepares
  per axis via `R`. Smoother `ΔP` junctions are a later overlay.
- A GUI application. gnuplot and HTML are **test artifacts**.
- A separately compiled FasNAxis translation unit, a virtual axis
  port, or heap-allocated lookahead. The class is a header template
  with member arrays.
- Isabelle or other machine-checked proofs. Theory is probed by
  PC tests (todo.md steps 2b / 3b), not a prover.

---

## 4. Constraints inherited from FastAccelStepper

FasNAxis does not get to pick a nicer command interface. These are
load-bearing. The feeder talks to `addQueueEntry()` only.

### 4.1 Decision: `addQueueEntry`, not `moveTimed`

The planner’s native quantity is already a **period in ticks**
(`calculate_ticks(P)`). That is `stepper_command_s.ticks`.

`moveTimed(steps, duration)` is a wrapper on top of
`addQueueEntry`. It computes `rate = duration / steps` (integer
`/`), leftover-stretches, and splits into ≤255-step commands.
FasNAxis would have to form `duration = k * ticks` and then let
that `/` quantize the period again. Accel is a **changing**
period; bundling several ramp steps into one duration flattens
them. The log2 map exists so production never divides to get a
period. Using `moveTimed` undoes that.

The FAS ramp generator already feeds the queue with
`addQueueEntry`. FasNAxis does the same.

What the feeder does itself (all small, already specified):

- split `k > 255`; pause-stuff `ticks > 65535`
- `ticks * steps >= MIN_CMD_TICKS` (multiply-compare)
- reserve two slots for a possible leftover DIR inject (Issue 370)
- idle axes: pause commands of the same tick sum
- DIR: planner-issued before/after pauses (§4.4)
- kick-off: `addQueueEntry(NULL, true)`
- underrun: `isQueueEmpty()` after kick-off while the plan still
  has motion
- tick sum of an accepted command: `steps==0 ? ticks : ticks*steps`,
  plus injected pause ticks on retry

**Rejected:** `moveTimed` as the FasNAxis API, `prepare_revert`,
`MOVE_TIMED_*` return codes, and a test-only `moveTimed` wrapper
on `SimPort`. Those stay FAS public API for single-axis callers;
this library does not call them.

### 4.1.1 `addQueueEntry` contract the feeder uses

```
AqeResultCode addQueueEntry(const struct stepper_command_s* cmd,
                            bool start = true);
```

```
struct stepper_command_s {
  uint16_t ticks;  // period, or pause length if steps==0
  uint8_t  steps;  // 0 = pause; else 1..255
  bool     count_up;
};
```

- `ticks >= getMaxSpeedInTicks()`. `TICKS_PER_S = 16_000_000` on
  PC-test / ESP32 / Pico / SAMD51.
- `steps = 0` is a pause of `ticks` ticks. **Required** to keep an
  idle axis on the common clock. `count_up` is whatever the
  caller passes; a timekeeping pause must repeat
  `queue_end.count_up`.
- `start = false` appends without starting the queue. First fill
  uses this on every axis, then `addQueueEntry(NULL, true)` per
  axis (interrupts off).
- A step command starts with a step at the beginning of the
  period, not in the middle.

Return codes:

| Code | Meaning for FasNAxis |
|------|----------------------|
| `AQE_OK` (0) | Appended. Consume `ticks` or `ticks*steps`. |
| `QueueFull` (1), `DirPinIsBusy` (2), `WaitForEnablePinActive` (3), `DeviceNotReady` (4) | Retry the whole coordinated slice later. |
| `DirChangePauseInjected` (6) / `DirPin2msPauseAdded` (5) | A DIR pause was injected; **cmd was not enqueued**. Globalize those ticks onto every other axis (§4.4.2). `_injected_pause_ticks` holds the pause. |
| `ErrorTicksTooLow` (-1) | Period `< getMaxSpeedInTicks()`. Planner bug. |
| `ErrorNoDirPinToToggle` (-3) | `count_up=false` without a DIR pin. |

The single-axis retry loop in the FAS API comment is
uncoordinated. N-axis must not use it. See §4.4.

Prefill: `isQueueEmpty()` true before the first append is
expected, not underrun. After kick-off, empty while the plan
still has motion **is** underrun.

### 4.2 Queue capacity

PC-test: `QUEUE_LEN = 16`, `MAX_STEPPER = 2`.
ESP32 / Pico / SAM: `QUEUE_LEN = 32`.
AVR: `QUEUE_LEN = 16`.

Each queue entry holds at most 255 steps. Periods `> 65535` ticks
need pause commands, which eat extra slots. The feeder reserves
two slots for a possible leftover DIR inject, so one coordinated
slice uses at most `QUEUE_LEN - 2` entries per axis.

FAS recommendation, which FasNAxis adopts as a hard slice rule:

> keep the number of queue commands a move generates well below
> `QUEUE_LEN/2` and split larger moves on the application side.

Practical slice budget on PC-test (`QUEUE_LEN = 16`):

- target ≤ 4 command entries per slice per axis
- duration in the **millisecond** range (1–4 ms is the sweet spot)
- never more than `4 * 255 = 1020` steps in one slice

At 16 MHz, 2 ms = 32 000 ticks. A 4000 step/s axis then issues 8
steps/slice — one queue entry. The capacity constraint is easy at
sane speeds and only bites for very slow motion (pause stuffing) or
absurdly long slices.

### 4.3 `MIN_CMD_TICKS`

On the PC-test / ESP32 / Pico path:

```
MIN_CMD_TICKS = TICKS_PER_S / 5000 = 3200 ticks ≈ 200 µs
```

A command with `steps > 0` must satisfy `ticks * steps ≥ MIN_CMD_TICKS`.
Very short slices with few steps fail this. FasNAxis therefore has a
**minimum slice duration** of at least `MIN_CMD_TICKS`, and in
practice 1 ms to leave margin for pauses.

### 4.4 Direction-change pauses

A direction change is a pause command whose `count_up` differs
from `queue_end.count_up`. Drivers may inject extra pauses in
front to drain a buffered pipeline (ESP32 RMT/I2S). At most one
pause is injected per `addQueueEntry()` call; `_injected_pause_ticks`
is that pause, and the submitted command is **not** enqueued.

Before-pauses (pipeline drain) use the **old** `queue_end.count_up`.
After-pauses and the external-pin 2 ms path use the **new**
`cmd.count_up`; `queue_end` is then already the new DIR.

On `pd_test`, `addDirChangePauseToQueue` returns `AQE_OK` for
`steps == 0` (no inject). Reversing **step** commands still go
through the injection path. The single-axis retry loop is
uncoordinated; FasNAxis must not use it. The planner **prepares**
DIR itself (§4.4.1) so a following reversing step should see
`queue_end.count_up` already matching.

#### 4.4.1 Planner-issued before / after DIR pauses

A direction change is two pauses, not one:

| Pause | `count_up` | Role |
|-------|------------|------|
| **Before** | old DIR | Drain the driver’s output pipeline so no old-DIR step is still in flight (RMT/I2S/MCPWM). |
| **After** | new DIR | The command that actually toggles the pin, plus the user `dir_change_delay`. |

Drivers insert **at most one** of these per `addQueueEntry` and
return Injected without enqueueing the caller’s command. If FasNAxis
only finds out then, the other axes have already been given the
next slice and they **keep stepping** while the reversing axis
sits — that is path deviation, not a time bubble.

So the planner issues both pauses **on purpose**, at a snapped
vertex, **before any next-block step** on **any** axis, and
duplicates the tick sum as timekeeping pauses on every
non-reversing axis.

**Why this is feasible at reversal.** A reversing axis is not at
configured max: Linear zeros path speed at a non-collinear /
reversing vertex; Overshoot zeros `P` on that axis. The last
commands on that axis are already long-period (decel tail, often
`calculate_ticks(1)` ≫ `MIN_CMD_TICKS`). With
`SUPPORT_PAUSE_CMD_COUNTING`, a last pause/step whose ticks
already cover `BEFORE_DIR_CHANGE_DELAY_TICKS` means the driver
will skip the before-inject. The planner still emits an explicit
before-pause if the credited tail is short (buffered RMT can want
**two** `MIN_CMD_TICKS` halves, idf5/6). The after-pause (new DIR)
is always issued; it is the toggle.

Continuing axes may still be at high `P` (Overshoot through a
corner). They **dwell** for the same ticks. That does not lower
their `P`; it is a sit-at-vertex, then they resume at the same
period. If they were allowed to run during τ_dir they would leave
the vertex and the chord would pick up unplanned bulge.

Tick budgets (per reversing axis, then copied to the others):

```
τ_before = max(MIN_CMD_TICKS, BEFORE_DIR_CHANGE_DELAY_TICKS(q))
           // 0 if the last command on this axis already
           // counted as a pause ≥ that (pause-cmd counting)
τ_after  = max(MIN_CMD_TICKS,
               dir_change_delay_ticks,
               AFTER_DIR_CHANGE_DELAY_TICKS(q))
```

Typical drivers (from `pd_esp32/esp32_queue.h` comments):

| Driver | before | after (plus user delay) |
|--------|--------|-------------------------|
| AVR / Pico / SAM, delay 0 | 0 | 0 (toggle on the step command; FasNAxis still issues a min after-pause if `dir_change_delay_ticks > 0`) |
| RMT idf4 / MCPWM | 1 × `MIN_CMD_TICKS` | user delay |
| RMT idf5/6 | 2 × `MIN_CMD_TICKS` | user delay |
| I2S GPIO DIR | 2 × `I2S_BLOCK_TICKS` | user delay |
| I2S mux DIR | 0 | `max(I2S_BLOCK_TICKS, user delay)` |
| External DIR pin | drain until no steps | 2 ms (`US_TO_TICKS(2000)`) |

Read the budget from the stepper at `addAxis` /
`setLimitsFromSteppers()`:

```
τ_before = s->getDirChangeBeforeTicks()     // 0 = none
n_before = s->getDirChangeBeforePauseCount()
τ_after  = s->getDirChangeAfterTicks()      // user delay and driver after
```

Issue `n_before` pauses of `τ_before` (old DIR), then one
`τ_after` (new DIR). `FasNAxisConfig::dir_before_ticks` /
`dir_after_ticks` override when non-zero (tests). Wrong values ⇒
leftover injection, handled next.

Sequence at vertex `p[k]`, axis i reversing, others not:

1. All axes have already issued the last command of the incoming
   block (vertex is a sample). **No** outgoing-block steps yet.
2. Before-pause on i: `{steps=0, ticks=τ_before, count_up=old}`.
   Same tick sum, old `count_up`, on every other axis.
3. After-pause on i: `{steps=0, ticks=τ_after, count_up=new}`.
   Same tick sum, **unchanged** `count_up`, on every other axis.
4. Only then: first commands of the next block (i steps the new
   way; continuing axes resume at their `P`).

`pd_test` skips injection for `steps==0`; the explicit pauses
still go on the queue and the following reversing step must
return `AQE_OK` with no Injected.

#### 4.4.2 Leftover injection: globalize, do not ignore

If a call still returns `DirChangePauseInjected` /
`DirPin2msPauseAdded`, the driver wanted a pause the planner did
not fully pre-issue (under-counted RMT halves, external 2 ms,
stale config). That pause is already on **one** axis.

**Do not** continue the other axes. Immediate compensation:

1. Read the injected ticks (`_injected_pause_ticks`).
2. Enqueue a timekeeping pause of **exactly those ticks** on
   every other participating axis (`count_up` unchanged).
3. Add the ticks to every `T_i` and to `T_plan`.
4. Inspect `queue_end.count_up` on the injecting axis (same
   rule as before): old DIR ⇒ retry the planned pause/step
   (more before-drain); new DIR ⇒ remaining after-pause is
   timekeeping, do not XOR back.
5. Then continue the planned sequence.

Effect: a **time bubble** at the vertex. All motors stationary,
so the path does not pick up a chordal error from the injection.
Clocks stay together. There is no later “catch up” by speeding
one axis — that would be the path deviation the pauses were
meant to avoid.

If next-block steps were already queued on another axis, it is
too late to globalize (those steps will run during the pause).
That is a feeder bug: DIR sequence must precede any outgoing
command. Tests fail if a non-revert `addQueueEntry` returns
Injected.

#### 4.4.3 Tests (F12)

- F12: planner before+after; following step has no Injected.
- F12b: Overshoot corner, continuing axis at high `P`; it must
  not step during τ_dir (position frozen at the vertex).
- F12c: SimPort injects one extra before-pause anyway; all axes
  dwell it; XY does not leave the vertex; `|T_i−T_j|` still bound.

### 4.5 Enable-on delay

Auto-enable can insert pauses at queue start. The FAS multi-axis note
already says: do not use enable-on delay for synchronized starts.
FasNAxis requires enable to be settled before kick-off (manual enable,
or auto-enable with zero on-delay).

### 4.6 Ramp generator must be idle

`addQueueEntry` does not stop the ramp generator.
`manageSteppers()` (~4 ms) will keep filling the same queues if a
prior `moveTo()` / `runForward()` is active, and `pump()` would
race it.

Precondition: every participating stepper is idle,
`!isRampGeneratorActive()`, before the first prefill. `addAxis`
fails if the stepper is running or the ramp is active. The sketch
must `stopMove` / `forceStop` and wait if it was using the ramp.

### 4.7 PC-test `MAX_STEPPER = 2`

The existing `pd_test` platform has two queues. A 3-axis run cannot
be backed by live `FastAccelStepper` objects without raising
`MAX_STEPPER`. The class is templated on the stepper type: production
is `FasNAxis<N>` (`Stepper = FastAccelStepper`); PC tests that need
n > 2 use `FasNAxis<N, HORIZON, SimPort>`. 1- and 2-axis golden
paths still use real FAS queues.

---

## 5. Architecture

Three stacked stages, two buffers. Only the last stage calls FAS.

```
          waypoints / dwells / end-of-path
                        │
                        ▼
             ┌─────────────────────┐
             │  Block planner      │  lookahead: parse to end or
             │  Linear: one ramp   │  direction change → max P;
             │  Overshoot: ramps_i │  path direction implies speed
             └──────────┬──────────┘
                        │ committed blocks (ramp state + T)
                        ▼
             ┌─────────────────────┐
             │  Time-slice         │  common tick budget, snap at vertices
             │  interpolator       │  commands: {ticks, steps, count_up}
             └──────────┬──────────┘
                        │ per-axis queue commands (same tick sum)
                        ▼
             ┌─────────────────────┐
             │  addQueueEntry      │  prefill, kick-off, DIR pauses
             │  feeder             │
             └──────────┬──────────┘
                        │
                        ▼
              FastAccelStepper queues
```

The two buffers are different lengths on purpose:

| Buffer | Unit | Typical depth | Job |
|--------|------|---------------|-----|
| Lookahead / block buffer | n-dim waypoints | tens to thousands | parse to end or direction change → max `P`; last point is rest |
| Slice / command buffer | time slices | 10–50 ms | enough **time** to keep `QUEUE_LEN` from underrunning |

Confusing them is the usual design mistake. A 2 ms slice buffer can
never be the n-dim lookahead. The velocity decision lives in the
block buffer: short `R` lowers `P`, it does not stall the feeder.

PC tests record at three taps: planner output (ramp-steps / ticks per
axis), interpolator output (integer slices), and feeder output
(commands actually accepted by `addQueueEntry`, including injected
pauses). gnuplot and HTML overlay the commanded polyline and the
realized path so Linear vs Overshoot is visible at a glance.

---

## 6. Coordinate frames and operating modes

### 6.1 Step space, in ticks

v1 plans in **motor step space**. Axis i has:

- position `p_i ∈ int32` (same as FAS)
- configured period `ticks_i_cfg = getSpeedInTicks()` (ticks/step,
  `RampGenerator::_parameters.min_travel_ticks`). This is **not**
  steps/s. Never form `getSpeedInTicks() / TICKS_PER_S`.
- device floor `ticks_min = getMaxSpeedInTicks()` (never-violate
  minimum period)
- acceleration as `log2_accel = log2_from(getAcceleration())`

Oracles and plots may convert a period to steps/s with
`TICKS_PER_S / ticks` (or `getSpeedInMilliHz() / 1000`). That
conversion is test-only. Production stays in ticks and log2.

### 6.2 Two modes

Both modes visit every trajectory point. Both use a common segment
time `T` so all axes arrive together. They differ in the **time-law
between** those points.

```
Linear                         Overshoot
p0 ●─────────────────● p1      p0 ●───╮
                                   \   ╰──╮ bulge / slight overshoot of the chord
                                    \      ╰──● p1
shared ramp, p on the chord        each axis its own ramp
path = chord                       path leaves the chord, still hits p0 and p1
```

| | **Linear** | **Overshoot** |
|--|------------|----------------|
| Geometry | `p(t)` lies on the chord `p0→p1` (½-step rounding) | `p(t)` may leave the chord; `p(t_start)=p0`, `p(t_end)=p1` |
| Time-law | One ramp, slaved DDA | One ramp per axis, common `T` |
| Who binds | Longest `\|Δ\|` first; rebind if a slave would exceed its v/a (§6.3) | Each axis binds on its own remaining steps; `T` is the slowest axis |
| Non-binding axes | DDA along the chord — scaled **down** relative to the binder | May run *slower* than their ramp (longer period) so they occupy all of `T`; never faster |
| Sharp corner | Whole path speed → 0 unless collinear | Only axes that reverse go through 0; continuing axes keep ramp-steps |
| Typical use | Plotter, laser, exact contour | Faster point-to-point, circles, shallow corners |

One-axis motion is identical in both modes (and matches a FAS
single-axis ramp of the same `ticks_cfg` / `a` over the same
remaining steps, within slice quantization).

Path progress `s` is **steps of the binding axis** on the current
block (0 … `|Δ_bind|`), not a real in `[0,1]`.

### 6.3 Linear

Waypoints are connected by straight lines in step space. The
trajectory stays on the polyline.

**Who sets the speed.** On a block, the axis with the largest
`|Δ_i|` is the tentative binder: it runs as fast as *its* ramp
and *its* remaining-steps budget allow. Every other axis is
**scaled down** to finish in the same time (DDA onto the binder’s
step count). A shorter axis therefore never runs faster than the
longest one.

**Unless that is too fast for a slave.** Scaling down is in
*steps*, not in that slave’s own v/a envelope. If the binder
coasts at `ticks_b_cfg`, wall-clock for the block is proportional
to `|Δ_b| * ticks_b`. Slave i needs at least `|Δ_i| * ticks_i_cfg`
at its own max. Integer compare, no division:

```
if |Δ_i| * ticks_i_cfg  >  |Δ_b| * ticks_b     // slave would have
                                                 // to go faster than v_i_max
    binder is too fast; rebind to i (or lengthen ticks_b)
```

Same for acceleration. DDA gives the slave
`a_i ≈ a_b * |Δ_i| / |Δ_b|`. That exceeds `a_i_max` when:

```
a_b * |Δ_i|  >  a_i_max * |Δ_b|
```

(`a` stays `log2_accel` in production; the compare is
`log2_multiply` of those integers.) Then the binder must ramp
slower — equivalently, treat the slave as binder and scale the
long-distance axis **down**.

Equal `|Δ|` (45° with identical motors): either axis, tie-break
on larger `ticks_cfg` (slower motor). Asymmetric motors on a
near-square diagonal: the *slow* motor binds even if it is not
the longest distance (F9, F18).

Once the binder is known, it runs the FAS ramp against remaining
steps until the next path-stop (§8): a non-collinear vertex, an
axis reversal, `endPath()`, or the **last buffered waypoint**
(open path: unknown next angle, last point is rest). Slaves:

```
err_i += |Δ_i|
if 2*err_i >= |Δ_bind|:  step i, err_i -= |Δ_bind|
```

No division after the block is accepted. The path is the chord
because every slave step is locked to the binder’s step count.

That lock **is** the path-direction implication: the binder’s `P`
(already capped by `R`) sets every slave’s step rate. A short
lookahead on the polyline lowers the binder, hence every axis.

A path-stop corner (Linear `v = 0`) is any vertex that is not
collinear in the same sense (§8.5), plus the last buffered point
while the path is open. Remaining steps for the binder is the
sum of `|Δ_bind|` over blocks up to that stop.

### 6.4 Overshoot

Each axis is a 1-D FAS ramp on its own remaining steps in the
**current direction** (sum of `|Δ_i|` until that axis reverses,
the path ends, or the last buffered point). Segment boundaries
stay **time-synchronized**:
axis i is at `p_i[k]` at the same `t_k` as every other axis.

```
T_opt_i = duration of axis i’s ramp over |Δ_i| of this block,
          with P_in, P_out from the remaining-steps law (§7)
T       = max_i T_opt_i
```

Non-binding axes occupy the whole of `T` by using a **longer**
period than their ramp allows as a minimum (they may not go
faster, they may not sit at the start or the target). Period for
a one-step stretch is `T` itself; for `|Δ_i| > 1` the log2 map is

```
log2_ticks_i = log2_from(T) − log2_from(|Δ_i|)   // log2_divide
ticks_i      = max(log2_to_u32(log2_ticks_i), ticks_from_ramp, ticks_min)
```

Because `Ta`, `Tc`, `Td` then differ across axes, normalized
progress differs, and `p(t)` leaves the chord. That is the
overshoot. It is not a 1-D overshoot: each axis still lands on
`p_i[k]` exactly. It is not CNC junction-deviation: that *cuts
inside* a corner and misses the vertex.

On an axis-aligned square the two modes coincide at the corners
(the continuing axis of the next side is the one that was idle, so
everyone is at rest). A lone diagonal is not faster in Overshoot
either (`T` is the binding axis in both modes); the path just
bulges. Overshoot pays off on **continuing-axis corners** and on
a sampled circle (the X extrema reverse X while Y keeps
ramp-steps — Linear with exact chords would stop the whole path).

### 6.5 Keeping overshoot slight: `overshoot_max`

On a single rest-to-rest segment the bulge is usually already
modest: the short axis cannot race ahead and wait, so its period
drops to ~`T / |Δ_i|` and `d(t)` is the difference between that
near-linear 1-D motion and the binding axis’s ramp.

The cap exists because **corners** are not modest. If X continues
through a vertex at high ramp-steps while Y is at 0, the path
flattens along X. Uncapped, “slight” would not hold.

Overshoot mode therefore carries a cap `overshoot_max` (steps).
Compare **squared** distance so production never takes a sqrt:

```
d²(t) = distance² of p(t) to segment p0–p1
require  max_t d²(t)  ≤  overshoot_max²
```

If the raw per-axis ramps would violate the cap, the planner
**mixes toward Linear** on the axes that are ahead of the linear
fraction: lengthen that axis’s period until `max d²` equals the
cap. The binding axis of the segment is left at its ramp, so a
lone segment’s `T` does not grow. At a continuing-axis corner the
mix *does* slow the continuing axis — that is the price of staying
slight. Linear is the `overshoot_max = 0` limit of the same
construction.

Recommended defaults:

- Linear mode: cap is 0 (use the DDA, do not iterate a mix).
- Overshoot mode: `overshoot_max = 8` steps, configurable. `∞`
  (`UINT16_MAX`) is allowed for tests that want the raw profiles.

The HTML / gnuplot viewer draws the chord, the realized path, and
the sample of maximum `d(t)`. That sample is the definition of
“slight” for a given run.

### 6.6 Number system

Production (`src/FasNAxis.h` and anything it includes):

- `log2_value_t` for period, acceleration, and any ratio of ticks
  to steps. Helpers: `log2_from`, `log2_to_u32`, `log2_multiply`,
  `log2_divide` (subtraction), `log2_sqrt` (shift), `log2_square`.
- Reuse `ramp_config_s::calculate_ticks` /
  `calculate_ramp_steps`. Do not re-derive `v = sqrt(2 a s)` in
  integer.
- Remaining-steps sums, DDA error, sign tests, `d²` compares:
  integer add/sub/mul/compare only.
- No `float`, no `double`, no integer `/` on the hot path.

PC tests and `FAS_NAXIS_TRACE` oracles may use double to plot
steps/s and Euclidean `d(t)`. That code is not in the production
header.

---

## 7. Time-optimal profile on one block

A block is the motion between two consecutive trajectory points.
Kinematics are the FAS ramp, not a separate `v²/2a` formula.

### 7.1 Ramp-steps ↔ period

`RampCalculator` (already in the library):

```
ticks(P) = calculate_ticks(P)          // period at ramp position P
P(ticks) = calculate_ramp_steps(ticks) // inverse
```

`P` is `performed_ramp_up_steps`. It **starts at 0** and is
never a precomputed `min(P_stop, R/2)`. Acceleration is counting
`P` up; deceleration is counting `P` down toward 0. Coast is `P`
held, period clipped to `ticks_i_cfg`.

The two caps are applied live, the same way FAS
`_getNextCommand` already compares `remaining_steps` to
`performed_ramp_up_steps`:

- **max speed:** `P` stops increasing once the period has reached
  `ticks_i_cfg` (`P_coast` / `P_stop` = `calculate_ramp_steps`)
- **remaining-to-standstill:** when `R == P` the next steps count
  `P` down (there are exactly `P` steps of braking room left).
  This is not estimated in advance.

```
R = remaining steps to stand still in the current direction  // §8
P = performed_ramp_up_steps   // starts at 0

if R > P:   accelerate (P++) or coast if already at ticks_i_cfg
if R == P:  decelerate (P--)
if R < P:   too fast — P exceeded R (planner bug); treat as
            reverse/decel (same as FAS overshoot handling)

ticks = max(calculate_ticks(P), ticks_i_cfg, ticks_min)
```

A rest-to-rest move of `N` steps **coasts** if `N/2 > P_stop`
(equivalently `N > 2 P_stop`): there is room to reach configured
max and still stop. If `N/2 < P_stop`, the move is a triangle
whose peak is whatever `P` has grown to when `R` catches it
(about `N/2`). That peak is an **outcome** of the law, not an
input the planner writes as `P`.

No sqrt, no `/`. `R` is an integer sum. `P` changes by the steps
just issued, so `|P_out − P_in| ≤ |Δ|` on a block — a 1-step
block cannot take an axis from rest to high speed. Period is
taken after the `P` update: first step from rest is
`calculate_ticks(1)`; `calculate_ticks(0)` is never called.

### 7.2 Linear — one ramp, DDA slaves

The binder of §6.3 (longest `|Δ|`, rebound if a slave would
exceed v/a) runs §7.1 with `R` = remaining binder-steps until the
next Linear path-stop (angle change, reversal, last point, or
`endPath()`). Each planning chunk of `planning_steps`
(same 2 ms rule FAS uses) is one interpolator slice: one
`addQueueEntry` on the binder at `calculate_ticks(P)`, DDA slaves
take 0 or 1 step per binder step, idle axes get a pause of the
same tick sum. Pause stuffing when `ticks > 65535`.

### 7.3 Overshoot — ramp per axis, common T

For each axis, §7.1 with that axis’s `R_i` (remaining steps until
*that* axis reverses). That yields `T_opt_i` for the block
(sum of periods of the `|Δ_i|` steps, with `P` evolving). Then
`T = max_i T_opt_i`.

A non-binding axis must cover exactly `|Δ_i|` in time `T` without
delayed start and without sitting at the target:

- Average period `T / |Δ_i|` via `log2_divide`, then
  `max(…, ticks_from_ramp, ticks_min)` so it never exceeds the
  ramp.
- If that average period would require **faster** than the ramp
  (should not happen if `T ≥ T_opt_i`), the axis was mislabeled
  non-binding.
- Shape: use the ramp (`P` vs `R`) as the default; lengthen
  periods (never shorten) if `overshoot_max` requires a mix
  toward Linear.

`P` cannot jump over `|Δ|`, so the old infeasible case “`v_out`
high after 1 step from rest, then occupy the long axis’s `T` at
`a_max`” does not arise as a trapezoid-at-`a_max` problem. A 1-step
axis occupying a long `T` is one slow step (long period), which
`overshoot_max` will pull toward the linear fraction if the early
step bows the chord.

If mixing toward Linear cannot meet the cap without slowing the
binder, lower the continuing axis’s `P` at the vertex (that is
lowering `R` consumed / allowed speed into the next block) and
re-run §7.1 on the adjacent blocks. That *does* iterate `T` at
continuing-axis corners; rest-to-rest lone segments (F4) do not.

---

## 8. Lookahead

Lookahead is a ring of n-dimensional waypoints, not a Euclidean
`v(s)` pass. Three facts, in order:

1. Parse until **end of path** or an axis **direction change**.
   That remaining-step count `R_i` is the upper limit on
   ramp-steps, hence on speed: `P_i ≤ R_i`.
2. The **path direction** (the n-dim `Δ` of the current run)
   maps that cap onto every axis.
3. **Angle changes** of the path change those implied speeds, so
   motors must accel/decel. That needs **preparation** before the
   vertex. It is the remaining hard part; v1’s answer is
   conservative (§8.4). It is not a `LookaheadTooShort` error.

### 8.1 Parse until end or direction change

For axis i, walking the block ring from the executing head:

```
R_i = 0
sign = 0
for each block b in order:
    if Δ_i[b] == 0:                 // idle on this block
        if sign != 0: break         // direction ended
        continue
    s = sign_of(Δ_i[b])
    if sign == 0: sign = s
    if s != sign: break             // reversal: must be at 0 here
    R_i += |Δ_i[b]|
    if path ended after b: break    // endPath() or last buffered point
```

The scan **stops** at the first of: path end, that axis going
idle after moving, or `sign(Δ_i)` flipping. The last buffered
waypoint of an open path is treated as **rest** — the next
angle is unknown, so the only safe `P` at that point is 0.
When more waypoints arrive, `R` may grow and the unexecuted
tail is replanned faster. `R` is therefore exact for the
current buffer, not a lower bound the planner waits to fill.

Linear additionally ends the *binder’s* `R` at a non-collinear
vertex (§8.5), even if that axis would not reverse there.

### 8.2 `R` is the cap on ramp-steps

The ramp law of §7.1 with this `R` is the whole reverse pass:

```
P_i ≤ R_i
ticks_i = max(calculate_ticks(P_i), ticks_i_cfg, ticks_min)
```

Configured max is

```
P_stop_i = calculate_ramp_steps(ticks_i_cfg)
```

(`addAxis` / `setLimitsFromSteppers`). That is how many steps
the motor needs from rest to `v_max`, and therefore from
`v_max` to rest. Oracle language:

| v_max | a_max | `P_stop` | t_stop |
|------:|------:|---------:|-------:|
| 1 000 step/s | 1 000 step/s² | 500 | 1.0 s |
| 4 000 | 2 000 | 4 000 | 2.0 s |
| 20 000 | 5 000 | 40 000 | 4.0 s |
| 40 000 | 2 000 | 400 000 | 20 s |

If `R_i < P_stop_i`, axis i **cannot be at configured max**.
`P` still starts at 0 and follows §7.1 against this `R`; there
is no separate `P = min(P_stop, R/2)` estimate. From rest a
triangle’s peak is about `R/2` only because accel stops when
`R` catches `P`. Rest-to-rest coast at `ticks_cfg` needs
`N/2 > P_stop` (i.e. `S ≥ 2 P_stop`) and, while coasting,
`R ≥ P_stop`. None of that is an error: the move is a slower
triangle. `endPath()` uses the same law (last point is rest,
and `R` will not grow).

A 2 ms slice is 8 steps at 4000 step/s. The cap is **one**
integer `R`, not 500 slices.

`HORIZON` (the second template argument, default 64 on MCU)
is the max number of **n-dim points**, not of steps. One
10 000-step `addLine` with `HORIZON = 8` still has `R = 10000`
and can reach `P_stop`. A stream of 1-step micro-segments
cannot: `R ≤ HORIZON`, so `P` stays small. That is the
lookahead–speed relation. It is **not** a configuration
error at `addAxis`. PC tests instantiate
`FasNAxis<N, 4096, SimPort>` when they want a long point
horizon without putting a 4096-block array on every AVR
sketch that merely includes the header.

`kappa_stop_q8` (default 320 = 1.25 in Q8) is only a
**diagnostic** threshold:
`R_i * 256 < P_stop_i * kappa_stop_q8` (path open) means
speed is limited by lookahead. Kinematics stay `P ≤ R`.
Integer, no `/`.

### 8.3 Path direction implies the other axes’ speeds

`R_i` is per axis. The motion is a **path**, so one axis’s
cap limits the others.

**Linear.** One ramp, DDA slaves (§6.3). The binder’s `R`
is remaining binder-steps to the next path-stop (including
the last buffered point). Binder `P ≤ R` sets the time-law;
every slave steps in lock with the binder. Short lookahead
on the polyline therefore slows **every** axis, in the
ratios of `Δ`. Rebind still applies: if a slave would exceed
its own `ticks_i_cfg` / `a_i_max` under that time-law, that
slave binds and the long axis is scaled down. The
lookahead-capped binder period is the `ticks_b` in that
compare.

**Overshoot.** Each axis runs §7.1 on its own `R_i`. Segment
time `T = max_i T_opt_i`. A short-`R` axis has a larger
`T_opt` (triangle, long periods). That `T` lengthens every
other axis’s period. DIR pauses at a reversal still freeze
the continuing axes at the vertex (§4.4.1).

So: max steps in one direction → max `P` of that axis →
path direction → max `P` / period of the related axes.

### 8.4 Angle changes need preparation

A direction change of an **axis** is a sign flip of `Δ_i`
and is already handled: `R_i` ends, `P_i` must be 0 there
(§8.1).

An **angle change of the path** is a jump from `Δ` to `Δ'`
that is not collinear. Even when no axis reverses, the
implied per-axis speeds change (new ratios, maybe a new
binder). Each motor must accel or decel by some `ΔP`.
That change takes `|ΔP|` steps and must be **prepared
before the vertex**, using the incoming `R` — this is why a
path-angle change reduces speed **before the next trajectory
point**, so that the acceleration to the new implied speeds
stays inside `a_max`.

v1 preparation, by case:

| What the lookahead shows | Preparation |
|--------------------------|-------------|
| Nothing past the last point (path open) | Last n-dim point is rest. Prepared for *any* next angle, including reversal. Short buffer ⇒ low `P`. |
| Collinear continuation (≤ 2°, §8.5) | No `P` change. `R` continues through the vertex. |
| Axis reversal | That axis to `P = 0` at the vertex (`R` ends). Linear also zeros the path. |
| Finite path-angle change, no reversal | **Linear:** path-stop (`P → 0`). Maximum preparation, exact chords. **Overshoot:** only reversing / going-idle axes to 0; continuing axes keep `P`; bulge capped by `overshoot_max` (which may itself lower the continuing `P`). |

Why this is tricky: a smoother junction would look *past*
the vertex at `Δ'`, compute each `P_i'` from the new
direction, and require `|P_i − P_i'|` to fit in the
incoming remaining steps — a partial-speed blend, not a
full stop. v1 does **not** do that. Linear’s 2° test is
the “angle change is negligible” threshold; anything
larger path-stops. Overshoot pays with chordal bulge
instead of a blended Linear speed. A `ΔP` junction is a
later overlay, not a reason to raise `LookaheadTooShort`.

FasNAxis does **not** use slice length `Δt` to invent a
corner speed. Inside-corner shortcuts (GRBL `δ`) miss the
vertex and are not v1.

### 8.5 Junction — Linear

Collinear, same sense (keep cruising):

- no axis reverses: `sign(Δ_i)` matches `sign(Δ'_i)` for
  every i with either component nonzero, **and**
- the unsigned angle is ≤ 2° so 1° sampled arcs count as
  collinear and 90° corners do not:

```
(Δ · Δ')² * 100000  >=  99878 * |Δ|² * |Δ'|²
```

(`cos²(2°) ≈ 0.99878`. Integer mul/compare, no division, no
sqrt.) `ε` is this test; it is not a free real.

Otherwise the vertex is a path-stop: binder `R` ends here,
`P` must reach 0. That *is* the preparation of §8.4.

### 8.6 Junction — Overshoot

Per axis, the scan of §8.1 **is** the junction rule:

- reversal (`Δ_i` and `Δ'_i` opposite signs) or going idle
  after moving: `R_i` ends, so `P_i` must be 0 at that vertex
- continuation (same sign, including through a 90° corner on
  the axis that does not reverse): `R_i` includes the next
  blocks, so `P_i` may stay high

A 90° corner where X continues and Y reverses: `P_x` may
stay high, `P_y = 0`. Linear would have zeroed the binder,
hence both. Overshoot is faster here, and the path through
the vertex flattens, bounded by `overshoot_max`. Y’s DIR
before/after pauses still freeze **X at the vertex** for
τ_dir (§4.4.1); X does not keep stepping through the toggle.

A 90° corner of an axis-aligned square: the “continuing”
axis of the next side was idle (`Δ = 0`), so both `P` are
already 0. Linear and Overshoot match.

### 8.7 What is committed

Feed a **stoppable** plan to the last path-stop in the
current buffer (reversal, Linear non-collinear vertex,
`endPath()`, or last buffered point). Do not wait for
`R ≥ P_stop` before moving.

Commands already in the hardware queues stay. The
unexecuted tail is **replanned** when `R` grows (more
waypoints, collinear continuation) — typically faster, same
`P ≤ R` law. Speculative decel that was queued because the
last point was rest may already be in the queue; that is
acceptable streaming (you slowed, then sped up). It is not
a feed-hold.

There is no separate `v_in[]` / `v_out[]` array. Endpoint
speed *is* `P` at the vertex. Initialize nothing to
`v_path_max`; the ramp starts at `P = 0` and is clipped by
`ticks_i_cfg` and by `R`.

Tests: F11 dribbles waypoints so `R < P_stop` while the
path is open — motion continues at the `R`-capped speed,
`pump()` stays `Running`, `P ≤ R`, cruise only after `R`
grows. F19 is a small `HORIZON` of micro-segments: same
cap, `addAxis` succeeds; contrast one long `addLine` with
the same `HORIZON`, which *can* reach `P_stop`.

### 8.8 Reversals of a single axis

Even on a smooth polyline an axis can reverse (a circle’s X
axis at the left and right extrema). That is a direction
change in the scan of §8.1, and an angle change of the path
(§8.4).

**Linear:** at the extremum some `Δ_i` changes sign, which
fails §8.5, so the path stops. A sampled circle would stop
twice per revolution. That is correct for exact chords, and
the wrong mode for a circle. (1° chords *without* a sign
change pass the 2° collinear test, so Linear does not stop
at every chord.)

**Overshoot:** only the reversing axis has `R → 0`; the
others keep `P`. A sampled circle is the motivating case.

True arc blocks remain v2. v1 tests a coarse square in
Linear (must stop) and a fine circle in Overshoot (must not).

---

## 9. Time-slice interpolator

### 9.1 Common clock

All axes of a slice share one duration `Δt` in ticks. Suggested
default:

```
Δt = 2 ms = 32000 ticks at 16 MHz
```

Configurable in `[max(MIN_CMD_TICKS, 1 ms), 4 ms]`. Shorter Δt:
better acceleration staircase, more feeder CPU, more queue traffic.
Longer Δt: coarser corners, fewer calls.

### 9.2 Sampling, snapped to vertices

The interpolator walks time **inside one block**. It must not
cross a vertex inside a single queue command:

- one signed `steps` is a net Δ; an out-and-back inside `Δt`
  would drop the reversal
- a vertex that is not a sample can skip `p[k]` (a 2 ms grid
  around a square corner at `v_peak ≈ 1789` can jump from
  `(1598,0)` to `(1600,2)` and miss `(1600,0)`)
- Linear DDA / Overshoot ramps are defined per block

```
while t < T_block:
    dt = min(Δt, T_block − t)          // last slice of the block
    emit one slice for this block only
    t += dt
vertex sample: integer p equals p[k]
if some axis reverses next: coordinated revert (§4.4.1)
next block
```

`T_block − t` is an integer tick subtract. `min` is a compare.

**Linear.** Binder takes `k` steps in this slice (from §7.2).
Slaves: DDA `k` times. `Δsteps_bind = k`, `Δsteps_i` from the
accumulator. Positions are exact integers; the oracle still
checks distance to the chord ≤ `0.5 √n` (that `√n` is
test-only).

**Overshoot.** Each axis’s planned `p_i(t)` comes from summing
the ramp periods (log2 ticks already computed). Integer steps:

```
P_i(t)     = steps completed on i at tick t   // integer
Δsteps_i   = P_i(t+dt) − P_i(t)
duration   = dt                               // nominal; see drift in §10
```

The last slice of the block **snaps**: `Δsteps_i` is whatever
remains to `p_i[k]`, duration is the remaining ticks of `T`.

If every `Δsteps_i = 0`, the slice is still emitted as a pause
command (`steps = 0`, `ticks = dt`) on **all** axes (a dwell).
That happens at exact stops and during coordinated direction
pauses.

### 9.3 16-bit `steps` and queue stuffing

If some `|Δsteps_i|` exceeds what one `addQueueEntry` will accept
for this `dt` (255 steps, or pause stuffing past `QUEUE_LEN-2`):

- **Too fast:** period would be `< ticks_min`. Planner bug.
  Reject at plan time: `|Δsteps_i|` steps in `dt` ticks must
  satisfy `dt >= |Δsteps_i| * ticks_min` (compare after
  multiplying, no division).
- **Too many commands** (slow axis, pause stuffing): split the
  slice in half in time (two command sequences, same ratio).
  Recurse until it fits or `dt` hits `MIN_CMD_TICKS` — then it is
  a planner bug (`ErrorTicksTooLow` / too many commands) and the
  planner must have issued a longer pause-only strategy (one step
  per several slices).

### 9.4 Acceleration as a staircase

A queue command is constant rate. Acceleration lives at command
boundaries. A raw per-slice `a = Δv/Δt` at `Δt = 2 ms` is
useless (one extra step is `2.5·10^5` step/s²).

**Oracle rule:** do not evaluate `a` on adjacent slices. Compare
the integer path’s period, low-pass filtered at
`T_a = max(Δt, 10 ms)`, to `calculate_ticks(P)` of the planner.
Position tracking remains exact (integer).

Speed oracle: every command with `steps ≠ 0` has
`ticks ≥ ticks_min` and `ticks ≥ ticks_i_cfg` except for one-step
quantization slack.

---

## 10. Feeder: clocks, drift, direction, start

The feeder issues `addQueueEntry` only (§4.1).

### 10.1 Global time vs per-axis actual time

Each accepted command contributes a known tick sum:
`steps == 0 ? ticks : ticks * steps`. Injected DIR pauses add
their ticks on the retry. FasNAxis keeps one **planned** clock
`T_plan` and one **actual** clock `T_i` per axis:

```
T_plan += dt
cmd.ticks, cmd.steps chosen so the tick sum closes T_plan − T_i
rc = addQueueEntry(&cmd, start)
T_i += tick_sum(cmd) + extra_i_consumed
```

`extra_i` is injected-pause ticks not yet folded into the
coordinated timeline. After a successful append, `T_i` should
equal `T_plan` within a few ticks. A growing `|T_i − T_j|` is a
feeder bug.

Idle axes in the same slice get pause commands of the same tick
sum.

### 10.2 Coordinated direction-change pauses

At a snapped vertex, if axis i’s next `count_up` disagrees with
`queue_end.count_up`, run §4.4.1 **before** any outgoing-block
command on any axis:

1. Optional credit: if i’s last command already has
   `steps==0` or a long period ≥ `τ_before`, skip the explicit
   before-pause (the driver will too, via pause-cmd counting).
2. Before-pause on i (old DIR) + matching timekeeping pauses
   on every other axis.
3. After-pause on i (new DIR) + matching timekeeping pauses
   on every other axis.
4. If any of those `addQueueEntry` calls returns Injected:
   §4.4.2 — copy those ticks to every other axis immediately,
   then retry. Do not speed anyone up later to “catch” the
   lost time; that is path error.
5. Only then issue the first reversing step / continuing steps.

The continuing axis in Overshoot keeps its `P` across this
bubble; it does not step during it.

### 10.3 Prefill and kick-off

First motion from rest. Participating steppers are idle (§4.6).

1. Interpolate commands until every participating queue has at
   least `Q_prefill` entries (e.g. half the queue) or the path
   ends, using `start = false`. An empty queue before the first
   append is expected; check `isQueueEmpty()` yourself if you
   care, do not treat it as underrun.
2. With interrupts disabled (no-op on PC), call
   `addQueueEntry(NULL, true)` on every participating stepper.
3. Subsequent commands use `start = true`. After kick-off, a
   queue that is empty while the plan still has motion is
   underrun.

Non-participating axes (n-axis machine, this move uses a subset)
are left alone in v1; they are not in the feeder set.

### 10.4 Pump

FasNAxis does **not** run in the FAS ramp interrupt. The host calls
`pump()` often enough that the slice buffer plus the hardware
queues always cover `> 5 ms` (more than one manage-steppers period,
more than a typical loop hiccup).

`pump()` does, in order:

1. Accept newly queued waypoints into the block buffer.
2. Recompute `R_i` (parse to end or direction change) and
   `P_stop_i` if the buffer or limits changed.
3. Plan with `P ≤ R` to the last path-stop in the buffer
   (last point is rest if the path is open). Short `R`
   lowers speed; it does not set an error flag.
4. Commit / replan under §8.7 (feed the stoppable plan;
   replan the unexecuted tail if `R` grew).
5. Interpolate commands into the slice buffer until it holds
   `T_slice_buf` (default 20 ms) or the committed path is exhausted.
6. While any axis has free queue room, pop a command and
   `addQueueEntry` it (with the retry/dwell protocol).
7. Return a status: running / idle / underrun / error.

On PC tests, `pump()` is called in a deterministic loop that also
**drains** the simulated queues by the elapsed ticks, exactly as
`RampChecker` drains `fas_queue[].read_idx` today.

### 10.5 Underrun

After kick-off, `isQueueEmpty()` on any participating axis while
`T_plan` still has motion is a failed test and a production error
flag. There is no attempt to “catch up”: the common clock is
already broken. The gnuplot / HTML dump still includes samples up
to the fault.

An empty queue during prefill is not underrun.

---

## 11. Delivery and API sketch

### 11.1 Header-only, on a need basis

FasNAxis is **one public header** with the class definition and its
methods. It is not a second Arduino library, not a `.cpp` next to
`FastAccelStepper.cpp`, and not included by `FastAccelStepper.h`.

```
#include <FastAccelStepper.h>   // always, if you use FAS
#include <FasNAxis.h>           // only if you want coordinated n-axis
```

Arduino’s `dot_a_linkage=true` already omits unused `.cpp` files;
an unused *header* is never compiled at all. Sketches that never
include `FasNAxis.h` pay **zero** flash, RAM, and compile time.

Layout:

```
src/FasNAxis.h              // public include; class + inline methods
src/fas_naxis/*.h           // optional split, included only from FasNAxis.h
                            // never a .cpp under src/
extras/tests/pc_based/      // test_26.cpp, naxis_sim_port.h, …
extras/n_axes/              // test-only HTML template + dump helper
```

Rules:

- No virtual `AxisPort`. The stepper type is a template parameter
  with duck typing (`addQueueEntry`, `isQueueEmpty`, `queueEntries`,
  `getSpeedInTicks` / `getAcceleration` / `getMaxSpeedInTicks` /
  `getDirChangeBeforeTicks` / `getDirChangeBeforePauseCount` /
  `getDirChangeAfterTicks` / `isRampGeneratorActive` / `isRunning`).
  Production default is `FastAccelStepper`.
- No `new` / `malloc`. Lookahead and slice state are member arrays
  sized by template parameters.
- G10: no `float`, no integer `/` in kinematics; `overshoot_max`
  in steps, `kappa_stop_q8` in 1/256 units (diagnostic
  lookahead-speed flag only).
- Trace / HTML dump is compiled only if the TU defines
  `FAS_NAXIS_TRACE`. The production class has no viewer dependency.
- `FasNAxis.h` may include `FastAccelStepper.h` and
  `fas_ramp/RampCalculator.h`. PC tests pass `SimPort` as the
  third template argument.

### 11.2 Class

C++11 (no designated initializers). Default member initializers
are the documented defaults; `FasNAxisConfig{}` is a valid Linear
config.

```cpp
enum PumpStatus {
  Idle,
  Running,
  Underrun,
  Error
};

struct FasNAxisConfig {
  uint32_t dt_ticks = 32000;       // 2 ms at 16 MHz; 0 means this default
  uint16_t kappa_stop_q8 = 320;    // 1.25 in Q8
  uint16_t overshoot_max = 8;      // steps; ignored in Linear
  uint16_t dir_before_ticks = 0;   // 0 = use stepper getDirChangeBeforeTicks()
  uint16_t dir_after_ticks = 0;    // 0 = use stepper getDirChangeAfterTicks()
  enum Mode { Linear, Overshoot } mode = Linear;
};

template <uint8_t NAXES, uint16_t HORIZON = 64,
          typename Stepper = FastAccelStepper>
class FasNAxis {
 public:
  explicit FasNAxis(const FasNAxisConfig& cfg);

  bool addAxis(uint8_t i, Stepper* s);   // i < NAXES; fails if running/ramp
  void setLimitsFromSteppers();          // re-read ticks_cfg / log2_accel
  void syncFromSteppers();               // p[] from stepper positions
  void setCurrentPosition(const int32_t p[NAXES]);

  void addLine(const int32_t p[NAXES]);  // absolute positions, steps
  void addDwellTicks(uint32_t ticks);    // zero-displacement block; v=0 at both ends
  void endPath();                        // decelerate to rest at last p

  PumpStatus pump();                     // plan + feed
  bool isBusy() const;
  bool hasUnderrun() const;
  bool isSpeedLimitedByLookahead() const;  // some R_i < kappa*P_stop, path open; not an error
  uint32_t stopDistanceSteps(uint8_t i) const;  // P_stop_i
  uint32_t remainingSteps(uint8_t i) const;     // R_i
  const char* lookaheadHint() const;     // diagnostic: axis, R vs P_stop, HORIZON

#if defined(FAS_NAXIS_TRACE)
  void enableTrace(Trace* t);
#endif

 private:
  Stepper* _s[NAXES];
  AxisLimits _lim[NAXES];
  Block _blocks[HORIZON];
  // slice / clock / sign / P / R state, also fixed-size
};
```

If a constructor argument is `dt_ticks == 0`, store 32000 (so a
zeroed config still means “default slice”, not a zero-duration
slice). Same for `kappa_stop_q8 == 0` → 320 (diagnostic only).
`overshoot_max == 0` is Linear-like cap and is legal.

`addLine` to the current position (`L = 0`, every `Δ_i = 0`) is a
dwell of 0 ticks (no-op). `addDwellTicks` appends a
zero-displacement block: lookahead treats it as a path-stop,
dwells, then the next `addLine` starts from rest. Calling it
mid-path is therefore a planned stop-and-wait, not a pause
command while still moving at speed.

Sketch (C++11):

```cpp
#include <FastAccelStepper.h>
#include <FasNAxis.h>

FastAccelStepperEngine engine;
FastAccelStepper *x, *y;

void setup() {
  engine.init();
  x = engine.stepperConnectToPin(...);
  y = engine.stepperConnectToPin(...);
  x->setSpeedInHz(4000);
  x->setAcceleration(2000);
  y->setSpeedInHz(4000);
  y->setAcceleration(2000);

  FasNAxisConfig cfg;
  cfg.mode = FasNAxisConfig::Linear;
  FasNAxis<2> path(cfg);
  path.addAxis(0, x);
  path.addAxis(1, y);
  path.syncFromSteppers();
  int32_t a[2] = {10000, 100};
  path.addLine(a);
  path.endPath();
}

void loop() {
  path.pump();
}
```

The first `addLine` is illegal until `syncFromSteppers()` or
`setCurrentPosition()`. FAS positions default to 0, so a machine
that is not at the origin must sync.

Limits are read from the steppers at `addAxis` and on
`setLimitsFromSteppers()`. Changing limits of an in-flight path is
v2. v1: limits are frozen from `pump()` of the first slice until
`endPath()` completes.

2D/3D is `NAXES = 2` or `3`. gnuplot / HTML dump the first two or
three axes.

---

## 12. PC-only test strategy

### 12.1 Why PC exclusive

The planner is discrete math plus a deterministic feeder. Hardware
adds driver drain pauses, interrupt jitter, and RMT/I2S buffering —
already covered by FAS’s own tests (`test_24`, `test_25`, Issue 370
replay). FasNAxis asserts:

- constraint satisfaction of the **plan** (`P ≤ R`, period ≥
  `ticks_min` / `ticks_cfg`)
- integer path error
- feeder protocol against a **queue simulator that implements the
  published `addQueueEntry` contract** (DIR pauses, kick-off,
  `isQueueEmpty`)
- optionally, 1- and 2-axis identity with real `pd_test` queues
- gnuplot of path and per-axis period, same workflow as `test_02`

None of that needs an MCU.

### 12.2 Layout

```
src/FasNAxis.h
src/fas_naxis/*.h
extras/doc/n_axes_whitepaper.md
extras/tests/pc_based/
  test_26.cpp                  // FasNAxis suite; Makefile wildcard picks it up
  naxis_sim_port.h
  naxis_plot.h                 // gnuplot helper (RampChecker-style)
  naxis_html_dump.h            // FAS_NAXIS_TRACE helper
extras/n_axes/
  viewer_template.html
  tests/out/                   // generated HTML, gitignored
```

`test_??.cpp` is already in the pc_based `TESTS` wildcard. No
Makefile special case unless a test must not link `LIB_O`. 3-axis
tests use `FasNAxis<3, …, SimPort>` and do not need `pd_test`
`MAX_STEPPER` raised. 1–2 axis golden runs against real
`FastAccelStepper` objects use the default `Stepper` parameter.

Each fixture writes `test_26_<id>.gnuplot` the way `test_02`
writes `test_02_f5.gnuplot` (path XY, per-axis speed vs time,
period vs time). `make -C extras/tests/pc_based clean` already
removes `*.gnuplot`.

### 12.3 Simulator port

`SimPort` reproduces, in software:

- `QUEUE_LEN` (configurable, default 16)
- command tick sum `steps==0 ? ticks : ticks*steps`
- `AQE_QUEUE_FULL` / `ErrorTicksTooLow` / injected DIR pauses
- pause `steps=0` with explicit `count_up` (never implicit flip)
- revert = pause with `count_up = !old_dir`; by default does
  **not** return Injected (match `pd_test`). A hook can inject
  before/after pauses for F12.
- reversing step commands still inject if DIR was not prepared;
  FasNAxis tests must not take that path
- `isQueueEmpty()` / `queueEntries()` / `isRampGeneratorActive()`
- a `drain(ticks)` that consumes commands the way a 16 MHz ISR
  would, advancing a clock and a position

A `SimRig` holds n ports, a global clock, and a drain scheduler:
`pump()` then `drain(Δ)` until idle or failure. Every accepted
command is appended to a trace:

```
{ t_start, t_end, axis, steps, duration_cmd, actual, rc, pos, P, R }
```

### 12.4 Oracle

After a run, from the trace:

1. **Step sum:** per axis, `Σ steps == p_end − p_start`.
2. **Common start:** first non-zero queue activity within a
   documented kick-off bound (PC: same simulated tick).
3. **No underrun** unless the test is the underrun-negative case.
   Prefill empty queue is not underrun.
3b. **Lookahead:** `P_i ≤ R_i` at every committed sample. Last
   buffered point of an open path is rest. If `R_i < P_stop_i`,
   that axis never claims configured max (F11/F19: speed cap,
   `pump()` still `Running`, `isSpeedLimitedByLookahead()`).
   `endPath()` is the same law with `R` frozen.
4. **Period:** for every command with `steps ≠ 0`,
   `ticks ≥ ticks_min` and `ticks ≥ ticks_cfg` (one-step slack).
5. **Ramp law:** `P ≤ R` at every committed sample. Period is never
   shorter than `calculate_ticks(P)`.
6. **Path, Linear:** distance from each reconstructed integer
   point to the polyline `≤ 0.5 √n` steps (rounding box). Every
   vertex is a sample.
7. **Path, Overshoot:** distance to the planned `p(t)` within the
   rounding box; `d²` to the polyline `≤ overshoot_max²` plus
   rounding. Every trajectory point is visited (integer position
   equals the waypoint at the vertex sample).
8. **Lookahead effect:** a fixture with a long fast segment and a
   late 90° Linear corner must have started decelerating when
   `R` reached `P` (angle-change preparation). A planner that
   only sees the current block and does not sum `R` across
   micro-segments fails F10. F11: short open-path `R` caps speed
   without an error.
9. **Time optimality (weak):** on a single long collinear segment
   from rest to rest, both modes match `calculate_ticks` of a FAS
   1-D ramp within a few Δt. Overshoot wins on polylines where at
   least one axis *continues* through a vertex (F6, F7):
   `T_overshoot ≤ T_linear`.

### 12.5 Fixture list (minimum)

| ID | Setup | Asserts | Plot |
|----|--------|---------|------|
| F1 | 1 axis, 10 000 steps | Both modes match `calculate_ticks` of the FAS ramp | `test_26_f1.gnuplot` speed/pos/time |
| F2 | 2 axis, 45° line, equal limits | Linear: equal `\|Δsteps\|` every slice. Overshoot: `d ≈ 0` | XY overlay |
| F3 | 2 axis, (10000, 100), Linear | Short axis well below its ramp; long axis binds; path on the chord | XY vs chord |
| F4 | Same segment, Overshoot, cap 8 | `max d² ≤ 64`; waypoints hit | XY bulge vs F3 |
| F4b | Same, Overshoot, cap `∞` | Raw profile bulge (order 10 steps, not an L); still hits endpoints | XY |
| F5 | Square 1600, Linear | `P → 0` at each corner (angle change prepared on the side) | XY + v(t) |
| F6 | 45° dog-leg, Overshoot | X continues, Y reverses; `P_x ≠ 0` at the vertex; `P_y = 0`; `T <` Linear | XY flatten |
| F6b | `(0,0)→(4000,1)→(4000,4000)`, Overshoot | Feasible (`P_y` after 1 step is ≤ 1); vertex hit; cap holds | XY |
| F7 | Circle r = 1600, 1° chords, Overshoot | Near-constant path speed; X/Y reverse without stopping the other axis; `d²` per chord ≤ cap | XY circle |
| F8 | 3 axis helix, both modes | HTML 3D overlay of chord vs path; no axis above limits | HTML + gnuplot |
| F9 | Asymmetric limits `ticks_x = 10*ticks_y`, Linear, equal `\|Δ\|` | X is slower so X binds; Y scaled down | XY |
| F10 | Micro-segments totalling a long line, Linear | `R` sees through them; does not stop at each | v(t) no dips |
| F11 | Streaming with `R < P_stop`, path open | speed capped by `R`; `pump()` `Running`; `isSpeedLimitedByLookahead()`; `P ≤ R`; cruise after `R` grows | v(t) capped then recovers |
| F12 | Axis reversal + `dir_after` / `dir_before` | planner issues before (old DIR) + after (new DIR) on all axes; following step has no Injected | event marks |
| F12b | Overshoot corner, X continues at high `P` | X does not step during τ_dir; vertex held; X resumes at same `P` | XY frozen at corner |
| F12c | SimPort injects one extra before-pause | all axes dwell it; no XY leave-vertex; clocks together | event marks |
| F13 | Queue underrun (pump starved) | Flag set; plot still dumped | — |
| F14 | Drift over 60 s simulated | `\|T_i − T_j\|` bounded by a few ticks | T_i − T_j |
| F15 | Slice would exceed `ticks_min` | Planner rejects / clips at plan time, never `ErrorTicksTooLow` at feed | — |
| F16 | `addAxis` while ramp active | `addAxis` fails; no race with `manageSteppers` | — |
| F17 | First fill on empty queue | Not underrun; path completes | — |
| F18 | Linear `(10000, 9000)`, Y 40× slower | Longest is X but Y would exceed `v_max` if scaled to X; Y binds, X scaled down | XY + v(t) |
| F19 | `HORIZON` too small to hold `P_stop` as micro-segments | `addAxis` succeeds; `P` never reaches `P_stop`; same `HORIZON` with one long `addLine` *does* coast | v(t) capped |

F7 is the regression sibling of `examples/MoveTimed`. F5 is Linear
lookahead. F6 / F6b is where `overshoot_max` and `P ≤ \|Δ\|` bite.
F1 is the identity with `RampCalculator`. F11/F19 are lookahead
**speed caps** (not errors, not silent starve). F18 is the
“longest axis too fast for a slave” rebind.

### 12.6 FAS adapter tests

When `pd_test` is linked (`MAX_STEPPER = 2`):

- F1 and F2 run twice, once on `SimPort`, once on real
  `FastAccelStepper` + `fas_queue` drain (the `test_16` pattern).
- Command-by-command identity is **not** required (FAS may insert
  pauses the sim was not configured for). Step sums, duration
  within a small tick budget, and return-code classes are.

3-axis tests never link `LIB_O` unless `pd_test` `MAX_STEPPER` is
raised. That raise is *not* part of this concept.

---

## 13. Plots and static HTML viewer

### 13.1 gnuplot (CI / every fixture)

Same pattern as `RampChecker::start_plot` / `finish_plot` in
`test_02`, `test_08`, `test_15`:

```
test_26_f5.gnuplot → test_26_f5.png
  2×2: XY path (polyline grey, realized on top),
       v_i(t) in steps/s (oracle conversion),
       P_i(t) / R_i(t),
       d(t) or period
```

The C++ oracle is pass/fail. gnuplot is for humans, produced
unconditionally on PC tests (small). `make clean` deletes
`*.gnuplot`.

### 13.2 HTML (optional, `FAS_NAXIS_TRACE`)

For 2D and 3D fixtures, a test may write **one** `.html` file that
opens in a browser with no network. All samples live in the file.
No CDN, no build step for the viewer.

```html
<!DOCTYPE html>
<meta charset="utf-8">
<title>FasNAxis F7 circle</title>
<style>/* inlined */</style>
<canvas id="path"></canvas>
<canvas id="plots"></canvas>
<input type="range" id="scrub">
<pre id="hud"></pre>
<script>
const DATA = { /* … entire payload … */ };
</script>
<script>
/* inlined viewer: draw, scrub, 3D orbit */
</script>
```

The viewer source is a checked-in template
(`extras/n_axes/viewer_template.html`). The test-only dump helper
splices `const DATA = …` into a copy. Tests do not generate JS.
None of this is referenced from `src/FasNAxis.h` unless
`FAS_NAXIS_TRACE` is defined.

### 13.3 Payload

```js
{
  meta: {
    fixture: "F7",
    n: 3,
    names: ["X", "Y", "Z"],
    dt_ticks: 32000,
    ticks_per_s: 16000000,
    limits: [{ ticks_cfg: 4000, a_max: 2000, ticks_min: 80 }, …],
    mode: "Overshoot",
    overshoot_max: 8,
    kappa_stop_q8: 320
  },
  polyline: [[0, 1600, 0], [28, 1599, 4], …],
  samples: [
    { t: 0.000, p: [0, 1600, 0], ticks: [0, 0, 0], P: [0, 0, 0], R: [0, 0, 0],
      steps: [0, 0, 0], actual: [32000, 32000, 32000] }
  ],
  events: [
    { t: 0.0, type: "kickoff" },
    { t: 0.412, type: "junction", vertex: 17, P: [0, 0] },
    { t: 1.002, type: "dir_dwell", axis: 0, ticks: 32000 },
    { t: 2.500, type: "underrun", axis: 1 }
  ]
}
```

Keep samples as a compact typed layout if files get large.
Fixtures should downsample the HTML trace to every k-th slice if
needed, while the oracle still sees every command.

### 13.4 Viewer behaviour

- **2D (`n ≥ 2`):** XY path, colour by path speed; grey commanded
  polyline; realized path on top. In Overshoot, a hair at the
  sample of `max d(t)` and a dashed circle of radius
  `overshoot_max`. Linear runs should lie on the grey line.
  Vertices marked; in Linear, path-stop corners are a distinct
  mark.
- **3D (`n ≥ 3`):** the same with a drag-to-orbit canvas 2D
  projection. No WebGL required.
- **Strip charts:** period and `P_i(t)` / `R_i(t)`, `ticks_cfg`
  hairlines. Limit violations in red. A small `d(t)` strip for
  Overshoot.
- **Scrubber:** time slider; HUD shows `t`, positions, `P`, `R`,
  queue depth if recorded.
- **n > 3:** path view uses the first three axes; strips show all.

The page is for humans to see that F5 (Linear square) really stops
at corners, F4 (Overshoot diagonal) slightly leaves the chord, and
F7 (Overshoot circle) does not stop at the axis extrema.

---

## 14. Worked mini-examples

These numbers are **oracle language** (steps/s). Production uses
`calculate_ticks(P)` / `R`.

### 14.1 One-axis sanity

`ticks_cfg` for 4000 step/s, `a = 2000` step/s², `S = 10 000` steps.

```
P_coast = calculate_ramp_steps(ticks_cfg)   // 4000 steps at these limits
T_acc   ≈ 2 s
s_coast = 10000 − 8000 = 2000
T_coast = 0.5 s
T = 4.5 s
```

`P` starts at 0. Coasting happens because `N/2 = 5000 > P_coast`.
At `Δt = 2 ms`: 2250 slices. Lookahead: a single block, path
ended, `R = 10000`. Feeder issues mostly 8-step slices. Decel
when `R == P`, tapering `P` to 0.

### 14.2 Square, Linear

Side 1600 steps, same limits. `P_coast = 4000 > 1600`, so each
side is a triangle if it starts and ends at rest:

```
v_peak ≈ 1789 step/s
T_side ≈ 1.789 s
T_loop ≈ 7.156 s
```

Lookahead does not shorten this in Linear: every corner fails
§8.5 (axis-aligned 90°), so `R` ends at the vertex and `P → 0`.
It only ensures the triangle is planned *before* the side starts.
Overshoot on the same square is numerically the same path (idle
axis ⇒ everyone at rest at the corner).

### 14.3 Fine circle, Overshoot

Radius 1600, 360 chords of 1°, same limits. Chord length
`2 * 1600 * sin(0.5°) ≈ 27.9` steps.

Linear would stop at the four axis extrema (`Δ_x` or `Δ_y` changes
sign). Overshoot only zeros the reversing axis; the other keeps
`P`. Chordal error per 1° is a fraction of a step — under a
default `overshoot_max = 8`.

### 14.4 Horizon size, Linear

Long axis 10 000 steps at 4000 step/s, 2000 step/s², then a 90°
corner into a 100-step stub, Linear. `R` of the binder ends at
the corner (angle change → path-stop, preparation = full stop).
Decel distance 4000 steps, so `P` must start falling by
`s = 6000`. Split into 100-step blocks: without summing `R`
across them the planner would not see the corner in time. F10 is
that test.

### 14.5 Overshoot bulge on (10000, 100)

Equal limits. Binding axis is X, `T ≈ 4.5 s` as in §14.1. Y
occupies the whole of `T` with a long period
(`log2_divide(log2_T, log2_from(100))`). Y is almost linear in
time; X is the trapezoid.

At `t = 1 s` (X still accelerating): `x ≈ 1000`, `y ≈ 22`, the
chord wants `y = 10`, so `d ≈ 12` steps. Uncapped rest-to-rest
bulge is already in the “slight” ballpark (F4b). A cap of 8
mixes Y a little closer to X’s DDA fraction (F4). Linear (F3)
has `d ≈ 0`.

### 14.6 Anisotropic continuing corner (F6b)

`(0,0) → (4000,1) → (4000,4000)`, `a = 2000`. X rest-to-stop over
4000 steps (`R_x` ends at the first vertex). Y continues, but
`|Δ_y| = 1` on the first block so `P_y ≤ 1` there. Stretching Y
to X’s `T` is one slow step, not a high-`v_out` trapezoid at
`a_max`. `overshoot_max` places that step near the linear
fraction. The second block then accels Y from `P ≤ 1`.

### 14.7 Short lookahead is a speed cap, not an error

Same limits, `P_stop = 4000`. Stream 800-step collinear chunks,
path open, `pump()` between chunks.

After the first chunk the last n-dim point is 800 steps away and
is rest. From rest, peak `P ≈ 400` (accel until `R == P`). Configured
4000 step/s is never reached. `pump()` returns `Running`.
`isSpeedLimitedByLookahead()` is true.

After ten chunks are buffered (`R = 8000` at the head, collinear
so the scan does not stop), `R ≥ P_stop` and the head may coast.
F11 is that recovery. F19 is the same cap with `HORIZON` too
small to ever hold 4000 steps of micro-segments: `addAxis`
succeeds; contrast one 10 000-step `addLine` at the same
`HORIZON`, which *can* coast because `R` is steps, not points.

A 2-axis first chunk `(800, 400)`, Linear: binder X, `R = 800`
to the last point. Y is DDA-slaved onto that triangle — path
direction implies Y’s speed. Overshoot: `R_x = 800`, `R_y = 400`,
`T = max T_opt`, so Y’s shorter `R` lengthens X as well.

---

## 15. Open issues in FAS that this library cares about

These are not FasNAxis bugs. They are interface facts the white
paper refuses to paper over.

1. **`MAX_STEPPER = 2` on `pd_test`.** 3-axis HTML is sim-port
   only unless someone raises that define.
2. **Kick-off is “near” synchronous, not atomic across queues.**
   PC sim can start all queues on the same tick. Hardware cannot,
   unless a future FAS API starts a mask of queues in one
   interrupt-off section. FasNAxis documents the inherited skew
   (one `addQueueEntry(NULL, true)` call per axis).
3. **Buffered ESP32 drivers inject extra drain pauses.**
   `getDirChangeBeforePauseCount()` (RMT idf5/6: 2) plus
   planner-issued pauses should cover it; leftover inject is
   globalized (§4.4.2). The PC sim has this as a configurable
   hook; it does not emulate RMT.

None of these block a PC-tested v1 on `SimPort`.

---

## 16. Implementation phasing (concept only)

Not a commitment. Matches `todo.md` at the repo root. Each phase
is test-first on `extras/tests/pc_based`.

| Phase | Delivers | Tests |
|-------|----------|-------|
| P0 | `RampCalculator` identity + `R` scan (end or direction change), no queues | F1 planner-only, F10 `R`, F19 speed cap, steps 2b/3b theory probes |
| P1 | Linear DDA + longest-then-rebind + `addQueueEntry` feeder | F1–F3, F5, F9, F12, F14–F18 |
| P2 | gnuplot dumps (always) + HTML (`FAS_NAXIS_TRACE`) | F5, F8 Linear pages |
| P3 | Overshoot ramps, `overshoot_max`, continuing-axis corners | F4, F4b, F6, F6b, F7 |
| P4 | Open-path lookahead speed cap + recovery; underrun | F11, F13 |
| P5 | Same header against real `FastAccelStepper` (1–2 axis) | F1/F2 identity-class |

P0 proves `P ≤ R` and that short `R` caps speed. P1 is the first
time queue quantization and DIR pauses exist. The header is in
`src/` from P0; nothing is added to `FastAccelStepper.cpp`.

---

## 17. Summary

FasNAxis is a coordinated-motion planner that uses FAS only as a
tick-exact multi-queue executor. Kinematics are the **existing
log2 ramp map** (`calculate_ticks(P)`). Lookahead is parsed until
**end of path or a direction change**; that `R` is the cap on
ramp-steps (`P ≤ R`). The path direction then implies the other
axes’ speeds. Short lookahead **reduces speed**, it is not an
error. Angle changes need motor accel/decel and therefore
preparation (v1 Linear path-stops; Overshoot prepares per axis).
Execution is `addQueueEntry` commands with a shared tick sum.
Reversals are planner-issued before/after DIR pauses plus
matching dwells. Underrun is a hard error after kick-off.
Production does not use float or integer division for planning.

Two geometry modes, both hitting every trajectory point:

- **Linear** — the longest-distance axis sets the speed; others
  scale down, unless that would make a slave exceed its v/a
  (then that slave binds). DDA on the chords. Path-stop at
  non-collinear / reversing vertices.
- **Overshoot** — per-axis ramps, slight chordal bulge capped by
  `overshoot_max`. Continuing axes keep `P` through a vertex;
  reversing axes go through 0. DIR before/after pauses are
  issued by the planner; leftover injection is a shared dwell,
  not a one-axis time warp.

Delivery is a single header (`src/FasNAxis.h`) with a class
template. Include it when you need coordinated motion; omit it and
the FAS binary is unchanged. Tests, gnuplot, and 2D/3D HTML dumps
live beside the existing PC harness and are not part of that
header.

The existing `examples/MoveTimed` circle is the spiritual
predecessor of the interpolator in Overshoot mode. FasNAxis adds
the missing constraints, the missing horizon, the Linear
alternative, and a checkable trace.
