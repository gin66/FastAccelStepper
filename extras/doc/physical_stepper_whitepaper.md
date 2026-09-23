# Physical Stepper Simulation — A Whitepaper

## On replacing the ideal stepper in the PC-based test suite with a
# rotordynamic, step-loss-aware plant model

*Author: the FastAccelStepper TDD effort*
*Status: concept whitepaper*

---

## 1. Abstract

The PC-based test suite (`extras/tests/pc_based/`) processes the real
FastAccelStepper ramp generator through an **ideal stepper**. The ideal stepper
is an *exact* integrator: it counts pulses and never lags, stalls, or rings.
Position is exact, speed is instantaneous, direction reversals are free, and
no command can ever be executed incorrectly. This is excellent for verifying
the planner, but it cannot answer a very practical question:

> *When the planner's ramp is too aggressive for the load, does the motor
> actually stall — and if so, how?*

This whitepaper defines the concept of a **`physical_stepper`**: a lightweight,
header-only rotor model with a *single degree of freedom* — the rotor position
— that turns a stream of `steps`/`direction`/`ticks` commands into physically
believable rotor motion. Between the commanded position and the actual rotor
sits a **bounded, sinusoidal magnetic coupling** (the holding/detent torque
curve), plus inertia and viscous drag. When the actual rotor lags its commanded
reference by a full step, the coupling loses grip: the motor **loses a step**,
the only 100 % error condition. The model emits a sample-per-step time series,
so every run can be rendered to gnuplot as position / speed / acceleration /
force / step-loss / stall — the same visualisation the FasNAxis suite already
uses (`test_26.cpp` via `naxis_plot.h`).

### 1.1 Unit convention

All positions and displacements in this paper are in **steps** — the plant's
finest position quantum. Whether a step corresponds to a full step or to a
fraction of one is a *private constant of the plant*: the plant knows its
full-step span `D`, and it knows what the step quantum is; nothing above the
plant ever needs the word "microstep". Every interface exposed to the queue,
to the tests and to the figures speaks only of steps and full-step spans.

---

## 2. Motivation

### 2.1 The gap the ideal stepper leaves

The ideal stepper is, in effect, an exact integrator:

```
x_commanded += Σ steps           (signed by direction)
t           += Σ ticks×steps
```

Every test that cares about *kinematics* (path following, ramp symmetry,
lookahead) is happy with that. Several of them (`test_26.cpp` F1–F26,
`rmc_test.cpp`, `ramp_helper.cpp`) treat the stepper as a perfect
step/pulse counter, and that is the right tool for what they ask.

But a real stepper driven by a real driver has three properties the ideal
model ignores, and all three show up as *error conditions* we would like the
tests to see:

1. **Inertia.** The rotor has mass. Its position cannot track a command that
   changes faster than its drivetrain can accelerate it. During an
   acceleration phase the rotor *lags*; during deceleration it *runs ahead*.
2. **A non-linear (magnetic) coupling.** The stepper's holding torque is a
   bounded, periodic function of the position error: it is **zero when the
   rotor is exactly on its commanded position**, rises to a maximum in the
   *opposing* direction as the error grows, and returns to **zero again one
   full step away**, where the next stable equilibrium sits. It is *not*
   Hookean, and it is *not* unbounded.
3. **Loss of synchronism.** If the lag between the command and the rotor ever
   reaches one full step, the rotor sits on the wrong stable point — the
   command has slipped past an equilibrium. The motor *loses a step*: a 100 %
   positional error that no ideal model can ever represent.

### 2.2 Why it is opt-in, not the default

The ideal stepper is the *baseline* every test already gets, and it must stay
that way. It is the right tool for kinematics — path following, ramp
symmetry, lookahead — and it never fails. `physical_stepper` is a *layer a
test opts onto*, like a scope probe clipped onto one axis of a machine tool:
you attach it to observe what that axis's rotor is really doing, but you do
not change the machine by clipping the probe on.

So the plant is turned on **by request, per test**, with a compile-time gate
(`FAS_PHYSICAL_STEPPER_ENABLED`, off by default). Every existing test keeps the
ideal stepper untouched; only a dynamics-focused test flips the macro on. This
keeps the kinematic suite bit-identical while giving `test_26` and friends a
physically real rotor they can assert stalling on. The gate means a test that
forgets to opt in cannot silently lose the ideal model.

The FastAccelStepper engine only ever hands the queue a *command*; the driver
is responsible for turning it into pulses. For testing we want a driver whose
behaviour we can *reason about* — one that is deterministic, runs on a host
without timers, and can *fail in physically meaningful ways*. The ideal stepper
never fails. `physical_stepper` fails the way a real motor does, which means
tests can assert:

* "At 200 % above the load's pull-out, every such move loses ≥ N steps",
* "After a hard reversal the rotor settles back to within K steps",
* "No stall is reported during a correctly braked rest-to-rest move",
* "A move whose ramp out-runs the rotor by a full step always loses a step".

That turns *motor dynamics* from an untested assumption into a testable
contract.

---

## 3. Relationship to the existing model

FastAccelStepper already separates *what* to do from *how* to do it:

```
RampGenerator  ──getNextCommand──▶  NextCommand (stepper_command_s)
                                            │
                                            ▼
        StepperQueueBase · addQueueEntry() · (ring of queue_entry)
                                            │
                                            ▼
                              hardware pulse driver (RMT / I2S / timer)
```

`physical_stepper` occupies the bottom two boxes — it **consumes the same
`queue_entry` ring the ISR consumes** and produces the same two things a real
driver produces: an **advanced position** and an **advanced wall-clock
time**. Nothing above the queue (`RampGenerator`, `addQueueEntry`,
`moveTimed`) needs to know which driver is attached.

This is the same discipline `SimPort` already follows in `naxis_sim_port.h`:
"a duck-typed stand-in for the platform `StepperQueue` that the feeder talks
to through `addQueueEntry()`." `physical_stepper` is the *rotordynamic*
descendant of that idea, but scoped to a single axis's rotor.

### 3.1 Opt-in, not a drop-in replacement

The plant is **never the default consumer**. Every existing test in the PC
suite keeps the ideal stepper — a step/pulse counter that never lags,
never stalls, and reverses for free — because that is what tests of
*kinematics* need. `PhysicalStepper` is turned on **by request, per test**:

* A **compile-time gate** (`FAS_PHYSICAL_STEPPER_ENABLED`, off by default) in
  `physical_stepper.h` makes a header-only build fail if the physics is used
  without turning it on, so the ideal model cannot be silently replaced.
* A **per-test hook** `setPhysicalStepper(&plant)` on the queue base swaps
  only the *position* source to the plant's rotor while leaving the ideal
  counter intact. Tests that care about motor dynamics call this once; tests
  that do not leave the ideal counter in place untouched.

The consequence is deliberately conservative:

```cpp
#if FAS_PHYSICAL_STEPPER_ENABLED
  PhysicalStepper plant{64};
  stepper.setPhysicalStepper(&plant);   // opt-in, only where desired
  ...
  test(plant.stalls() == 0, "no stall in a correctly braked move");
#else
  // ideal stepper — unchanged everywhere else
#endif
```

So the ideal model is the **baseline every test already gets**; the plant is a
**layer a test opts onto**. This keeps the kinematic suite bit-identical while
giving dynamics-focused tests (`test_26` and friends) a physically real rotor
they can assert stalling on.

---

## 4. The queue_entry contract (what the plant receives)

Each command the plant sees is a `struct queue_entry` (from
`src/fas_queue/base.h`):

```cpp
struct queue_entry {
  uint8_t steps;            // if 0, the command only adds a delay (pause)
  uint8_t toggle_dir : 1;
  uint8_t countUp : 1;      // +1 / −1 direction
  uint8_t moreThanOneStep : 1;
  uint8_t hasSteps : 1;
  uint8_t dirPinState : 1;
  uint16_t ticks;           // period between steps, in TICKS_PER_S units
#if defined(SUPPORT_QUEUE_ENTRY_END_POS_U16)
  uint16_t end_pos_last16;
#endif
};
```

The plant only needs three fields:

| Field | Meaning for the plant |
|-------|-----------------------|
| `steps` | integer count of step commands dispatched in this update (0 ⇒ dwell) |
| `countUp` | signed direction of the commanded advance |
| `ticks` | commanded *period* between steps; `steps*ticks` is the dwell time |

The driver expands each command into `steps` sub-steps, each `ticks` apart
(and, when `ticks > 65535`, the ramp generator has already split it into a
`steps=1` command plus a `steps=0` pause — so the plant only ever sees
`steps·ticks` in range, but the model must still guard against it).

### 4.1 Position quantisation

The plant advances its *commanded reference* by one step per `steps` count:

```
x_c[k] += (+1 or −1) · 1 step        per step command
```

The plant privately knows how many steps make up a full step — call it `D` —
because the magnetic force curve (§5.4) is periodic over one full step. `D` is
a constructor parameter (`units_per_full_step`, default 64, see §5). Nothing
above the plant ever sees `D`; the public position is always a plain step
count.

---

## 5. The plant model

The model is a **one-body rotordynamics integrator**. The single degree of
freedom is the *rotor position* `x`. Its commanded reference is `x_c`.
Everything else derives from the **position error**

```
delta = x − x_c          (actual minus target, in steps)
```

### 5.1 State

```
x          rotor position         (steps)
w          rotor speed            (steps/s, signed)
a          rotor acceleration     (steps/s², internal only)
x_c        commanded position     (steps, from the queue)
D          steps per full step    (== units_per_full_step)
```

Position reported to the queue base is the *rotor's* position, `round(x)`,
exactly mirroring `getCurrentPosition()` — so the planner still sees a
continuous, *physical* step count instead of the perfect one.

### 5.2 The governing equation

Newton's second law for the rotor, with two torques:

```
J · dw/dt  =  τ_magnetic(delta)   −   B · w
```

with the *position error* `delta = x − x_c`. A positive `delta` (rotor ahead
of its command) produces a **negative** (retarding) torque; a negative
`delta` (rotor lagging) produces a positive (accelerating) torque. Dividing by
the inertia `J`:

```
dw/dt  =  (1/J) · ( − Fmax · sin(π·delta/D)   −   B · w )
dx/dt  =  w
```

This is the deterministic core. An advanced model can add a broadband noise
term `η(t)` on the right-hand side (§13.4) so the traces carry the jitter a
real encoder would; the mean trajectory above is what the whitepaper builds
first.

Three parameters, all physically named:

| Symbol | Name | Typical PC-test value | Units |
|--------|------|-----------------------|-------|
| `J` | rotor moment of inertia | `1.0e-6` | kg·m² |
| `B` | viscous drag coefficient | `1.0e-6` | N·m·s |
| `Fmax` | peak magnetic torque | `1.0e-1` | N·m |
| `D` | steps per full step | `64` | – |

The coupling is deliberately shaped to *feel* magnetic, not Hookean:

* **It is zero at zero error.** When the rotor sits on its command there is no
  force; the motor is at rest in a stable detent. (The quadratic form used in
  the first draft grew without bound and never returned to zero — that is not
  a detent curve.)
* **Magnitude is bounded and peaks in opposition.** For a positive error the
  force is negative (retarding) and reaches its maximum at `delta = +D/2`; for
  a negative error it is positive and peaks at `delta = −D/2`.
* **Sign follows the error**, so the force always acts to pull the rotor back
  toward the command — exactly the "force is accelerating/decelerating the
  rotor depending on the orientation" behaviour requested.

Drag `B·w` is a linear (Sutherland-free, first-order) viscous term so the
rotor comes to rest in finite time with no limit-cycle chatter; it also makes
the model's response well-posed for the gnuplot traces.

### 5.3 Time-stepping

The plant integrates with a fixed **Euler-Cromer (semi-implicit) scheme** at
the update period implied by each command, which keeps it stable through the
whole ramp:

```
a     = ( − Fmax·sin(π·delta/D) − B·w ) / J
w    ← w + a · Δt
x    ← x  + w · Δt          ← note: use the *new* w
```

`Δt` for one step at the current `ticks` is

```
Δt = ticks / TICKS_PER_S     seconds per commanded step
```

For a multi-step command (`steps > 1`) the plant takes `steps` sub-steps; for a
pause (`steps == 0`) it takes one dwell of `ticks` with `x_c` frozen. A pause
does **not** release the motor: the same holding torque of §5.4 is still
applied to the frozen command, so the rotor is held (and rings down) at its
commanded position instead of coasting. The whole integration runs in
*integer-ish* arithmetic (steps + ticks are `uint`/`uint16`), but the physics
state is `double` so the traces are smooth — see §9 for the fixed-point
variant.

### 5.4 The magnetic torque curve τ(delta) and its full-step span

The magnetic coupling is a **single bounded curve** — the entire motor
behaviour is this one plot of torque versus position error, `τ(delta)`:

```
τ(delta) = − Fmax · sin(π · delta / D)      for |delta| < D
τ(delta) = 0                                 for |delta| ≥ D   (grip lost)
```

It is smooth, odd, and periodic over a full step:

| delta | τ | meaning |
|-------|---|---------|
| `0` | `0` | rotor aligned with its command — stable detent |
| `+D/2` | `−Fmax` | maximum retarding force (rotor far ahead) |
| `−D/2` | `+Fmax` | maximum accelerating force (rotor far behind) |
| `±D` | `0` | one full step of error — the next stable point; grip lost |

There is no separate "pull-in" vs "pull-out" branch: the pull-out limit is
simply the point where the curve crosses zero, one full step of error. A
motor that pulls out sooner under load is modelled by a smaller `D` or a
smaller `Fmax` — the underlying function is still one `τ(delta)`.

In the example curves `D = 64`, so the force peaks at `delta = ±32` and
returns to zero at `delta = ±64`.

### 5.5 Step loss = one full step of error (100 %)

The hard condition is:

> *If the lag between actual and commanded position reaches one full step, a
> step is lost — which is a 100 % error condition.*

With the bounded curve of §5.4 this is no longer a bolt-on detector: it is the
**domain boundary of the force law itself**. Once `|delta| ≥ D`, the coupling
torque is zero and the rotor can no longer be pulled to its command. The plant
records a **stall event**, increments `stall_count`, and the rotor position is
left where it is (it cannot be where the command says it should be).

* The condition is on the **actual lag**, `delta = x − x_c`, not on the
  commanded delta.
* It fires the instant that lag reaches the curve's zero crossing `D`.
* It is reported separately from the *continuous* inertial following (§5.2),
  which is always bounded and recoverable. A step loss requires the command to
  come back within a full step before the rotor can be recaptured (§5.5.1).

Two failure modes therefore coexist:

| Mode | Cause | Recoverable? | Reported as |
|------|-------|--------------|-------------|
| **Inertial lag** | rotor can't keep up during accel/decel | yes (settles back) | `delta` trace |
| **Step loss / stall** | `\|delta\| ≥ D` (one full step of error) | only after re-sync | `stall` event, `stall_count` |

### 5.5.1 Stall is a latch: over-drive pulses impart no motion

Once grip is lost the motor is no longer synchronised: the field keeps rotating
at the commanded rate while the rotor stays on the detent where it fell. The
defining, testable consequence is:

> *A stalled motor does not move under high-speed pulses. Feeding more fast
> pulses advances the **commanded** position while the **actual** rotor stays
> put.*

The plant models this directly: while `|delta| ≥ D` the coupling torque is
zero, so a stalled rotor is subject to drag alone and holds its position
(any residual speed decays, §5.6). The command, meanwhile, keeps advancing one
step per pulse, so `delta` grows one step per command. The gap between `x_c`
(racing ahead) and `x` (flat-lining) *is* the visible signature of a stall: on
the position panel the commanded trace climbs while the rotor trace is a
horizontal line; on the `error` panel `delta` grows linearly; and every
over-drive command increments `stall_count`.

Recovery happens through the same curve and only through it. If the command is
slowed, stopped and walked *back* toward the rotor, `x_c` returns toward `x`;
the instant `|delta| < D` the sine re-engages and the rotor is captured again
by the nearest detent. Between the two regimes the plant stays fully
deterministic, so a test can assert both halves of the behaviour:

* *"after a stall, N further fast pulses move the rotor by 0 steps while the
  commanded position advances by N"*, and
* *"after walking the command back within `D`, the rotor moves again"*.

This is exactly what a real machine does when a driver command outruns its
load: the pulses keep coming, the driver counts them, and the axis simply does
not move.

### 5.6 Drag and holding torque

The viscous drag `B` governs the *post-manoeuvre* behaviour. After a
rest-to-rest move the rotor does not snap to `x_c`; it rings down with an
envelope `w(t) ∝ e^(−(B/J)t)`. The gnuplot `speed` trace therefore shows a
decaying oscillation — the "motor ringing on stop" that a perfect model never
shows. The *holding torque* is what makes that ringing end at the commanded
position rather than drifting: during a pause the command is frozen but the
`τ(delta)` force is still applied, so the rotor is clamped to its detent. A
pause is a hold, not a release.

### 5.7 Reversal dynamics

A commanded reversal (`countUp` flips) makes `delta` *jump sign*, which for
the odd force curve means `τ` flips too. The rotor, now moving fast in the old
direction, decelerates through zero (high force, because `|delta|` is large)
and accelerates back. This is the same stiff, *springy* behaviour real hybrid
steppers show through a reversal, and it is exactly what the
`speed`/`force` gnuplot traces are there to reveal. The model reports
`peak_reversal_mis` — the maximum `|delta|` reached during a reversal — a
clean scalar to assert on.

### 5.8 Acoustic emission (and a playable WAV)

A stepper is audible, so a complete model predicts the *sound*, not just the
motion. The sound is driven by the **actual position** of the rotor, not by
the commanded step train: a rotor that cannot follow its command (a lagging or
desynced rotor) emits less, and a stalled rotor goes quiet. The model uses a
**hybrid** source:

```
f           = full-step (electrical) rate, |w| / D    (Hz)
gate(τ)     = |τ(delta)| / Fmax                       ∈ [0, 1]
p(t)        = gate(τ) · Σ_k A_k · sin(2π k f t)       electromagnetic hum
            + c · d²x/dt²                             displacement term
loudness    ≈ sqrt( Σ_k A_k² ) → dB ≈ 20·log10(loudness)  (relative to a ref)
```

The pitch tracks the *actual* rotor motion, but at the **full-step rate**
`|w|/D` — the rate the magnetic field turns — not the finest step rate. (Using
the step rate itself would place the fundamental in the multi-kHz range and
fold its harmonics into aliasing noise; the full-step rate is the physically
audible hum, e.g. `10000/64 ≈ 156 Hz` for the canonical trapezoid.)

The amplitude is gated by the magnitude of the magnetic force, so a rotor
aligned with its command (`delta = 0`) and a rotor that has lost grip
(`|delta| ≥ D`) are both silent or near-silent, while the mid-range error —
where the motor is working hardest — is loudest. The displacement term
`c·d²x/dt²` adds the mechanical knock of the rotor itself.

The same signal, sampled on a fixed **44.1 kHz** grid, is the payload of a
plain 16-bit PCM `.wav` file. The plant records one sample per output interval
*while it steps*, so the file spans the **whole simulated move** — not a capped
note. `PhysicalStepper` gains a

```cpp
bool to_wav(const char* path) const;   // 16-bit PCM, mono, 44100 Hz
```

so a test does not only *assert* a stall drops loudness: it can **play it
back** and hear the hum drop when the rotor loses grip. The canonical trapezoid
of §10.1 runs for 10 s, so its WAV is

```
10 s × 44100 samples/s × 2 bytes = 882000 bytes   (+ a 44-byte header)
```

which is exactly the "around 882000 bytes" a test asserts. This is the natural
companion to the gnuplot traces in §7 and needs no external tool.

---

## 6. Outputs / observables

Every plant instance exposes a small `observed_s` snapshot after each
`step()` call. The harness records one row per command (or per step for
smoothness), giving a full time series:

| Field | Meaning | Gnuplot panel |
|-------|---------|---------------|
| `t` | accumulated wall clock | x-axis (all) |
| `x` | rotor position in steps | position |
| `x_c` | commanded position in steps | position |
| `delta` | `x − x_c`, the position error in steps | position / error |
| `w` | rotor speed, steps/s | speed |
| `a` | rotor acceleration, steps/s² | accel |
| `tau` | magnetic force `−Fmax·sin(π·delta/D)` (scaled) | force |
| `stall` | 1 if this step hit `\|delta\| ≥ D`, else 0 | steploss / stall |
| `stall_count` | cumulative step losses | — |

These are exactly the six panels requested: **pos / speed / accel / force /
steploss / stall-event**.

---

## 7. Gnuplot time-series integration

`test_26.cpp` already knows how to write multi-panel gnuplot files through
`naxis_plot.h`. `physical_stepper` adds the same capability to a single axis
with a helper in the same header:

```cpp
// Emit one row to the six-panel physical-stepper figure.
void PhysicalStepper::plot_row(Gnuplot* g, uint8_t panel,
                               const observed_s& o);
```

The produced `.gnuplot` file has:

1. **position** — `x` vs `x_c` overlaid (you see the lag and the step-loss
   discontinuity),
2. **error** — `delta` (inertial lag is bounded and decays; a step loss is a
   jump),
3. **speed** — `w` with its reversal overshoot and post-stop decay,
4. **accel** — `a`, the derivative that exposes the sine curve's changing sign,
5. **force** — `τ`, the magnetic torque curve,
6. **stall** — a stem/impulse trace, high exactly at the `|delta| ≥ D` events,
   plus a running `stall_count` title line.

The figure is written next to `test_26_f*.gnuplot` and rendered with the
existing `make png` rule — no new tooling.

---

## 8. Determinism and testability

The model has **no randomness and no shared hardware state**. For a fixed
`queue_entry` stream and fixed `(J, B, Fmax, D)`, every `step()` returns
bit-identical results. This makes the assertions in §2.2 exact:

* Step-loss count is reproducible — assert `stall_count == 3`.
* Settling bound is reproducible — assert `|delta| < D/2` after the move.
* Reversal peak is reproducible — assert `peak_reversal_mis > X`.

A *mutation* that removes the force-curve domain check (a hypothetical
`-DNO_STALL_DETECT`) must make `stall_count` assertions fail — proving the
check is load-bearing, the same discipline `prove_mutations.sh` enforces for
FasNAxis.

---

## 9. Portability of the model

* **Host (this paper).** `double` physics, driven by a small integer
  `queue_entry` parser. This is the reference implementation.
* **Target (future).** The physics state can be moved to fixed-point: scale
  `x, w, a` by `2^F` into `int32/int64`, quantise `Fmax`, `B`, `J` to integer
  gains, and replace `sin(π·delta/D)` with a small phase lookup table (or a
  minimax polynomial) indexed by `delta·(table_size/D)`. That is exactly the
  fixed-point / log2 table style the existing `log2/` library uses. No FP unit
  required.
* **Cost.** Per step: one table lookup, one multiply, a few adds. The
  `steps>1` burst is bounded by `MIN_CMD_TICKS`, so worst case per command is
  a small, fixed number of FMA-like ops — ISR-feasible.

---

## 10. Concrete worked examples (for the harness and the figures)

### 10.1 The canonical trapezoid

The reference test move is a rest-to-rest trapezoid:

```
accelerate  0 → 10000 step/s   in 1 s    (acceleration 10000 step/s²)
coast       10000 step/s       for 8 s
decelerate  10000 step/s → 0   in 1 s
distance = ½·10000·1 + 10000·8 + ½·10000·1 = 90000 steps
```

This is the shape a healthy axis runs every day. Fed to the plant it produces
the six panels of §6: a bounded lag that is *negative* during acceleration
(rotor behind) and *positive* during deceleration (rotor ahead), a clean
trapezoid on the `speed` panel, and a force trace that follows
`−Fmax·sin(π·delta/D)`. If the plant parameters are chosen so the curve's
peak can supply the required acceleration, `stall_count == 0` and the rotor
lands on `90000` within a step or two. If `Fmax` is too small for the
requested `J` and ramp, the lag grows monotonically until `|delta| ≥ D`, and
the `stall` panel fires.

Because the move lasts exactly `10 s`, its `.wav` (§5.8) is `10 · 44100 · 2 =
882000` bytes of PCM — the test renders it and checks the file is that size,
which proves the acoustic stream really spans the whole simulation and not
just a snippet.

### 10.2 A move that is *too fast*

```
D = 64,  J = 1e-6,  B = 2e-5,  Fmax = 2e-3
Commanded trapezoid peak: 100 steps per step-update
Total: 2000 steps.
```

The ramp asks for a step every few microseconds — far faster than the rotor's
`Fmax/J` can accelerate it. The required `|delta|` grows past `D`, the force
curve loses grip, and `stall_count` climbs by one for every step whose lag
reaches a full step. The `error` trace shows a lag that *snaps* at each loss;
the `speed` trace shows the decay; the `stall` trace fires a spike for every
dropped step. The harness asserts `stall_count > 0` — a green test that
*detects* an underspecified ramp.

The inverse case (a slow, correctly braked trapezoid) settles with
`|delta| < D/2` and `stall_count == 0`, proving the plant can also be *perfect*
when the physics agrees with the plan.

---

## 11. Specimen API

All of the code below is behind an **opt-in gate**. `physical_stepper.h`
defines `FAS_PHYSICAL_STEPPER_ENABLED` (default 0); the header compiles an
inert stub while the macro is 0, so no build can lose the ideal stepper by
accident. Only a test that *wants* dynamics turns it on — one `#define` at the
top of its file — so every kinematic test keeps the ideal model untouched.

```cpp
#ifndef FAS_PHYSICAL_STEPPER_H
#define FAS_PHYSICAL_STEPPER_H

#include "fas_queue/base.h"

// Observed snapshot after one step() — one row of the time series.
struct observed_s {
  uint32_t t;      // accumulated wall clock (ticks)
  double   x;      // rotor position, steps (physical, reported to queue)
  double   x_c;    // commanded position, steps
  double   delta;  // position error x - x_c, steps
  double   w;      // rotor speed, steps / s
  double   a;      // rotor accel, steps / s^2
  double   tau;    // magnetic force, -Fmax * sin(pi * delta / D)
  bool     stall;  // |delta| >= D fired this step
};

class PhysicalStepper {
 public:
  // D = steps per full step (default 64).
  // Physical gains J, B, Fmax.
  PhysicalStepper(uint8_t units_per_full_step = 64,
                  double inertia   = 1.0e-6,
                  double drag      = 1.0e-6,
                  double force_gain= 1.0e-1);

  // Advance the plant by one queue_entry. Advances the simulated clock by
  // steps*ticks ticks (or ticks for a pause) and advances the commanded
  // position by steps in the entry's direction.
  observed_s step(int steps, bool count_up, uint16_t ticks);

  // Run a whole command list; returns the last observed_s.
  observed_s run(const Cmd* cmds, int len);

  // The position the queue base would report.
  int32_t getCurrentPosition() const { return (int32_t)llround(state_.x); }
  uint32_t stalls() const { return stalls_; }
  uint32_t total_ticks() const { return ticks_; }

  // Emit one row to a multi-panel gnuplot writer (see naxis_plot.h).
  void plot_row(void* gnuplot, uint8_t panel, const observed_s& o);

  // Acoustic emission (§5.8): a hybrid source driven by the actual rotor
  // position, gated by the force curve. Returns true on success.
  bool to_wav(const char* path, uint32_t sr = 44100);

  const observed_s& last() const { return last_; }

 private:
  struct { double x, x_c, w; } state_;   // x (rotor), x_c (cmd), w
  uint8_t  units_per_full_step_;         // D
  double   J_, B_, Fmax_;
  uint32_t ticks_, stalls_;
  observed_s last_;
};

#endif
```

### 11.1 step() pseudocode

```
step(steps, count_up, ticks):
  dir    = count_up ? +1 : -1
  dt     = ticks / TICKS_PER_S
  for k in 0 .. steps-1:
     x_c   = state_.x_c + dir                     // command advances one step
     delta = state_.x - x_c                        // actual - target
     if |delta| >= D:                              // full-step (100%) error
        stall = true; tau = 0                      // grip lost
     else:
        tau = -Fmax * sin(pi * delta / D)
     a       = (tau - B*state_.w) / J
     state_.w += a * dt
     state_.x += state_.w * dt
     state_.x_c = x_c
  if steps == 0:                                   // pause: hold at x_c
     delta = state_.x - state_.x_c
     tau   = |delta| < D ? -Fmax*sin(pi*delta/D) : 0
     a     = (tau - B*state_.w) / J
     state_.w += a * dt
     state_.x += state_.w * dt
  ticks_ += (steps == 0) ? ticks : steps*ticks
  record last_ {...}
```

The domain check is the single `stall` boolean; the bounded sine is the `tau`
branch; drag is the pause decay.

---

## 12. Test matrix (proposed `test_27.cpp`)

| Section | Input | Assert |
|---------|-------|--------|
| T1 dwell | single pause | `w → 0`, `|delta| < D/2` |
| T2 single step | one step at 1000 ticks | `stall==0`, settles |
| T3 burst | 64 steps @ 1000 ticks | no stall, bounded lag |
| T4 too-fast | 64 steps @ 100 ticks | `stall_count > 0` |
| T5 reversal | forward then back | `peak_reversal_mis > 0`, recovers |
| T6 canonical | 0→10000/s in 1 s, coast 8 s, decel 1 s | tracks in 10 s, `stall_count == 0`, lands on 90000, wav ≈ 882000 B |
| T7 full-step loss | lag driven to `D` in one update | `stall==1` when `|delta| ≥ D` |
| T8 brake | correctly braked trapezoid | `stall_count == 0`, `|delta| < D/2` at rest |
| T9 **stalled axis is inert** | stall, then N more high-speed pulses | `Δx == 0` while `Δx_c == N`; `stall_count` grows by N |
| T10 **re-sync** | stall, then walk `x_c` back within `D` | rotor moves again, `stall_count` stops growing |

For **T6** the canonical trapezoid of §10.1 is the reference: a correctly
parameterised plant follows it exactly and reports no stall; reducing `Fmax`
(make the physics too weak for the ramp) must make `stall_count > 0`. For
**T7**, the plant is driven until the accumulated lag reaches a full step, and
the step-loss event is checked at the exact crossing. **T9** is the
"over-drive pulses are not able to move the motor anymore" contract of
§5.5.1, and **T10** its converse: the axis is recoverable only by walking the
command back within a full step.

---

## 13. Open questions

1. **Load-dependent pull-out.** Should `D` or `Fmax` shrink with the external
   load (a loaded motor pulls out sooner)? That would be a parametric scaling
   of the *single* `τ(delta)` curve, not a second curve.
2. **Stick-slip / detent.** A small sawtooth ripple on `τ(delta)` could model
   the detent tooth and the step-to-step ripple in the `position` trace.
3. **Coupling to FasNAxis.** The six-axis planner (`test_26.cpp`) already
   tracks commanded vs realized per axis; `physical_stepper::getCurrentPosition()`
   drops straight into that loop, making step loss *visible to the binder*.
4. **Fixed-point port.** Convert §9's `double` model to the `log2/` table
   style; validate bit-identical stall detection.

---

## 14. Summary

`physical_stepper` is a header-only, deterministic, rotordynamic plant that
replaces the ideal stepper *at the queue boundary*. Its public vocabulary is
**steps**; the full-step span `D` is the plant's private business. It models
inertia (`J`), viscous drag (`B`), and a **bounded, sinusoidal magnetic
coupling**
`τ(delta) = −Fmax·sin(π·delta/D)`
between the commanded and actual rotor positions — zero at the command, peak
opposition at half a full step, and zero again one full step away. The 100 %
error condition is no longer a separate rule: it *is* the domain boundary of
that curve, a **step loss when `|delta| ≥ D`**. A hybrid acoustic model derives
the emitted hum from the actual rotor position, gated by the force curve, and
writes it to a playable `.wav`. It reports position, speed, acceleration,
force and every step-loss event as a gnuplot time series, giving the PC test
suite the same *physical* failure modes a real motor exhibits — without a
timer, without hardware, and without non-determinism.
