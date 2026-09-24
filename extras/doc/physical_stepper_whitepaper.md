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
curve), plus inertia and a **single friction force**. The whole model is a
single force integration: the magnetic field drives the rotor, friction is the
only inverse force that slows it, and the rotor position, speed and acceleration
are just the integrated result.

Stall is not something the model *does*. There is no step-loss latch and no
step counter. Stall is an **observation**: the difference between the rotor and
the field position, `Δ = x − x_c`, is simply compared. The magnetic coupling has
a **stable detent once per full step**; as long as `|Δ| < D` the rotor tracks
the field, but once the field out-runs it the periodic detent force *averages
itself out* over a period and the rotor can no longer keep up — friction slows
it and `|Δ|` grows. That growth of the rotor/field position difference is the
only 100 % error condition. The decisive case is a **half-step slip during
coasting**: a momentary mechanical hold that parks the rotor on the separatrix
between two detents — the field then out-runs it and it stalls; a *full-step*
slip parks it on the next detent and it only loses one step. Re-engagement is
likewise not implemented: as the field decelerates the same integration
re-captures the rotor onto the nearest whole-step detent. Every run can be
rendered to
gnuplot as position / speed / acceleration / force / step-error / stall, and the
model emits a sample-per-step time series — the same visualisation the FasNAxis
suite already uses (`test_26.cpp` via `naxis_plot.h`).

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

But a real stepper driven by a real driver has four properties the ideal
model ignores, and all four show up as *error conditions* we would like the
tests to see:

1. **Inertia.** The rotor has mass. Its position cannot track a command that
   changes faster than its drivetrain can accelerate it. During an
   acceleration phase the rotor *lags*; during deceleration it *runs ahead*.
2. **A non-linear (magnetic) coupling.** The stepper's holding torque is a
   bounded, periodic function of the position error with **one stable detent
   per full step**: it is **zero when the rotor is exactly on a detent** (every
   `D` steps), rises to its peak `Fmax` a quarter step away in the *opposing*
   direction, and crosses **zero again at the half step** — the separatrix
   between two detents. It is *not* Hookean, and it is *not* unbounded.
3. **Friction — the only inverse force.** The only thing that opposes the
   magnetic drive is friction. It is small at low speed (an order of magnitude
   below the peak torque), grows with rotor speed, and saturates at the peak
   torque at the motor's pull-out speed. There is no separate viscous-drag
   coefficient; the field uses `Fmax` and friction does the rest.
4. **Loss of synchronism.** If the lag between the command and the rotor ever
   crosses the half-step separatrix, the rotor falls into the next detent — the
   command has slipped past an equilibrium. The motor *loses one or more
   steps*: a whole-step positional error that no ideal model can ever
   represent. A half-step slip during coasting is the cleanest way to force a
   runaway: parking the rotor on the separatrix leaves it unable to follow the
   moving field.

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
  test(plant.last().stall == false, "no stall in a correctly braked move");
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

Newton's second law for the rotor. The magnetic field provides the only
commanded torque; friction is the **only inverse force**:

```
J · dw/dt  =  τ_magnetic(delta)   −   F_friction(w)
```

with the *position error* `delta = x − x_c`. A positive `delta` (rotor ahead
of its command) produces a **negative** (retarding) torque; a negative
`delta` (rotor lagging) produces a positive (accelerating) torque. Dividing by
the inertia `J`:

```
dw/dt  =  (1/J) · ( − Fmax · sin(2π·delta/D)   −   F_friction(w) )
dx/dt  =  w
```

Friction is an **independent mechanical load** — it does *not* scale with the
magnetic torque `Fmax` (a stronger motor does not gain more bearing/load
friction). It has a Coulomb floor and a viscous term:

```
F_friction(w) = friction_static + friction_viscous · |w|      opposite to w
F_friction    ≤ Fmax                                          (capped)
```

The pull-out speed is then *derived* from the balance `F_friction = Fmax`:

```
v_max = (Fmax − friction_static) / friction_viscous
```

so a stronger motor simply has a higher pull-out speed. There is deliberately
no separate `B·w` magnetic-drag term: the force that spins the rotor up is the
field (`Fmax`), and the load is the only thing braking it.

This is the deterministic core. An advanced model can add a broadband noise
term `η(t)` on the right-hand side (§13.4) so the traces carry the jitter a
real encoder would; the mean trajectory above is what the whitepaper builds
first.

Five parameters, all physically named:

| Symbol | Name | Typical PC-test value | Units |
|--------|------|-----------------------|-------|
| `J` | rotor moment of inertia | `1.0e-6` | kg·m² |
| `Fmax` | peak magnetic torque | `1.0` | N·m |
| `friction_static` | Coulomb friction floor | `0.1` | N·m |
| `friction_viscous` | friction per step/s | `2.25e-5` | N·m·s |
| `D` | steps per full step (one detent spacing) | `64` | – |

With these, `v_max = (1.0 − 0.1)/2.25e-5 = 40000` step/s.

The coupling is deliberately shaped to *feel* magnetic, not Hookean:

* **It is zero at each detent.** When the rotor sits on a stable point there is
  no force; the motor rests in the detent. Every full step `D` there is another
  identical stable detent.
* **Magnitude is bounded and peaks in opposition.** For a positive error the
  force is negative (retarding) and reaches its maximum at `delta = +D/4`; for a
  negative error it is positive and peaks at `delta = −D/4`. At the half step
  (`±D/2`) the force is zero again — the separatrix between two detents.

Friction is applied so that it opposes the motion but can never *reverse* it:
each sub-step removes up to `F_friction·Δt/J` of speed, and if that is enough
to reach zero the rotor is held at zero. This gives a rotor at rest a genuine
static friction: while `|τ_magnetic| ≤ friction_static` the rotor stays put, so
a rotor left near a detent stays there.

### 5.3 Time-stepping

The plant integrates with a fixed **Euler-Cromer (semi-implicit) scheme** at
the update period implied by each command, which keeps it stable through the
whole ramp. Friction is applied as a velocity budget that can cancel the motion
but never reverse it:

```
tau    = − Fmax·sin(2π·delta/D)
Ff     = min(Fmax, friction_static + friction_viscous·|w|)
w_free = w + (tau/J)·Δt
dw_fric= Ff·Δt/J
w     ← (|w_free| ≤ dw_fric) ? 0 : w_free − sign(w_free)·dw_fric
x     ← x  + w · Δt          ← note: use the *new* w
```

`Δt` for one step at the current `ticks` is

```
Δt = ticks / TICKS_PER_S     seconds per commanded step
```

For a multi-step command (`steps > 1`) the plant takes `steps` sub-steps; for a
pause (`steps == 0`) it takes one dwell of `ticks` with `x_c` frozen. A pause
does **not** release the motor: the same holding torque of §5.4 is still
applied to the frozen command. Each commanded period is integrated in
sub-steps no longer than `kMaxDt = 250 µs`, so a long period (up to 65535
ticks = 4.1 ms) never out-runs the ≈63 ms detent dynamics — the step size is a
pure accuracy knob and the stall/re-capture result is converged with respect to
it. The whole integration runs in *integer-ish* arithmetic (steps + ticks are
`uint`/`uint16`), but the physics state is `double` so the traces are smooth —
see §9 for the fixed-point variant. A **one-shot rotor displacement**
(`rotate`, the simulation stand-in for a mechanical hold) is applied between
sub-steps, with velocity preserved.

### 5.4 The magnetic torque curve τ(delta) — a single periodic force

The magnetic coupling is a **single bounded, periodic curve** — the entire
motor behaviour is this one plot of torque versus position error, `τ(delta)`:

```
τ(delta) = − Fmax · sin(2π · delta / D)
```

It is smooth, odd, and periodic with period **one full step** (`D` steps), so
`τ(delta) = τ(delta ± D)`. Over the interval `delta ∈ [0, D)` it is the classic
sine detent curve:

| delta | τ | meaning |
|-------|---|---------|
| `0` | `0` | rotor on a detent — stable equilibrium |
| `+D/4` | `−Fmax` | maximum retarding force (rotor ahead) |
| `+D/2` | `0` | the separatrix — unstable, between two detents |
| `−D/4` | `+Fmax` | maximum accelerating force (rotor behind) |
| `±D` | `0` | one full step of error — the next stable detent |

There is no separate “grip lost” branch and no domain boundary: the force is
evaluated for **any** `delta`, and the curve repeats every full step. This
periodicity — combined with the half-step separatrix — is precisely what makes
stall emerge, as described in §5.5.

In the example curves `D = 64`, so the force peaks at `delta = ±16`, the
separatrix is at `delta = ±32`, and the next detent is at `delta = ±64`.

### 5.5 Stall is an observation, not a latch

Stall is **not implemented**. There is no step-loss counter, no grip-lost
branch, no re-capture condition. The only thing the model computes is the
force integration of §5.2 applied to a *periodic* force curve (§5.4). From
that integration, stall is simply an **observation of the rotor/field position
difference**: `|delta| = |x − x_c|` growing beyond a full step.

Why does `|delta|` grow under a fast command? The magnetic torque is the
periodic sine `−Fmax·sin(2π·delta/D)`, repeating every full step. While the
field (labeled “command”) drifts slowly, the rotor tracks it within the detent
well: it sits near a zero of the sine and the force pulls it back whenever it
wanders — `|delta|` stays small. But the field also changes position one step
at a time at the stepping rate. As that rate rises, the rotor — limited by its
inertia `J` and the finite torque `Fmax` — cannot follow each small step;
it is perpetually accelerating but perpetually behind. Over one full step of
command travel the sine force therefore takes a positive half then a negative
half and **averages itself out**: the nearer the field moves than the rotor can
track, the closer the time-average of `τ` gets to zero. What was a restoring
force that holds the rotor becomes, at high rate, a force that cancels — and
friction (the only inverse force, §5.6) then slows the rotor. The gap `|delta|`
between the rotor and the field position then grows without bound (the field
keeps stepping while the rotor drifts back). That growth of the rotor/field
position difference *is* the stall.

**A half-step slip is the cleanest way to force it.** "Mechanically holding"
the rotor is modelled as a one-shot displacement with velocity preserved. Hold
it for **half a full step** and it is parked exactly on the separatrix between
two detents, where the restoring force vanishes; the field then out-runs it and
`|delta|` runs away — a stall induced *shortly after coasting is reached*. Hold
it for a **full step** instead and it is parked on the next stable detent, so it
simply keeps tracking with exactly **one step lost**. The two outcomes are the
same integration, differing only by the separatrix.

There is no step-loss detector and no step-loss counter. The `stall` panel is
a **dynamic observation of slipping**, not a latch on the offset: the rotor is
stalled while it is at least a full step behind (`|delta| > D`) **and** its
speed still differs from the commanded field speed (`w ≠ v_field`). The speed
mismatch is low-passed so a commanded direction change or an acceleration
transient does not read as a stall. A rotor that lost whole steps but has
re-locked and is tracking the field (delta constant, `w = v_field`) is **not**
stalled — the flag returns to `0` as soon as it re-captures, even though the
position offset is permanent. (A plain `|delta| ≥ D` test would latch high
forever on that offset.) A companion latch, `stall_ever()`, records whether a
slip ever occurred, for "a stall happened" assertions.

**Why the two failure modes still differ.** The *inertial lag* of §5.2 is the
bounded, always-recoverable following error during accel/decel: the command
brakes back to rest, the rotor catches up, `|delta|` shrinks again. The
*stall* is the same quantity, but run away: the slip (or a too-fast command)
outran the rotor for long enough that friction carried `|delta|` past the
separatrix and on. Same equation, same observable, different regimes.

### 5.5.1 Re-engagement is not implemented either

Because the force integration runs for *any* `delta` (§5.4, there is no
domain boundary), recovery is a *consequence*, not a rule. If the command is
slowed or stopped, the field no longer outruns the rotor: the detent force is
no longer self-averaged to zero, and the sine pulls the rotor onto the *nearest
whole-step detent*. That is the physically honest meaning of a lost step: the
rotor re-engages, but a whole number of full steps behind the command — the
detent never gives those steps back. Once re-engaged it tracks a fresh move
exactly. If the command is walked *back* toward the rotor by that offset, `x_c`
returns toward `x` and the rotor can even re-capture the original detent. In
all cases it is just the integration continuing. Between the regimes the plant
stays fully deterministic, so a test can assert both halves of the behaviour:

* *“after a stall, a fast command races ahead while the rotor does not keep
  up — the commanded position advances much faster than the rotor”*, and
* *“after the command is slowed, the rotor re-syncs onto the nearest whole-step
  detent and a fresh slow move is then tracked exactly — with no re-capture
  code, just the same integration”.*

This is exactly what a real machine does when a driver command outruns its
load: the pulses keep coming, the driver counts them, the axis slips a whole
number of full steps, and it runs smoothly again once the driver slows down.

### 5.6 Friction and holding torque

Friction is the **only inverse force** to the motor, and it is an *independent
mechanical load* — it does not scale with `Fmax` (a stronger motor does not
have more bearing/load friction). It is a Coulomb floor plus a viscous term,
capped at `Fmax`, and the pull-out speed `v_max = (Fmax − static)/viscous` is
where it balances the field. This gives the two behaviours the review asked
for:

* **Friction is small at small speeds** (a fraction of `Fmax`), so the detent
  force can spin the rotor up from standstill and hold it in a detent against
  small disturbing torques.
* **Friction reaches `Fmax` at `v_max`**, so further acceleration is impossible:
  the motor has reached its pull-out speed. Because the load is independent, a
  stronger motor (`Fmax` up) simply gets a higher `v_max`.

A **half-step slip** is what forces the stall (§5.5). A momentary mechanical
hold displaces the rotor half a full step; that parks it on the separatrix,
where the restoring force vanishes and it can no longer follow the field. A
full-step displacement instead parks it on the next detent, so it only loses
one step. Once a runaway has begun, the detent force self-averages and friction
is left as the only net retarding force, so the rotor drifts while the field
runs on — until the field slows enough (during deceleration) for the detent to
re-capture the rotor onto the nearest whole-step detent.

### 5.7 Mid-band resonance (why a stalled rotor rings)

A stepper is a mass-spring system: the "mass" is the rotor/load inertia `J`, the
"spring" is the slope of the magnetic torque curve at a detent. Its natural
frequency is

```
f_r = (1/2π)·sqrt(K_t / J),     K_t = 2π·Fmax / D   (torsional stiffness)
```

For the canonical plant (`Fmax = 1.0`, `D = 64`, `J = 1e-6`) this is
`f_r ≈ 50 Hz` — inside the documented NEMA-17 **mid-band resonance** band
(50–100 Hz for a typical hybrid stepper). Two well-known consequences follow,
and the model reproduces both:

* **Every step overshoots and rings.** The rotor lands past the detent and
  oscillates; normally the next step damps it, but when the field rate passes
  through `f_r` each step *reinforces* the oscillation.
* **A stalled rotor rings as the field sweeps down through `f_r`.** Once the
  rotor is free (out of synchronism) and the decelerating field crosses the
  resonance, the rotor can swing past the field speed before it settles. With
  weak damping (little friction) it is large; with strong damping it is small.

The amplitude is set by the **damping factor**, which depends on the *ratio of
friction to inertia*. Increasing the inertia alone makes the ringing **worse**
(less relative damping); increasing friction (or microstepping) makes it
better. Because the friction is an independent load, it is set by its own
`friction_static`/`friction_viscous` parameters rather than by `Fmax`.

### 5.8 Reversal dynamics

A commanded reversal (`countUp` flips) makes `delta` *jump sign*, which for
the odd force curve means `τ` flips too. The rotor, now moving fast in the old
direction, decelerates through zero (high force, because `|delta|` is large)
and accelerates back. This is the same stiff, *springy* behaviour real hybrid
steppers show through a reversal, and it is exactly what the
`speed`/`force` gnuplot traces are there to reveal. The model reports the peak
of `|delta|` (`peak_abs_delta()`) — a clean scalar to assert on.

### 5.9 Acoustic emission (and a playable WAV)

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
aligned with its command (`delta = 0`) and a rotor that has run away past a
full step (`|delta| ≥ D`) are both silent or near-silent — here the detent
force has self-averaged to zero, so there is no restoring torque to drive the
hum. The mid-range error — where the motor is working hardest — is loudest. The displacement term
`c·d²x/dt²` adds the mechanical knock of the rotor itself.

A stall is not silent, however. The instant `|delta|` runs away past `D` the
motor is a rotor sitting on a detent while a fast field sweeps past it; the
driver still supplies holding current, and that shows up as a broad, noisy
**buzz** riding on the hum. §5.5.1 records this: the hum drops to the gating
floor exactly when the detent force cancels, and the buzz rises there on top —
so the `.wav` of a stalled move carries a loud, broadband stall transient
rather than silence. A merely lagging rotor (bounded `|delta| < D`) keeps its
hum and stays quiet; only a run-away stall buzzes.

The same signal, sampled on a fixed **44.1 kHz** grid, is the payload of a
plain 16-bit PCM `.wav` file. The plant records one sample per output interval
*while it steps*, so the file spans the **whole simulated move** — not a capped
note. `PhysicalStepper` gains a

```cpp
bool to_wav(const char* path) const;   // 16-bit PCM, mono, 44100 Hz
```

so a test does not only *assert* the sound of the motor: it can **play it
back** and hear the hum drop to the gating floor when the detent force cancels
and the buzz rise on top as the rotor runs away. The canonical trapezoid
of §10.1 runs for 8 s, so its WAV is

```
8 s × 44100 samples/s × 2 bytes = 705600 bytes   (+ a 44-byte header)
```

which is exactly the "around 441000 bytes" a test asserts. This is the natural
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
| `a` | rotor acceleration, steps/s² (raw, to the rotor) | accel |
| `tau` | magnetic force `−Fmax·sin(2π·delta/D)` (scaled) | force |
| `friction` | friction force, signed against the motion | force |
| `stall` | `1` while `\|delta\| ≥ D` — stall *observation* (§5.5) | steploss / stall |

These are exactly the six panels requested: **pos / speed / accel / force /
steploss / stall**.

---

## 7. Gnuplot time-series integration

`test_27` dumps its decimated trace to `.dat` files and writes a companion
`.gnuplot` multiplot. The plant itself exposes the recorder:

```cpp
void   trace_begin(unsigned long decim);   // 1 row per `decim` sub-steps
void   trace_set_phase(int phase);         // accel / coast / decel / pause tag
size_t trace_dump(const char* path);       // t x x_c delta w a tau friction stall phase
```

The produced figure has:

1. **position** — `x` vs `x_c` overlaid (you see the lag and, for the stall
   run, the commanded trace racing ahead of the rotor),
2. **error** — `delta` (inertial lag is bounded and decays; a stall runs away),
3. **speed** — `w` with its reversal overshoot and the pause braking it down,
4. **accel** — `a`, the **raw rotor acceleration**, which exposes the magnetic
   sine's changing sign and the ring-down in the detent,
5. **force** — `τ`, the magnetic torque curve, together with the signed
   `friction` force (the only inverse force),
6. **stall** — the `stall` flag, high while `\|delta\| ≥ D` (§5.5): a
   continuous observation of the rotor/field position difference running away,
   not a counted event.

For the stall case the trapezoid's own trace is plotted alongside, so the
shared acceleration ramp and the divergent ending can be read off directly.

---

## 8. Determinism and testability

The model has **no randomness and no shared hardware state**. For a fixed
`queue_entry` stream and fixed `(J, Fmax, friction, D)`, every `step()` returns
bit-identical results. This makes the assertions in §2.2 exact:

* The stall observation is reproducible — over a fast command `stall` goes
  high while the rotor slips, and returns to `0` once it re-captures.
* Settling bound is reproducible — assert `|delta| < D/2` after the move.
* Reversal peak is reproducible — assert the peak `|delta|` exceeds `X`.
* The re-engagement is reproducible — slow the command and assert `stall`
  returns to `0` with no re-capture branch involved.

A *mutation* that stops the rotor/field position difference from growing (a
hypothetical `-DFORCE_ALWAYS_RESTORING`) must make the stall assertion go
wrong — proving the self-averaging force is load-bearing, the same
discipline `prove_mutations.sh` enforces for FasNAxis.

---

## 9. Portability of the model

* **Host (this paper).** `double` physics, driven by a small integer
  `queue_entry` parser. This is the reference implementation.
* **Target (future).** The physics state can be moved to fixed-point: scale
  `x, w, a` by `2^F` into `int32/int64`, quantise `Fmax`, the friction, `J` to
  integer gains, and replace `sin(2π·delta/D)` with a small phase lookup table
  (or a minimax polynomial) indexed by `delta·(table_size/D)`. That is exactly
  the fixed-point / log2 table style the existing `log2/` library uses. No FP
  unit required.
* **Cost.** Per step: one table lookup, one multiply, a few adds. The
  `steps>1` burst is bounded by `MIN_CMD_TICKS`, so worst case per command is
  a small, fixed number of FMA-like ops — ISR-feasible.

---

## 10. Concrete worked examples (for the harness and the figures)

### 10.1 The canonical rest-to-rest profile

The reference test move is a rest-to-rest profile with a slow re-join segment:

```
accelerate  0 → 2000 step/s      in 0.5 s
coast       2000 step/s          for 1 s
accelerate  2000 → 10000 step/s  in 0.5 s
coast       10000 step/s         for 1 s
coast       10000 step/s         for 1 s
decelerate  10000 → 2000 step/s
coast       2000 step/s          for 3 s
decelerate  2000 → 0 step/s
```

Total exactly `8 s`. This is the shape a healthy axis runs every day. Fed to
the plant it produces the six panels of §6: a bounded lag that is *negative*
during acceleration (rotor behind) and *positive* during deceleration (rotor
ahead), a clean speed profile, and a force trace that follows
`−Fmax·sin(2π·delta/D)`. If the plant parameters are chosen so the curve's
peak can supply the required acceleration, the rotor tracks the command
(`stall` stays `0`, i.e. `|delta|` never reaches `D`) and lands on the target
within a step or two. If `Fmax` is too small for the requested `J` and ramp,
or if the command keeps racing past the rotor, the rotor/field position
difference runs away and the `stall` panel fires — the
emergent stall of §5.5.

Because the move lasts exactly `7 s`, its `.wav` (§5.9) is `7 · 44100 · 2 =
617400` bytes of PCM — the test renders it and checks the file is that size,
which proves the acoustic stream really spans the whole simulation and not
just a snippet.

### 10.2 A half-step slip mid-coast — the decisive stall

The stall case shares the reference's **exact profile**. After the first coast
(`t ≈ 3 s`) the rotor is displaced **half a full step** — the simulation
stand-in for a momentary mechanical hold. That parks it on the separatrix, so
the field out-runs it, `|delta|` runs away and the `stall` panel fires. The
profile then continues: it **decelerates to a small speed (2000 step/s)**, which
the rotor is able to follow again — the detent re-captures it there, and during
the 3 s small-speed coast its position advance and speed match the field — and
finally decelerates both to standstill. `test_27` also runs the counterpart — a
**full-step** slip — which parks the rotor on the next detent and only loses
one step, following the whole profile. Both runs are dumped as gnuplot traces
(`test_27_trapezoid.dat`, `test_27_coast_stall.dat`) so they can be read off
side by side — position, error, speed, raw rotor acceleration, force/friction
and the stall observation.

### 10.3 A move that is *too fast*

```
D = 64,  J = 1e-6,  Fmax = 2e-3,  friction ~ Fmax/10
Commanded trapezoid peak: 100 steps per step-update
Total: 2000 steps.
```

The ramp asks for a step every few microseconds — far faster than the rotor's
`Fmax/J` can accelerate it. The rotor cannot follow each step: the periodic
force self-averages to zero (§5.5) and friction slows the rotor, so the
rotor/field position difference `|delta|` grows past `D` — the emergent stall
of §5.5. The `error` trace shows a lag that *runs away* past a full step; the
`speed` trace shows it decay to rest; the `stall` panel stays high while the
commanded trace races ahead of the stalled rotor. The harness asserts that
`|delta|` crosses `D` (`stall` goes high) — a green test that *detects* an
underspecified ramp.

The inverse case (a slow, correctly braked trapezoid) settles with
`|delta| < D/2` and `stall == 0`, proving the plant can also be *perfect* when
the physics agrees with the plan. And because re-engagement is not implemented,
a command that is slowed after stalling simply re-captures the rotor — the
integration carrying it forward.

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
  double   tau;    // magnetic force, -Fmax * sin(2*pi * delta / D)
  bool     stall;  // 1 while slipping: |delta|>D and w != v_field (§5.5)
};

class PhysicalStepper {
 public:
  // D = steps per full step (default 64).
  // Physical gains: J inertia, Fmax peak torque, and the independent friction
  // load (Coulomb floor + viscous coefficient). v_max is derived.
  PhysicalStepper(uint8_t units_per_full_step = 64,
                  double inertia          = 1.0e-6,
                  double force_gain       = 1.0,
                  double friction_static  = 0.1,
                  double friction_viscous = 2.25e-5);

  // Advance the plant by one queue_entry. Advances the simulated clock by
  // steps*ticks ticks (or ticks for a pause) and advances the commanded
  // position by steps in the entry's direction.
  observed_s step(int steps, bool count_up, uint16_t ticks);

  // Run a whole command list; returns the last observed_s.
  observed_s run(const Cmd* cmds, int len);

  // The position the queue base would report.
  int32_t getCurrentPosition() const { return (int32_t)llround(state_.x); }
  uint32_t total_ticks() const { return ticks_; }

  // Peak |x - x_c| since reset (a clean stall scalar).
  double peak_abs_delta() const;

  // Whether the rotor was ever observed slipping since reset (the stall
  // observation of §5.5 clears when the rotor re-captures; this latches).
  bool stall_ever() const;

  // One-shot external displacement of the rotor by `steps` (velocity
  // preserved): the simulation stand-in for a mechanical hold.
  void rotate(double steps);

  // Emit one row to a multi-panel gnuplot writer (see naxis_plot.h).
  void plot_row(void* gnuplot, uint8_t panel, const observed_s& o);

  // Acoustic emission (§5.9): a hybrid source driven by the actual rotor
  // position, gated by the force curve. Returns true on success.
  bool to_wav(const char* path, uint32_t sr = 44100);

  const observed_s& last() const { return last_; }

 private:
  struct { double x, x_c, w; } state_;   // x (rotor), x_c (cmd), w
  uint8_t  units_per_full_step_;         // D
  double   J_, Fmax_, friction_static_, friction_viscous_;
  uint32_t ticks_;
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
     x_c   = x_c + dir                     // command advances one step
     delta = x - x_c                       // actual - target
     tau   = -Fmax * sin(2*pi*delta/D)     // field for ANY delta (§5.4)
     Ff    = friction_mag(w)               // only inverse force (§5.6)
     w_free = w + (tau / J) * dt
     w     = (|w_free| <= Ff*dt/J) ? 0 : w_free - sign(w_free)*Ff*dt/J
     x    += w * dt                        // uses the *new* w
     stall = |delta|>D && w != v_field     // §5.5: dynamic slip obs
  if steps == 0:                          // pause: hold at x_c
     delta = x - x_c
     tau   = -Fmax * sin(2*pi*delta/D)     // field still acts on frozen cmd
     (same friction-limited velocity update as above)
  ticks_ += (steps == 0) ? ticks : steps*ticks
  record last_ {...}

// Mechanical hold, applied between step() calls:
rotate(steps):  x += steps                // velocity preserved
```

The force is evaluated for **any** `delta` (§5.4); there is no domain
boundary. `stall` is the dynamic slip observation of §5.5 — it is high
while the rotor is a full step behind *and* still slipping, and clears when
the rotor re-captures even though the position offset stays. There is no step-loss counter and no
re-capture branch: recovery (§5.5.1) is just the same integration running
again once the field stops outrunning the rotor.

---

## 12. Test matrix (`test_27.cpp`)

| Section | Input | Assert |
|---------|-------|--------|
| T1 dwell | single pause | `|delta| < D/2`, no drift |
| T2 single step | one step at 200000 ticks | `stall==0`, settles onto command |
| T3 burst | 64 steps @ 16000 ticks | no stall, bounded lag |
| T4 too-fast | 80 steps @ 1 tick | `stall` (dynamic slip) goes high |
| T5 reversal | forward then back | no stall, recovers below the peak |
| T6 canonical | the §10.1 profile | tracks, `stall==0`, lands on target, wav spans 8 s, trace rows |
| T7 weak motor | same profile with `Fmax=2e-3` | loses synchronism (`stall`) |
| T8 over-drive | 80 steps @ 1 tick | emergent `stall` (slipping), `stall_ever()` |
| T9 re-engagement | stall, then slow the command | latches to a whole-step detent, then keeps up |
| T10 wav | moving vs stalled plant | valid RIFF/`to_wav()` both |
| T12 **half-step slip** | §10.1 profile + `rotate(-D/2)` after the first coast | `stall` high during the slip, clears on re-capture; follows at 2000 step/s |
| T13 **full-step slip** | §10.1 profile + `rotate(-D)` | no runaway, ends exactly one full step behind, follows throughout |

For **T6** the canonical profile of §10.1 is the reference: a correctly
parameterised plant follows it exactly and never stalls (`stall` stays `0`);
reducing `Fmax` (make the physics too weak for the ramp, T7) must make `stall`
go high. **T8** is the over-drive case where the field out-runs the rotor's
inertia. **T12/T13** are the decisive slip cases of §10.2: the same profile,
with the rotor displaced half a step (stalls, then re-joins at the small speed)
or a full step (loses exactly one step, follows throughout). Both are dumped as
gnuplot traces so they can be compared on equal footing.

### 12.1 Gnuplot output

`test_27` dumps one decimated row per integration sub-step to
`test_27_trapezoid.dat` and `test_27_coast_stall.dat`, and writes a
`test_27.gnuplot` multiplot beside them. Columns are
`t x x_c delta w a tau friction stall phase`; the six panels are position,
step error, speed, **raw rotor acceleration**, force/friction, and the stall
observation — exactly the parameters the review asked to see.

### 12.2 FasNAxis coupling (`test_26.cpp` F21)

`SimPort` (`naxis_sim_port.h`) is the n-axis suite's duck-typed queue. Its
opt-in `setPhysicalStepper(&plant)` attaches a plant to one axis: `drain_one()`
feeds the plant the very command the ideal counter consumes, so the plant's
rotor is that axis's *realized* position while `position()` stays the ideal
commanded count the planner's DDA binds against. `realizedPosition()` /
`realizedSpeed()` / `realizedDelta()` / `realizedStall()` expose the plant's
observables to the test.

F21 drives the F5 Linear square (1600 steps per side, ticks 4000, accel 2000)
through `FasNAxis<2, 64, SimPort>` with a plant on X and Y. It writes
`test_26_f21.gnuplot` (the realized rotor path on the commanded square, per-axis
rotor speed, P/R, period, and the commanded-minus-realized deviation) and one
16-bit PCM **stereo** wav (`test_26_f21.wav`, X on the left channel, Y on the
right): the two plants' `audio_sample()` streams are interleaved. The asserts
are that neither rotor loses synchronism, the per-axis lag stays under one full
step, both reach a real side speed, the realized position returns to the
origin, and the wav carries audible signal.

The hum is normalized (so it no longer clips for most of a move) and faded with
a rotor-speed envelope, so a ramp does not fade in and out of a clipped,
low-frequency rumble at the start and end of every side.

---

## 13. Open questions

1. **Load-dependent pull-out.** Should `D` or `Fmax` shrink with the external
   load (a loaded motor pulls out sooner)? That would be a parametric scaling
   of the *single* `τ(delta)` curve, not a second curve.
2. **Stick-slip / detent.** A small sawtooth ripple on `τ(delta)` could model
   the detent tooth and the step-to-step ripple in the `position` trace.
3. **Coupling to FasNAxis.** Realized (see §12.2): `test_26.cpp` F21 attaches
   a plant to each `SimPort` axis, so `physical_stepper::getCurrentPosition()`
   supplies the realized position while the planner still binds against the
   ideal commanded count. Open: make step loss *visible to the binder* (a
   re-sync / error response), rather than only observable in the test.
4. **Fixed-point port.** Convert §9's `double` model to the `log2/` table
   style; validate bit-identical stall detection.

---

## 14. Summary

`physical_stepper` is a header-only, deterministic, rotordynamic plant that
replaces the ideal stepper *at the queue boundary*. Its public vocabulary is
**steps**; the full-step span `D` is the plant's private business. It models
inertia (`J`), a **single friction force** that grows with speed from a small
Coulomb floor and saturates at `Fmax` (there is *no* separate viscous-drag
term), and a **bounded, sinusoidal magnetic
coupling**
`τ(delta) = −Fmax·sin(π·delta/D)`
between the commanded and actual rotor positions — zero at the command, peak
opposition at half a full step, and zero again one full step away (and
periodic thereafter). The 100 % error condition is no longer a separate rule:
it is the *observation* that the rotor/field position difference
§5.5 (a full step behind and still slipping). Because the force is periodic,
a command that outruns the
rotor makes that force self-average to zero, friction slows the rotor, and
`|delta|` runs away — the stall. A pause is the decisive case: the detent force
stays in full effect on the frozen command, brakes the coasting rotor hard, and
the resumed command then out-runs `Fmax`. Re-engagement (§5.5.1) is likewise not
implemented: slow the command again and the same integration re-captures the
rotor onto the nearest whole-step detent — the lost steps stay lost. A hybrid
acoustic model derives the emitted hum from the actual rotor position, gated by
the force curve, and writes it to a playable `.wav`. It reports position,
speed, acceleration, force/friction and the stall observation as a gnuplot time
series, giving the PC test suite the
same *physical* failure modes a real motor exhibits — without a timer, without
hardware, and without non-determinism.
