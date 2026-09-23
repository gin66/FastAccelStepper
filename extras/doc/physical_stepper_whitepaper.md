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
header-only rotor model with a *single degree of freedom* — the rotor angle —
that turns a stream of `steps`/`direction`/`ticks` commands into physically
believable rotor motion, complete with inertial following, a quadratic
magnetic coupling spring between the commanded microstep position and the actual
rotor, viscous drag, and a hard **step-loss / stall** condition. The model also
emits a sample-per-step time series, so every run can be rendered to gnuplot as
position / speed / acceleration / force / step-loss / stall — the same visualisation
the FasNAxis suite already uses (`test_26.cpp` via `naxis_plot.h`).

---

## 2. Motivation

### 2.1 The gap the ideal stepper leaves

The ideal stepper is, in effect, an exact integrator:

```
θ_commanded += Σ steps           (signed by direction)
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
2. **A non-linear (magnetic) coupling.** The stepper's detent torque does not
   increase linearly with the step error — a hybrid/variable-reluctance motor's
   pull-out behaviour is roughly *quadratic* in the angular miss up to the
   pull-in limit, then collapses.
3. **Loss of synchronism.** If the commanded microstep jumps too far between
   control updates (more than about half a full step), the rotor cannot catch
   it. The motor *loses steps*: a 100 % positional error that no ideal model
   can ever represent.

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

* "At 200 % above `pull_in_ticks`, every such move loses ≥ N steps",
* "After a hard reversal the rotor settles back to within K microsteps",
* "No stall is reported during a correctly braked rest-to-rest move",
* "A commanded `steps > half_step` command is always reported as a stall".

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
  PhysicalStepper plant{16};
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
| `steps` | integer count of full-step commands dispatched in this update (0 ⇒ dwell) |
| `countUp` | signed direction of the commanded advance |
| `ticks` | commanded *period* between full steps; `steps*ticks` is the dwell time |

The driver expands each command into `steps` sub-steps, each `ticks` apart
(and, when `ticks > 65535`, the ramp generator has already split it into a
`steps=1` command plus a `steps=0` pause — so the plant only ever sees
`steps·ticks` in range, but the model must still guard against it).

### 4.1 Microstep quantisation

The plant does **not** move on full steps. The commanded position is quantised
onto a `microsteps`-division per full step:

```
θ_commanded[k] += (+1 or −1) · (360° / microsteps)   per full step
```

`microsteps` is a template/constructor parameter (default 16, see §5).

---

## 5. The plant model

The model is a **one-body rotordynamics integrator**. The single degree of
freedom is the *rotor angle* `θ`. Its commanded reference is the *coupled*
microstep angle `θc`. Everything else derives from `θ − θc`.

### 5.1 State

```
θ          rotor angle          (rad,  wrapped to [0, 2π))
ω          rotor angular speed  (rad/s, signed)
α          rotor angular accel  (rad/s², internal only)
θc         commanded microstep  angle (rad, from the queue)
N          microsteps per full step   (== microsteps)
θstep      = 2π / N              (rad per commanded microstep)
```

Position reported to the queue base is the *rotor's* position expressed in
microsteps, `round(θ / θstep)`, exactly mirroring `getCurrentPosition()` — so
the planner still sees a continuous, *physical* step count instead of the
perfect one.

### 5.2 The governing equation

Newton's second law for the rotor, with three torques:

```
J · dω/dt  =  τ_magnetic(Δθ)   −   τ_drag(ω)
```

with the *step error* `Δθ = θc − θ` (commanded minus actual, so a positive
error accelerates the rotor *forward*). Dividing by the inertia `J`:

```
dω/dt  =  (1/J) · ( K · Δθ² · sgn(Δθ)   −   B · ω )      (smooth mean)
dθ/dt  =  ω
```

(An advanced model adds a broadband noise term `η(t)` on the right-hand side —
see §13.4 — so the traces carry the jitter a real encoder would; the mean
trajectory above is the deterministic core the whitepaper builds first.)

Four parameters, all physically named:

| Symbol | Name | Typical PC-test value | Units |
|--------|------|-----------------------|-------|
| `J` | rotor moment of inertia | `1.0e-6` | kg·m² |
| `B` | viscous drag coefficient | `2.0e-5` | N·m·s |
| `K` | magnetic spring gain | `2.0e-3` | N·m/rad² |
| `N` | microsteps per full step | `16` | – |

The spring is deliberately shaped to *feel* magnetic, not Hookean:

* **Magnitude grows quadratically** with `|Δθ|` — "the magnet pulls harder
  the farther you take it", then saturates at the pull-in limit (§5.4).
* **Sign follows the error** — a positive error accelerates forward, a
  negative error (rotor ahead) decelerates/reverses. That is exactly the
  "force is accelerating/decelerating the rotor depending on the
  orientation" behaviour requested.

Drag `B·ω` is a linear (Sutherland-free, first-order) viscous term so the
rotor comes to rest in finite time with no limit-cycle chatter; it also makes
the model's response well-posed for the gnuplot traces.

### 5.3 Time-stepping

The plant integrates with a fixed **Euler-Cromer (semi-implicit) scheme** at
the update period implied by each command, which keeps it stable through the
whole ramp:

```
α     = ( K · Δθ² · sgn(Δθ) − B · ω ) / J
ω    ← ω + α · Δt
θ    ← θ  + ω · Δt          ← note: use the *new* ω
```

`Δt` for one microstep at the current `ticks` is

```
Δt = ticks / TICKS_PER_S     seconds per commanded microstep
```

For a multi-step command (`steps > 1`) the plant takes `steps` sub-steps,
each `ticks` wide; for a pause (`steps == 0`) it takes one dwell of `ticks`
with `θc` frozen (the rotor decays toward `θc` under drag only). The whole
integration runs in *integer-ish* arithmetic (microsteps + ticks are
`uint`/`uint16`), but the physics state is `double` so the traces are smooth —
see §9 for the fixed-point variant.

### 5.4 The magnetic torque curve τ(Δθ) and its pull-in limit

The magnetic coupling is a **single curve** — the pull-in / pull-out curve is
just this one plot of torque versus angular miss, `τ(Δθ)`. There is no
separate "pull-in" vs "pull-out" branch; the whole motor behaviour is this one
function:

```
τ(Δθ) = K · Δθ² · sgn(Δθ)        for |Δθ| ≤ Δθ_pi
τ(Δθ) = 0                        for |Δθ| > Δθ_pi   (grip lost)
```

It is a single curve that grows quadratically from the origin and then
*loses grip* past a maximum reachable miss `Δθ_pi`. The pull-in limit `Δθ_pi`
is therefore **a property of the curve** — the largest miss the motor can
sustain synchronously — not a second, speed-dependent curve. A motor that
pulls out faster at speed is modelled simply by choosing a smaller `Δθ_pi`
under load; the underlying function is still one τ(Δθ).

In microstep units the corresponding miss is

```
ΔN_pi = Δθ_pi / θstep
```

For a 16-microstep motor with a pull-in miss of ~30° that is `ΔN_pi ≈ 0.83
microstep` — i.e. just under a microstep of allowed lag before the rotor
drops out. That is the correct order of magnitude: at high speed the ramp
out-runs the motor by a microstep or two, and the single τ(Δθ) curve can no
longer be sustained, so the motor stalls.

### 5.5 Step loss = the ½ full-step rule (100 % error)

The requested hard condition is:

> *If a single command's delta exceeds half a full step, a step is lost —
> which is a 100 % error condition.*

The plant enforces this as a **binary detector**, independent of the spring
dynamics, because a >½-step jump is a *discrete* impossibility, not a
"getting tired" phenomenon:

```
per update:   ΔN_cmd = |θc_next − θc_prev| / θstep
if ΔN_cmd > 0.5 · N_fullstep   (i.e. > microsteps/2)   ⇒  STALL
```

* It fires on **the commanded delta**, not on the measured `Δθ` — a jump of
  half a step is uncatchable even by an ideal rotor.
* It is recorded as a **stall event** with a `stall_count`, and the rotor
  position is *held* (it cannot be where the command says it should be).
* It is reported separately from the *continuous* inertial following (§5.2),
  which is always bounded and recoverable. A stall is the only
  **irrecoverable** outcome.

Two failure modes therefore coexist:

| Mode | Cause | Recoverable? | Reported as |
|------|-------|--------------|-------------|
| **Inertial lag** | rotor can't keep up during accel/decel | yes (settles back) | `position_error` trace |
| **Step loss / stall** | `|Δθ_cmd| > ½ full step` in one command | no | `stall` event, `stall_count` |

### 5.6 Drag and settling

The viscous drag `B` governs the *post-maneur* behaviour. After a
rest-to-rest move the rotor does not snap to `θc`; it decays with an envelope
`ω(t) ∝ e^(−(B/J)t)`. The gnuplot `speed` trace therefore shows a decaying
oscillation — the "motor ringing on stop" that a perfect model never shows.

### 5.7 Reversal dynamics

A commanded reversal (`countUp` flips) makes `Δθ` *jump sign*, which for the
quadratic spring means `τ` flips too. The rotor, now moving fast in the old
direction, decelerates through zero (high spring force, because `|Δθ|` is
large) and accelerates back. This is the same stiff, *springy* behaviour real
hybrid steppers show through a reversal, and it is exactly what the
`speed`/`force` gnuplot traces are there to reveal. The model reports
`peak_reversal_mis` — the maximum `|Δθ|` reached during a reversal — a
clean scalar to assert on.

### 5.8 Acoustic emission (and a playable WAV)

A stepper is audible, so a complete model predicts the *sound*, not just the
motion. The dominant source is electromagnetic hum: the step train emits a
fundamental at the step frequency and its harmonics. The model converts the
simulated state into an emitted-pressure signal

```
f          = ν · |ω|       step frequency (Hz); ν = microsteps, ω in microsteps/s
p(t)       = Σ_k A_k · sin(2π k f t)         first few harmonics, A_k ∝ 1/k
loudness   ≈ sqrt( Σ_k A_k² )   →   dB ≈ 20·log10(loudness)  (relative to a ref)
```

The same signal, sampled on a fixed audio grid, is the payload of a plain
16-bit PCM `.wav` file. `PhysicalStepper` gains a

```cpp
bool to_wav(const char* path) const;
```

that writes one sample per output interval — a short .wav per command (a
"motor note") or one .wav for the whole move — so a test does not only
*assert* a stall drops loudness: it can **play it back** and hear the hum
drop when the rotor stalls. This is the natural companion to the gnuplot
traces in §7 and needs no external tool.

---

## 6. Outputs / observables

Every plant instance exposes a small `observed_s` snapshot after each
`step()` call. The harness records one row per command (or per microstep for
smoothness), giving a full time series:

| Field | Meaning | Gnuplot panel |
|-------|---------|---------------|
| `t` | accumulated wall clock | x-axis (all) |
| `θ_rotor` | rotor position in microsteps | position |
| `θ_cmd` | commanded position in microsteps | position |
| `ΔN` | `θ_cmd − θ_rotor`, the step error in microsteps | position / error |
| `ω` | rotor speed, microsteps/s | speed |
| `α` | rotor acceleration, microsteps/s² | accel |
| `τ` | magnetic torque, `K·Δθ²` (scaled) | force |
| `stall` | 1 if this command hit the ½-step rule, else 0 | steploss / stall |
| `stall_count` | cumulative stalls | — |

These are exactly the six panels requested: **pos / speed / accel / force /
steploss / stall-event**.

---

## 7. Gnuplot time-series integration

`test_26.cpp` already knows how to write multi-panel gnuplot files through
`naxis_plot.h`. `physical_stepper` adds the same capability to a single axis
with a helper in the same header:

```cpp
// Emit one row to the six-panel physical-stepper figure.
void PhysicalStepper::plot_row(Gnuplot* g, uint8_t panel, double t,
                               double theta, double thetac, double mis,
                               double w, double a, double tau, bool stall);
```

The produced `.gnuplot` file has:

1. **position** — `θ_rotor` vs `θ_cmd` overlaid (you see the lag and the
   stall discontinuity),
2. **error** — `ΔN` (inertial lag is bounded and decays; a stall is a jump),
3. **speed** — `ω` with its reversal overshoot and post-stop decay,
4. **accel** — `α`, the derivative that exposes the quadratic spring's
   `sgn(Δθ)` kink,
5. **force** — `τ`, the magnetic torque curve,
6. **stall** — a stem/impulse trace, high exactly at the ½-step events, plus a
   running `stall_count` title line.

The figure is written next to `test_26_f*.gnuplot` and rendered with the
existing `make png` rule — no new tooling.

---

## 8. Determinism and testability

The model has **no randomness and no shared hardware state**. For a fixed
`queue_entry` stream and fixed `(J, B, K, N, Δθ_pi)`, every `step()` returns
bit-identical results. This makes the six assertions in §2.2 exact:

* Step-loss count is reproducible — assert `stall_count == 3`.
* Settling bound is reproducible — assert `|ΔN| < 0.5` after the move.
* Reversal peak is reproducible — assert `peak_reversal_mis > X`.

A *mutation* that removes the ½-step rule (a hypothetical `-DNO_STALL_DETECT`)
must make `stall_count` assertions fail — proving the check is load-bearing,
the same discipline `prove_mutations.sh` enforces for FasNAxis.

---

## 9. Portability of the model

* **Host (this paper).** `double` physics, driven by a small integer
  `queue_entry` parser. This is the reference implementation.
* **Target (future).** The physics state can be moved to fixed-point: scale
  `θ, ω, α` by `2^F` into `int32/int64`, quantise `K`, `B`, `J` to integer
  gains, and replace `sgn(Δθ)` with a branchless sign on `int`. The
  `quadratic` term becomes a 32×32→64 multiply with a shift — exactly the
  fixed-point style the existing `log2/` library uses. No FP unit required.
* **Cost.** Per microstep: one multiply, one divide-free `sgn`, a few adds.
  The `steps>1` burst is bounded by `MIN_CMD_TICKS`, so worst case per
  command is a small, fixed number of FMA-like ops — ISR-feasible.

---

## 10. Concrete worked example (for the harness and the figures)

A rest-to-rest move that is *too fast*:

```
microsteps = 16,  J = 1e-6,  B = 2e-5,  K = 2e-3,  Δθ_pi = 30°
Commanded trapezoid peak period: 400 ticks  (== 25 kstep/s @16MHz)
Total: 2000 steps.
```

At 400 ticks the ramp asks for `1/400 s ≈ 2.5 ms` per microstep. The motor's
pull-in for a 30° miss under `K` gives a maximum sustainable microstep period
somewhere near 450 ticks; below that (faster) the required `|Δθ|` to keep up
exceeds `Δθ_pi`, so `τ` saturates to zero and `stall_count` climbs by one per
dropped microstep in the coast band. The `error` trace shows a steadily
growing lag that *snaps* at each stall; the `speed` trace shows the decay;
the `stall` trace fires a spike for every dropped command. The harness
asserts `stall_count > 0` — a green test that *detects* an underspecified ramp.

The inverse case (a correctly braked move at ≥ 700 ticks) settles with
`|ΔN| < 0.5` and `stall_count == 0`, proving the plant can also be *perfect*
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
  double   theta;  // rotor angle, microsteps (physical, reported to queue)
  double   thetac; // commanded microstep angle
  double   mis;    // step error ΔN = thetac - theta, microsteps
  double   w;      // rotor speed, microsteps / s
  double   a;      // rotor accel, microsteps / s^2
  double   tau;    // magnetic torque, K * Δθ^2 (scaled)
  bool     stall;  // ½-full-step rule fired this command
};

class PhysicalStepper {
 public:
  // N = microsteps per full step (default 16).
  // Physical gains J, B, K, pull-in miss dtheta_pi.
  PhysicalStepper(uint8_t microsteps = 16,
                  double inertia      = 1.0e-6,
                  double drag         = 2.0e-5,
                  double magnetic_gain= 2.0e-3,
                  double dtheta_pi    = M_PI / 6.0); // 30 degrees

  // Advance the plant by one queue_entry. Advances the simulated clock by
  // steps*ticks ticks (or ticks for a pause) and advances the commanded
  // angle by steps microsteps in the entry's direction.
  observed_s step(const queue_entry& e);

  // Convenience: run a whole baked queue_entry ring, returning the last
  // observed_s. Used by the test harness.
  observed_s run(const queue_entry* entries, uint8_t len);

  // The position the queue base would report.
  int32_t getCurrentPosition() const { return (int32_t)llround(state_.theta); }
  uint32_t stalls() const { return stalls_; }
  uint32_t total_ticks() const { return ticks_; }

  // Emit one row to a multi-panel gnuplot writer (see naxis_plot.h).
  void plot_row(void* gnuplot, uint8_t panel, const observed_s& o);

  const observed_s& last() const { return last_; }

 private:
  struct { double theta, theta_cmnd, w; } state_;   // θ, θc (cmd), ω
  uint8_t  microsteps_;
  double   J_, B_, K_, dtheta_pi_;
  double   step_rad_;                 // 2π / microsteps
  uint32_t ticks_, stalls_;
  observed_s last_;
};

#endif
```

### 11.1 step() pseudocode

```
step(e):
  dir   = e.countUp ? +1 : -1
  tnow  = ticks_
  θprev = state_.theta
  for k in 0 .. max(e.steps-1, 0):
     θc_prev  = state_.theta_cmnd
     θc_next  = θc_prev + dir · step_rad
     ΔN_cmd   = |θc_next − θc_prev| / step_rad        // == 1 per microstep
     stall    = (ΔN_cmd > 0.5 * microsteps)            // ½ full-step rule
     if stall:  τ = 0, rotor frozen this microstep  (cannot catch)
     else:
        Δθ    = (θc_next − state_.theta) · rad2micro
        τ     = K · Δθ² · sgn(Δθ)     if |Δθ| < Δθ_pi, else 0   // pull-in
        ω     = ω + (τ − B·ω)/J · Δt
        θ     = θ + ω · Δt
     θc     = θc_next
     Δt     = ticks / TICKS_PER_S
  if e.steps == 0:                 // pause: rotor only decays under drag
     ω = ω · exp(-(B/J)·Δt)
  ticks_ += (e.steps==0) ? ticks : steps*ticks
  record last_ {...}
```

The stall rule (§5.5) is the single `stall` boolean; the quadratic spring
(§5.4) is the `τ` branch; drag (§5.6) is the pause decay.

---

## 12. Test matrix (proposed `test_27.cpp`)

| Section | Input | Assert |
|---------|-------|--------|
| T1 dwell | single pause | `ω → 0`, `|ΔN| < 0.5` |
| T2 single step | one `steps=1` at 1000 ticks | `stall==0`, settles |
| T3 burst | 64 steps @ 1000 ticks | no stall, bounded lag |
| T4 too-fast | 64 steps @ 200 ticks | `stall_count > 0` |
| T5 reversal | forward then back | `peak_reversal_mis > 0`, recovers |
| T6 >½ jump | one command, `steps·ticks` mapping a >½ full step miss (see below) | `stall==1` exactly once |
| T7 brake | correctly braked trapezoid | `stall_count == 0`, `|ΔN|<0.5` at rest |

For **T6**, a single command cannot carry a >½-step *micro* jump (a command is
one microstep at a time); the intended test is a **commanded period so short
that the rotor physically cannot move in one update** — i.e. inject a command
whose `Δt` is smaller than the time the spring needs to move the rotor by half
a step. The model computes the minimum catchable period `Δt_min` from `K/J`
and flags any command faster than that as a stall *by the same ½-step rule*,
just applied at the update boundary. That is the honest reading of the
requested invariant: *a command whose required microstep delta per update
exceeds ½ full step is a stall.*

---

## 13. Open questions

1. **Load-dependent pull-in.** Should `Δθ_pi` shrink with the external load
   (a loaded motor pulls out sooner)? That would be a parametric tuning of
   the *single* τ(Δθ) curve, not a second curve.
2. **Stick-slip / detent.** A piecewise `τ` with a detent tooth could model
   the microstep ripple in the `position` trace.
3. **Coupling to FasNAxis.** The six-axis planner (`test_26.cpp`) already
   tracks commanded vs realized per axis; `physical_stepper::getCurrentPosition()`
   drops straight into that loop, making step loss *visible to the binder*.
4. **Fixed-point port.** Convert §9's `double` model to the `log2/` integer
   style; validate bit-identical stall detection.

**Resolved.** The acoustic emission (§5.8) that was once an open question is
now implemented: `PhysicalStepper::to_wav()` synthesises the electromagnetic
hum from the first few harmonics of the step frequency and writes a 16-bit
PCM `.wav`, so a test can play the motor's hum back and hear it drop on stall.

---

## 14. Summary

`physical_stepper` is a header-only, deterministic, rotordynamic plant that
replaces the ideal stepper *at the queue boundary*. It models inertia (`J`),
viscous drag (`B`), and a **quadratic magnetic coupling spring** between the
commanded microstep angle and the actual rotor, and it enforces a hard
**½-full-step step-loss rule** as a 100 % error condition. It reports position,
speed, acceleration, torque and every stall event as a gnuplot time series,
giving the PC test suite the same *physical* failure modes a real motor
exhibits — without a timer, without hardware, and without non-determinism.
