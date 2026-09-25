# Pluggable motion sources — engine generalization

Status: implemented design record. The decision (Path A, single driver)
and the stop/emergency-stop design are in the code and covered by the PC
tests (F22). The two-class prototype was reverted.

Replaces the former items "Running `pump()` from `manageSteppers()`"
(item 7) and "Generalize ramp generator and naxes" (item 9).

## Goal

Pair a motor with a **motion source** instead of hard-wiring the ramp
generator into `FastAccelStepper`:

| Source | Cardinality | Notes |
|---|---|---|
| Ramp generator | per stepper | default, current behaviour |
| Nothing | per stepper | `moveTimed` / raw `addQueueEntry` users |
| naxes planner | **one per engine** | coordinated multi-axis |

Constraints:

- Unused `RampGenerator` must cost **zero flash and zero RAM** — 328p is
  RAM-constrained.
- As backward compatible as possible across all supported
  architectures.
- Mixed mode on one engine must work, e.g. 4 steppers on the naxes
  planner + 1 stepper on the ramp generator.

## Decision

- `FastAccelStepper` **keeps its name and stays the ramp class**,
  backward compatible; existing sketches, `FastAccelStepper*`, `moveTo`
  and `runForward` are unchanged.
- New `FastAccelStepperNaxes`: no ramp, no `moveTo`; driven by the
  engine-level naxes planner via `addQueueEntry`.
- Shared base `FastAccelStepperBase` (queue/pin driver). `pd_*` refer to
  `FastAccelStepperEngineBase*` and do not change per configuration.
- The engine is config-typed:

  ```cpp
  FastAccelStepperEngineT<RampGenerator>               // == FastAccelStepperEngine (default, back-compat)
  FastAccelStepperEngineT<NoSource>                    // zero ramp flash + RAM
  FastAccelStepperEngineT<RampGenerator, NaxesPlanner> // mixed
  ```

  Compile-time selection is deliberate: link-time GC
  (`dot_a_linkage` / `--gc-sections`) is not guaranteed across every
  target, so it must not be relied on.
- **Dispatch is `engine->fill`**, not `stepper->fill`: the engine keeps
  one typed array per source (`FastAccelStepper*`, `FastAccelStepperNaxes*`)
  and calls `fill()` non-virtually in `manageSteppers()`. No per-stepper
  vtable. `FastAccelStepperEngineBase` exposes a single virtual
  `manageSteppers()` — one vtable for the whole program.
- Factories follow the config: `stepperConnectToPin()` → `FastAccelStepper*`
  for a ramp config; the naxes planner's `addAxis` → `FastAccelStepperNaxes*`.
  A no-ramp config never references `FastAccelStepper`, so its ramp code
  is not emitted.
- Item 7 folds in: `manageSteppers()` ticks the configured sources.
  `FasNAxis::pump()` is **not** called inside FAS; it becomes the naxes
  source's tick. The whitepaper §4.6 idle precondition still applies on
  the ramp side (ramp and naxes never share a queue).

## Stop and emergency stop

Per-axis and group stop must be first-class, not inferred from timing.

Current state: `forceStop()` / `stopMove()` exist only on
`FastAccelStepper` (the ramp class); `FastAccelStepperNaxes` has **no**
stop API. `forceStop()` only sets `q->ignore_commands = true` and calls
`_rg.forceStop()`; `fill_queue()` clears `ignore_commands`, so it is a
transient latch, not a queryable event. A planner therefore cannot
reliably detect a per-axis stop from existing state.

Design:

- Move `forceStop()` / `stopMove()` down to `FastAccelStepperBase` (all
  steppers need a stop path).
  - `Base::forceStop()` — immediate: `ignore_commands = true`,
    `q->forceStop()`, then notify the observer.
  - `FastAccelStepper::stopMove()` — `_rg.initiateStop()` (controlled
    decel). `FastAccelStepperNaxes::stopMove()` — notify the planner,
    which plans the coordinated group stop.
- Per-stepper back-pointer to a motion observer (`MotionObserver*
  _observer`, set at `addAxis`). One pointer of RAM per stepper; the
  observer's vtable exists once (the planner), so no per-stepper vtable.
- Per-axis E-stop → `_observer->onAxisStopped(this, kind)`. The planner
  then runs the group E-stop: `forceStop()` on every member with a
  re-entrancy guard, mark fault, positions **untrusted** (re-home before
  re-assembling).
- Group E-stop: `FasNAxis::emergencyStop()` / an engine-level
  `emergencyStopAll()` calling the same base primitive.
- ISR-safety: a limit-switch handler may call `forceStop()` from an ISR.
  The immediate per-axis abort is queue-level and ISR-safe; the planner
  reaction should be either tiny (urgent global abort only) or deferred
  via a `volatile` flag consumed in `pump()` (latency < one pump).

## `manageSteppers()` context per backend

Where the tick actually runs determines what a registration hook may do:

| Backend | Call site | Context |
|---|---|---|
| AVR | `pd_avr/avr_queue.cpp:210` inside `StepperISR` with `sei()` | ISR, nested IRQs enabled |
| ESP32 | `pd_esp32/esp32_queue.cpp:451` `StepperTask` | RTOS task |
| Pico | `pd_pico/pico_queue.cpp:272` `StepperTask` | RTOS task |
| SAM | `pd_sam/sam_queue.cpp:38` `TC5_Handler` | ISR |
| SAMD | `pd_samd/samd_queue.cpp:52` `FAS_TC_HANDLER` | ISR |
| Teensy | `pd_teensy/teensy_queue.cpp:50` `fas_ramp_tick_isr` | ISR |

4 of 6 are ISR (AVR nested, SAM, SAMD, Teensy). Any `manageSteppers()`
hook may therefore only do **light, ISR-safe** work; heavy planner
planning stays caller-pumped (`pump()` from `loop()`), matching
whitepaper §10.4.

## Decision: single driver (Path A)

Chosen over the two-class prototype. The two-class direction
(`FastAccelStepperBase`, `FastAccelStepperNaxes`,
`FastAccelStepperEngineBase`, `FastAccelStepperEngineT`, `test_28`) was
reverted: an application's physical axis must be **ramp-homed first, then
join a group**, and a no-ramp `FastAccelStepperNaxes` cannot do the
homing. Keeping `FastAccelStepper` as the one driver lets the same object
home with its ramp and later be driven by the planner through
`addQueueEntry()` while the ramp is idle (§4.6). The ramp is always
linked — acceptable, since homing uses it.

Consequences:

- No `FastAccelStepperNaxes`; `FasNAxis` keeps `Stepper =
  FastAccelStepper` as its production default.
- No config-typed engine and no engine `virtual`; the C++-runtime link
  cost of the prototype is avoided.
- `FasNAxis` stays caller-pumped. A `manageSteppers()` registration hook
  is **not** required for now; if added later it may only do light,
  ISR-safe work (see the backend table above).

Implemented:

- Driver: `StepperStopCause` lives in `fas_arch/result_codes.h` (shared
  with the duck-typed planner) and is set by
  `stopMove()`/`forceStop()`/`forceStopAndNewPosition()`, read-and-cleared
  by `FastAccelStepper::takeStopCause()`.
- Planner: `FasNAxis::pump()` polls each member's `takeStopCause()`; a
  non-None cause aborts the plan and returns `PumpStatus::Stopped`
  (`isFaulted()`), positions untrusted until re-synced. `addAxis()`
  discards a pre-registration cause. `emergencyStop()` force-stops every
  member (no re-entrancy); `clearFault()` resets without re-syncing.
- Test helper `SimPort` models the hook (`takeStopCause()`, plus
  `setStopCause()`/`forceStop()` injection); `test_26` case F22 exercises
  the injected cause, a member `forceStop()`, and `emergencyStop()`.

## Not done (deliberate)

- A `manageSteppers()` registration hook for auto-feed. `FasNAxis` stays
  caller-pumped; if a hook is added later it may only do light, ISR-safe
  work (see the backend table above).

## References

- `src/FastAccelStepper.h` — `StepperStopCause`, `takeStopCause()`.
- `src/FastAccelStepperEngine.cpp:172` — `manageSteppers()`.
- `src/FastAccelStepper.cpp:48` — `fill_queue()` and the member `_rg`.
- `extras/doc/n_axes_whitepaper.md` §3.2, §4.6, §10.4.
