# Pluggable motion sources — engine generalization

Status: design decision, not v1.

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

## References

- `src/FastAccelStepperEngine.cpp:172` — `manageSteppers()`.
- `src/FastAccelStepper.cpp:85` — `fill_queue()` and the member `_rg`.
- `extras/doc/n_axes_whitepaper.md` §3.2, §4.6, §10.4.
