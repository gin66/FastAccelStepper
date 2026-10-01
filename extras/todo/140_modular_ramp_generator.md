# 140 Modular ramp generator

## Goal

Replace the current hand-written, monolithic ramp generator with a
**modular, separately testable, and well-documented** implementation.

## Current state

The ramp generator is tightly coupled to the queue driver and platform
abstraction.  Its algorithms (acceleration profiling, speed calculation,
Bresenham line-drawing) are embedded in large functions with limited
unit-test coverage.

## Target architecture

```
RampGenerator (core)
├── AccelerationProfile  — compute speed vs. position curve
├── SpeedCalculator      — fixed-point log2 arithmetic (reuse log2/)
├── StepScheduler        — Bresenham step distribution
└── TickCalculator       — convert position → timer ticks
```

Each module has:
- A **pure C++ header + source** pair (no platform dependencies).
- **Unit tests** in the PC-based harness (`pc_based/`).
- **Documentation** comments explaining the algorithm and invariants.

## Migration plan

1. **Extract** `AccelerationProfile` from the existing code.  Write PC
   tests that verify acceleration/deceleration curves against a golden
   reference (floating-point reference implementation).
2. **Extract** `StepScheduler` (Bresenham variant).  Test with known
   step ratios and edge cases (overflow, wrap-around).
3. **Extract** `TickCalculator` (position → timer ticks).  Verify
   tick values match platform-specific expectations.
4. **Integrate** the modular components back into the existing queue
   driver, keeping the public API unchanged.
5. **Regression test** — all existing PC and SimAVAR tests must pass.

## Documentation

- Algorithm descriptions in `extras/doc/ramp_generator_design.md`.
- Invariant preconditions in every function comment.
- Example usage in a new `examples/ramp_demo/` directory.

## Status

_idea — not started_
