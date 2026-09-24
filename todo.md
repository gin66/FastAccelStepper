# FasNAxis — open items

Source of truth: `extras/doc/n_axes_whitepaper.md`.

The test-driven implementation plan (Steps 0–14) is complete; its
tests live in `extras/tests/pc_based/test_26.cpp`. The items below
remain out of scope for v1.

## Out of scope

- Raising `pd_test` `MAX_STEPPER`
- simavr / hardware / PlatformIO jobs for FasNAxis
- Cubic start (`s_h`) overlay
- Per-block feedrate `F`
- Faithful timed trajectory (whitepaper §3.3 problem 2 /
  §3.3.1, GitHub #363): polyline plus **speed at position**,
  execute or `TimingNotAchievable`. Not Δpos per 1 ms frame
  (`moveTimed(Δ, 1 ms)` hunts 1 kHz/2 kHz). v1 is as-fast-as-
  possible only; `naxis_ref` is the duration bound; smoothness
  is a second check (near-exact period, step-separated
  commands)
- Inverse kinematics
- Running `pump()` from `manageSteppers()`
- A Linear oracle that is faster by leaving the chord, cutting
  a corner, or skipping a vertex (not constraint-faithful;
  whitepaper §12.4.1). Overshoot is the mode that may leave
  the chord.
- Generalize ramp generator and naxes using e.g. template class and associate with stepper. If a user needs only naxes, no need to compile ramp calculator. This should then automatically record pump() or the ramp generator equivalent in manageSteppers().
