# Cubic start (`s_h`) overlay

Priority: **P6** — later feature, not v1.

Status: later feature, not v1.

## Scope

Relevant only for the **as-fast-as-possible** planner problem (§3.3
problem 1). With timed/constraint input (problem 2) the requested
trajectory dictates jerk, so a cubic start adds nothing there.

## Hard part

The FasNAxis planner computes time-optimal ramps itself
(`src/fas_naxis/ramp_law.h`, `ramp_map.h`) under a constant-acceleration
model. To remain time-optimal with a cubic start, the planner must model
the cubic ramp. Merely forwarding
`FastAccelStepper::setLinearAcceleration()` to each stepper is **not**
sufficient and would change the profile without the planner accounting
for it.

## References

- `src/FastAccelStepper.h:305-334` — `s_h` handover, `v_h`,
  jump-start speed.
- `extras/doc/ramp_cubic_quadratic.md` — cubic/quadratic ramp concept.
