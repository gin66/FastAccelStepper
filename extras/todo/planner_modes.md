# AFAP vs timed — two implementations

Status: decision.

## Decision

`FasNAxis` implements only the **as-fast-as-possible** planner
(whitepaper §3.3 problem 1). The **faithful timed trajectory** (problem
2) is a **separate implementation**, not a `FasNAxis` mode. An
application selects one mode or the other; it does not mix them on the
same axis set.

## Rationale

- v1/AFAP has no requested time, so no timing can be infeasible and no
  feasibility error can occur. All error paths (`TimingNotAchievable`,
  "too fast to do smoothly") presuppose a caller-supplied
  time/speed constraint and therefore exist only in the timed world.
- Forcing one planner to serve both inputs would drag timed-only
  machinery and semantics into `FasNAxis` for no benefit.
- An app uses either mode, never both, so a shared runtime is not
  required.

## Consequences

- `FasNAxis` public API stays AFAP-only; the timed trajectory runs
  under its own class/module.
- Shared between the two: geometry conventions, the `addQueueEntry`
  contract, `SimPort`, the PC test rig, and the whitepaper theory.
  Optionally the same axis/pin setup.
- The timed implementation owns per-block feedrate `F`
  (`F`-as-command) and the feasibility error (`TimingNotAchievable`).
- The error boundary is explicit: **no time constraint ⇒ no feasibility
  error**.
