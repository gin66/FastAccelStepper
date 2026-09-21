# FasNAxis — test-driven implementation plan

Source of truth: `extras/doc/n_axes_whitepaper.md`.
Tests live in `extras/tests/pc_based/` and are picked up by the
existing `test_??.cpp` wildcard (`make -C extras/tests/pc_based test`).

Rules for every step:

1. Write the failing test first. Run it. Then implement until green.
2. Production code is header-only (`src/FasNAxis.h` and optional
   `src/fas_naxis/*.h`). Not included from `FastAccelStepper.h`.
   No new `src/*.cpp`.
3. No `float` / `double` / integer `/` in the production header.
   Period and accel use `log2_value_t` and `RampCalculator`.
   No Isabelle. Theory is probed in steps 2b–2g, 2ref, and 3b.
4. Each motion fixture writes `test_26_<id>.gnuplot` the way
   `test_02` writes `test_02_f5.gnuplot` (via a small helper modeled
   on `RampChecker::start_plot` / `finish_plot`). `make clean`
   already deletes `*.gnuplot`.
5. HTML dumps are extra, behind `-DFAS_NAXIS_TRACE`, not required
   to pass `make test`.
6. Format with `bash extras/scripts/format_code.sh` before commit.
   Revert unrelated clang-format noise.
7. Do not link `LIB_O` unless the step says so. Planner-only tests
   can be self-contained; 1–2 axis FAS golden runs use the default
   `test_%: test_%.o $(LIB_O)` rule.

Run one binary:

```
make -C extras/tests/pc_based test_26 && extras/tests/pc_based/test_26
```

Render a plot (optional, local):

```
gnuplot extras/tests/pc_based/test_26_f1.gnuplot
```

---

## Step 0 — harness (no planner yet) ✅

**Test:** `test_26.cpp` compiles, `main()` prints `FasNAxis TDD` and
exits 0. Makefile wildcard picks it up; `make -C extras/tests/pc_based test`
still passes the existing suite.

**Add:**

- `extras/tests/pc_based/test_26.cpp`
- `extras/tests/pc_based/naxis_plot.h` — copy the plot file protocol
  from `RampChecker` (open `$data <<EOF`, write rows, close with a
  `multiplot` of XY / speed-time / P-vs-R / period). No kinematics.

**Done when:** `make -C extras/tests/pc_based test_26` succeeds and
is part of `make test`.

**Fifth panel (done):** `naxis_plot.h` emits a fifth panel — step
deviation distance (commanded − realized) over time on `set yrange
[-10:10]`. `row()` now takes a `deviation` argument (column 4); per-axis
overlay columns are offset by 4*i so multi-axis fixtures plot their own
columns.

---

## Step 1 — log2 ramp map identity (F1 kernel) ✅

**Test first:** for `a = 2000`, `ticks_cfg` of 4000 step/s, and
`P = 1, 2, 4, …, 4000`:

- `calculate_ticks(P)` equals `ramp_config_s::calculate_ticks(P)`
  (same object the library uses).
- `calculate_ramp_steps(ticks)` round-trips within the documented
  log2 error band (compare to `rmc_test` / `test_03.h` style).
- Period is monotone non-increasing in `P`.
- `P = 0` is “stopped”; never call `calculate_ticks(0)` (FAS
  starts ramps at `P >= 1`).

**Implement:** a thin wrapper in `src/fas_naxis/ramp_map.h` that
constructs `ramp_config_s` from `ticks_cfg` + `log2_from(accel)`
and forwards. FasNAxis must not re-derive `v = sqrt(2 a s)`.

**Plot:** `test_26_f1_map.gnuplot` — period vs `P`, overlay of
wrapper vs `RampCalculator`.

**Done when:** F1 kernel assertions pass; plot exists.

---

## Step 2 — remaining-steps scan `R` (lookahead) ✅

**Test first:** feed polylines as arrays of `Δ[NAXES]`, no queues.

| Case | Polyline | Assert |
|------|----------|--------|
| One axis, one block | `(10000,)` | `R = 10000` |
| Two blocks, same sign | `(4000,) + (4000,)` | `R = 8000` |
| Reversal | `(4000,) + (−100,)` | `R = 4000` at start; after first block `R = 100` the other way |
| Idle then move | `(0,) + (100,)` | first block does not start a direction; `R = 100` |
| Linear path-stop | 2-D square corner | binder `R` ends at the 90° vertex even if that axis would continue |
| F10 micro-segments | 100 × 100-step collinear | `R` at head is the full 10000, not 100 |
| F6b | `(4000,1)` then `(0,3999)` | `R_x = 4000` (then idle); `R_y = 4000` (continues) |

Integer add/sign only. The 2° collinear test of §8.5 lives here:

```
(Δ · Δ')² * 100000  >=  99878 * |Δ|² * |Δ'|²
```

Also compute `P_stop_i = calculate_ramp_steps(ticks_i_cfg)`.
A fixture that only feeds 800 steps with `P_stop = 4000` (path
open, last point = rest) must **reduce** allowed `P` (`P ≤ R`,
live remaining-to-stop so peak `P < R`) and must **not** raise
an error. `P` starts at 0 and is capped by `P_stop` **and** by
remaining-to-standstill; it is not estimated as
`min(P_stop, R/2)`. Coasting happens when `N/2 > P_stop`.
`R` is the parse to end or direction change, not a lower bound
the planner waits to fill.

Path-angle change: speed is reduced **before** the next
trajectory point so per-axis accel stays in limits (§8.4).
v1 Linear path-stops (`P → 0` at the vertex); Overshoot
prepares per axis via `R`. F5 is the Linear fixture. A blended
`ΔP` junction is not v1.

**Implement:** `src/fas_naxis/remaining.h` (or methods on the class).
No queue I/O yet.

**Plot:** `test_26_f10.gnuplot` — live ramp over the collinear
10000: `P` starts at 0, coasts because `N/2 > P_stop`, `R(t)` is
remaining-to-stop (does not drop to 100 at every micro-segment),
commanded and realized share the chord (deviation ~ 0).

**Done when:** table above is green; F19 kernel (`HORIZON` of
micro-segments, `R < P_stop`) caps speed and still plans; same
`HORIZON` with one long block can reach `P_stop`.

---

## Step 2b — theory probes for `R`, binder, collinear (no queues) ✅

These must fail on a wrong *model*, even if later F-fixtures are
green. Oracle is a small pure function, not FasNAxis state.

1. **Reference oracle.** Given waypoints + `ticks_cfg` / accel,
   compute `R` (parse to end or direction change; last buffered
   point is rest), `P_stop`, Linear binder (longest `|Δ|`, rebind
   if `|Δ_i|*ticks_i > |Δ_b|*ticks_b`), DDA step counts to the
   vertex. Path direction maps the binder’s `P ≤ R` onto slaves.
   The planner must match this function on the same inputs.
2. *(moved to 2c–2g)* Exhaustive tiny Linear was too large as one
   step. DDA walk, one-block Linear, issued-period `P ≤ R`,
   two-block path-stop, then the exhaustive set.
3. **Rebind neighbourhood.** `|Δ_x| ∈ {99,100,101}`,
   `ticks_y/ticks_x` in `{1, 2, 99/100}` (integer ticks, not
   a `/` in production). Especially `|Δ_x|*ticks_x ≈
   |Δ_y|*ticks_y`.
4. **Collinear boundary.** 1° must pass, 2° is the documented
   edge, 3° and 90° must stop (angle change → Linear path-stop,
   preparation = `P → 0` at that vertex). Include n=3 with one
   tiny component.
5. **Lookahead speed cap.** Open path of 800 steps, `P_stop = 4000`:
   last point is rest, allowed `P ≤ R`, live remaining-to-stop so
   peak `P < R`, no error. Append collinear blocks until
   `N/2 > P_stop`: coasting to `P_stop` becomes legal. Path
   direction: `(800, 400)` Linear slaves Y onto X’s `R = 800`
   triangle.

**Done when:** items 1, 3, 4, 5 green; disabling the rebind
rule makes the neighbourhood fail; disabling remaining-to-stop
makes (5) fail to brake (`peak P` is not `< R`).
`make -C extras/tests/pc_based mutations` runs
`prove_mutations.sh` next to the tests (it is a test, not a
library script under `extras/scripts`).

---

## Step 2c — DDA walk, one block (geometry only) ✅

No ramp, no period, no queues. `Remaining::dda_steps` already
counts; this step *walks* the error accumulator one binder
step at a time.

**Test first:** 2-axis hand cases, integer positions only.

| `Δ` | Binder | Assert |
|-----|--------|--------|
| `(5,0)` | X | Y never steps; end `(5,0)` |
| `(0,4)` | Y | X never steps; end `(0,4)` |
| `(5,5)` | X (tie → smaller index) | Y steps every binder step |
| `(5,3)` | X | classic Bresenham; end `(5,3)` |
| `(5,−3)` | X | Y steps are `−1`; end `(5,−3)` |
| `(−4,2)` | X | signs follow `Δ` |

Per binder step each slave takes 0 or 1 step (never 2). Walked
count equals `Remaining::dda_steps`. Chord invariant (integer,
no `/`): `|2·err| ≤ |Δ_bind|` after every binder step. Test-only
double may check distance to the chord `≤ 0.5√n` (§12.4).

**Implement:** walker in `src/fas_naxis/dda.h` (or methods on
`Remaining`). Error accumulator + `2*err >= |Δ_bind|` compare
already used by `dda_steps`. No `float` / `/`.

**Plot:** `test_26_f2c.gnuplot` — XY of `(5,3)` on the chord.

**Done when:** table green; walked counts match `dda_steps`.

**Done:** `src/fas_naxis/dda.h` holds `DdaWalk{bind, slave}` that walks
the Bresenham error accumulator one binder step at a time (`err +=
|slave|`; step + `err -= |bind|` when `2*err >= |bind|`). `f2c_dda_walk()`
runs the six 2-axis hand cases (table above) through `DdaWalk`: each binder
step issues exactly one binder step and 0 or 1 slave step (never 2), the
walked per-axis count equals `Remaining::dda_steps(bind, slave)` (compared
as magnitude, so `(5,-3)` gives `-3`), end position equals the waypoint,
the chord invariant `|2*err| <= |bind|` holds after every step, and a
test-only double confirms distance to the chord `<= 0.5*sqrt(n)`
(section 12.4). Plot: `test_26_f2c.gnuplot`.

---

## Step 2d — Linear one-block rest-to-rest (binder ramp + DDA)

Still no queues. One committed block: binder runs `RampLaw` on
`|Δ_bind|`; each binder step is one DDA tick from 2c.

**Test first:** equal `ticks_cfg`, rest-to-rest, §14.1 accel.

- `(20, 8)`: X binds; issued `|steps|` per axis equals `|Δ|`;
  end is the vertex; path on the chord.
- `(20, 0)`: Y never steps.
- `(8, 20)`: Y binds.
- Binder `P ≤ R` from `RampLaw` fields is allowed here (sanity).
  Reconstruction from issued periods is 2e, not this step.

**Implement:** `src/fas_naxis/linear.h` — one block, no ring, no
`addQueueEntry`. Emit a per-binder-step trace `{ticks, step[NAXES]
in {−1,0,1}}`. Reuse `Remaining::longest_axis` (DDA master) and
the 2c walker. Time-law rebind is `ticks_floor`, not a DDA rebind.

**Plot:** `test_26_f2d.gnuplot` — XY of `(20,8)` plus binder
speed vs time.

**Done when:** step sums match `Δ`; path on the chord; plot exists.

---

## Step 2e — `P ≤ R` from issued periods (not planner fields)

Same interpolator as 2d. Ignore `RampLaw.P`. A wrong model that
only writes planner fields must fail here.

**Test first:** replay the `(20, 8)` trace from 2d.

- Binder `P_issued = calculate_ramp_steps(ticks)` (`P = 0` if
  that record is a pause). Never read `RampLaw.P`.
- Remaining after `k` binder steps = `|Δ_bind| − k` (last point
  is rest).
- Every sample: `P_issued ≤ remaining`. Peak `P_issued < |Δ_bind|`
  on this short block (live remaining-to-stop).
- Envelope: every moving command `ticks ≥ ticks_i_cfg` (one-step
  slack, §12.4).

Second hand case: `(20, 8)` with Y 4× slower (integer ticks).
Wall-clock rebind would pick Y, but the DDA master stays X
(longest `|Δ|`). The time-law lengthens `ticks_b` to Y’s
`ticks_cfg` (`ticks_floor`) so Y never exceeds `v_max`.
Reconstruction and envelope run on X; issued `|steps|` per axis
equals `|Δ|` (X is scaled down in speed, not in count). A third
hand case `(5, 3)` with ticks `(4000, 8000)` is the 2g bite:
Y wins wall-clock, X stays master and still issues 5 steps.

**Implement:** a trace oracle in `test_26.cpp` (or a helper next
to it). Production interpolator only needs to record issued
`ticks`. No queues.

**Plot:** none required.

**Done when:** all three hand cases green without reading planner `P`;
rebind cases still issue `|steps| == |Δ|` on every axis.

---

## Step 2ref — PC reference track (`naxis_ref.h`) ✅

The globally fastest constraint-faithful Linear track **is**
this reference (whitepaper §12.4.1). Same object as the
proposal: on the chords, path speed 0 at a non-collinear
vertex, `RampCalculator` + DDA. Not a second oracle, not a
replay of `LinearBlock`. A faster trace that leaves the chord
or misses a vertex is not Linear.

PC-only. `double` allowed. Not production. Not included from
`src/FasNAxis.h`. Must not read interpolator `P` / `R` fields.

**Test first:** the three 2e hand cases as reference inputs
(even though 2e already passed on `LinearBlock`):

- issued `|steps_i| == |Δ_i|`; end is the vertex
- DDA master is longest `|Δ|`; `ticks_floor` lengthens the
  period when a slow short slave would exceed `v_max`
- `P_issued = calculate_ramp_steps(ticks)` from
  `RampCalculator` (the FAS map); `P_issued ≤ remaining`;
  peak `P_issued < |Δ_master|` on these shorts
- envelope: when axis `i` steps, `ticks ≥ ticks_i_cfg`
- the existing 2d/2e interpolator must match this trace
  within a few Δt (log2 / one-step slack), not the reverse

Also the three 2f polylines, as *reference* traces (2f’s
interpolator comes next):

| Polyline | Assert |
|----------|--------|
| `(5,0)+(0,5)` | vertex `(5,0)` is a sample; `P → 0` there; Y is DDA master after the vertex |
| `(3,3)+(2,2)` | collinear; end `(5,5)`; no rest at the joint |
| `(5,0)+(−3,0)` | reversal; `P → 0` at `(5,0)`; then X the other way |

**Implement:** `extras/tests/pc_based/naxis_ref.h`. Linear
only (Overshoot `T_opt` is Step 11). Infinite `HORIZON`.
`Remaining` scan + `longest_axis` / `ticks_floor` +
`RampCalculator` + `dda_steps` / `DdaWalk`. Emit the same
per-step `{ticks, step[NAXES] in {−1,0,1}}` as `LinearBlock`.

Do **not** copy `LinearBlock` and call it a reference. The 1-D
law is `RampCalculator` on remaining master-steps; DDA is the
2c walker.

**Plot:** optional overlay vs 2d on `(20,8)`.

**Also F20** (several hundred waypoints, one polyline):

- Seeded LCG (`seed = 26`): 80 random blocks `|Δ_i| ∈ [1,12]`,
  then a connecting block to `(1600,0)`, then a half-circle
  `r = 1600` in 180 chords of 1°, then 80 more random blocks.
- `ticks = (4000, 8000)`, accel = 2000.
- Assert: `n_blocks ≥ 200`; every vertex hit; envelope;
  law `P ≤ R`; reconstructed `P` within a 2-step log2 band;
  issued `|steps| == |Δ|`; at least one path-stop joint and
  one collinear-cruise joint.
- Plot: `test_26_f20.gnuplot`.

On the arc the DDA master switches; `P`/`R` stay in **path
steps** (one DDA tick per command), not per-axis remaining.
A 1° rounded chord may be a 90° jog — that is a path-stop,
not a reason to leave the vertex.

**Done when:** `naxis_ref` emits the globally fastest Linear
trace on the 2e hand cases, the three 2f rows, and F20,
without reading interpolator fields. 2f/2g/2h compare the
interpolator to this file.

**Not this step:** a faster track that leaves the chord, cuts
a corner, or skips a vertex. That is not Linear (Overshoot
`T_opt` is Step 11). Nonzero corner speed on a kink is not
constraint-faithful Linear: the unit tangent jumps. Faithful
timed trajectory (whitepaper §3.3 problem 2) is not this
step: `naxis_ref` is the bound, not a timed executor.

**Done:** `extras/tests/pc_based/naxis_ref.h` walks a polyline
with `RampMap` + `DdaWalk`. `P`/`R` are path steps (one DDA
tick) so a collinear run may rebind the DDA master. One-block
traces match `LinearBlock` (2e). Two-block L / collinear /
reversal hold. F20 (341 blocks, seed 26, ticks 4000/8000):
vertices, envelope, law `P ≤ R`, recon slack ≤ 2, both
path-stop and cruise joints. Plot: `test_26_f20.gnuplot`.

---

## Step 2f — two-block Linear: path-stop vs collinear ✅

Still no queues. The interpolator walks `Remaining` across two
blocks: snap at vertices; `R` is `remaining_linear_binder`;
rebind per block.

**Test first:**

| Polyline | Assert |
|----------|--------|
| `(5,0)+(0,5)` | vertex `(5,0)` is a sample; reconstructed binder `P → 0` there; Y is the binder after the vertex |
| `(3,3)+(2,2)` | collinear; end `(5,5)`; no rest at the joint (`P` does not return to 0) |
| `(5,0)+(−3,0)` | reversal; `P → 0` at `(5,0)`; then X the other way |

Issued step sums equal the polyline. Envelope as in 2e. `P ≤ R`
from issued periods, including at the vertex sample.

**Implement:** extend `linear.h` to a two-block walk. Compare
the interpolator trace to `naxis_ref.h` (Step 2ref), not to
planner `P` fields. DIR pauses are Step 9 (queues); here a
reversal is only `P → 0` then the other sign.

**Plot:** `test_26_f2f.gnuplot` — the L, vertices marked.

**Done when:** the three rows green.

**Done:** `src/fas_naxis/linear.h` holds `LinearPoly`, an N-block Linear
interpolator (the two-block case of Step 2f, generalized for Step 2h). It
walks a `Remaining` ring: the DDA master is `Remaining::longest_axis` per
block, `R` is `remaining_path_steps` (path-step currency so a collinear run may
rebind the master), `P` resets at a path-stop (non-collinear vertex or last
block) and carries across a collinear joint. `f2f_two_block()` runs the three
rows — `(5,0)+(0,5)` L, `(3,3)+(2,2)` collinear, `(5,0)+(−3,0)` reversal —
through the interpolator and the `naxis_ref` oracle: envelope, `P_issued ≤ R`
from issued periods, issued `|steps| == |Δ|`, end is the last vertex, a vertex
sample per block, `P → 0` at a path-stop joint / no rest at a collinear joint,
and the two tracks agree within a 2-step log2 band. Plot:
`test_26_f2f.gnuplot` (the L). DIR pauses remain Step 9.

---

## Step 2g — exhaustive tiny Linear ✅

No new production if 2c–2f are right. Nested loops in
`test_26.cpp` only.

**Test first:** all 2-axis polylines with `|Δ_i| ≤ 5`, 3 vertices
(2 blocks) and 4 vertices (3 blocks), three integer `ticks_cfg`
pairs: `(4000,4000)`, `(4000,8000)`, `(100,99)`. Skip the
all-zero polyline. Idle-then-move is included (first block may
be `0`). Accel = 2000 (§14.1).

On every polyline:

- vertices exact (integer position equals the waypoint)
- no axis exceeds envelope (`ticks ≥ ticks_i_cfg`, one-step slack)
- `P ≤ R` from issued periods as in 2e, including path-stops
- collinear joints do not rest; non-collinear / reversal joints
  have `P → 0` at that vertex

`|Δ| ≤ 5` is combinatorics (DDA / vertex / `P ≤ R`), not
coasting. Do not assert `P_stop` cruise here.

**Implement:** enumeration + `naxis_ref.h` (Step 2ref) as the
trace oracle. Interpolator must stay O(steps) so ~10⁶ polylines
finish in a few seconds.

**Plot:** none (too many). Optional one representative
`test_26_f2g.gnuplot` if a failure needs a picture.

**Done when:** exhaustive set is green.

**Done:** `f2g_exhaustive()` in `test_26.cpp` walks every 2-axis polyline with
`|Δ_i| ≤ 5`, 3 vertices (2 blocks) and 4 vertices (3 blocks), across three
integer `ticks_cfg` pairs `(4000,4000)`, `(4000,8000)`, `(100,99)`, skipping the
all-zero first block. `f2g_walk` checks, against the `naxis_ref` oracle, the
envelope (`ticks ≥ ticks_i_cfg`), `P_issued ≤ R` from issued periods, issued
`|steps| == |Δ|`, end is the last vertex, every emitted vertex lands on a
cumulative waypoint, and the joint P semantics (path-stop `P_issued ≤ 1` vs
collinear cruise `> 1` once the live remaining path exceeds `P_coast`). ~5.3 M
polylines tested (42264 path-stop joints, 936 collinear joints).

---

## Step 2h — N-block interpolator vs F20 ✅

`linear.h` still one- or two-block after 2f. This step walks
an arbitrary polyline and must match `naxis_ref` on F20
(tick-for-tick within log2 / one-step slack, vertices, envelope).

**Test first:** F20 interpolator trace equals `naxis_ref`
(same asserts as Step 2ref F20).

**Implement:** generalize `linear.h` to N blocks (collinear
`P` carries; path-stop resets `P`; `R` is remaining **path**
steps in the Linear scan). DIR pauses still Step 9.

**Plot:** overlay on `test_26_f20.gnuplot` or
`test_26_f20_lin.gnuplot`.

**Done when:** interpolator matches `naxis_ref` on F20.

**Done:** `LinearPoly` (Step 2f) was already the N-block interpolator, so this
step adds `f2h_nblock_vs_f20()`: the same F20 polyline is now built by the shared
`build_f20_blocks` helper (seed 26), and both the `NaxisRefLinear` oracle and
the `LinearPoly` interpolator walk it. Asserts envelope, `P_issued ≤ R`, 2-step
log2 recon slack, issued `|steps| == |Δ|`, end is the last vertex, identical
vertex count and per-vertex positions, joint P within 2-step log2, and matching
path-stop joint count. 341 blocks, 341 vertices, 206 path-stop joints — both
tracks agree. Plot: `test_26_f20_lin.gnuplot`.

---

## Step 3 — ramp law `P` vs `R` (still no queues) ✅

**Test first:** one axis, rest-to-rest `S = 10000`, limits of §14.1.

- While `R > P`: `P` increases (accel) until period hits `ticks_cfg`
  (coast).
- When `R == P`: `P` decreases (decel).
- Never `P > R`.
- Step count issued equals `S`.
- Duration vs `RampCalculator` within a few Δt (oracle may convert
  ticks to seconds).

This is `_getNextCommand` without a `StepperQueue`. Planning chunk
size follows the FAS 2 ms rule, but the test may step `P` by 1 for
clarity and a second pass with `planning_steps`.

**Implement:** planner state `{P, R, ticks}` per axis.

**Plot:** `test_26_f1.gnuplot` — speed vs time, speed vs position,
`P` vs time, `R` vs time. Compare by eye to `test_02` trapezoids.

**Done when:** F1 rest-to-rest matches the analytic/FAS ramp.

**Done:** `src/fas_naxis/ramp_law.h` holds `RampLaw{P,R,ticks}` applying the
§7.1 law (`P` starts at 0; `R > P` accel / coast, `R == P` decel, period after
the `P` update so the first step is `calculate_ticks(1)`). `f3_ramp()` drives
`S = 10000` rest-to-rest: `P <= R` every step, peak `P == P_coast` (coasts
because `N/2 > P_stop`), decel starts at `S - P_coast`, total ticks match an
independent trapezoid, batched chunks land on the same P trajectory. Plot:
`test_26_f3.gnuplot`.

---

## Step 3b — stoppability from the command trace

Ignore planner `P`. From issued periods and the leftover
polyline, `calculate_ramp_steps(current_ticks) ≤ remaining`
on every axis at every sample (can still stop).

**Mutations** (must be documented in the test file): disable
cross-block `R` → F10 fails; disable rebind → F18 fails;
disable vertex snap → F5 misses `(1600,0)`.

**Done when:** the trace oracle (`naxis_ref.h` for Linear) is
green on F1/F5/F10, and the three mutations are listed as
comments (or `#if` hooks) next to those fixtures.

---

## Step 4 — `SimPort` `addQueueEntry` contract ✅

**Test first:** a duck-typed port used as
`FasNAxis<1, 64, SimPort>`. Feeder is `addQueueEntry` (§4.1).

- `addQueueEntry({ticks, steps, count_up}, start=false)` on an
  empty queue appends; `isQueueEmpty()` was true before — not
  underrun.
- Kick-off `addQueueEntry(NULL, true)`. After that,
  `isQueueEmpty()` while the plan still has motion is underrun.
- Pause `steps=0` uses the `count_up` you pass (no implicit flip).
- Revert = pause with `count_up = !old_dir`; default pd_test:
  no Injected, DIR already new.
- Hook: inject `DirChangePauseInjected` with old DIR (before) and
  with new DIR (after); after-pause must flip `queue_end` so a
  naive retry would XOR back.
- `drain(ticks)` advances position and clock.
- `isRampGeneratorActive()` is false unless the test forces it.

**Implement:** `extras/tests/pc_based/naxis_sim_port.h`.
No production header change except perhaps a documented duck-type
list.

**Plot:** none required.

**Done when:** a self-contained section of `test_26` covers the
table in whitepaper §4.1 / §4.4.1 without `FasNAxis` planning.

**Done:** `extras/tests/pc_based/naxis_sim_port.h` holds `SimPort`, a
duck-typed StepperQueue stand-in the feeder talks to through
`addQueueEntry()` only. `f4_sim_port()` in `test_26.cpp` runs the
§4.1/§4.4.1 table: append on an empty queue (`isQueueEmpty()` was
true before, no underrun), kick-off `addQueueEntry(NULL, true)` and the
empty-after-kick-off underrun (empty before kick-off is not underrun),
kick-off on an empty queue is `ErrorEmptyQueueToStart`, a pause
(`steps = 0`) uses the caller's `count_up` with no implicit flip, a
reverting pause (`count_up = !old`) leaves the port in the new DIR
with no inject (the pd_test default), the `InjectDirPauses` hook
injects a before-pause (old DIR) then an after-pause (new DIR, which
flips `queue_end` so a naive retry would XOR back) before the step
enqueues, a following same-direction step sees no further injection,
ticks below `max_speed_in_ticks` is `ErrorTicksTooLow`, `drain()`
advances signed position and the simulated clock, and
`isRampGeneratorActive()` is false unless forced.

---

## Step 5 — header skeleton + `addAxis` / position

**Test first (F16):**

- `FasNAxisConfig{}` has `dt_ticks=32000`, `kappa_stop_q8=320`,
  `overshoot_max=8`, `mode=Linear`. `dt_ticks==0` in a raw struct
  still becomes 32000 in the constructor. `kappa_stop_q8==0` → 320
  (diagnostic threshold for `isSpeedLimitedByLookahead` only).
- `PumpStatus` has `Idle`, `Running`, `Underrun`, `Error` — no
  `LookaheadTooShort`.
- `addAxis` fails if `i >= NAXES`, if the pointer is null, or if
  `isRampGeneratorActive()` / `isRunning()`. Small `HORIZON`
  relative to `P_stop` is **not** an `addAxis` failure.
- `addLine` before `syncFromSteppers()` / `setCurrentPosition()`
  is illegal.
- `addLine` to the current position is a no-op (`L=0`).

**Implement:** `src/FasNAxis.h` class template, config, axis
registration, current position. No motion yet.

Sketch in the whitepaper is C++11 (no designated initializers).

**Done when:** F16 and config-default tests pass.

---

## Step 6 — Linear interpolator, one block (F1, F2, F3)

**Test first:**

- F1 Linear through `SimPort`: step sum, `P≤R`, gnuplot.
- F2 Linear: 45° equal limits, `|Δx|==|Δy|` every slice, on chord.
- F3 Linear: `(10000, 100)`, binder is X, Y well below its ramp,
  reconstructed path within `0.5√n` of the chord. Vertex samples
  exist (start and end).

Interpolator snaps at vertices (§9.2). Linear DDA as in §6.3:
longest `|Δ|` binds; slaves scale down. Commands go to
`SimPort.addQueueEntry(..., start=false)` then kick-off
`addQueueEntry(NULL, true)`. Empty queue during prefill is not
underrun; empty after kick-off fails the test.

**Implement:** block buffer of one committed segment, Linear path,
feeder without reversals.

**Plots:** `test_26_f1.gnuplot`, `test_26_f2.gnuplot`,
`test_26_f3.gnuplot` (XY + speed/time).

**Done when:** F1–F3 Linear green; F17 (first-fill empty queue is
not underrun) green.

---

## Step 7 — Linear lookahead across blocks (F5, F9, F10)

**Test first:**

- F5 square 1600 Linear: `P→0` at each corner; decel starts on
  the side, not in the last slice; every corner is a sample.
- F9 `ticks_x = 10 * ticks_y` on a 45° line (equal `|Δ|`): X is
  slower so X binds; Y scaled down.
- F18 `(10000, 9000)` with Y 40× slower: longest is X (DDA
  master) but Y would exceed `v_max` if X ran at `ticks_x`; Y
  lengthens `ticks_b`, X scaled down in speed, both axes issue
  full `|Δ|` (`|Δ_i|*ticks_i_cfg` compare).
- F10 100-step micro-segments totalling 10000: `R` sees through;
  no rest at each joint (collinear test).

**Implement:** block ring `HORIZON`, `R` scan of §8.1 (end or
direction change) + Linear path-stop of §8.5 (angle change),
commit / replan of §8.7. Last buffered point is rest.

**Plots:** `test_26_f5.gnuplot` (XY square, v(t) touching 0 at
corners), `test_26_f9.gnuplot`, `test_26_f18.gnuplot`,
`test_26_f10.gnuplot` (v(t) with no per-segment dips).

**Done when:** F5, F9, F10, F18 green.

---

## Step 8 — feeder: drift, busy, too-large, ticks_min (F14, F15)

**Test first:**

- F14: 60 s simulated two-axis Linear, `|T_i−T_j|` a few ticks.
- F15: a slice that would violate `ticks_min` is rejected at plan
  time; feeder never sees `ErrorTicksTooLow`.
- `BUSY` / `QueueFull` / `DirPinIsBusy`: retry the **whole**
  slice; clocks stay together.
- Slice split when pause stuffing would exceed `QUEUE_LEN-2`.

**Implement:** §10.1 clocks, §9.3 split, retry class of §4.1.

**Plot:** `test_26_f14.gnuplot` — `T_i−T_j` vs time.

**Done when:** F14, F15 green.

---

## Step 9 — planner-issued before/after DIR pauses (F12, F12b, F12c)

**Test first:**

- At a vertex, **before any next-block step**, the planner emits
  a before-pause (`count_up = old`) then an after-pause
  (`count_up = new`) on the reversing axis, and matching
  timekeeping pauses (same ticks, DIR unchanged) on every other
  axis. Reversal is at low `P` on that axis; continuing axes
  may be fast — they still dwell.
- Credit: if the last command on the reversing axis already
  has ticks ≥ `dir_before`, the explicit before-pause may be
  omitted; SimPort then must not Inject on the after/step.
- Following reversing step: no Injected.
- F12b: Overshoot dog-leg, X continuing at high `P`. X’s
  position is frozen at the vertex for `τ_before+τ_after`;
  then X resumes at the same period. Fail if X steps during
  τ_dir (that is the uncoordinated-injection path error).
- F12c: SimPort injects one extra before-pause anyway
  (under-counted drain). Feeder copies those ticks to all
  other axes **immediately**. XY does not leave the vertex.
  No later speed-up to catch the time. `|T_i−T_j|` still bound.
- Injected on a non-revert step command (next-block already
  queued on another axis) fails the test.

**Implement:** §4.4.1 / §4.4.2 / §10.2. Read
`getDirChangeBeforeTicks()` / `getDirChangeBeforePauseCount()` /
`getDirChangeAfterTicks()` from the stepper. Config
`dir_before_ticks` / `dir_after_ticks` override when non-zero.

**Plot:** `test_26_f12.gnuplot` — DIR, pauses, and XY at the
vertex vs time; F12b overlay.

**Done when:** F12, F12b, F12c green.

---

## Step 10 — gnuplot helper completeness + HTML stub (P2)

**Test first:** F5 Linear writes a 2×2 gnuplot that gnuplot can
parse (header `$data <<EOF`, `EOF`, `set multiplot`). Optional
`-DFAS_NAXIS_TRACE` writes `extras/n_axes/tests/out/F5.html`
from `extras/n_axes/viewer_template.html`.

**Implement:** `naxis_plot.h` finished; `naxis_html_dump.h` +
checked-in viewer template. Production header unchanged unless
`FAS_NAXIS_TRACE`.

**Done when:** F5 plot exists after `test_26`; HTML only when
the macro is set; `src/FasNAxis.h` has no viewer include by
default.

---

## Step 11 — Overshoot rest-to-rest (F4, F4b)

**Test first:**

- Same `(10000, 100)` as F3, mode Overshoot, `overshoot_max=8`:
  both endpoints hit; `max d² ≤ 64`; bulge visible vs F3.
- F4b cap `UINT16_MAX`: bulge order 10 steps, not an L (short
  axis does not finish and wait).
- Lone diagonal: `T` equals Linear `T` (binding axis).

**Implement:** per-axis ramp, `T = max T_opt_i`, stretch by
**lengthening** period (`log2_divide(log2_T, log2_|Δ|)`), never
faster than the ramp, no delayed start. Extend `naxis_ref.h`
(Step 2ref) with Overshoot `T_opt` so F4/F4b duration is
compared to that oracle, not to interpolator fields.

**Plots:** `test_26_f4.gnuplot`, `test_26_f4b.gnuplot` — XY with
chord in grey.

**Done when:** F4, F4b green.

---

## Step 12 — Overshoot corners and circle (F6, F6b, F7)

**Test first:**

- F6 45° dog-leg: `P_x ≠ 0` at the vertex, `P_y = 0`, vertex is
  a sample, `T_overshoot ≤ T_linear`, `d²` ≤ cap.
- F6b `(0,0)→(4000,1)→(4000,4000)`: `P_y ≤ 1` after the first
  block; vertex hit; cap holds; no “high v_out in 32 ms stretched
  to 4 s at a_max”.
- F7 circle r=1600, 1° chords: reversing axis only goes through
  0; the other keeps `P`; `d²` per chord ≤ cap.

**Implement:** `R` spanning same-sign blocks; `overshoot_max` mix
toward Linear; snap + revert at extrema.

**Plots:** `test_26_f6.gnuplot`, `test_26_f6b.gnuplot`,
`test_26_f7.gnuplot` (circle, colour by speed).

**Done when:** F6, F6b, F7 green.

---

## Step 13 — dwell, starve, underrun, lookahead speed cap (F11, F13, F19)

**Test first:**

- `addDwellTicks` mid-path: planned stop, wait, continue from
  rest. Not a pause command at speed.
- F11: dribble waypoints so `R < P_stop` while the path is still
  open. Last point is rest. **Not** an error and **not** a
  feed-hold: `pump()` returns `Running`,
  `isSpeedLimitedByLookahead()` is true, `P ≤ R`, peak `P` fits
  in `R` (about `R/2` from rest). After enough collinear
  waypoints, `R` grows and cruise at `ticks_cfg` is allowed.
  `lookaheadHint()` may name axis / `R` / `P_stop` / `HORIZON`
  as a diagnostic, not as recovery advice that the caller must
  act on before motion continues.
- F13: starve `pump()` on purpose after kick-off; `hasUnderrun()`;
  plot still written up to the fault.
- F19: `HORIZON` too small to hold `P_stop` as micro-segments.
  `addAxis` succeeds. `P` never reaches `P_stop`. Same
  `HORIZON` with one long `addLine` *does* coast (`R` is steps,
  not points).

**Implement:** zero-displacement blocks; §8.2 / §8.7 speed cap
and replan; underrun flag after kick-off only. No
`LookaheadTooShort` status.

**Plots:** `test_26_f11.gnuplot` (v(t) capped then recovers),
`test_26_f13.gnuplot` (cut off at underrun event),
`test_26_f19.gnuplot` (micro-segment cap vs long-line coast).

**Done when:** F11, F13, F19 green.

---

## Step 14 — 3-axis SimPort + HTML (F8)

**Test first:** helix on `FasNAxis<3, 4096, SimPort>`, both modes.
No `MAX_STEPPER` change. Oracle: no axis above limits; vertices
hit. HTML 3D overlay when `FAS_NAXIS_TRACE`.

**Plot:** `test_26_f8.gnuplot` (XY and XZ) + optional HTML.

**Done when:** F8 green without linking extra queues.

---

## Step 15 — real `FastAccelStepper` 1–2 axis (P5)

**Test first:** F1 and F2 again with default `Stepper =
FastAccelStepper`, drain `fas_queue[]` like `test_16` /
`RampChecker`. Step sums and duration class match SimPort; command
streams need not be identical.

Link `LIB_O` (normal `test_%` rule). Skip if you split this into
`test_27.cpp` that must link the library while `test_26` stays
SimPort-only — prefer one binary unless the link set fights
(then Makefile comments, same pattern as `test_24` / `test_25`).

**Plots:** `test_26_f1_fas.gnuplot`, `test_26_f2_fas.gnuplot`.

**Done when:** F1/F2 identity-class green on real queues.

---

## Order and merge points

| After step | Whitepaper phase | What a reviewer can check |
|------------|------------------|---------------------------|
| 1–3 | P0 | log2 + `R` + `P≤R` without queues |
| 2b | P0 | theory probes (oracle, rebind, collinear, lookahead speed cap, mutations) |
| 2c–2h | P0 | tiny Linear + PC reference + F20 long polyline vs interpolator |
| 3b | P0 | trace stoppability on F1/F5/F10 |
| 4–9 | P1 | Linear through `addQueueEntry`; F18 rebind |
| 10 | P2 | gnuplot always; HTML optional |
| 11–12 | P3 | Overshoot |
| 13 | P4 | lookahead speed cap / underrun |
| 14–15 | P5 | 3-axis sim + 1–2 axis FAS |

Do not start a later step until the earlier step’s tests are
green. If a step uncovers a spec hole, fix the whitepaper in the
same change as the test.

---

## Out of scope until the list above is green

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
