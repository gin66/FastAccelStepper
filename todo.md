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
8. An open step is a patch. Make the edit named **First edit**
   before reading further whitepaper sections. Do not replace
   `feed_one` with a 2 ms multi-step slicer. Do not edit
   `naxis_sim_port.h` unless that step lists a SimPort addition.

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

## Step 2d — Linear one-block rest-to-rest (binder ramp + DDA) ✅

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

## Step 2e — `P ≤ R` from issued periods (not planner fields) ✅

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

**Done:** `f2e_issued_periods()` in `test_26.cpp` replays the three
hand cases on `LinearBlock`. `P_issued` comes from
`RampMap::calculate_ramp_steps(ticks)`, not from `RampLaw.P`.
Master stays the longest `|Δ|`; `ticks_floor` lengthens the period;
issued `|steps| == |Δ|`.

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

## Step 3b — stoppability from the command trace ✅

Ignore planner `P`. From issued periods and the leftover
polyline, `calculate_ramp_steps(current_ticks) ≤ remaining`
on every axis at every sample (can still stop).

**Mutations** (must be documented in the test file): disable
cross-block `R` → F10 fails; disable rebind → F18 fails;
disable vertex snap → F5 misses `(1600,0)`.

**Done when:** the trace oracle (`naxis_ref.h` for Linear) is
green on F1/F5/F10, and the three mutations are listed as
comments (or `#if` hooks) next to those fixtures.

**Done:** `f3b_stoppability()` in `test_26.cpp` walks the
`NaxisRefLinear` oracle on F1 (1-axis `10000`), F5 (square `1600`)
and F10 (100×100 collinear) and checks, at every moving sample,
the per-axis reconstructed `P_issued = calculate_ramp_steps(ticks)
≤ remaining(i, block)` with the documented two-step log2
slack (§12.4, same convention as `walk_polyline`). F1/F10 coast to
`P_coast` (within 1%), F5 path-stops at every 90° corner
(`P → 0`) and each side is too short to coast. The three mutation
hooks are documented next to the fixtures and proven by
`make mutations`: `FAS_NAXIS_NO_CROSS_BLOCK_R` (naxis_ref.h, F10
joints rest), `FAS_NAXIS_NO_REBIND` (remaining.h, Step 2b
neighbourhood), `FAS_NAXIS_NO_REST_CAP` (ramp_law.h, peak `P` is
not `< R`).

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

## Step 5 — header skeleton + `addAxis` / position ✅

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

**Done:** `src/FasNAxis.h` holds `FasNAxis<NAXES, HORIZON=64,
Stepper=FastAccelStepper>` (header-only, NOT included from
`FastAccelStepper.h`; the default `FastAccelStepper` is forward
declared so the template parses when only this header is included,
as in the self-contained `test_26`). `FasNAxisConfig{}` carries the
documented defaults (`dt_ticks=32000`, `kappa_stop_q8=320`,
`overshoot_max=8`, `dir_before/after_ticks=0`, `mode=Linear`); the
constructor recovers `dt_ticks==0 → 32000` and `kappa_stop_q8==0 →
320` so a raw zeroed struct still means the defaults. `PumpStatus` is
a scoped `enum class {Idle=0, Running=1, Underrun=2, Error=3}` — no
`LookaheadTooShort`. `addAxis(i, s)` fails on `i >= NAXES`, a null
pointer, or `isRampGeneratorActive()` / `isRunning()`, and reads
`ticks_cfg` from `getMaxSpeedInTicks()`; a small `HORIZON` still
registers cleanly (the ramp cap is F19's concern). `syncFromSteppers()`
reads `getCurrentPosition()` per axis, `setCurrentPosition(p)` takes an
array; both open `addLine`, which is illegal (returns `false`) before
a sync and a no-op (returns `true`, records no block, verifiable via
`block_count() == 0`) when the target equals the current position
(`L = 0`). Backed by `SimPort` (a `getCurrentPosition()` / 
`getMaxSpeedInTicks()` duck-type was added to `naxis_sim_port.h`) so
F16 exercises the same query surface as the real stepper without
raising `MAX_STEPPER` or linking extra queues. `f16_skeleton()` in
`test_26.cpp` runs the full contract table; no motion is planned
(that is Step 6+).

---

## Step 6 — Linear interpolator, one block (F1, F2, F3) ✅

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

**Done:** `src/FasNAxis.h` now plans and feeds one committed rest-to-rest
segment. `addLine` commits the segment (delta from the current target);
`endPath` closes the path. `pump()` runs the §10.4 loop: `feeder_start`
picks the DDA master (`Remaining::longest_axis`) and the `RampLaw` at
`ticks_floor`/`addAxis` accel, prefill appends with `start=false`, then
each axis is kicked off with `addQueueEntry(NULL, true)` and later
commands use `start=true`. `feed_one` emits at most one entry per axis per
call so the axes stay in lockstep; a master period above 65535 is
represented the FAS way as a half-period step entry plus pause entries
(§4.2/§9.3), keeping the common clock. `pump` flags `Underrun` only after
kick-off with the plan still moving, and clears the segment to `Idle` once
the plan is done and the queues empty. Getters `performedRampUp`,
`remainingToStop`, `lastTicks`, `masterAxis`, `isBusy`, `hasUnderrun` feed
the oracle. `SimPort` gained duck-typed `getAcceleration`,
`queueEntries`, `isQueueFull` (one reserved slot) and a `drain_one` that
advances position/clock by exactly one command. `f6_linear_sim()` in
`test_26.cpp` runs F1 `(10000,0)` (idle Y issues nothing), F2 `(1600,1600)`
45° equal limits (a step on each axis every slice), F3 `(10000,100)` (X
master, path within `0.5*sqrt(2)` of the chord) and F17 (first fill on an
empty queue is not underrun). All issued sums equal the target, `P <= R`
holds on every sample, and no run underruns. Plots: `test_26_f1_lin.gnuplot`,
`test_26_f2.gnuplot`, `test_26_f3_lin.gnuplot` (`f1_map`/`f3` were already
taken by Steps 1 and 3).

---

## Step 7 — Linear lookahead across blocks (F5, F9, F10) ✅

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

**Done:** `src/FasNAxis.h` now commits points into a block ring of up to
`HORIZON` n-dim points (`addLine` appends; a full ring returns `false` for
backpressure) and feeds the whole Linear path. The DDA master is the longest
`|delta|` of the current block; `R` is `remaining_path_steps(head)` — remaining
master steps to the next Linear path-stop, summed across collinear blocks — so
a collinear run carries `P` and `R` across joints whereas a non-collinear vertex
resets `P` (section 8.1/8.5/8.7). A path that catches up to the buffer re-opens
when more points arrive. The 65535 split now keeps the step entry `>= ticks_law`
(`max(T/2, ticks_law)`, remainder as pauses) — halving alone could send a step
faster than `v_max` for a slow axis just above the 16-bit boundary; the
whitepaper section 9.3 records this. `f7_linear_lookahead()` drives each fixture
through `SimPort`, coalesces the split entries back to full periods, and
compares the trace tick-for-tick with the `naxis_ref` oracle: F5 square 1600
(`P -> 0` at every corner, decel starts on the side, exact four vertex samples),
F9 45° with `ticks_x = 10·ticks_y` (X stays the DDA master, Y scaled down:
shared period `>= ticks_x`), F18 `(10000,9000)` with Y 40x slower (X is DDA
master, Y binds the time law: shared period `>= ticks_y`, both issue `|delta|`),
and F10 100×100-step collinear (one vertex sample per micro-segment, no rest at
any of the 99 collinear joints, coasts to `P_coast`). `SimPort`'s max-speed
floor is now scoped to step commands (a pause is a delay, floor
`MIN_CMD_TICKS`). Plots: `test_26_f5.gnuplot`, `test_26_f9.gnuplot`,
`test_26_f18.gnuplot`, `test_26_f10.gnuplot`.

---

## SimPort — what stays, what each open step may add

`extras/tests/pc_based/naxis_sim_port.h` is the stepper the
feeder already talks to. Steps 0–7 are green on it. Do not
rewrite it, and do not add a second clock, an undo, a multi-step
packer, or an ISR.

Already enough, leave these alone:

- `addQueueEntry`, kick-off `addQueueEntry(NULL, true)`,
  `AQE_ERROR_TICKS_TOO_LOW` when `steps > 0` and
  `ticks < max_speed_in_ticks`
- `isQueueFull` / `isQueueEmpty` / `queueEntries` / `drain` /
  `drain_one` / `clock()` / `position()`
- `InjectNone` and `InjectDirPauses` (before-pause at the old
  DIR, after-pause at the new DIR, step not enqueued until both
  have been returned)
- `injectedPauseTicks()`, `getMaxSpeedInTicks()`,
  `getAcceleration()`, `getCurrentPosition()`,
  `isRampGeneratorActive()`, `isRunning()`
- `drain_one`’s returned tick sum **is** the period, because the
  feeder issues `steps` of 0 or 1 only

Not expressible today, so later steps add exactly the methods
named there and nothing else:

| Step | Add on `SimPort` | Why the current port cannot do it |
|------|------------------|-----------------------------------|
| 8 | `void failNext(AqeResultCode rc)` | `AQE_QUEUE_FULL` is returned only when `isQueueFull()` is already true, and `pump()` refuses to call `addQueueEntry` in that case. Filling one axis by hand makes `all_have_room()` false, so no command is sent and the clocks cannot diverge. `failNext` returns `rc` once, enqueues nothing, and leaves `isQueueFull()` false. Default: off. |
| 9 | `getDirChangeBeforeTicks()`, `getDirChangeBeforePauseCount()`, `getDirChangeAfterTicks()`, default 0. Setter `setDirChangeBudget(before, n_before, after)`. | `FasNAxis` must call the same names as `FastAccelStepper`. A default of 0 keeps today’s F5 reversal as a plain `count_up` flip. |
| 9 | `void forceExtraBefore(uint16_t ticks)` | `InjectDirPauses` fires only when `count_up` disagrees with `queue_end`. After the planner has already issued the after-pause, the following step is not a reversal, so F12c’s extra before-pause never happens. One shot: the next `steps > 0` command enqueues a pause of `ticks` at the **old** `count_up`, sets `injectedPauseTicks()`, returns `AQE_DIR_CHANGE_PAUSE_INJECTED`, and does not enqueue the step. Then the flag clears. Use it with `InjectNone`. |

Steps 10–15 add nothing to `SimPort`. Step 14 is three `SimPort`
objects. Step 15 uses real `FastAccelStepper` queues.

`isQueueFull()` keeps reserving one slot. The `QUEUE_LEN - 2`
reserve is a `FasNAxis` check on `queueEntries()`, not a new
`SimPort` predicate. `QUEUE_LEN` is 16 (`pd_test/pd_config.h`).
Construct `SimPort(ticks, 16)` when a test cares; the ring length
must be a power of two.

---

## Step 8 — feeder: drift, retry, room (F14, F15)

Do not introduce a 2 ms slice. A slice is one `feed_one()`
emission: one command per registered axis, `steps` 0 or 1, the
same tick sum on every axis. The 65535 split (`t_step =
max(T/2, _ticks_law)`, remainder in `_pause_left`) already
exists. If a new assert passes without editing that formula, do
not edit it.

`send_to` ignores the `addQueueEntry` result. That is the bug
this step fixes. There is no undo: a command that returned
`AQE_OK` stays in that axis’s queue.

**First edit:** `extras/tests/pc_based/test_26.cpp`, new
`f8_feeder()`, called from `main()` after `f7_linear_lookahead()`.
Three fixtures, in this order. Run after each one.

**8.1 F14, expect green with no `FasNAxis.h` change.** Linear
`(240000, 0)`, both axes `SimPort(4000)`, accel 2000, `endPath`,
pump/drain like `run_linear_segment`. Do not store a per-step
array. After every paired `drain_one`, track
`max |px.clock() - py.clock()|`. Assert that max is `<= 2`, both
final positions are `(240000, 0)`, and `px.clock() >= 60ull *
16000000ull` (240000 coast steps at 4000 ticks is 60 s; accel
makes it longer). Plot `test_26_f14.gnuplot` with
`NaxisPlot::start_scalar`: column 1 time in seconds, column 2
`clock_x - clock_y`, subsample every 1000th pair. If this is
already green, leave the command stream alone. Do not add
`T_plan` inside `FasNAxis`.

**8.2 Retry, this one requires a code change.** Add
`SimPort::failNext` as in the table above. Test:

1. Run a `(500, 500)` Linear move for a few paired drains.
2. `py.failNext(AQE_QUEUE_FULL)`.
3. `pump()` once.
4. Assert X’s queue gained the new command and Y’s did not, and
   `pump()` did not plan a second command onto X.
5. `pump()` again with no fault. Y receives that same command.
   Then paired `drain_one` (if one queue is empty, `pump()`
   again instead of draining the other). Assert
   `|px.clock() - py.clock()| <= 2` after every pair, and the
   move still ends at `(500, 500)`.
6. Repeat the fault with `AQE_DIR_PIN_IS_BUSY`. Same asserts.
   One code path handles `QueueFull`, `DirPinIsBusy`,
   `WaitForEnablePinActive`, and `DeviceNotReady`.

Implementation, only in `src/FasNAxis.h`:

- Add a held command per axis: `bool waiting`, `uint16_t ticks`,
  `uint8_t steps`, `bool count_up`, plus `bool _slice_open`.
- `feed_one` builds a command only when `!_slice_open`, using
  today’s body (`_law.step()`, DDA, 65535 split), stores it, sets
  `waiting` on registered axes, sets `_slice_open`. It must not
  call `_law.step()` again until the held slice is done.
- `flush_held()` sends only axes with `waiting`. `AQE_OK` clears
  `waiting`. A retry code leaves `waiting` set and does not send
  a new plan step. When no `waiting` remains, `_slice_open` is
  false.
- `AQE_ERROR_TICKS_TOO_LOW` and, until Step 9, both pause-injected
  codes: set an error flag and return `PumpStatus::Error`. Do not
  retry those. Mark the injected branch
  `// Step 9: time bubble, not Error`.

**8.3 Room.** `SimPort px(4000, 16), py(4000, 16)`. One long
Linear move. `pump()` until it stops making progress, and do not
drain. Assert `queueEntries() <= QUEUE_LEN - 2` (14) on both.
Today `all_have_room()` uses `isQueueFull()`, which allows 15.
Change it to `queueEntries() + 2 >= QUEUE_LEN`. Do not change
`SimPort::isQueueFull`. Then drain to the end and assert the
target is hit and no drained step has tick sum `< 4000`
(F15: the `_ticks_law` floor already rejects a too-fast step
before `addQueueEntry`; this assert is the test).

**Done when:** 8.1, 8.2, and 8.3 are green and the existing
`test_26` fixtures still pass.

```
make -C extras/tests/pc_based test_26 && extras/tests/pc_based/test_26
```

---

## Step 9 — planner-issued before/after DIR pauses (F12, F12b, F12c)

Do not change Step 8’s held-slice retry. DIR pauses are ordinary
held commands (`steps = 0`) issued **before** the first
outgoing-block step is built. Default budgets stay 0, so F5’s
trace must still match `naxis_ref`. If a reversal emits a pause
when both the config and the getters are 0, F5 is broken.

**First edit:** `SimPort` methods from the table
(`setDirChangeBudget`, the three getters, `forceExtraBefore`),
then `f9_dir_pauses()` in `test_26.cpp`, called from `main()`
after `f8_feeder()`.

**Budget.** On the reversing axis, `τ_before` is
`cfg.dir_before_ticks` when that field is non-zero, otherwise
`s->getDirChangeBeforeTicks()`. `τ_after` is the same with
`dir_after_ticks` / `getDirChangeAfterTicks()`. `n_before` is
`getDirChangeBeforePauseCount()`, or 1 when the config override
is non-zero and the getter count is 0. A zero `τ` skips that
pause.

**Sequence** at a vertex where the next step’s `count_up` differs
from `_dir[i]`, and no outgoing step has been stored yet:

1. `n_before` times: `{steps=0, ticks=τ_before, count_up=old}` on
   the reversing axis, and the same tick value with **that**
   axis’s unchanged `count_up` on every other axis.
2. Once: `{steps=0, ticks=τ_after, count_up=new}` on the reversing
   axis; other axes get the same ticks and their own unchanged
   `count_up`.
3. Only then build the next block’s step slice.

Credit: if the reversing axis’s previous command already has
`ticks >= τ_before`, skip the before-pause (still issue the
after-pause). SimPort with `InjectNone` must then accept the
following step with `AQE_OK`.

**Injected, replacing the Step 8 error branch.** On
`AQE_DIR_CHANGE_PAUSE_INJECTED` or `AQE_DIR_PIN_2MS_PAUSE_ADDED`:
read `injectedPauseTicks()` — add that same method on
`FastAccelStepper`, next to `getDirChangeAfterTicks`, returning
`_queue()->_injected_pause_ticks`. Do not include the queue
header from `FasNAxis.h`. Enqueue a timekeeping pause of exactly
those ticks on every other axis (`count_up` unchanged) before any
later step. Do not shorten a later period to catch up. If a
non-pause step on another axis was already accepted for the
outgoing block, return `PumpStatus::Error` (the test fails the
run).

**Fixtures:**

- F12: square corner, `setDirChangeBudget(3200, 1, 3200)`. At the
  first corner the trace shows a before-pause (old `count_up`)
  then an after-pause (new `count_up`) on the reversing axis, and
  pauses of those same tick sums on the other axis, before any
  step of the next side. The following step returns without
  `injectedPauseTicks() != 0`. Both axes end on the square.
- F12b is Step 12’s dog-leg. Here, only the Linear square: the
  other axis’s position does not change across the pause pair.
  Leave a `// F12b: Overshoot continuing axis, Step 12` comment
  and do not invent Overshoot in this step.
- F12c: `InjectNone` plus `forceExtraBefore(8000)` armed at the
  corner. After the injected return, the other axis has a pause
  of 8000 and neither position has left the vertex. Final
  `|clock_x - clock_y| <= 2`.

Plot `test_26_f12.gnuplot` with `NaxisPlot` (XY plus the pause
samples). **Done when:** F12 and F12c are green, F5 still matches
`naxis_ref`, and `failNext` from Step 8 still retries.

---

## Step 10 — gnuplot file check + HTML stub (P2)

`naxis_plot.h` already writes `$data <<EOF`, `EOF`, and
`set multiplot layout 3,2` for F5 (`test_26_f5.gnuplot`). That
file is the plot. Do not rebuild `NaxisPlot` and do not change
the layout to 2×2.

**First edit:** in `f7_linear_lookahead()`’s F5 block, after the
run, open `test_26_f5.gnuplot` and `test()` that the bytes
`$data <<EOF`, a line `EOF`, and `set multiplot` occur. If that
passes, stop touching `naxis_plot.h`.

**Then:** add `extras/tests/pc_based/naxis_html_dump.h` (test
only, not included from `src/FasNAxis.h` unless
`FAS_NAXIS_TRACE` is defined). Under that macro, F5 also writes
`extras/n_axes/tests/out/F5.html` by copying
`extras/n_axes/viewer_template.html` and embedding the same
samples the gnuplot file already has (t, x, y). The template is
a checked-in static page with a `<pre id="trace">` placeholder.
Without the macro, the html file is not created and
`src/FasNAxis.h` has no viewer include.

**Done when:** `test_26` is green either way, and a rebuild with
`-DFAS_NAXIS_TRACE` produces `F5.html`.

---

## Step 11 — Overshoot rest-to-rest (F4, F4b)

Do not fold this into Linear `feed_one`. Linear mode must keep
calling the current DDA path.

**First edit:** `extras/tests/pc_based/test_26.cpp` function
`f11_overshoot_rest()`, and a new header
`src/fas_naxis/overshoot.h` (no float, no integer `/`).
`FasNAxis::feed_one` calls it only when
`_cfg.mode == FasNAxisConfig::Overshoot`.

`OvershootBlock` for one rest-to-rest segment, two axes:

- Each axis: a `RampLaw` over `|Δ_i|` with that axis’s
  `ticks_cfg`. `T_opt_i` is the sum of those periods (walk a
  copy). `T = max T_opt_i`. The binding axis is the one whose
  `T_opt` equals `T` (lower index on a tie).
- Binding axis: one `RampLaw::step()` per command, duration =
  that period. It never pauses and never waits at the end.
- Other axis: exactly `|Δ_i|` steps spread across `T`, no
  delayed start. Integer test only, multiply-compare, no `/`:
  after binding time `t`, the short axis should have completed
  `k` steps when `|Δ_short| * t >= k * T`. Catch-up inside one
  binding period is `steps` on that command (1..255) or a split
  of the same duration; it is not a pause after the short axis
  has already finished. Sum of short-axis tick sums equals `T`
  within a few ticks.
- `overshoot_max == 0` is Linear (do not call this class).
  `UINT16_MAX` is the raw schedule above. A finite cap mixes the
  short axis toward the Linear DDA fraction by **lengthening**
  its early periods (never shortening the binder) until the
  test’s `d²` holds. `d²` in the test is double; production
  compares an integer squared distance to
  `overshoot_max * overshoot_max`.

**Fixtures** (ticks 4000, accel 2000), same harness as
`run_linear_segment`:

- F4: `(10000, 100)`, `mode = Overshoot`, `overshoot_max = 8`.
  End positions exact. `max d² <= 64`. `max d²` is greater than
  F3’s (the bulge is visible). Plot `test_26_f4.gnuplot`.
- F4b: same segment, `overshoot_max = UINT16_MAX`. Short axis
  still has steps left at mid-time (not an L: it must not finish
  and then pause). End positions exact. Plot
  `test_26_f4b.gnuplot`.
- Lone diagonal `(1600, 1600)`, both modes. Overshoot total
  `clock()` equals Linear `clock()` within 2 ticks (same binding
  ramp). `d²` stays under 1.

Extend `naxis_ref.h` with an Overshoot duration: sum of the
binding `RampLaw` only. Compare F4/F4b `clock()` to that sum,
not to a field inside `FasNAxis`.

**Done when:** F4, F4b, and the diagonal are green, and F1–F3
Linear plots are unchanged.

---

## Step 12 — Overshoot corners and circle (F6, F6b, F7)

`OvershootBlock` from Step 11 is one segment from rest. This step
lets `P` cross a vertex.

**First edit:** `f12_overshoot_corners()` in `test_26.cpp`.

Per axis, `R` is the sum of `|Δ_i|` while the sign stays the same
(or the axis is idle). A sign change or a zero-length axis sets
that axis’s `R` to the current block only and forces its `P` to 0
at the vertex. The other axis keeps `P`. `T` for the block is
still `max T_opt_i` with those entry `P` values. Snap positions
to the vertex before the next block’s steps (Step 9 pauses, if
the budget is non-zero, sit between the two).

**Fixtures** (ticks 4000, accel 2000, `overshoot_max = 8`):

- F6: `(0,0) → (1600,1600) → (3200,0)`. At the vertex, sample
  position is exactly `(1600, 1600)`, `P` of the reversing axis
  is 0, `P` of the continuing axis is not 0. Total `clock()` is
  `<=` the same polyline in Linear. `d²` per chord `<= 64`.
  Plot `test_26_f6.gnuplot`.
- F6b: `(0,0) → (4000,1) → (4000,4000)`. After the vertex,
  `P` on Y is `<= 1`. Vertex position exact. `d² <= 64`. Y does
  not spend the first block at a high rate and then stretch; its
  single step is one long period. Plot `test_26_f6b.gnuplot`.
- F7: circle radius 1600, 180 chords of 1° (integer `x,y` via
  the same rounding F20 already uses in `test_26.cpp`). Only the
  axis whose sign flips has `P == 0` at that vertex. `d²` of
  each chord `<= 64`. Plot `test_26_f7.gnuplot`.

Fill in the Step 9 `F12b` comment with this dog-leg: the
continuing axis’s position is unchanged for `τ_before + τ_after`
and its `P` after the bubble equals its `P` before the bubble.
Budget `(3200, 1, 3200)`.

**Done when:** F6, F6b, F7, and F12b are green.

---

## Step 13 — dwell, starve, underrun, lookahead speed cap (F11, F13, F19)

The speed cap is already how `remaining_path_steps` treats the
last buffered point as rest. This step exposes it and adds dwell
plus a real underrun. Do not add a `LookaheadTooShort` status.

**First edit:** `f13_lookahead()` in `test_26.cpp`.

**Methods on `FasNAxis`:**

- `bool addDwellTicks(uint32_t ticks)`. Legal only when the path
  is synced. Inserts a zero-motion block that issues pauses of
  `ticks` (split at 65535 the same way `_pause_left` already
  does) on every axis, from rest to rest: `P` is 0 on the way in
  and on the way out. It is not a pause stuffed into a moving
  slice.
- `bool isSpeedLimitedByLookahead() const`. True when the live
  `R < P_stop` of the master while the path is still open
  (`!_path_closed` and `_head < _n_blk`). False on a closed path
  and when `R` is large enough to coast.
- `void lookaheadHint(uint8_t* axis, uint32_t* R, uint32_t* P_stop, uint16_t* horizon) const`. Fills those four outs. No string, no heap.

**Fixtures** (ticks 4000, accel 2000; `P_stop` is
`RampMap(4000, 2000).P_coast()`):

- Dwell: `(0,0) → (400,0)`, `addDwellTicks(80000)`, then
  `(800,0)`. Positions stay `(400, 0)` for exactly 80000 ticks
  of `clock()`, and the second half starts at `P == 0`.
- F11: `HORIZON` large. `addLine` one 800-step collinear chunk on
  X, `pump` a few commands **without** `endPath`. Assert
  `pump()` is `Running`, `isSpeedLimitedByLookahead()` is true,
  `P <= R`, and peak `P` is about `R/2` (under `P_stop`). Then
  `addLine` nine more 800-step collinear chunks so `R` at the
  head is 8000. Assert a later moving sample reaches `P_stop`
  (coast). Plot `test_26_f11.gnuplot`.
- F13: start F1 `(10000, 0)`, `pump` once, kick-off has happened,
  then drain both queues to empty and do **not** call `pump`.
  Assert `hasUnderrun()` and `pump()` returns `Underrun`. Write
  `test_26_f13.gnuplot` with the samples collected before the
  drain.
- F19: `FasNAxis<2, 4, SimPort>`. Four `addLine`s of 50 steps,
  open path. `addAxis` returned true. Peak `P` stays below
  `P_stop`. A second instance, same `HORIZON` 4, one `addLine`
  of 10000 steps, `endPath`: peak `P` reaches `P_stop`. Plot
  `test_26_f19.gnuplot` with both runs.

**Done when:** those four asserts are green. F10 (closed
collinear path) still coasts.

---

## Step 14 — 3-axis SimPort (F8)

No `SimPort` change. No `MAX_STEPPER` change. No new queue
objects beyond three `SimPort`s.

**First edit:** `f14_helix()` in `test_26.cpp` on
`FasNAxis<3, 4096, SimPort>`.

Helix: 180 chords, radius 1600, one full turn, `Z` increases by
10 steps per chord (integer XY via the F20 rounding). Run it
twice, `mode = Linear` and `mode = Overshoot` with
`overshoot_max = 8`. Assert each vertex position is hit on all
three axes, every drained step has tick sum `>= 4000`, and
Overshoot `d²` in XY `<= 64`. Plot `test_26_f8.gnuplot` (the
existing `NaxisPlot` XY panel plus a second scalar file
`test_26_f8_xz.gnuplot` via `start_scalar` for X and Z). When
`FAS_NAXIS_TRACE` is set, also write `extras/n_axes/tests/out/F8.html`
through the Step 10 dumper (add Z as a third column; do not
invent a new viewer).

**Done when:** both modes finish at the last vertex and `test_26`
still does not link a third hardware queue.

---

## Step 15 — real `FastAccelStepper`, 1–2 axis (P5)

`test_26.cpp` already provides `inject_fill_interrupt` /
`noInterrupts` / `interrupts` and the Makefile already links
`LIB_O`. Stay in this file.

**First edit:** `#include "FastAccelStepper.h"` and
`#include "fas_queue/stepper_queue.h"`, then `f15_fas()` called
from `main()`. If that include fails to compile, move only
`f15_fas` into `extras/tests/pc_based/test_27.cpp` and copy the
`test_24` / `test_25` Makefile exception. Do not do that split
until the include has actually failed.

Copy the `test_16` setup: `fas_queue[i]._initVars()`,
`FastAccelStepper s; s.init(NULL, i, 0)`, `setSpeedInTicks(4000)`,
`setAcceleration(2000)`. `FasNAxis<2, 64>` (the default stepper
type). `addAxis`, `setCurrentPosition({0,0})`, the F1 target
`(10000, 0)` and the F2 target `(1600, 1600)`.

Drain like `test_16`: while `isBusy()`, `pump()`, then if
`fas_queue[i].read_idx != next_write_idx`, consume one entry and
add `steps == 0 ? ticks : ticks * steps` to a per-axis clock.
Assert issued step sums equal the target and
`|clock_0 - clock_1| <= 2` at the end. Command bytes need not
match `SimPort`. Dir-change getters on this `init(NULL, …)` path
are 0, so no DIR pauses are required here.

Plots: `test_26_f1_fas.gnuplot`, `test_26_f2_fas.gnuplot`,
`start_scalar` of position vs time is enough.

**Done when:** F1 and F2 step sums match and the SimPort half of
`test_26` is still green.

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
