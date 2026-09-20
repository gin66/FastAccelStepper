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
   No Isabelle. Theory is probed in steps 2b and 3b.
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

## Step 2 — remaining-steps scan `R` (lookahead)

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

Integer add/sign only. The 2° collinear test of §8.2 lives here:

```
(Δ · Δ')² * 100000  >=  99878 * |Δ|² * |Δ'|²
```

Also compute `P_stop_i = calculate_ramp_steps(ticks_i_cfg)` and
assert `LookaheadTooShort` logic (path open ⇒ `R_i ≥ P_stop_i`)
on a fixture that only feeds 800 steps with `P_stop = 4000`.

**Implement:** `src/fas_naxis/remaining.h` (or methods on the class).
No queue I/O yet.

**Plot:** `test_26_f10.gnuplot` — `R(t)` as blocks are consumed;
must not drop to 100 at every micro-segment.

**Done when:** table above is green; F19 kernel (`HORIZON` /
`R < P_stop`) fails closed with a hint that names acc/vel.

---

## Step 2b — theory probes for `R`, binder, collinear (no queues)

These must fail on a wrong *model*, even if later F-fixtures are
green. Oracle is a small pure function, not FasNAxis state.

1. **Reference oracle.** Given waypoints + `ticks_cfg` / accel,
   compute `R`, `P_stop`, Linear binder (longest `|Δ|`, rebind if
   `|Δ_i|*ticks_i > |Δ_b|*ticks_b`), DDA step counts to the
   vertex. The planner must match this function on the same
   inputs.
2. **Exhaustive tiny Linear.** All 2-axis polylines with
   `|Δ_i| ≤ 5`, 3–4 vertices, two or three `ticks_cfg` ratios.
   Vertices exact; no axis exceeds its envelope; `P ≤ R` is
   checked from the *issued step counts*, not from planner
   fields.
3. **Rebind neighbourhood.** `|Δ_x| ∈ {99,100,101}`,
   `ticks_y/ticks_x` in `{1, 2, 99/100}` (integer ticks, not
   a `/` in production). Especially `|Δ_x|*ticks_x ≈
   |Δ_y|*ticks_y`.
4. **Collinear boundary.** 1° must pass, 2° is the documented
   edge, 3° and 90° must stop. Include n=3 with one tiny
   component.

**Done when:** exhaustive set is green; disabling the rebind
rule in a local `#if 0` makes the neighbourhood fail.

---

## Step 3 — ramp law `P` vs `R` (still no queues)

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

---

## Step 3b — stoppability from the command trace

Ignore planner `P`. From issued periods and the leftover
polyline, `calculate_ramp_steps(current_ticks) ≤ remaining`
on every axis at every sample (can still stop).

**Mutations** (must be documented in the test file): disable
cross-block `R` → F10 fails; disable rebind → F18 fails;
disable vertex snap → F5 misses `(1600,0)`.

**Done when:** the trace oracle is green on F1/F5/F10, and the
three mutations are listed as comments (or `#if` hooks) next
to those fixtures.

---

## Step 4 — `SimPort` `addQueueEntry` contract

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

---

## Step 5 — header skeleton + `addAxis` / position

**Test first (F16):**

- `FasNAxisConfig{}` has `dt_ticks=32000`, `kappa_stop_q8=320`,
  `overshoot_max=8`, `mode=Linear`. `dt_ticks==0` in a raw struct
  still becomes 32000 in the constructor.
- `addAxis` fails if `i >= NAXES`, if the pointer is null, or if
  `isRampGeneratorActive()` / `isRunning()`.
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
- F18 `(10000, 9000)` with Y 40× slower: longest is X but Y would
  exceed `v_max` if scaled to X; Y binds, X scaled down
  (`|Δ_i|*ticks_i_cfg` compare).
- F10 100-step micro-segments totalling 10000: `R` sees through;
  no rest at each joint (collinear test).

**Implement:** block ring `HORIZON`, `R` scan of §8.1 + Linear
path-stop of §8.2, commit rule of §8.4.

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
faster than the ramp, no delayed start.

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

## Step 13 — dwell, starve, underrun (F11, F13)

**Test first:**

- `addDwellTicks` mid-path: planned stop, wait, continue from
  rest. Not a pause command at speed.
- F11: dribble waypoints so `R < P_stop` while the path is still
  open. **Not** a silent feed-hold: `pump()` returns
  `LookaheadTooShort`, `lookaheadHint()` names acc/vel/`HORIZON`,
  `P ≤ R`. After enough waypoints, cruise is allowed again.
- F13: starve `pump()` on purpose after kick-off; `hasUnderrun()`;
  plot still written up to the fault.
- F19: `HORIZON` too small to ever hold `P_stop`; error at
  `addAxis` / first `pump`, same hint.

**Implement:** zero-displacement blocks; §8.5 lookahead error;
underrun flag after kick-off only.

**Plots:** `test_26_f11.gnuplot` (v(t) capped then recovers),
`test_26_f13.gnuplot` (cut off at underrun event).

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
| 2b, 3b | P0 | theory probes (oracle, exhaustive, rebind, collinear, trace stoppability) |
| 4–9 | P1 | Linear through `addQueueEntry`; F18 rebind |
| 10 | P2 | gnuplot always; HTML optional |
| 11–12 | P3 | Overshoot |
| 13 | P4 | lookahead error / underrun |
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
- Inverse kinematics
- Running `pump()` from `manageSteppers()`
