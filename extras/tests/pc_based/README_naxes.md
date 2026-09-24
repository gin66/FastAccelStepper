# FasNAxis TDD Suite — test_26

## Overview

test_26 implements a test-driven development (TDD) suite for the FasNAxis
multi-axis stepper planner, built up against the white-paper steps in
extras/doc/n_axes_whitepaper.md.

Each fixture writes `test_26_fN.gnuplot` via `NaxisPlot`.  Failure aborts the
run through the `test()` macro.  Most fixtures use the duck-typed `SimPort`
queue (`naxis_sim_port.h`); the planner arithmetic ones (F1, F2, F2b–F2ref,
F2f–F2h, F3b) are self-contained and compare against `naxis_ref.h`.

## Fixture Summary

| Fixture        | Function                                      | Paper         | What it checks |
|----------------|-----------------------------------------------|---------------|----------------|
| **F1**         | `f1_kernel`, `f3_ramp`                        | 7.1           | Ramp-map identity with the FAS `ramp_config_s`; monotone period in P; single-axis rest-to-rest ramp (coast because `N/2 > P_stop`) |
| **F2**         | `f2_remaining`                                | 8             | Remaining-steps scan `R`; collinear continuation vs Linear path-stop; `P ≤ R`; F10/F19 kernels |
| **F2b**        | `f2b_oracle`                                  | 6.3 / 8.3     | Theory probes: DDA master, rebind neighbourhood, 2° collinear boundary, lookahead speed cap; `make mutations` proves the hooks are load-bearing |
| **F2c**        | `f2c_dda_walk`                                | 6.3 / 9.2     | DDA walk of one block; walked per-axis counts equal `Remaining::dda_steps`; chord invariant |
| **F2d**        | `f2d_linear_one_block`                        | 6.3 / 9.2     | One-block Linear interpolator: issued steps equal deltas, path on the chord, vertex hit |
| **F2e**        | `f2e_issued_periods`                          | 6.3 / 12.4    | P reconstructed from *issued periods* (never planner fields); per-axis envelope; coast standby |
| **F2ref**      | `f2ref_reference`                             | 2ref          | Globally fastest constraint-faithful Linear track; one-block traces match `LinearBlock` |
| **F2f**        | `f2f_two_block`                               | 2f            | Two-block Linear: path-stop vs collinear vs reversal; `LinearPoly` matches the reference |
| **F2g**        | `f2g_exhaustive`                              | 2g            | Exhaustive tiny Linear: every 2-axis polyline with `\|Δ_i\| ≤ 5` (~5.3 M) |
| **F2h**        | `f2h_nblock_vs_f20`                           | 2h            | N-block interpolator matches the F20 reference (same seeded blocks) |
| **F3**         | `f3_ramp`, `f6_linear_sim`                    | 3 / 14.1      | Ramp law P vs R; F3 `(10000,100)` path on the chord, vertex samples |
| **F3b**        | `f3b_stoppability`                            | 3b            | Stoppability from the command trace on F1/F5/F10; three mutation hooks documented |
| **F4 / F4b**   | `f11_overshoot_rest`                          | 4 / 11        | Overshoot rest-to-rest `(10000,100)`: cap 8 (`d² ≤ 64`) and raw cap; lone diagonal |
| **F5**         | `f7_linear_lookahead` (also F3b)              | 5 / 8.5       | Linear square 1600: `P → 0` at every 90° corner, decel on the side, 4 vertex samples |
| **F6 / F6b**   | `f12_overshoot_corners`                       | 6             | Overshoot corners: X continues `P_x ≠ 0` while Y reverses `P_y = 0`; anisotropic `(4000,1)+(0,3999)` |
| **F7**         | `f12_overshoot_corners`                       | 7             | Overshoot circle r=1600: every chord vertex sampled; only the reversing axis has `P = 0`; `d² ≤ 64` |
| **F8**         | `f14_helix`                                   | 8             | 3-axis SimPort helix (r=8000, 240 chords), Linear and Overshoot; vertices, smooth path speed |
| **F9**         | `f7_linear_lookahead`                         | 9             | 45° line with `ticks_x = 10·ticks_y`: X is DDA master, Y scaled down in speed |
| **F10**        | `f2_remaining`, `f7_linear_lookahead`         | 10            | 100×100 collinear micro-segments: `R` sees through, coasts, no per-joint rest |
| **F11**        | `f13_lookahead`                               | 11            | One 800-step chunk caps speed (`isSpeedLimitedByLookahead`); ten chunks recover to `P_stop` |
| **F12**        | `f9_dir_pauses`                               | 12 / 4.4      | DIR pause appended after the last step (period kept, no jump); before (old DIR) + after (new DIR) on that axis only |
| **F12b**       | `f9_dir_pauses`                               | 12            | Overshoot dog-leg: continuing axis keeps its planned steps, no copied DIR pause |
| **F12c**       | `f9_dir_pauses`                               | 12            | An injected pause the plan did not carve → `pump()` `Error`; no tick copied to the other axis |
| **F13**        | `f13_lookahead`                               | 13            | Queue starve after kick-off: `hasUnderrun()` and `pump()` `Underrun`; pre-starve plot |
| **F14**        | `f8_feeder`                                   | 14            | Drift over a 240 000-step move: `|clock_x − clock_y| ≤ 2` |
| **F15**        | `f8_feeder`                                   | 15            | Queue room: `queueEntries() ≤ QUEUE_LEN−2`; move completes on `QUEUE_LEN=16` |
| **F16**        | `f16_skeleton`                                | —             | `addAxis`/`addLine` legality, config defaults, no-op zero delta, SimPort-backed queries |
| **F17**        | `f6_linear_sim`                               | 17            | First fill on an empty queue is not underrun |
| **F18**        | `f7_linear_lookahead`                         | 18            | `(10000,9000)` with Y 40× slower: X DDA master, Y time-law binds, both issue full `\|Δ\|` |
| **F19**        | `f13_lookahead`                               | 19            | Small `HORIZON` caps P below `P_stop`; same HORIZON with one long block coasts |
| **F20**        | `f20_long_polyline`, `f2h_nblock_vs_f20`, `f20_physical_wav` | 2ref / 12.5 | Seeded random walk + collinear half-circle (r=4800, 190 chords) + random walk, ticks `(4000,8000)`: every vertex, envelope, `P ≤ R`, path-stops on the random walk, cruise on the arc, DDA rebind; stereo wav |
| **F21**        | `f21_physical`                                | 3.1 / 13.3    | PhysicalStepper on each axis's SimPort: rotor path vs commanded, speed, force, friction, stall, stereo wav |

## Plots and audio

- Every fixture writes `test_26_fN.gnuplot`; run `gnuplot test_26_fN.gnuplot`
  for the `.png`.  `make clean` deletes `*.gnuplot`.
- Velocity panels plot the **axis speed** (EMA of position delta, or, for the
  F12 carve, the time between consecutive step pulses), never the raw
  per-command step rate — that avoids phantom "pulses" when a command is
  shortened or split.
- The "performed ramp-up" panels use `FasNAxis::performedRampUpAxis(i)` /
  `remainingToStopAxis(i)`: in Overshoot each axis has its own persistent `P`,
  so the binder scalar must not be mirrored onto every axis.
- F21 writes `test_26_f21.gnuplot` (realized path, rotor speed, P/R, period,
  deviation), `test_26_f21_phys.gnuplot` (position, speed, error, force,
  friction, stall from `test_26_f21_{x,y}.dat`), and `test_26_f21.wav` (stereo,
  X left / Y right).
- F20 writes `test_26_f20.gnuplot` (oracle), `test_26_f20_lin.gnuplot`
  (interpolator vs oracle) and `test_26_f20.wav` (stereo, X left / Y right).

## Build Notes

- Links with g++ (not plain gcc) because `physical_stepper.h` uses
  `std::vector`.
- `test_26.cpp` defines `FAS_PHYSICAL_STEPPER_ENABLED` (with `<vector>`
  included before the PC `test` macro).  Only the F20/F21 fixtures attach a
  `PhysicalStepper`; every other fixture keeps the ideal SimPort counter
  bit-identical.
- Compile flags: `-Werror -g -DF_CPU=16000000`.
- `extras/tests/pc_based/prove_mutations.sh` (`make mutations`) rebuilds with
  mutation hooks to prove key probes are load-bearing.

See extras/doc/n_axes_whitepaper.md for the mathematical foundations and
section cross-references, and extras/doc/physical_stepper_whitepaper.md for
the rotordynamic plant used by F20/F21.
