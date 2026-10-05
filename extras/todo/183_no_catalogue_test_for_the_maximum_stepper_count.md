# 183 No catalogue test steps the maximum number of steppers

## Priority

**HIGH** — the catalogue cannot answer "how many steppers does this driver
drive?", which is the one question a driver answers by refusing, and it is the
question whose answer is a hardware fact rather than a constant in a header.

## Finding

The SR catalogue's largest stepper count is **three**. `CONFIGS` tops out at
`2ch`, and SR_14/15/16/17 are 2–3 steppers. Nothing in `ALL_TESTS` runs at a
driver's maximum, on any driver.

The only thing that reaches a high count is `--mode scale`, which sweeps
n = 1…N. It is not in `ALL_TESTS`, has no SR number, and cannot be reached
through `--tests SR_xx`. So it is not part of the characterization set, is not
tagged per-test, and does not appear in a matrix row as its own result.

This is not a cosmetic gap. It is the mechanism by which
182 (closed as not reproducible, see `README.md` → Done) was filed with "zero
measurements above n = 8" and stayed open through a full six-row matrix run: a
defect that only appears past n = 8 cannot be caught by a catalogue whose
largest case is 3.

## What a test has to be

`nodir`, one driver named per stepper, at **the driver's own maximum**:

- `i2s_mux` — **32**, bounded by the 32-bit word (a stepper costs one bit and no
  analyzer channel)
- a GPIO driver — **min(its queue count, 8)**, bounded by the analyzer channels
  and by what the library can allocate (6 for MCPWM/PCNT on ESP32)

The count comes from the bound, never from a literal — the same rule as
`MAX_STEPPERS_PER_MODE`. One shared program, all steppers at the same commanded
period.

Evaluated as `eval_scale` already is: **each stepper's measured step count and
its period, from the capture**. That is the load-bearing part. Asserting the
board's own `POS` tally instead would pass on a driver emitting complete
garbage — the position is the queue's bookkeeping, not the pin.

n+1 is already covered by the existing refusals (`ERR CONFIG n=33 max=32`,
`ERR CONFIG mux n=17 needs 34 slots, max=32`), so the same scenario need not
re-test the boundary.

## What to find

- **Add the scenario to the catalogue**, as the next free SR number, with the
  count derived per driver rather than hardcoded.
- **Two known reasons it may record red**, both independent of the work:
  - [023](023_i2s_mux_dir_phantom_steps_at_24ms.md) — the 24 MS/s sampling race
    makes the mux decode unreliable at high n, so the `i2s_mux` row may fail on
    decode alone.
  - the intermittent dropped pulse (`AGENTS.md`, `r7_virtual_i2s_mux.md` §5).
    **Observed while measuring this item's premise**: a forced
    `--mode scale --driver i2s_mux --pin-mode nodir` sweep to 32 passed 31 of 32
    points, with n = 3 reporting **63 of 64 steps** on stepper C and one
    off-grid period. An immediate re-run of n = 1…4 passed all four, so it is
    the known flake and not a new defect — but a max-count scenario inherits it.

  Adding a permanently-red row is worse than having no row, so **023 first**, or
  land the scenario as `skipped` on mux rows with the reason recorded.

## Note

Found while closing 182 (closed as not reproducible, see `README.md` → Done):
the mux accepts n = 32 and all 32 steppers run, so 182's premise — that the
number could not be *tested* — was right for the wrong reason. The parser was
never the obstacle. **No catalogue scenario steps more than three steppers**,
and `--mode scale`, the only thing that reaches n = 32, sits outside
`ALL_TESTS` with no SR number. That is the obstacle, and this is the item for
it.