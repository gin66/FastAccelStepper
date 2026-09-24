# naxes hardware / simavr validation

Status: open.

## Motivation

The FasNAxis design is currently validated only on the host
(`extras/tests/pc_based/test_26.cpp`). A theoretical design is only
worth something once it runs on a target. simavr is accepted as an
adequate stand-in for hardware. Listening to real steppers is enough to
spot algorithm-level bugs; a multi-axis test bed is not required.

## Deliverables

- `examples/naxes/`: 3-axis FasNAxis sketch using one
  `FastAccelStepper` per axis (individual steppers, no gantry). Pin
  assignment reuses the `examples/StepperDemo` concept. A compile-time
  switch disables the third axis on ATmega328p.
- Fixed path sequence, comparable run to run: **helix → hexagon →
  square/cube → return to origin**.
- CI: a `pio_dirs/naxes/` wrapper (symlink `platformio.ini` to
  `extras/ci/platformio.ini`, `src/` to the example) so the build matrix
  in `extras/ci/build_matrix.yaml` compiles the example on every
  architecture.
- simavr: a test under `extras/tests/simavr_based/` that links the
  example, runs to completion, and reports no planner failure or queue
  underrun.
- **New detector**: reconstruct the generated curve from the
  `StepA/B/C` and `Dir` pin traces and validate the geometry. The
  existing `eval.awk` / `judge.awk` only compare timings/positions
  against `expect.txt`, which is insufficient for a multi-axis curve.

## Whitepaper consequences

- Remove/replace §3.2 non-goal "Hardware-in-the-loop or simavr coverage
  for FasNAxis".
- Revise §12.1 ("Why PC exclusive" / "None of that needs an MCU"). The
  whitepaper should not argue at length about validation; it should
  describe the design, and validation belongs here.
