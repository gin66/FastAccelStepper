# naxes — three-axis FasNAxis example

A hardware example for the multi-axis planner `FasNAxis`
(see `extras/doc/n_axes_whitepaper.md`). One `FastAccelStepper` drives each
axis independently (X, Y, Z — no gantry); a single `FasNAxis` planner feeds all
axis queues from one polyline so the axes stay time-synchronized.

## The path

The path is fixed and deterministic, so a run is comparable to the next:

    helix      →  hexagon      →  square      →  return to origin

* **helix** — `NAXES_HELIX_TURNS` full revolutions in the XY plane while Z
  climbs `NAXES_HELIX_Z_PER_TURN` per turn, sampled from a quarter-sine table
  (`naxes_path.h`, float-free).
* **hexagon** — six corners of a regular hexagon in the XY plane.
* **square** — four corners of an axis-aligned square in the XY plane (the
  cube footprint on a two-axis build).
* **return to origin** — a direct line back to the start position.

The planner runs in **Linear** mode: each `addLine()` target is an exact-chord
vertex, so the realized curve is the polygon through the vertices. The geometry
detector reconstructs that polygon from the `Step`/`Dir` pin traces.

## Axis count

A compile-time switch (`NAXES_HW`) drops the third axis on the ATmega168/328/
328p, which expose only two step channels (OC1A / OC1B). Every other platform
wires three axes:

```
#if defined(__AVR_ATmega328P__) || __AVR_ATmega328__ || __AVR_ATmega168__
#define NAXES_HW 2
#else
#define NAXES_HW 3
#endif
```

## Pin configuration

Following the StepperDemo pattern, the pin table is selected by architecture via
`StepperPins_naxes_<plat>.h` (avr, sam, pico, esp32). Each header defines the
`naxes_config_0[]` array of `stepper_config_s` (from `StepperConfig.h`), with
`NAXES_HW` valid entries plus the `STEPPER_CONFIG_END` sentinel. Enable is
low-active and set manually so it is settled before the planner kicks off
(whitepaper §4.5).

## Kick-off synchronization

The planner is constructed with the engine
(`FasNAxis<NAXES_HW, NAXES_HORIZON> naxes_planner(FasNAxisConfig{}, engine)`)
and its kick-off uses the engine's synchronized start: when a committed path
is released, every active axis queue is started by one engine operation
instead of one `addQueueEntry(NULL, true)` per axis, so the axes share a
single start event. Today each platform implements that operation as a single
critical section around the per-stepper starts; platform-specific mechanisms
(see `extras/todo/engine_synchronized_start.md`) replace it as they are
written.

* **CI / PlatformIO** — `pio_dirs/naxes/` is a symlink wrapper (like
  `pio_dirs/MoveTimed/`): `platformio.ini` → `extras/ci/platformio.ini`,
  `FastAccelStepper/src` → the library, and `src/naxes.ino` → this sketch.
  The example is built for every architecture in `extras/ci/build_matrix.yaml`.
* **SimAVR** — `extras/tests/simavr_based/test_naxes/` builds this example for
  the ATmega328p (two axes), runs it to completion under `run_avr`, and the
  geometry detector validates the reconstructed helix/hexagon/square/origin
  curve.
* **PC** — the planner itself is covered by `extras/tests/pc_based/test_26`.
