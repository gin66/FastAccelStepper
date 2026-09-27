# NaxesTimed — two-axis FasTimed example

A hardware sketch for `FasTimed`, the timed-trajectory planner. The call list is
[FasTimed.md](../../extras/doc/FasTimed.md); the design record is
[timed_trajectory.md](../../extras/doc/implemented/timed_trajectory.md).

`FasTimed` does not invent a ramp between waypoints. The caller submits one
constant rate per chunk and a shared duration in driver ticks. This example
shows how a caller draws a ramp on top of that: a pitch ladder that starts
and ends each side at rest.

## The path

Both axes move together on every chunk, so the head follows a 45-degree
diamond:

    (+1,+1)  ->  (-1,+1)  ->  (-1,-1)  ->  (+1,-1)  ->  home

Each of the four sides plays `NaxesTimed_ramp.h` once: it rises from about
440 Hz to 2500 Hz in eight coarse periods, then falls back to a stop in
thirty-one ramp-step moves. Because `|dx| == |dy|` on every chunk, both axes
see the same step count and the same ramp-step, so the diagonal stays at 45
degrees and the axes share one clock.

The pitch ladder is the one `extras/tests/pc_based/test_29` renders as
`test_29.wav`; its `RampMap` does not depend on the platform's
`getMaxSpeedInTicks()` as long as the periods stay well above it, so the same
table is legal on AVR, Pico, SAM and ESP32.

Set each stepper's acceleration to `kNaxesTimedAccel` (100000 steps/s²) before
`addAxis`. The periods were chosen for that acceleration at 16 MHz ticks; a
different acceleration makes `addDelta` return `TimingNotAchievable`.

`FasTimed` ignores the configured speed limit: only the acceleration feeds the
ramp map, and the caller owns the rate. There is no `setSpeedInHz` call.

## Pin configuration

Following the StepperDemo / [NaxesAFAP](../NaxesAFAP/README.md) pattern, the
pin table is selected by architecture via `StepperPins_NaxesTimed_<plat>.h`
(avr, sam, pico, esp32). Each header defines the `NaxesTimed_config_0[]` array
of `stepper_config_s` (from `StepperConfig.h`) with two axes plus the
`STEPPER_CONFIG_END` sentinel. Enable is low-active and set manually so it is
settled before the planner kicks off (whitepaper §4.5).

On AVR the axes are the Timer 1 channels OC1A / OC1B (digital 9 / 10). Every
other platform uses placeholder GPIOs to be adapted to the actual wiring.

## Direction changes

A reversal needs the axis at ramp-step 0, which the ladder provides at every
corner. A driver that injects its own direction-change pause (ESP32
RMT/MCPWM/I2S) is reported by `pump()` as `TimedStatus::Error`; the
timer-based AVR and Pico drivers set DIR directly and do not inject one. Build
it for those targets to see the full diamond.

## Backpressure

Each chunk is already one queue command, so the ring is small
(`NAXES_TIMED_HORIZON = 8`). `loop()` calls `addDelta` until it returns
`Rejected` (ring full), then `pump()` to drain, and adds again. The
`g_done` flag stops the state machine after the fourth side and lets the
queues drain.

## Build

* **CI / PlatformIO** — `pio_dirs/NaxesTimed/` is a symlink wrapper (see
  `build-pio-dirs.sh`): `platformio.ini` → `extras/ci/platformio.ini`,
  `FastAccelStepper/src` → the library, and `src/NaxesTimed.ino` → this
  sketch.
* **PC** — the planner itself is covered by `extras/tests/pc_based/test_29`.
