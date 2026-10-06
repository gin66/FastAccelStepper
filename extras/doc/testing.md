# Test Strategy

[Back to README](../../README.md)

The library is tested with different kind of tests:

* PC only (sub folder `../tests/pc_based`)

  These tests focussing primarily the ramp generator and part of the API
* simavr based for avr (sub folder `../tests/simavr_based`)

  The simavr is an excellent simulator for avr microcontrollers. This allows to
  check the avr implementation thoroughly: number of steps generated, virtual
  stepper position and even timing. Tested code is mainly the StepperDemo, which
  gets fed in a one line sequence of commands to execute. These tests are focused
  on avr, but help to check the whole library code, used by esp32, too.
* esp32 tests with another pulse counter attached (e.g. `test_seq_08` in StepperDemo)

  The FastAccelStepper-API supports to attach another free pulse counter to a
  stepper's step and dir pins. This counter counts in the range of -16383 to
  16383 with wrap around to 0. The test condition is, that the library's view of
  the position should match the independently counted one. These tests are still
  evolving
* Test for pulse generation using examples/Pulses

  This has been intensively used to debug the esp32 ISR code
* esp32 hw tests

  These tests live under sub folder `../tests/esp32_hw_based`
* Saleae-based hardware verification (sub folder `../tests/saleae_based`)

  Captures the step and dir pins with a logic analyzer as the oracle, so what is
  measured is the waveform that comes out of the MCU on real silicon: pulse
  high time, dir→step timing, spurious or swallowed steps, cross-stepper skew,
  and how closely the emitted step rate follows the commanded one. Runs on
  ESP32, AVR and RP2040/RP2350. The measured results are
  [ESP32 platform matrix](../tests/saleae_based/reports/esp32_platform_matrix.md)
  and
  [RP2350 platform matrix](../tests/saleae_based/reports/rpipico2_platform_matrix.md);
  the harness itself is documented in
  [saleae_based/README.md](../tests/saleae_based/README.md).

  **Virtual I2S decoder — 32 steppers on 3 wires, decoded by software.** The
  ESP32 I2S peripheral can multiplex up to 32 stepper signals into one 32-bit
  word, sent repeatedly on three physical wires (data, bit-clock, word-select).
  A Saleae logic analyzer captures only those three wires (plus any additional
  physical stepper channels), and a **software decoder** (`scripts/i2s_mux_decoder.py`)
  reconstructs the 32 individual stepper channels from the time-multiplexed
  stream. No extra hardware is needed beyond the three bus wires — the decoding
  is entirely in software.

  How it works:

  1. The firmware brings up the I2S multiplexer (`IMUX` command). The bus
     occupies the **last three** analyzer channels (D5 = data, D6 = bclk,
     D7 = ws); the remaining channels carry any physical steppers.
  2. The ESP32 sends a 32-bit word every I2S frame: each bit position
     corresponds to one stepper's STEP signal (and a second bit for DIRECTION
     when in `dir` mode). The word is two 16-bit halves, low half first.
  3. `i2s_mux_decoder.py` reads the 8-channel `.sr` capture, extracts the
     bclk rising edges, samples the data line one sample before each edge
     (to avoid the ESP32's non-50 % clock duty), and reconstructs the 32
     slot values. It writes a VCD with 37 channels — 5 passthrough + 32
     decoded slots — which the ordinary evaluators read with no mux-specific
     code.

  Key constraints (measured on hardware):

  * **Sample rate:** 24 MS/s is the floor (3 samples per 8 MHz bit-clock
    period). 48 MS/s is unusable on this analyzer because it truncates an
    8-channel capture to 0.18 ms.
  * **Channel budget:** a multiplexed stepper costs a **bit of the 32-bit
    word, not an analyzer channel**. In `nodir` mode up to 32 steppers can
    be tested on 8 analyzer channels (5 physical + 3 bus); in `dir` mode
    up to 16 (a direction bit uses a second bit of the same word).
  * **Frame-quantized periods:** a multiplexed step can only start on an
    I2S frame boundary, so its period is a discrete set of frame counts
    rather than a continuous value. The decoder and evaluators handle this
    via `signal_parser.grid_period_defects()`.

  The full design and measured results are in
  [r7_virtual_i2s_mux.md](../implemented/r7_virtual_i2s_mux.md). The harness
  integrates it through `--driver i2s_mux --imux` on `harness.py` and
  `--mode scale --driver i2s_mux --pin-mode nodir` for a full 1…32 sweep.
* manual tests using examples/StepperDemo

  These are unstructured tests with listening to the motor and observing the behavior

## Test sequences from StepperDemo

Short info, what the test sequences, embedded in StepperDemo in the test mode, do:

- 01 - Run the stepper like a clock for one minute
- 02 - Run the stepper towards positive position and back to zero repeatedly
- 03/04 - same like 02, both different speed/acceleration
- 05 - Perform 800 times a single step and then 800 steps back in one command
- 06 - Run 32000 steps with speed changes every 100ms in order to reproduce issue #24

All those tests have no internal test passed/failed criteria. Those are used for
automated tests with `../tests/simavr_based` and `../tests/esp32_hw_based`. The
test pass criteria are: They should run smoothly without hiccups and return to
start position.

- 07 - measures timing of several moveByAcceleration(). Should stop at position
  zero. (should be started from position 0).
- 08 - is an endless running test to check on esp32, if the generated pulses are
  successfully counted by a second pulse counter. The moves should be all
  executed in one second with alternating direction and varying speed/acceleration
- 09 - is an endless running test with starting a ramp with random
  speed/acceleration/direction, which after 1s is stopped with
  `forceStopAndNewPosition()`. It contains no internal test criteria, but looking
  at the log, the match of generated and sent pulses can be checked. And the
  needed steps for a `forceStopAndNewPosition()` can be derived out of this
- 10 - runs the stepper forward and every 200 ms changes speed with increasing
  positive speed deltas and then decreasing negative speed deltas.
- 11 - runs the stepper to position 1000000 and back to 0. This tests, if
  `getCurrentPosition()` is counting monotonously up or down respectively.
- 14 - ESP32 pulse counter only. Replays the Issue370 sampled stroke via
  `moveTimed()` (motor running). Pass if PCNT is -592 and the ramp finishes in < 1s.

## Running the tests

See the [installation documentation](installation.md) and the
`Makefile`s under `../tests/pc_based` and `../tests/simavr_based`.
