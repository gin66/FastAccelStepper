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
