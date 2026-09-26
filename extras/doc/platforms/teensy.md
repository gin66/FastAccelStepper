# Teensy 4.0/4.1 Platform (EXPERIMENTAL)

[Back to README](../../../README.md) | [Platform index](../hardware.md)

The experimental Teensy 4.0/4.1 port was contributed and brought up on real
hardware by seifemad (GitHub: ARDUTECH0), see
[pull request #374](https://github.com/gin66/FastAccelStepper/pull/374).

## Usage limits

* up to 16 stepper motors: 4 QuadTimer (TMR) modules x 4 channels each
* step/dir pins are plain GPIO (toggled via `digitalWriteFast()`), not restricted
  to specific muxed pins - any digital pin works
* two ISR calls per step (rising + falling edge), each hardware-timed by a
  QuadTimer compare match, the same principle this library's AVR backend uses
* Steppers' command queue depth: 32

## Testing status

Tested on a real Teensy 4.0 with a DM556 industrial stepper driver:

* a single axis was speed-swept up to 200 kHz with no missed steps (verified by
  marking the shaft, since there is no pulse-counter/encoder feedback to check
  this in software)
* all 16 steppers were run simultaneously across all 4 QuadTimer modules with no
  cross-talk between channels
* pulse width/edge timing has not been checked on a scope or logic analyzer yet -
  if you have one, please verify and report back what you find
