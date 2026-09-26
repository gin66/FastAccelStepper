# Move Semantics

[Back to README](../../README.md)

## General behaviour of moves

* The desired end position to move to is set by calls to `moveTo()` and `move()`
* The desired end position is in case of `moveTo()` given as absolute position
* For `move()` the delta is added to the latest desired end position
* The stepper tries to reach the given desired end position as fast as possible
  with adherence to acceleration/deceleration
* If the stepper is e.g. running towards position 1000 and `moveTo(0)` is called
  at position 500, then the stepper will
    1. decelerate, which means it will overshoot position 500
    2. stop and accelerate towards 0
    3. eventually coast for a while and then decelerate
    4. stop

## 32-bit position wraparound

The stepper position is a 32bit integer variable, which wraps around for
continuous movement. Example:

- Assume counting up turns stepper clockwise, and counting down, anti-clockwise.
- Current position is -2.000.000.000, move to 2.000.000.000.
- Apparently the position has to count up, and count should run clockwise.
- Implementation is done via difference of 32bit signed numbers, which can
  overflow (being legit).
- The calculation is then:
      2.000.000.000 - (-2.000.000.000) = 4.000.000.000
- But 4.000.000.000 interpreted as signed 32bit is -294.967.296 => count down,
  turn anti-clockwise. Means the position will count:

```
-2.000.000.000
-2.000.000.001
-2.000.000.002
    :
-2.147.483.647
-2.147.483.648
 2.147.483.647
 2.147.483.646
 2.147.483.645
    :
 2.000.000.000
```

## Pin sharing

* Enable pin sharing: the common pin will be enabled for as long as one motor is
  running + delay off. Every motor will adhere to its auto enable delay, even if
  other motors already have enabled the pin.
* Direction pin sharing: The direction pin will be exclusively driven by one
  motor. If one motor is operating, another motor will wait until the direction
  pin comes available.

See [Usage](usage.md) for auto enable/disable details.
