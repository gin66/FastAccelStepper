Tests;

- test_01
  check queue functionality

- test_02
  checks ramp timing

- pmf_test
  checks Log2Representation implementation

- test_04
  one test case with speed change during ramp

- test_05
  check for move/moveTo while ramp is processing
  Introduce concept of interrupt generation during noInterrupts call

- test_06
  check for stop during move

- test_07
  test case with varying speed

- test_08
  ramp to max speed with step wise increased length

- test_09
  simple test case for V30 A1000000 R53 W R53

- test_10
  test case for V30 a17164 w2000 a-1000

- test 11
  test case for M1 A1000 V10000 f w300 V100000 U
  This is stuck in state RED.
  Revised test: M1 A1000 V10000 P100 w300 V100000 U

- test 12
  ramp up with 1 step/s^2 to 1000us/step

- test 13
  tests with maximum high acceleration

- test 14
  test case for issue #178: Speed jump instead of decrease

- test 15
  test case for setForwardPlanningTimeInMs()

- test 16
  tests for moveTimed()

- test 17
  test to specifically test for esp32 rmt implementation