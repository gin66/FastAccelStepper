#include "FastAccelStepper.h"
#include "generic.h"

#define TEST_STATE_ERROR 0xffff
struct test_seq_s {
  bool (*test)(FastAccelStepper *stepper, struct test_seq_s *,
               uint32_t time_ms);
  uint16_t state;
  uint32_t u32_1;
  int32_t s32_1;
  int16_t s16_1;
  int16_t s16_2;
};

#if !defined(__AVR_ATmega32U4__)
#define NUM_TEST_SEQUENCE 13
#else
#define NUM_TEST_SEQUENCE 12
#endif
extern struct test_seq_def_s {
  char code[4];
  bool (*test)(FastAccelStepper *stepper, struct test_seq_s *,
               uint32_t time_ms);
} test_sequence[];

// a test_seq should return true, if finished.

// This sequence turns stepper like a clock's second hand
bool test_seq_01(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);

// Run the stepper towards positive position and back to zero repeatedly
bool test_seq_02(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);

// same like 02, both different speed/acceleration
bool test_seq_03(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);

// same like 02, both different speed/acceleration
bool test_seq_04(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);

// Perform 800 times a single step and then 800 steps back in one command
bool test_seq_05(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);

// Run 32000 steps with speed changes every 100ms in order to reproduce issue
// #24
bool test_seq_06(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);

// measures timing of several moveByAcceleration(). Should stop at position
// zero. (should be started from position 0).
bool test_seq_07(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);

// is an endless running test to check on esp32, if the generated pulses are
// successfully counted by a second pulse counter. The moves should be all
// executed in one second with alternating direction and varying
// speed/acceleration
bool test_seq_08(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);

// is an endless running test with starting a ramp with random
// speed/acceleration/direction, which after 1s is stopped with
// forceStopAndNewPosition(). It contains no internal test criteria, but looking
// at the log, the match of generated and sent pulses can be checked. And the
// needed steps for a forceStopAndNewPosition() can be derived out of this
bool test_seq_09(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);

// runs the stepper forward and every 200 ms changes speed with increasing
// positive speed deltas and then decreasing negative speed deltas.
bool test_seq_10(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);

// runs the stepper to position 1000000 and back to 0. This tests, if
// getCurrentPosition() is counting monotonously up or down respectively.
bool test_seq_11(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);

// test case for one stepper to reproduce issue #103
bool test_seq_12(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);

// test case for one stepper to reproduce issue #113
bool test_seq_13(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);
