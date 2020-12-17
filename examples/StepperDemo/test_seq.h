#include "FastAccelStepper.h"
struct test_seq_s {
  bool (*test)(FastAccelStepper *stepper, struct test_seq_s *,
               uint32_t time_ms);
  uint16_t state;
  uint32_t u32_1;
};

// a test_seq should return true, if finished.

// This sequence turns stepper like a clock's second hand
bool test_seq_01(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);
bool test_seq_02(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);
bool test_seq_03(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);
bool test_seq_04(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);
bool test_seq_05(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);
