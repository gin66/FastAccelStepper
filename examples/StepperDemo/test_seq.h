#include "FastAccelStepper.h"
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
bool test_seq_06(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);
bool test_seq_07(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);
bool test_seq_08(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);
bool test_seq_09(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);
bool test_seq_10(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);
bool test_seq_11(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms);
