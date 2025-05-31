#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "FastAccelStepper.h"
//#include "StepperISR.h"

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

#define SUPPORT_ESP32_RMT
#define SUPPORT_EXTERNAL_DIRECTION_PIN
#define IRAM_ATTR
#define RMT_CHANNEL_T int
#define LL_TOGGLE_PIN(dirPin)
#include "StepperISR_esp32xx_rmt.cpp"

StepperQueue fas_queue[NUM_QUEUES];

struct TestCase {
  uint8_t forward_planning;
  int32_t position;
  uint32_t speed_us;
  uint32_t acceleration;
  // check step rate between from_step and to_step
  bool check_step_rate;
  uint32_t from_step;
  uint32_t to_step;
  uint32_t period_ticks;
};

class FastAccelStepperTest {
 public:
 uint32_t offset;
 uint32_t rmt_entries[1000000];

  void init_queue() {
    fas_queue[0].read_idx = 0;
    fas_queue[0].next_write_idx = 0;
  }

  void ramp(TestCase tc) {
    printf("Running ramp test with forward_planning=%d, position=%d, speed_us=%d, "
           "acceleration=%d\n",
           tc.forward_planning, tc.position, tc.speed_us, tc.acceleration);
    init_queue();
    offset = 0;
    FastAccelStepper s = FastAccelStepper();
    s.init(NULL, 0, 0);
    assert(0 == s.getCurrentPosition());

    assert(s.isQueueEmpty());
    s.setAbsoluteSpeedLimit(80);
    s.setSpeedInUs(tc.speed_us);
    s.setAcceleration(tc.acceleration);
    s.setForwardPlanningTimeInMs(tc.forward_planning);
    s.fill_queue();
    assert(s.isQueueEmpty());
    float old_planned_time_in_buffer = 0;

    s.moveTo(tc.position);
    for (int i = 0; i < 100000; i++) {
      if (true) {
        printf(
            "Loop %d: Queue read/write = %d/%d    Target pos = %d, Queue End "
            "pos = %d  QueueEmpty=%s\n",
            i, fas_queue[0].read_idx, fas_queue[0].next_write_idx,
            s.targetPos(), s.getPositionAfterCommandsCompleted(),
            s.isQueueEmpty() ? "yes" : "no");
      }
      if (!s.isRampGeneratorActive()) {
        break;
      }
      s.fill_queue();
      if (!s.isQueueEmpty()) {
        if (offset+PART_SIZE > sizeof(rmt_entries) / sizeof(rmt_entries[0])) {
          printf("RMT buffer overflow\n");
          exit(1);
        }
        rmt_fill_buffer(&fas_queue[0], true, &rmt_entries[offset]);
        offset += PART_SIZE;
      }
    }
    while (!s.isQueueEmpty()) {
      if (offset+PART_SIZE > sizeof(rmt_entries) / sizeof(rmt_entries[0])) {
        printf("RMT buffer overflow\n");
        exit(1);
      }
      rmt_fill_buffer(&fas_queue[0], true, &rmt_entries[offset]);
      offset += PART_SIZE;
    }
    // test(!s.isRampGeneratorActive(), "too many commands created");
    printf("current position = %d, target position %d\n", s.getCurrentPosition(), tc.position);
    test(s.getCurrentPosition() == tc.position, "target position not reached");

    evaluate_rmt_entries(tc);
  }

  void evaluate_rmt_entries(TestCase tc) {
    printf("RMT entries: %d\n", offset);
    uint64_t ticks = 0;
    #define MAX_STEPS 100000
    uint32_t steps_time[MAX_STEPS];
    uint32_t steps = 0;
    bool step_high = false;
    for (uint32_t i = 0; i < offset; i++) {
      printf("Entry %d: %08" PRIx32 " steps=%" PRIu32 "\n", i, rmt_entries[i], steps);
      uint32_t entry = rmt_entries[i];
      for (int j = 0; j < 2; j++) {
        uint16_t subentry = (j==0) ? entry & 0xffff : entry >> 16;
        if (((subentry & 0x8000) != 0) && !step_high) {
          // this is a step transition
          step_high = true;
          steps++;
          if (steps < MAX_STEPS) {
            steps_time[steps] = ticks;
          } else {
            printf("Too many steps recorded\n");
            exit(1);
          }
        }
        else if ((subentry & 0x8000) == 0) {
          step_high = false;
        }
        // advance ticks
        ticks += subentry & 0x7fff;
      }
    }
    test(steps == tc.position, "number of steps does not match target position");
    float t = ticks / (float)TICKS_PER_S;
    printf("Total ticks: %" PRIu64" = %.3fs, steps: %d\n", ticks, t, steps);

    if (tc.check_step_rate) {
      // check step rate between from_step and to_step
      for (uint32_t i = tc.from_step; i < tc.to_step; i++) {
        if (i+1 >= steps) {
          printf("to_step out of range\n");
          exit(1);
        }
        uint32_t dt = steps_time[i+1] - steps_time[i];
        if (dt != tc.period_ticks) {
          printf("Step %d: dt=%" PRIu32 " != %" PRIu32 " ticks\n", i, dt, tc.period_ticks);
        }
        test(dt == tc.period_ticks, 
             "step rate does not match expected period ticks");
      }
    }
  }
};

int main() {

  FastAccelStepperTest test;
  // run the ramp twice with 20 and with 5ms planning time.
  // the ramp will change speed after half of the loops.
  // The 5ms ramp will not have 20ms coasting in the buffer and as such runs
  // much shorter.
  TestCase test_cases[] = {
      {.forward_planning = 20, .position=1000, .speed_us = 1000, .acceleration = 320}, 
      {.forward_planning = 5, .position=5000, .speed_us = 1000, .acceleration = 320,
        .check_step_rate=true, .from_step=1600, .to_step=3400, .period_ticks=16000},
      {.forward_planning = 5, .position=5000, .speed_us = 5, .acceleration = 10000000,
        .check_step_rate=true, .from_step=2000, .to_step=3000, .period_ticks=80},
  };
  for (int i = 0; i < sizeof(test_cases) / sizeof(test_cases[0]); i++) {
    test.ramp(test_cases[i]);
  }
  printf("TEST_17 PASSED\n");
  return 0;
}