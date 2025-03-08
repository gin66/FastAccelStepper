#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "FastAccelStepper.h"
#include "StepperISR.h"

char TCCR1A;
char TCCR1B;
char TCCR1C;
char TIMSK1;
char TIFR1;
unsigned short OCR1A;
unsigned short OCR1B;

StepperQueue fas_queue[NUM_QUEUES];

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

#include "RampChecker.h"

class FastAccelStepperTest {
 public:
  void init_queue() {
    fas_queue[0].read_idx = 0;
    fas_queue[1].read_idx = 0;
    fas_queue[0].next_write_idx = 0;
    fas_queue[1].next_write_idx = 0;
  }

  void ramp(uint8_t forward_planning, uint32_t expected_steps) {
    init_queue();
    FastAccelStepper s = FastAccelStepper();
    s.init(NULL, 0, 0);
    RampChecker rc = RampChecker();
    assert(0 == s.getCurrentPosition());

    uint32_t speed_us = 1000000 / 3600;
    assert(s.isQueueEmpty());
    s.setSpeedInUs(speed_us);
    s.setAcceleration(320);
    s.setForwardPlanningTimeInMs(forward_planning);
    s.fill_queue();
    assert(s.isQueueEmpty());
    float old_planned_time_in_buffer = 0;

    char fname[100];
    float sum_planning_time = 0;
    float points = 0;
    snprintf(fname, 100, "test_15_%dms.gnuplot", forward_planning);
    rc.start_plot(fname);
    s.runForward();
    for (int i = 0; i < 2000; i++) {
      if (i == 1000) {
        printf("Change speed\n");
        s.setSpeedInUs(10000);
        s.applySpeedAcceleration();
      }
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
      uint32_t from_dt = rc.total_ticks;
      while (!s.isQueueEmpty()) {
        rc.increase_ok = true;
        rc.decrease_ok = true;
        rc.check_section(
            &fas_queue[0].entry[fas_queue[0].read_idx & QUEUE_LEN_MASK]);
        fas_queue[0].read_idx++;
      }
      uint32_t to_dt = rc.total_ticks;
      float planned_time = (to_dt - from_dt) * 1.0 / 16000000;
      printf("planned time in buffer: %.6fs\n", planned_time);
      sum_planning_time += planned_time;
      points += 1.0;
      // This must be ensured, so that the stepper does not run out of
      // commands
      assert((i == 0) || (old_planned_time_in_buffer > 0.005));
      old_planned_time_in_buffer = planned_time;
      // stop after
      if (rc.total_ticks > TICKS_PER_S * 40) {
        break;
      }
    }
    rc.finish_plot();
    // test(!s.isRampGeneratorActive(), "too many commands created");
    printf("current position = %d\n", s.getCurrentPosition());
    test(s.getCurrentPosition() > expected_steps - 10, "stepper runs too slow");
    test(s.getCurrentPosition() < expected_steps + 10, "stepper runs too fast");
    printf("Total time  %f\n", rc.total_ticks / 16000000.0);
    float avg_time = sum_planning_time / points * 1000.0;
    printf("Average planning time: %f ms\n", avg_time);
    test(avg_time < forward_planning + 1, "too much forward planning");

#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
    printf("CHECKSUM for %d/%d/%d: %d\n", steps, travel_dt, accel, s.checksum);
#endif
  }
};

void tc_2() {
  int8_t ret;
  uint32_t actual;
  int64_t actual_sum = 0;
  char fname[100];

  FastAccelStepper s = FastAccelStepper();
  s.init(NULL, 0, 0);
  RampChecker rc = RampChecker();
  snprintf(fname, 100, "test_16_tc_2.gnuplot");
  rc.start_plot(fname);

  ret = s.moveTimed(QUEUE_LEN / 2 - 1, QUEUE_LEN / 2 * 100000, &actual);
  test(ret == MOVE_TIMED_EMPTY, "TC2_S1: valid pars");
  actual_sum += actual;

  ret = s.moveTimed(2, 200000, &actual);
  printf("queue = %d\n", s.queueEntries());
  test(ret != MOVE_TIMED_OK, "TC2_S2: move should be rejected");
  test(ret == MOVE_TIMED_BUSY, "TC2_S3: queue should be full");

  // process commands
  while (!s.isQueueEmpty()) {
    rc.increase_ok = true;
    rc.decrease_ok = true;
    rc.check_section(
        &fas_queue[0].entry[fas_queue[0].read_idx & QUEUE_LEN_MASK]);
    fas_queue[0].read_idx++;
  }
  ret = s.moveTimed(2, 200000, &actual);
  test(ret == MOVE_TIMED_EMPTY, "TC2_S4: valid pars");
  actual_sum += actual;

  // process commands
  while (!s.isQueueEmpty()) {
    rc.increase_ok = true;
    rc.decrease_ok = true;
    rc.check_section(
        &fas_queue[0].entry[fas_queue[0].read_idx & QUEUE_LEN_MASK]);
    fas_queue[0].read_idx++;
  }
  rc.finish_plot();

  uint64_t expected_ticks = QUEUE_LEN / 2 * 100000 + 200000;
  printf("current position = %d,  total_ticks=%" PRIu64 "\n",
         s.getCurrentPosition(), rc.total_ticks);
  test(s.getCurrentPosition() == QUEUE_LEN / 2 + 1, "TC2_S5: step mismatch");
  printf("expected ticks=%" PRIu64 ", actual=%" PRIu64 "\n", expected_ticks,
         actual_sum);
  test(rc.total_ticks == actual_sum, "TC2_S6: time algnment");
  int64_t drift = actual_sum - expected_ticks;
  printf("drift=%" PRId64 "\n", abs(drift));
  test(abs(drift) < 10, "accepted drift");
}

int main() {
  int8_t ret;
  uint32_t actual;

  FastAccelStepperTest test;

  puts("few simple tests");
  FastAccelStepper s = FastAccelStepper();
  s.init(NULL, 0, 0);

  ret = s.moveTimed(QUEUE_LEN * 255 + 1, 1000, &actual);
  test(ret == MOVE_TIMED_TOO_LARGE_ERROR,
       "TC1_1: too many steps for the queue");

  ret = s.moveTimed(QUEUE_LEN + 1, 65536 * (QUEUE_LEN + 1), &actual);
  test(ret == MOVE_TIMED_TOO_LARGE_ERROR,
       "TC1_2: too many steps for the queue");

  ret = s.moveTimed(1, MIN_CMD_TICKS - 1, &actual);
  test(ret == AQE_ERROR_TICKS_TOO_LOW, "TC1_3: too short duration");

  ret = s.moveTimed(100, 100 * (s.getMaxSpeedInTicks() - 1), &actual);
  test(ret == AQE_ERROR_TICKS_TOO_LOW, "TC1_4: still too short duration");

  ret = s.moveTimed(QUEUE_LEN / 2 + 1, (QUEUE_LEN / 2) * 100000, &actual);
  test(ret == MOVE_TIMED_TOO_LARGE_ERROR, "TC1_S5: too many commands");

  tc_2();

  printf("TEST_16 PASSED\n");
  return 0;
}
