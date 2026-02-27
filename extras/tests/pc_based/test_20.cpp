#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

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

static bool test_failed = false;
static const char* failure_msg = NULL;

void test_handler(bool condition, const char* msg) {
  if (!condition) {
    printf("TEST FAILURE: %s\n", msg);
    test_failed = true;
    failure_msg = msg;
  }
}

class FastAccelStepperTest {
 public:
  void init_queue() {
    fas_queue[0].read_idx = 0;
    fas_queue[1].read_idx = 0;
    fas_queue[0].next_write_idx = 0;
    fas_queue[1].next_write_idx = 0;
  }

  void test_issue_motor_stops_at_low_speed() {
    init_queue();
    FastAccelStepper s = FastAccelStepper();
    s.init(NULL, 0, 0);
    RampChecker rc = RampChecker();
    assert(0 == s.getCurrentPosition());

    uint32_t accel = 8164;
    uint32_t speed_hz = 81;
    int32_t target_position = 1720;

    printf("\n=== Test Issue: Motor stops at low speed ===\n");
    printf("acceleration=%u, speed=%u Hz, target=%d\n", accel, speed_hz,
           target_position);

    assert(s.isQueueEmpty());
    s.setSpeedInHz(speed_hz);
    s.setAcceleration(accel);
    s.fill_queue();
    assert(s.isQueueEmpty());

    MoveResultCode result = s.moveTo(target_position);
    test_handler(result == MOVE_OK, "moveTo should succeed");

    int consecutive_empty_queues = 0;
    int max_consecutive_empty = 0;
    bool reached_target = false;
    uint32_t last_pos = 0;
    int steps_moved = 0;

    for (int i = 0; i < 5000; i++) {
      if (!s.isRampGeneratorActive()) {
        printf("Ramp generator inactive at loop %d\n", i);
        break;
      }

      s.fill_queue();

      if (s.isQueueEmpty()) {
        consecutive_empty_queues++;
        if (consecutive_empty_queues > max_consecutive_empty) {
          max_consecutive_empty = consecutive_empty_queues;
        }
      } else {
        consecutive_empty_queues = 0;
      }

      while (!s.isQueueEmpty()) {
        rc.increase_ok = true;
        rc.decrease_ok = true;
        rc.check_section(
            &fas_queue[0].entry[fas_queue[0].read_idx & QUEUE_LEN_MASK]);
        fas_queue[0].read_idx++;
        if (rc.pos != last_pos) {
          steps_moved++;
          last_pos = rc.pos;
        }
      }

      if (s.getPositionAfterCommandsCompleted() >= target_position) {
        reached_target = true;
        printf("Reached target position at loop %d\n", i);
        break;
      }
    }

    printf(
        "Final position = %d, target = %d, steps_moved = %d, "
        "max_empty_cycles = %d\n",
        s.getCurrentPosition(), target_position, steps_moved,
        max_consecutive_empty);

    test_handler(reached_target || s.getCurrentPosition() >= target_position,
                 "motor should reach target position");
    test_handler(max_consecutive_empty < 20,
                 "motor should not stop prematurely due to empty queue");

    if (!test_failed) {
      printf("Test 1 PASSED\n");
    }
  }

  void test_issue_speed_change_from_high_to_low() {
    test_failed = false;
    init_queue();
    FastAccelStepper s = FastAccelStepper();
    s.init(NULL, 0, 0);
    RampChecker rc = RampChecker();
    assert(0 == s.getCurrentPosition());

    uint32_t accel = 8164;
    uint32_t high_speed_hz = 89;
    uint32_t low_speed_hz = 81;
    int32_t target_position = 1720;

    printf(
        "\n=== Test Issue: Speed change from high (%u Hz) to low (%u Hz) ===\n",
        high_speed_hz, low_speed_hz);
    printf("This test demonstrates the reported bug:\n");
    printf("- When speed is reduced from 89Hz to 81Hz during movement,\n");
    printf(
        "- the ramp generator produces invalid ticks (below MIN_CMD_TICKS),\n");
    printf("- causing the motor to stop prematurely.\n\n");

    assert(s.isQueueEmpty());
    s.setSpeedInHz(high_speed_hz);
    s.setAcceleration(accel);
    s.fill_queue();
    assert(s.isQueueEmpty());

    MoveResultCode result = s.moveTo(target_position);
    test_handler(result == MOVE_OK, "moveTo should succeed");

    bool speed_reduced = false;
    int reduce_at_loop = 50;
    int consecutive_empty_queues = 0;
    int max_consecutive_empty = 0;
    uint32_t last_pos = 0;
    int steps_before_reduce = 0;
    int steps_after_reduce = 0;
    bool position_changed_after_reduce = false;

    for (int i = 0; i < 5000; i++) {
      if (!s.isRampGeneratorActive()) {
        printf("Ramp generator inactive at loop %d\n", i);
        break;
      }

      if (!speed_reduced && i == reduce_at_loop) {
        steps_before_reduce = rc.pos;
        printf("At loop %d: Reducing speed from %u Hz to %u Hz (pos=%d)\n", i,
               high_speed_hz, low_speed_hz, rc.pos);
        s.setSpeedInHz(low_speed_hz);
        s.applySpeedAcceleration();
        speed_reduced = true;
      }

      s.fill_queue();

      if (s.isQueueEmpty()) {
        consecutive_empty_queues++;
        if (consecutive_empty_queues > max_consecutive_empty) {
          max_consecutive_empty = consecutive_empty_queues;
        }
      } else {
        consecutive_empty_queues = 0;
      }

      while (!s.isQueueEmpty()) {
        rc.increase_ok = true;
        rc.decrease_ok = true;
        rc.check_section(
            &fas_queue[0].entry[fas_queue[0].read_idx & QUEUE_LEN_MASK]);
        fas_queue[0].read_idx++;
        if (rc.pos != last_pos) {
          if (speed_reduced) {
            steps_after_reduce++;
            if (rc.pos > last_pos + 1) {
              position_changed_after_reduce = true;
            }
          }
          last_pos = rc.pos;
        }
      }

      if (s.getPositionAfterCommandsCompleted() >= target_position) {
        printf("Reached target position at loop %d\n", i);
        break;
      }

      if (i > reduce_at_loop + 100 && consecutive_empty_queues > 10) {
        printf("Motor appears to have stopped after speed reduction\n");
        break;
      }
    }

    steps_after_reduce = rc.pos - steps_before_reduce;

    printf("Final position = %d, target = %d\n", s.getCurrentPosition(),
           target_position);
    printf("Steps before speed reduction: %d\n", steps_before_reduce);
    printf("Steps after speed reduction: %d\n", steps_after_reduce);
    printf("Max consecutive empty queue cycles: %d\n", max_consecutive_empty);

    if (max_consecutive_empty >= 20 || !position_changed_after_reduce) {
      printf("\n*** BUG REPRODUCED ***\n");
      printf(
          "The motor stopped or failed to continue after speed was reduced.\n");
      printf("This matches the reported issue where reducing speed to 81 Hz\n");
      printf("with acceleration 8164 causes the motor to stop.\n");
    }

    test_handler(max_consecutive_empty < 20,
                 "BUG: motor stopped prematurely after speed reduction");
    test_handler(position_changed_after_reduce,
                 "BUG: motor should continue moving after speed reduction");

    if (!test_failed) {
      printf("Test 2 PASSED (bug not reproduced - may be fixed)\n");
    } else {
      printf("Test 2 FAILED (bug reproduced - test is working correctly)\n");
    }
  }

  void test_issue_multiple_speed_changes() {
    test_failed = false;
    init_queue();
    FastAccelStepper s = FastAccelStepper();
    s.init(NULL, 0, 0);
    RampChecker rc = RampChecker();
    assert(0 == s.getCurrentPosition());

    uint32_t accel = 8164;
    int32_t target_position = 1720;

    printf("\n=== Test Issue: Multiple speed changes ===\n");
    printf("Simulating user scenario: 73Hz -> 81Hz -> 89Hz -> 97Hz -> 81Hz\n");

    assert(s.isQueueEmpty());
    s.setSpeedInHz(73);
    s.setAcceleration(accel);
    s.fill_queue();
    assert(s.isQueueEmpty());

    MoveResultCode result = s.moveTo(target_position);
    test_handler(result == MOVE_OK, "moveTo should succeed");

    int consecutive_empty_queues = 0;
    int max_consecutive_empty = 0;
    uint32_t last_pos = 0;

    for (int i = 0; i < 5000; i++) {
      if (!s.isRampGeneratorActive()) {
        printf("Ramp generator inactive at loop %d\n", i);
        break;
      }

      if (i == 20) {
        printf("At loop %d: Setting speed to 81 Hz\n", i);
        s.setSpeedInHz(81);
        s.applySpeedAcceleration();
      } else if (i == 40) {
        printf("At loop %d: Setting speed to 89 Hz\n", i);
        s.setSpeedInHz(89);
        s.applySpeedAcceleration();
      } else if (i == 60) {
        printf("At loop %d: Setting speed to 97 Hz\n", i);
        s.setSpeedInHz(97);
        s.applySpeedAcceleration();
      } else if (i == 80) {
        printf("At loop %d: Setting speed to 81 Hz (problematic per report)\n",
               i);
        s.setSpeedInHz(81);
        s.applySpeedAcceleration();
      }

      s.fill_queue();

      if (s.isQueueEmpty()) {
        consecutive_empty_queues++;
        if (consecutive_empty_queues > max_consecutive_empty) {
          max_consecutive_empty = consecutive_empty_queues;
        }
      } else {
        consecutive_empty_queues = 0;
      }

      while (!s.isQueueEmpty()) {
        rc.increase_ok = true;
        rc.decrease_ok = true;
        rc.check_section(
            &fas_queue[0].entry[fas_queue[0].read_idx & QUEUE_LEN_MASK]);
        fas_queue[0].read_idx++;
        last_pos = rc.pos;
      }

      if (s.getPositionAfterCommandsCompleted() >= target_position) {
        printf("Reached target position at loop %d\n", i);
        break;
      }
    }

    printf("Final position = %d, target = %d, max_empty_cycles = %d\n",
           s.getCurrentPosition(), target_position, max_consecutive_empty);
    test_handler(s.getCurrentPosition() > target_position - 100,
                 "motor should be close to target position");
    test_handler(max_consecutive_empty < 20,
                 "motor should not stop prematurely");

    if (!test_failed) {
      printf("Test 3 PASSED\n");
    }
  }

  void test_issue_with_reduced_acceleration() {
    test_failed = false;
    init_queue();
    FastAccelStepper s = FastAccelStepper();
    s.init(NULL, 0, 0);
    RampChecker rc = RampChecker();
    assert(0 == s.getCurrentPosition());

    uint32_t accel = 8164 / 3;
    uint32_t high_speed_hz = 89;
    uint32_t low_speed_hz = 81;
    int32_t target_position = 1720;

    printf(
        "\n=== Test: Speed change with reduced acceleration (1/3 of 8164) "
        "===\n");
    printf("User reported this avoids the issue.\n\n");

    assert(s.isQueueEmpty());
    s.setSpeedInHz(high_speed_hz);
    s.setAcceleration(accel);
    s.fill_queue();
    assert(s.isQueueEmpty());

    MoveResultCode result = s.moveTo(target_position);
    test_handler(result == MOVE_OK, "moveTo should succeed");

    bool speed_reduced = false;
    int reduce_at_loop = 50;
    int consecutive_empty_queues = 0;
    int max_consecutive_empty = 0;

    for (int i = 0; i < 5000; i++) {
      if (!s.isRampGeneratorActive()) {
        printf("Ramp generator inactive at loop %d\n", i);
        break;
      }

      if (!speed_reduced && i == reduce_at_loop) {
        printf("At loop %d: Reducing speed from %u Hz to %u Hz\n", i,
               high_speed_hz, low_speed_hz);
        s.setSpeedInHz(low_speed_hz);
        s.applySpeedAcceleration();
        speed_reduced = true;
      }

      s.fill_queue();

      if (s.isQueueEmpty()) {
        consecutive_empty_queues++;
        if (consecutive_empty_queues > max_consecutive_empty) {
          max_consecutive_empty = consecutive_empty_queues;
        }
      } else {
        consecutive_empty_queues = 0;
      }

      while (!s.isQueueEmpty()) {
        rc.increase_ok = true;
        rc.decrease_ok = true;
        rc.check_section(
            &fas_queue[0].entry[fas_queue[0].read_idx & QUEUE_LEN_MASK]);
        fas_queue[0].read_idx++;
      }

      if (s.getPositionAfterCommandsCompleted() >= target_position) {
        printf("Reached target position at loop %d\n", i);
        break;
      }
    }

    printf("Final position = %d, target = %d, max_empty_cycles = %d\n",
           s.getCurrentPosition(), target_position, max_consecutive_empty);
    test_handler(s.getCurrentPosition() > target_position - 50,
                 "motor should reach target position with reduced accel");
    test_handler(max_consecutive_empty < 20,
                 "motor should not stop prematurely with reduced accel");

    if (!test_failed) {
      printf("Test 4 PASSED\n");
    }
  }
};

int main() {
  FastAccelStepperTest test;

  printf("========================================\n");
  printf("FastAccelStepper PC-Based Test 20\n");
  printf("Issue: Motor stops at low speed with\n");
  printf("       acceleration=8164, speed=81Hz\n");
  printf("========================================\n");

  test.test_issue_motor_stops_at_low_speed();
  test.test_issue_speed_change_from_high_to_low();
  test.test_issue_multiple_speed_changes();
  test.test_issue_with_reduced_acceleration();

  printf("\n========================================\n");
  printf("TEST_20 COMPLETED\n");
  printf("(Note: Test 2 may fail if bug is present)\n");
  printf("========================================\n");

  return test_failed ? 1 : 0;
}
