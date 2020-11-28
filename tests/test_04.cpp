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

class RampChecker {
 public:
  RampChecker();
  void check_section(struct queue_entry *e);

  uint32_t total_ticks;
  uint32_t last_dt;
  uint32_t min_dt;
  bool increase_ok;
  bool flat_ok;
  bool decrease_ok;
  bool first;
  uint32_t accelerate_till;
  uint32_t coast_till;
};

RampChecker::RampChecker() {
  total_ticks = 0;
  last_dt = ~0;
  min_dt = ~0;
  first = true;
  increase_ok = true;
  decrease_ok = false;
  coast_till = 0;
  accelerate_till = 0;
}
void RampChecker::check_section(struct queue_entry *e) {
  uint8_t steps = e->steps;
  if (!first) {
    assert((steps & 1) == 0);
  }
  steps >>= 1;
  assert(steps >= 1);
  uint32_t start_dt = PERIOD_TICKS;
  start_dt *= e->n_periods;
  start_dt += e->period;
  uint32_t end_dt = start_dt;

  min_dt = min(min_dt, min(start_dt, end_dt));
  float accel = 0;
  if (!first) {
    accel = (16000000.0 / end_dt - 16000000.0 / last_dt) /
            (1.0 / 16000000.0 * 0.5 * (start_dt + end_dt));
  }
  printf(
      "process command in ramp checker @%.6fs: steps = %d last = %d start = %d "
      " end = %d  min_dt "
      "= %d   accel=%.6f\n",
      total_ticks / 16000000.0, steps, last_dt, start_dt, end_dt, min_dt,
      accel);

  total_ticks += steps * start_dt;
  assert(steps * start_dt >= 0);

  if (last_dt > start_dt) {
    assert(increase_ok);
    accelerate_till = total_ticks;
    decrease_ok = true;
  } else if (last_dt < start_dt) {
    if (increase_ok) {
      coast_till = total_ticks;
    }
    assert(decrease_ok);
    increase_ok = false;
  }
  if (start_dt > end_dt) {
    assert(increase_ok);
  } else if (start_dt < end_dt) {
    assert(decrease_ok);
    increase_ok = false;
  }

  last_dt = end_dt;

  first = false;
}

void init_queue() {
  fas_queue[0].read_idx = 0;
  fas_queue[1].read_idx = 0;
  fas_queue[0].next_write_idx = 0;
  fas_queue[1].next_write_idx = 0;
}

int main() {
  init_queue();
  FastAccelStepper s = FastAccelStepper();
  RampChecker rc = RampChecker();
  assert(0 == s.getCurrentPosition());

  int32_t steps = 10000;

  // Increase speed to 400, then further to 300
  // Identified bug was a fast jump to 300 without acceleration
  assert(s.isQueueEmpty());
  s.setSpeed(400);
  s.setAcceleration(1000);
  s.manage();
  assert(s.isQueueEmpty());
  s.move(steps);
  s.manage();
  assert(!s.isQueueEmpty());
  float old_planned_time_in_buffer = 0;
  int speed_increased = false;
  for (int i = 0; i < steps; i++) {
    if (!speed_increased && (s.getCurrentPosition() >= 5000)) {
      puts("Change speed");
      s.manage();  // ensure queue is not empty
      speed_increased = true;
      s.setSpeed(300);
      s.move(steps);
    }
    if (true) {
      printf(
          "Loop %d: Queue read/write = %d/%d    Target pos = %d, Queue End "
          "pos = %d  QueueEmpty=%s\n",
          i, fas_queue[0].read_idx, fas_queue[0].next_write_idx, s.targetPos(),
          s.getPositionAfterCommandsCompleted(),
          s.isQueueEmpty() ? "yes" : "no");
    }
    if (!s.isRampGeneratorActive()) {
      break;
    }
    s.manage();
    uint32_t from_dt = rc.total_ticks;
    while (!s.isQueueEmpty()) {
      rc.check_section(
          &fas_queue[0].entry[fas_queue[0].read_idx & QUEUE_LEN_MASK]);
      fas_queue[0].read_idx++;
    }
    uint32_t to_dt = rc.total_ticks;
    float planned_time = (to_dt - from_dt) * 1.0 / 16000000;
    printf("planned time in buffer: %.6fs\n", planned_time);
    // This must be ensured, so that the stepper does not run out of commands
    assert((i == 0) || (old_planned_time_in_buffer > 0.005));
    old_planned_time_in_buffer = planned_time;
  }
  test(!s.isRampGeneratorActive(), "too many commands created");
  printf("Total time  %f\n", rc.total_ticks / 16000000.0);
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
  printf("CHECKSUM for %d/%d/%d: %d\n", steps, travel_dt, accel, s.checksum);
#endif

  printf("TEST_04 PASSED\n");
}
