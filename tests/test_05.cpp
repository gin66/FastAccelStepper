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

FastAccelStepper *stepper;
StepperQueue fas_queue[NUM_QUEUES];

int enable_inject_on_mark = -1;
bool enable_stepper_manage_on_interrupts = false;
bool enable_stepper_manage_on_noInterrupts = false;
bool in_manage = false;

void inject_fill_interrupt(int mark) {
  if ((mark == enable_inject_on_mark) && !in_manage) {
    in_manage = true;
    stepper->manage();
    in_manage = false;
  }
}
void noInterrupts() {
  if (enable_stepper_manage_on_noInterrupts && !in_manage) {
    in_manage = true;
    stepper->manage();
    in_manage = false;
  }
}
void interrupts() {
  if (enable_stepper_manage_on_interrupts && !in_manage) {
    in_manage = true;
    stepper->manage();
    in_manage = false;
  }
}

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
  uint32_t total_steps;
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
  total_steps = 0;
}
void RampChecker::check_section(struct queue_entry *e) {
  uint8_t steps = e->steps;
  if (!first) {
    assert((steps & 1) == 0);
  }
  steps >>= 1;
  assert(steps >= 1);
  uint32_t start_dt = e->period * e->n_periods;
  uint32_t end_dt = start_dt;

  min_dt = min(min_dt, min(start_dt, end_dt));
  assert(min_dt > 0);
  float accel = 0;
  if (!first) {
    accel = (16000000.0 / end_dt - 16000000.0 / last_dt) /
            (1.0 / 16000000.0 * 0.5 * (start_dt + end_dt));
  }
  printf(
      "process command in ramp checker @%.6fs - %d steps: steps = %d last = %d "
      "start = %d "
      " end = %d  min_dt "
      "= %d   accel=%.6f\n",
      total_ticks / 16000000.0, total_steps, steps, last_dt, start_dt, end_dt,
      min_dt, accel);

  total_steps += steps;
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
  fas_queue[0]._initVars();
  fas_queue[1]._initVars();
}

void do_test() {
  init_queue();
  FastAccelStepper s = FastAccelStepper();
  s.init(0, 0);
  stepper = &s;

  RampChecker rc = RampChecker();
  assert(0 == s.getCurrentPosition());

  int32_t steps = 10000;

  // Increase speed to 400, then further to 300
  // Identified bug was a fast jump to 300 without acceleration
  assert(s.isQueueEmpty());
  s.setSpeed(400);
  s.setAcceleration(10000);
  in_manage = true;
  s.manage();
  in_manage = false;
  assert(s.isQueueEmpty());
  s.moveTo(3000);
  in_manage = true;
  s.manage();
  in_manage = false;
  assert(!s.isQueueEmpty());

  float old_planned_time_in_buffer = 0;
  int moveto_done = false;
  for (int i = 0; i < steps; i++) {
    if (!moveto_done && (s.getCurrentPosition() >= 500)) {
      moveto_done = true;
      s.moveTo(4000);
    }
    if (true) {
      printf(
          "Loop %d: Queue read/write = %d/%d    Target pos = %d, Queue End "
          "pos = %d  QueueEmpty=%s\n",
          i, fas_queue[0].read_idx, fas_queue[0].next_write_idx, s.targetPos(),
          s.getPositionAfterCommandsCompleted(),
          s.isQueueEmpty() ? "yes" : "no");
    }
    in_manage = true;
    if (!s.isrSpeedControlEnabled() && s.isQueueEmpty()) {
      break;
    }
    s.manage();
    uint32_t from_dt = rc.total_ticks;
    while (!s.isQueueEmpty()) {
      rc.check_section(
          &fas_queue[0].entry[fas_queue[0].read_idx & QUEUE_LEN_MASK]);
      fas_queue[0].read_idx++;
    }
    in_manage = false;
    uint32_t to_dt = rc.total_ticks;
    float planned_time = (to_dt - from_dt) * 1.0 / 16000000;
    printf("%d: planned time in buffer: %.6fs\n", i, planned_time);
    // This must be ensured, so that the stepper does not run out of commands
    old_planned_time_in_buffer = planned_time;
  }
  test(!s.isrSpeedControlEnabled(), "too many commands created");
  printf("Total time  %f\n", rc.total_ticks / 16000000.0);
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
  printf("CHECKSUM for %d/%d/%d: %d\n", steps, travel_dt, accel, s.checksum);
#endif
  printf("Total steps = %d\n", rc.total_steps);
  assert(rc.total_steps == 4000);

  printf("TEST_05 Part PASSED\n");
}

int main() {
  enable_stepper_manage_on_interrupts = false;
  enable_stepper_manage_on_noInterrupts = false;
  enable_inject_on_mark = -1;
  do_test();

  enable_stepper_manage_on_interrupts = false;
  enable_stepper_manage_on_noInterrupts = false;
  enable_inject_on_mark = 0;
  do_test();

  enable_stepper_manage_on_interrupts = false;
  enable_stepper_manage_on_noInterrupts = false;
  enable_inject_on_mark = 1;
  do_test();

  enable_stepper_manage_on_interrupts = false;
  enable_stepper_manage_on_noInterrupts = false;
  enable_inject_on_mark = 2;
  do_test();

  enable_stepper_manage_on_interrupts = false;
  enable_stepper_manage_on_noInterrupts = true;
  enable_inject_on_mark = -1;
  do_test();

  enable_stepper_manage_on_interrupts = true;
  enable_stepper_manage_on_noInterrupts = false;
  enable_inject_on_mark = -1;
  do_test();

  printf("TEST_05 PASSED\n");
}
