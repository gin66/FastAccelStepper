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

uint8_t fas_q_readptr_A = 0;  // ISR stops if readptr == next_writeptr
uint8_t fas_q_next_writeptr_A = 0;
uint8_t fas_q_readptr_B = 0;
uint8_t fas_q_next_writeptr_B = 0;
struct queue_entry fas_queue_A[QUEUE_LEN], fas_queue_B[QUEUE_LEN];

uint8_t fas_autoEnablePin_A = 255;
uint8_t fas_autoEnablePin_B = 255;
uint8_t fas_dirPin_A = 255;
uint8_t fas_dirPin_B = 255;

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
  uint32_t start_dt = e->delta_msb * 16384 + e->delta_lsw;
  uint32_t end_dt = start_dt + (steps - 1) * e->delta_change;

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

  total_ticks += steps * start_dt + (steps - 1) * steps / 2 * e->delta_change;
  assert(steps * start_dt + (steps - 1) * steps / 2 * e->delta_change >= 0);

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
  fas_q_readptr_A = 0;
  fas_q_readptr_B = 0;
  fas_q_next_writeptr_A = 0;
  fas_q_next_writeptr_B = 0;
}

int main() {
  init_queue();
  FastAccelStepper s = FastAccelStepper(true);
  RampChecker rc = RampChecker();
  assert(0 == s.getCurrentPosition());

  int32_t steps = 10000;

  // Increase speed to 400, then further to 300
  // Identified bug was a fast jump to 300 without acceleration
  assert(s.isQueueEmpty());
  s.setSpeed(400);
  s.setAcceleration(1000);
  s.isr_fill_queue();
  assert(s.isQueueEmpty());
  s.move(steps);
  s.isr_fill_queue();
  assert(!s.isQueueEmpty());
  float old_planned_time_in_buffer = 0;
  int speed_increased = false;
  for (int i = 0; i < steps; i++) {
	if (!speed_increased && (s.getCurrentPosition() >= 5000)) {
	   puts("Change speed");
       s.isr_fill_queue(); // ensure queue is no empty
	   speed_increased = true;
       s.setSpeed(300);
       s.move(steps);
	}
    if (true) {
      printf(
          "Loop %d: Queue read/write = %d/%d    Target pos = %d, Queue End "
          "pos = %d  QueueEmpty=%s\n",
          i, fas_q_readptr_A, fas_q_next_writeptr_A, s.target_pos,
          s.getPositionAfterCommandsCompleted(),
          s.isQueueEmpty() ? "yes" : "no");
    }
    if (!s.isr_speed_control_enabled) {
      break;
    }
    s.isr_fill_queue();
    uint32_t from_dt = rc.total_ticks;
    while (!s.isQueueEmpty()) {
      rc.check_section(&fas_queue_A[fas_q_readptr_A]);
      fas_q_readptr_A = (fas_q_readptr_A + 1) & QUEUE_LEN_MASK;
    }
    uint32_t to_dt = rc.total_ticks;
    float planned_time = (to_dt - from_dt) * 1.0 / 16000000;
    printf("planned time in buffer: %.6fs\n", planned_time);
    // This must be ensured, so that the stepper does not run out of commands
    assert((i == 0) || (old_planned_time_in_buffer > 0.005));
    old_planned_time_in_buffer = planned_time;
  }
  test(!s.isr_speed_control_enabled, "too many commands created");
  printf("Total time  %f\n", rc.total_ticks / 16000000.0);
  test(s.isStopped(), "is not stopped");
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
  printf("CHECKSUM for %d/%d/%d: %d\n", steps, travel_dt, accel, s.checksum);
#endif

  printf("TEST_04 PASSED\n");
}
