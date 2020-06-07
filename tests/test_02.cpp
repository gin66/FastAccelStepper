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
  void check_section(struct queue_entry *e, uint8_t ramp_state);

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
void RampChecker::check_section(struct queue_entry *e, uint8_t ramp_state) {
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
    decrease_ok = true;
  } else if (last_dt < start_dt) {
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

  switch (ramp_state) {
    case RAMP_STATE_ACCELERATE:
      accelerate_till = total_ticks;
      break;
    case RAMP_STATE_DECELERATE_TO_STOP:
      break;
    case RAMP_STATE_DECELERATE:
      break;
    case RAMP_STATE_COAST:
      coast_till = total_ticks;
      break;
  }

  first = false;
}

void init_queue() {
  fas_q_readptr_A = 0;
  fas_q_readptr_B = 0;
  fas_q_next_writeptr_A = 0;
  fas_q_next_writeptr_B = 0;
}

void basic_test_with_empty_queue() {
  init_queue();
  FastAccelStepper s = FastAccelStepper(true);
  RampChecker rc = RampChecker();
  assert(0 == s.getCurrentPosition());

  assert(s.isQueueEmpty());
  s.setSpeed(10000);
  s.setAcceleration(100);
  s.isr_fill_queue();
  assert(s.isQueueEmpty());
  s.move(1000);
  s.isr_fill_queue();
  assert(!s.isQueueEmpty());
  for (int i = 0; i < 1000; i++) {
    if (false) {
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
    while (!s.isQueueEmpty()) {
      rc.check_section(&fas_queue_A[fas_q_readptr_A], s.ramp_state);
      fas_q_readptr_A = (fas_q_readptr_A + 1) & QUEUE_LEN_MASK;
    }
  }
  test(!s.isr_speed_control_enabled, "too many commands created");
  printf("%d\n", rc.min_dt);
  test(rc.min_dt == 160000, "max speed not reached");
}

void test_with_pars(int32_t steps, uint32_t travel_dt, uint16_t accel,
                    bool reach_max_speed, float min_time, float max_time,
					float allowed_ramp_time_delta) {
  printf("Test test_with_pars steps=%d travel_dt=%d accel=%d dir=%s\n", steps,
         travel_dt, accel, reach_max_speed ? "CW" : "CCW");
  init_queue();
  FastAccelStepper s = FastAccelStepper(true);
  RampChecker rc = RampChecker();
  assert(0 == s.getCurrentPosition());

  assert(s.isQueueEmpty());
  s.setSpeed(travel_dt);
  s.setAcceleration(accel);
  s.isr_fill_queue();
  assert(s.isQueueEmpty());
  s.move(steps);
  s.isr_fill_queue();
  assert(!s.isQueueEmpty());
  float old_planned_time_in_buffer = 0;
  for (int i = 0; i < steps; i++) {
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
      rc.check_section(&fas_queue_A[fas_q_readptr_A], s.ramp_state);
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
  printf("Total time  %f < %f < %f ?\n", min_time, rc.total_ticks / 16000000.0,
         max_time);
  test(rc.total_ticks / 16000000.0 > min_time, "ramp too fast");
  test(rc.total_ticks / 16000000.0 < max_time, "ramp too slow");
  if (reach_max_speed) {
    printf("%d = %d ?\n", rc.min_dt, travel_dt * 16);
    test(rc.min_dt == travel_dt * 16, "max speed not reached");
  } else {
    printf("%d > %d ?\n", rc.min_dt, travel_dt * 16);
    test(rc.min_dt > travel_dt * 16, "max speed reached");
  }
  float up_time, down_time;
  if (reach_max_speed) {
     printf("Ramp time up/coast/down/total=");
	 up_time = 1.0*rc.accelerate_till / 16000000.0;
	 down_time = (1.0*rc.total_ticks - 1.0*rc.coast_till)/16000000.0;
     printf(" %f", 1.0*rc.accelerate_till / 16000000.0);
     printf(" %f", 1.0*(rc.coast_till - rc.accelerate_till) / 16000000.0);
     printf(" %f", 1.0*(rc.total_ticks - rc.coast_till) / 16000000.0); 
	 assert(rc.total_ticks > rc.coast_till);
  }
  else {
     printf("Ramp time up/down/total =");
	 up_time = 1.0*rc.accelerate_till / 16000000.0;
	 down_time = (1.0*rc.total_ticks - 1.0*rc.accelerate_till)/16000000.0;
     printf(" %f", 1.0*rc.accelerate_till / 16000000.0);
     printf(" %f", 1.0*(rc.total_ticks - rc.accelerate_till) / 16000000.0); 
  }
  printf(" %f\n", 1.0*rc.total_ticks / 16000000.0);
  test(abs(up_time - down_time) < 0.5*(up_time + down_time) * allowed_ramp_time_delta, "assymmetric ramp");
  test(s.isStopped(), "is not stopped");
}

int main() {
  basic_test_with_empty_queue();
  //             steps  ticks_us  accel    maxspeed  min/max_total_time
  test_with_pars(10000, 5000, 100, true, 2 * 2.0 + 46.0 - 1.0,
                 2 * 2.0 + 46.0 + 2.0, 0.2);  // ramp time 2s, 400 steps TODO
  test_with_pars(1600, 5000, 10000, true, 7.9,
                 8.1, 0.2);  // ramp time 0.02s, 4 steps
  test_with_pars(1600, 5000, 1000, true, 2 * 0.2 + 7.8 - 0.1,
                 2 * 0.2 + 7.8 + 0.1, 0.2);  // ramp time 0.2s, 20 steps
  test_with_pars(15000, 100, 10000, true, 2 * 1.0 + 0.5 - 0.17,
                 2 * 1.0 + 0.5 + 0.1, 0.2);  // ramp time 1s, 5000 steps
  test_with_pars(100, 5000, 10000, true, 2 * 0.02 + 0.48 - 0.1,
                 2 * 0.02 + 0.48 + 0.1, 0.2);  // ramp time 0.02s, 4 steps
  test_with_pars(
      500, 50, 10000, false, 2 * 0.22 - 0.1,
      2 * 0.22 + 0.11, 0.2);  // ramp time 2s, 20000 steps => only ramp 0.22s
  test_with_pars(128000, 250, 1000, true, 2 * 2.0 + 30.0 - 0.1,
                 2 * 2.0 + 30.0 + 0.1 + 1.9, 0.2);  // ramp time 4s, 8000 steps
  test_with_pars(72000, 250, 1000, true, 2 * 2.0 + 15.0 - 0.1,
                 2 * 2.0 + 15.0 + 0.1 + 2 * 1.7, 0.2);  // ramp time 4s, 8000 steps
  test_with_pars(44000, 250, 1000, true, 2 * 2.0 + 7.5 - 0.1,
                 2 * 2.0 + 7.5 + 0.1 + 2 * 1.7, 0.2);  // ramp time 4s, 8000 steps
  test_with_pars(16002, 250, 1000, true, 2 * 2.0 + 0.0 - 0.1,
                 2 * 2.0 + 0.0 + 0.1 + 4.0, 0.2);  // ramp time 4s, 8000 steps
  test_with_pars(1000, 20, 1000, false, 2 * 1.0 - 0.1,
                 2 * 1.0 + 0.1, 0.2);  // ramp time 50s => 2s
  test_with_pars(500, 4000, 5, false, 19.0 - 0.1,
                 19.0 + 0.2, 0.2);
  test_with_pars(256000, 40, 5000, true, 15.2 - 0.1,
                 15.2 + 0.2, 0.2);  // jumps in speed in real
  //  test_with_pars(2000000, 40, 40, false, 2*223.0, 2*223.0); // ramp time
  //  625s, 7812500 steps
  printf("TEST_02 PASSED\n");
}
