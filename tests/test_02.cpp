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

		uint32_t last_dt;
		uint32_t min_dt;
		bool increase_ok;
		bool flat_ok;
		bool decrease_ok;
		bool first;
};

RampChecker::RampChecker() {
	last_dt = ~0;
	min_dt = ~0;
	first = true;
	increase_ok = true;
	decrease_ok = false;
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

	min_dt = min(min_dt, min(start_dt,end_dt));
	printf("%d\n", min_dt);

	if (last_dt > start_dt) {
		assert(increase_ok);
		decrease_ok = true;
	}
	else if (last_dt < start_dt) {
		assert(decrease_ok);
		increase_ok = false;
	}
	if (start_dt > end_dt) {
		assert(increase_ok);
	}
	else if (start_dt < end_dt) {
		assert(decrease_ok);
		increase_ok = false;
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
  s.set_dynamics(160000, 100.0);
  s.isr_fill_queue();
  assert(s.isQueueEmpty());
  s.move(1000);
  s.isr_fill_queue();
  assert(!s.isQueueEmpty());
  for (int i = 0; i < 1000; i++) {
    if (false) {
      printf(
          "Loop %d: Queue read/write = %d/%d    Target pos = %ld, Queue End "
          "pos = %ld  QueueEmpty=%s\n",
          i, fas_q_readptr_A, fas_q_next_writeptr_A, s.target_pos,
          s.getPositionAfterCommandsCompleted(),
          s.isQueueEmpty() ? "yes" : "no");
    }
    if (!s.isr_speed_control_enabled) {
      break;
    }
    s.isr_fill_queue();
    while (!s.isQueueEmpty()) {
	  rc.check_section(&fas_queue_A[fas_q_readptr_A]);
      fas_q_readptr_A = (fas_q_readptr_A + 1) & QUEUE_LEN_MASK;
    }
  }
  test(!s.isr_speed_control_enabled, "too many commands created");
  printf("%d\n", rc.min_dt);
  test(rc.min_dt == 160000, "max speed not reached");

  printf("TEST_02 PASSED\n");
}

int main() { basic_test_with_empty_queue(); }
