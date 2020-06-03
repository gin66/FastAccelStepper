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

void init_queue() {
  fas_q_readptr_A = 0;
  fas_q_readptr_B = 0;
  fas_q_next_writeptr_A = 0;
  fas_q_next_writeptr_B = 0;
}

void basic_test_with_empty_queue() {
  init_queue();
  FastAccelStepper s = FastAccelStepper(true);
  assert(0 == s.getCurrentPosition());

  assert(s.isQueueEmpty());
  s.set_dynamics(100.0, 100.0);
  s.isr_fill_queue();
  assert(s.isQueueEmpty());
  s.move(100);
  s.isr_fill_queue();
  assert(!s.isQueueEmpty());
  for (int i = 0; i < 100; i++) {
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
      fas_q_readptr_A = (fas_q_readptr_A + 1) & QUEUE_LEN_MASK;
    }
  }
  test(!s.isr_speed_control_enabled, "too many commands created");

  printf("TEST_02 PASSED\n");
}

int main() { basic_test_with_empty_queue(); }
