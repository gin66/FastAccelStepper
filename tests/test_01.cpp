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

void basic_test() {
  init_queue();
  FastAccelStepper s = FastAccelStepper(true);
  assert(0 == s.getCurrentPosition());
  assert(s.isQueueEmpty());
  assert(s.isQueueEmpty());
  s.addQueueEntry(10000, 100, true, 0);
  assert(!s.isQueueEmpty());
}

void queue_full() {
  puts("queue_full...");
  init_queue();
  FastAccelStepper s = FastAccelStepper(true);
  assert(0 == s.getCurrentPosition());
  assert(s.isQueueEmpty());
  assert(s.isQueueEmpty());
  printf("Queue read/write = %d/%d\n", fas_q_readptr_A, fas_q_next_writeptr_A);
  for (int i = 0; i < QUEUE_LEN - 2; i++) {
    s.addQueueEntry(10000, 100, true, 0);
    assert(!s.isQueueEmpty());
    assert(!s.isQueueFull());
    printf("%d: Queue read/write = %d/%d\n", i, fas_q_readptr_A,
           fas_q_next_writeptr_A);
  }
  s.addQueueEntry(10000, 100, true, 0);
  printf("Queue read/write = %d/%d\n", fas_q_readptr_A, fas_q_next_writeptr_A);
  assert(!s.isQueueEmpty());
  assert(s.isQueueFull());
  puts("...done");
}

void queue_out_of_range() {
  int8_t res;

  init_queue();
  FastAccelStepper s = FastAccelStepper(true);
  assert(0 == s.getCurrentPosition());
  assert(s.isQueueEmpty());
  assert(s.isQueueEmpty());

  res = s.addQueueEntry(1 << 23, 100, true, 0);
  test(res == AQE_TOO_HIGH, "Too high provided should trigger error");
  assert(s.isQueueEmpty());

  res = s.addQueueEntry(10000, 100, true, 30000);
  test(res == AQE_CHANGE_TOO_HIGH,
       "Too high change provided should trigger error");
  assert(s.isQueueEmpty());

  res = s.addQueueEntry(10000, 100, true, -30000);
  test(res == AQE_CHANGE_TOO_LOW,
       "Too low change provided should trigger error");
  assert(s.isQueueEmpty());

  res = s.addQueueEntry(10000, 100, true, -100);
  test(res == AQE_CUMULATED_CHANGE_TOO_LOW,
       "Too low change provided should trigger error");
  assert(s.isQueueEmpty());

  res = s.addQueueEntry(65536, 100, true, 32768 / 99 + 1);
  test(res == AQE_CHANGE_TOO_HIGH,
       "Too high change provided should trigger error");
  assert(s.isQueueEmpty());

  res = s.addQueueEntry(65535, 10, true, 728);
  test(res == AQE_OK, "Change just within limit should be ok");
  assert(!s.isQueueEmpty());

  res = s.addQueueEntry(65535, 100, true, 6);
  test(res == AQE_OK, "Change just within limit should be ok");
  assert(!s.isQueueEmpty());

  res = s.addQueueEntry(65535, 127, true, 4);
  test(res == AQE_OK, "Change just within limit should be ok");
  assert(!s.isQueueEmpty());

  res = s.addQueueEntry(65535, 128, true, 0);
  test(res == AQE_STEPS_ERROR, "Too high step count should trigger an error");
  assert(!s.isQueueEmpty());
}

void end_pos_test() {
  init_queue();
  FastAccelStepper s = FastAccelStepper(true);
  assert(0 == s.getPositionAfterCommandsCompleted());
  s.addQueueEntry(65535, 1, true, 0);
  assert(1 == s.getPositionAfterCommandsCompleted());
}

int main() {
  basic_test();
  queue_out_of_range();
  queue_full();
  end_pos_test();
  printf("TEST_01 PASSED\n");
}
