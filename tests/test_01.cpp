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

void init_queue() {
  fas_queue[0].read_idx = 0;
  fas_queue[0].next_write_idx = 0;
  fas_queue[1].read_idx = 0;
  fas_queue[1].next_write_idx = 0;
}

void basic_test() {
  init_queue();
  FastAccelStepper s = FastAccelStepper();
  assert(0 == s.getCurrentPosition());
  assert(s.isQueueEmpty());
  assert(s.isQueueEmpty());
  s.addQueueEntry(10000, 100, true);
  assert(!s.isQueueEmpty());
}

void queue_full() {
  puts("queue_full...");
  init_queue();
  FastAccelStepper s = FastAccelStepper();
  s.init(0, 0);
  assert(0 == s.getCurrentPosition());
  assert(s.isQueueEmpty());
  assert(s.isQueueEmpty());
  printf("Queue read/write = %d/%d\n", fas_queue[0].read_idx,
         fas_queue[0].next_write_idx);
  for (int i = 0; i < QUEUE_LEN - 1; i++) {
    s.addQueueEntry(10000, 100, true);
    assert(!s.isQueueEmpty());
    assert(!s.isQueueFull());
    printf("Queue read/write = %d/%d\n", fas_queue[0].read_idx,
           fas_queue[0].next_write_idx);
  }
  s.addQueueEntry(10000, 100, true);
  printf("Queue read/write = %d/%d\n", fas_queue[0].read_idx,
         fas_queue[0].next_write_idx);
  assert(!s.isQueueEmpty());
  assert(s.isQueueFull());
  puts("...done");
}

void queue_out_of_range() {
  int8_t res;

  init_queue();
  FastAccelStepper s = FastAccelStepper();
  s.init(0, 0);
  assert(s.isQueueEmpty());
  assert(0 == s.getCurrentPosition());
  assert(s.isQueueEmpty());
  assert(s.isQueueEmpty());

  res = s.addQueueEntry(0xfeff02, 100, true);
  test(res == AQE_TOO_HIGH, "Too high provided should trigger error");
  assert(s.isQueueEmpty());

  res = s.addQueueEntry(65535, 128, true);
  test(res == AQE_STEPS_ERROR, "Too high step count should trigger an error");
  assert(s.isQueueEmpty());

  res = s.addQueueEntry(0xfeff01, 100, true);
  test(res == AQE_OK, "In range should be accepted");
  assert(!s.isQueueEmpty());
}

void end_pos_test() {
  init_queue();
  FastAccelStepper s = FastAccelStepper();
  s.init(0, 0);
  assert(0 == s.getPositionAfterCommandsCompleted());
  s.addQueueEntry(65535, 1, true);
  assert(1 == s.getPositionAfterCommandsCompleted());
}

int main() {
  // assert(sizeof(struct queue_entry) == 6);
  basic_test();
  queue_out_of_range();
  queue_full();
  end_pos_test();
  printf("TEST_01 PASSED\n");
}
