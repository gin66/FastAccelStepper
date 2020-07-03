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

void init_queue() {
  fas_queue[0].read_ptr = 0;
  fas_queue[0].next_write_ptr = 0;
  fas_queue[1].read_ptr = 0;
  fas_queue[1].next_write_ptr = 0;
}

void basic_test() {
  init_queue();
  FastAccelStepper s = FastAccelStepper();
  assert(0 == s.getCurrentPosition());
  assert(s.isQueueEmpty());
  assert(s.isQueueEmpty());
  s.addQueueEntry(10000, 100, true, 0);
  assert(!s.isQueueEmpty());
}

void queue_full() {
  puts("queue_full...");
  init_queue();
  FastAccelStepper s = FastAccelStepper();
  s.init(0,0);
  assert(0 == s.getCurrentPosition());
  assert(s.isQueueEmpty());
  assert(s.isQueueEmpty());
  printf("Queue read/write = %d/%d\n", fas_queue[0].read_ptr,
         fas_queue[0].next_write_ptr);
  for (int i = 0; i < QUEUE_LEN - 2; i++) {
    s.addQueueEntry(10000, 100, true, 0);
    assert(!s.isQueueEmpty());
    assert(!s.isQueueFull());
    printf("Queue read/write = %d/%d\n", fas_queue[0].read_ptr,
           fas_queue[0].next_write_ptr);
  }
  s.addQueueEntry(10000, 100, true, 0);
  printf("Queue read/write = %d/%d\n", fas_queue[0].read_ptr,
         fas_queue[0].next_write_ptr);
  assert(!s.isQueueEmpty());
  assert(s.isQueueFull());
  puts("...done");
}

void queue_out_of_range() {
  int8_t res;

  init_queue();
  FastAccelStepper s = FastAccelStepper();
  s.init(0,0);
  assert(s.isQueueEmpty());
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
  FastAccelStepper s = FastAccelStepper();
  s.init(0,0);
  assert(0 == s.getPositionAfterCommandsCompleted());
  s.addQueueEntry(65535, 1, true, 0);
  assert(1 == s.getPositionAfterCommandsCompleted());
}

int main() {
  assert(sizeof(struct queue_entry) == 6);
  basic_test();
  queue_out_of_range();
  queue_full();
  end_pos_test();
  printf("TEST_01 PASSED\n");
}
