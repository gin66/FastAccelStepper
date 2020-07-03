#ifndef TEST
#include <Arduino.h>
#endif
#include <stdint.h>

#if defined(ARDUINO_ARCH_AVR)
#define stepPinStepperA 9  /* OC1A */
#define stepPinStepperB 10 /* OC1B */
#endif

// Here are the global variables to interface with the interrupts

// CURRENT QUEUE IMPLEMENTATION WASTES ONE UNUSED ENTRY => BUG/TODO

#if defined(TEST)
#define NUM_QUEUES 2
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#endif
#if defined(ARDUINO_ARCH_AVR)
#define NUM_QUEUES 2
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#endif

// These variables control the stepper timing behaviour
#define QUEUE_LEN 16
#define QUEUE_LEN_MASK (QUEUE_LEN - 1)
struct queue_entry {
  uint8_t steps;  // coding is bit7..1 is nr of steps and bit 0 is direction
  uint8_t delta_msb;
  uint16_t delta_lsw;    // using small values is not safe
  int16_t delta_change;  // change of delta on each step. delta_lsw +
                         // steps*delta_change must not over/underflow
};
class StepperQueue {
 public:
  struct queue_entry entry[QUEUE_LEN];
  uint8_t read_ptr;  // ISR stops if readptr == next_writeptr
  uint8_t next_write_ptr;
  uint8_t autoEnablePin;
  uint8_t dirPin;
  // This is used in the timer compare unit as extension of the 16 timer
#if defined(ARDUINO_ARCH_AVR)
  uint8_t skip;
#endif

  void init(uint8_t step_pin);
};

extern struct StepperQueue fas_queue[NUM_QUEUES];

