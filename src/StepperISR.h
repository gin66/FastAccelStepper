#include <Arduino.h>
#include <stdint.h>

// Here are the global variables to interface with the interrupts

// These variables control the stepper timing behaviour
// Current queue implementation cannot fill all elements. TODO
#define QUEUE_LEN (1 << 4)
#define QUEUE_LEN_MASK 15
extern uint8_t fas_q_readptr_A;  // ISR stops if readptr == next_writeptr
extern uint8_t fas_q_next_writeptr_A;
extern uint8_t fas_q_readptr_B;
extern uint8_t fas_q_next_writeptr_B;
extern struct queue_entry {
  uint8_t steps;  // coding is bit7..1 is nr of steps and bit 0 is direction
  uint8_t delta_msb;
  uint16_t delta_lsw;    // using small values is not safe
  int16_t delta_change;  // change of delta on each step. delta_lsw +
                         // steps*delta_change must not over/underflow
} fas_queue_A[QUEUE_LEN], fas_queue_B[QUEUE_LEN];

extern uint8_t fas_autoEnablePin_A;
extern uint8_t fas_autoEnablePin_B;
extern uint8_t fas_dirPin_A;
extern uint8_t fas_dirPin_B;

#define StepperA_Toggle TCCR1A = (TCCR1A | _BV(COM1A0)) & ~_BV(COM1A1)
#define StepperA_Zero TCCR1A = (TCCR1A | _BV(COM1A1)) & ~_BV(COM1A0)
#define StepperA_Disconnect TCCR1A = (TCCR1A & ~(_BV(COM1A1) | _BV(COM1A0)))
#define StepperA_IsToggling \
  ((TCCR1A & (_BV(COM1A0) | _BV(COM1A1))) == _BV(COM1A0))

#define StepperB_Toggle TCCR1A = (TCCR1A | _BV(COM1B0)) & ~_BV(COM1B1)
#define StepperB_Zero TCCR1A = (TCCR1A | _BV(COM1B1)) & ~_BV(COM1B0)
#define StepperB_Disconnect TCCR1A = (TCCR1A & ~(_BV(COM1B1) | _BV(COM1B0)))
#define StepperB_IsToggling \
  ((TCCR1A & (_BV(COM1B0) | _BV(COM1B1))) == _BV(COM1B0))
