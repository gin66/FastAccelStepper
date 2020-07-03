#include "StepperISR.h"

// Here are the global variables to interface with the interrupts

// These variables control the stepper timing behaviour
// Current queue implementation cannot fill all elements. TODO
StepperQueue fas_queue[NUM_QUEUES];

void StepperQueue::init() {
	dirPin = 255;
	autoEnablePin = 255;
	read_ptr = 0;
	next_write_ptr = 0;
#if defined(ARDUINO_ARCH_AVR)
	skip = 0;
#endif
}

#if defined(ARDUINO_ARCH_AVR)
#define AVR_STEPPER_ISR(CHANNEL,queue) ISR(TIMER1_COMP ## CHANNEL ## _vect) { \
  if (queue.skip) { \
    if ((--queue.skip) == 0) { \
      Stepper ## CHANNEL ## _Toggle; \
    } \
    OCR1 ## CHANNEL += 16384; \
    return; \
  } else if (Stepper ## CHANNEL ## _IsToggling) { \
    TCCR1C = _BV(FOC1 ## CHANNEL);  /* clear bit */ \
    uint8_t rp = queue.read_ptr; \
    struct queue_entry* e = &queue.entry[rp]; \
    if ((e->steps -= 2) > 1) { \
      /* perform another step with this queue entry */ \
      OCR1 ## CHANNEL += (e->delta_lsw += e->delta_change); \
      if (queue.skip = e->delta_msb) {  /* assign to skip and test for not zero */ \
        Stepper ## CHANNEL ## _Zero; \
      } \
      return; \
    } \
    rp = (rp + 1) & QUEUE_LEN_MASK; \
    queue.read_ptr = rp; \
    if (rp == queue.next_write_ptr) { \
      /* queue is empty => set to disconnect */ \
      Stepper ## CHANNEL ## _Disconnect; \
      if (queue.autoEnablePin != 255) { \
        digitalWrite(queue.autoEnablePin, HIGH); \
      } \
      /* Next Interrupt takes place at next timer cycle => ~4ms */ \
      return; \
    } \
  } else { \
    /* If reach here, then stepper is idle and waiting for a command */ \
    uint8_t rp = queue.read_ptr; \
    if (rp == queue.next_write_ptr) { \
      /* Next Interrupt takes place at next timer cycle => ~4ms */ \
      return; \
    } \
  } \
  /* command in queue */ \
  struct queue_entry* e = &queue.entry[queue.read_ptr]; \
  OCR1 ## CHANNEL += e->delta_lsw; \
  if (queue.skip = e->delta_msb) {  /* assign to skip and test for not zero */ \
    Stepper ## CHANNEL ## _Zero; \
  } else { \
    Stepper ## CHANNEL ## _Toggle; \
  } \
  uint8_t steps = e->steps; \
  if ((steps & 0x01) != 0) { \
    digitalWrite(queue.dirPin, digitalRead(queue.dirPin) == HIGH ? LOW : HIGH); \
  } \
  if (queue.autoEnablePin != 255) { \
    digitalWrite(queue.autoEnablePin, LOW); \
  } \
}
AVR_STEPPER_ISR(A, fas_queue_A)
AVR_STEPPER_ISR(B, fas_queue_B)
#endif
