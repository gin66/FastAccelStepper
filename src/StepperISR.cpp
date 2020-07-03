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
	skip = 0;
}

#if defined(ARDUINO_ARCH_AVR)
ISR(TIMER1_COMPA_vect) {
  if (fas_queue_A.skip) {
    if ((--fas_queue_A.skip) == 0) {
      StepperA_Toggle;
    }
    OCR1A += 16384;
    return;
  } else if (StepperA_IsToggling) {
    TCCR1C = _BV(FOC1A);  // clear bit
    uint8_t rp = fas_queue_A.read_ptr;
    struct queue_entry* e = &fas_queue_A.entry[rp];
    if ((e->steps -= 2) > 1) {
      // perform another step with this queue entry
      OCR1A += (e->delta_lsw += e->delta_change);
      if (fas_queue_A.skip = e->delta_msb) {  // assign to skip and test for not zero
        StepperA_Zero;
      }
      return;
    }
    rp = (rp + 1) & QUEUE_LEN_MASK;
    fas_queue_A.read_ptr = rp;
    if (rp == fas_queue_A.next_write_ptr) {
      // queue is empty => set to disconnect
      StepperA_Disconnect;
      if (fas_queue_A.autoEnablePin != 255) {
        digitalWrite(fas_queue_A.autoEnablePin, HIGH);
      }
      // Next Interrupt takes place at next timer cycle => ~4ms
      return;
    }
  } else {
    // If reach here, then stepper is idle and waiting for a command
    uint8_t rp = fas_queue_A.read_ptr;
    if (rp == fas_queue_A.next_write_ptr) {
      // Next Interrupt takes place at next timer cycle => ~4ms
      return;
    }
  }
  // command in queue
  struct queue_entry* e = &fas_queue_A.entry[fas_queue_A.read_ptr];
  OCR1A += e->delta_lsw;
  if (fas_queue_A.skip = e->delta_msb) {  // assign to skip and test for not zero
    StepperA_Zero;
  } else {
    StepperA_Toggle;
  }
  uint8_t steps = e->steps;
  if ((steps & 0x01) != 0) {
    digitalWrite(fas_queue_A.dirPin, digitalRead(fas_queue_A.dirPin) == HIGH ? LOW : HIGH);
  }
  if (fas_queue_A.autoEnablePin != 255) {
    digitalWrite(fas_queue_A.autoEnablePin, LOW);
  }
}

ISR(TIMER1_COMPB_vect) {
  if (fas_queue_B.skip) {
    if ((--fas_queue_B.skip) == 0) {
      StepperB_Toggle;
    }
    OCR1B += 16384;
    return;
  } else if (StepperB_IsToggling) {
    TCCR1C = _BV(FOC1B);  // clear bit
    uint8_t rp = fas_queue_B.read_ptr;
    struct queue_entry* e = &fas_queue_B.entry[rp];
    if ((e->steps -= 2) > 1) {
      // perform another step with this queue entry
      OCR1B += (e->delta_lsw += e->delta_change);
      if (fas_queue_B.skip = e->delta_msb) {  // assign to skip and test for not zero
        StepperB_Zero;
      }
      return;
    }
    rp = (rp + 1) & QUEUE_LEN_MASK;
    fas_queue_B.read_ptr = rp;
    if (rp == fas_queue_B.next_write_ptr) {
      // queue is empty => set to disconnect
      StepperB_Disconnect;
      if (fas_queue_B.autoEnablePin != 255) {
        digitalWrite(fas_queue_B.autoEnablePin, HIGH);
      }
      // Next Interrupt takes place at next timer cycle => ~4ms
      return;
    }
  } else {
    // If reach here, then stepper is idle and waiting for a command
    uint8_t rp = fas_queue_B.read_ptr;
    if (rp == fas_queue_B.next_write_ptr) {
      // Next Interrupt takes place at next timer cycle => ~4ms
      return;
    }
  }
  // command in queue
  struct queue_entry* e = &fas_queue_B.entry[fas_queue_B.read_ptr];
  OCR1B += e->delta_lsw;
  if (fas_queue_B.skip = e->delta_msb) {  // assign to skip and test for not zero
    StepperB_Zero;
  } else {
    StepperB_Toggle;
  }
  uint8_t steps = e->steps;
  if ((steps & 0x01) != 0) {
    digitalWrite(fas_queue_B.dirPin, digitalRead(fas_queue_B.dirPin) == HIGH ? LOW : HIGH);
  }
  if (fas_queue_B.autoEnablePin != 255) {
    digitalWrite(fas_queue_B.autoEnablePin, LOW);
  }
}
#endif
