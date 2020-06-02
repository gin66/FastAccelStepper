#include "StepperISR.h"

// Here are the global variables to interface with the interrupts

// These variables control the stepper timing behaviour
// Current queue implementation cannot fill all elements. TODO
uint8_t fas_q_readptr_A = 0;   // ISR stops if readptr == next_writeptr
uint8_t fas_q_next_writeptr_A = 0;
uint8_t fas_q_readptr_B = 0;
uint8_t fas_q_next_writeptr_B = 0;
struct queue_entry fas_queue_A[QUEUE_LEN],fas_queue_B[QUEUE_LEN];

uint8_t fas_autoEnablePin_A = 255;
uint8_t fas_autoEnablePin_B = 255;
uint8_t fas_dirPin_A = 255;
uint8_t fas_dirPin_B = 255;

// This is used in the timer compare unit as extension of the 16 timer
uint8_t fas_skip_A = 0;
uint8_t fas_skip_B = 0;

ISR(TIMER1_COMPA_vect) {
   if (fas_skip_A) {
      if ((--fas_skip_A) == 0) {
	 StepperA_Toggle;
      }
      OCR1A += 16384;
      return;
   }
   else if (StepperA_IsToggling) {
      TCCR1C = _BV(FOC1A); // clear bit
      uint8_t rp = fas_q_readptr_A;
      struct queue_entry *e = &fas_queue_A[rp];
      if ((e->steps -= 2) > 1) {
         // perform another step with this queue entry
	 OCR1A += (e->delta_lsw += e->delta_change);
         if (fas_skip_A = e->delta_msb) { // assign to skip and test for not zero
            StepperA_Zero;
	 }
	 return;
      }
      rp = (rp + 1) & QUEUE_LEN_MASK;
      fas_q_readptr_A = rp;
      if (rp == fas_q_next_writeptr_A) {
         // queue is empty => set to disconnect
         StepperA_Disconnect;
         if (fas_autoEnablePin_A != 255) {
            digitalWrite(fas_autoEnablePin_A, HIGH);
         }
	 // Next Interrupt takes place at next timer cycle => ~4ms
         return;
      }
   }
   else {
      // If reach here, then stepper is idle and waiting for a command
      uint8_t rp = fas_q_readptr_A;
      if (rp == fas_q_next_writeptr_A) {
	 // Next Interrupt takes place at next timer cycle => ~4ms
         return;
      }
   }
   // command in queue
   struct queue_entry *e = &fas_queue_A[fas_q_readptr_A];
   OCR1A += e->delta_lsw;
   if (fas_skip_A = e->delta_msb) { // assign to skip and test for not zero
      StepperA_Zero;
   }
   else {
      StepperA_Toggle;
   }
   uint8_t steps = e->steps;
   if ((steps & 0x01) != 0) {
      digitalWrite(fas_dirPin_A, digitalRead(fas_dirPin_A) == HIGH ? LOW : HIGH);
   }
   if (fas_autoEnablePin_A != 255) {
      digitalWrite(fas_autoEnablePin_A, LOW);
   }
}

ISR(TIMER1_COMPB_vect) {
   if (fas_skip_B) {
      if ((--fas_skip_B) == 0) {
	 StepperB_Toggle;
      }
      OCR1B += 16384;
      return;
   }
   else if (StepperB_IsToggling) {
      TCCR1C = _BV(FOC1B); // clear bit
      uint8_t rp = fas_q_readptr_B;
      struct queue_entry *e = &fas_queue_B[rp];
      if ((e->steps -= 2) > 1) {
         // perform another step with this queue entry
	 OCR1B += (e->delta_lsw += e->delta_change);
         if (fas_skip_B = e->delta_msb) { // assign to skip and test for not zero
            StepperB_Zero;
	 }
	 return;
      }
      rp = (rp + 1) & QUEUE_LEN_MASK;
      fas_q_readptr_B = rp;
      if (rp == fas_q_next_writeptr_B) {
         // queue is empty => set to disconnect
         StepperB_Disconnect;
         if (fas_autoEnablePin_B != 255) {
            digitalWrite(fas_autoEnablePin_B, HIGH);
         }
	 // Next Interrupt takes place at next timer cycle => ~4ms
         return;
      }
   }
   else {
      // If reach here, then stepper is idle and waiting for a command
      uint8_t rp = fas_q_readptr_B;
      if (rp == fas_q_next_writeptr_B) {
	 // Next Interrupt takes place at next timer cycle => ~4ms
         return;
      }
   }
   // command in queue
   struct queue_entry *e = &fas_queue_B[fas_q_readptr_B];
   OCR1B += e->delta_lsw;
   if (fas_skip_B = e->delta_msb) { // assign to skip and test for not zero
      StepperB_Zero;
   }
   else {
      StepperB_Toggle;
   }
   uint8_t steps = e->steps;
   if ((steps & 0x01) != 0) {
      digitalWrite(fas_dirPin_B, digitalRead(fas_dirPin_B) == HIGH ? LOW : HIGH);
   }
   if (fas_autoEnablePin_B != 255) {
      digitalWrite(fas_autoEnablePin_B, LOW);
   }
}
