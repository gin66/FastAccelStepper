#include "FastAccelStepper.h"

#define stepPinStepperA   9  /* OC1A */
#define stepPinStepperB   10 /* OC1B */

// Here are the global variables to interface with the interrupts

// To realize the 1 Hz debug led
uint8_t fas_ledPin = 255;   // 255 if led blinking off
uint16_t fas_debug_led_cnt = 0;

// These variables control the stepper timing behaviour
// Current queue implementation cannot fill all elements. TODO
#define QUEUE_LEN (1<<4)
#define QUEUE_LEN_MASK 15
uint8_t fas_q_readptr_A = 0;   // ISR stops if readptr == next_writeptr
uint8_t fas_q_next_writeptr_A = 0;
uint8_t fas_q_readptr_B = 0;
uint8_t fas_q_next_writeptr_B = 0;
struct queue_entry {
   uint8_t steps;          // coding is bit7..1 is nr of steps and bit 0 is direction
   uint8_t delta_msb;
   uint16_t delta_lsw;     // using small values is not safe
   int16_t  delta_change;  // change of delta on each step. delta_lsw + steps*delta_change must not over/underflow
} fas_queue_A[QUEUE_LEN],fas_queue_B[QUEUE_LEN];

// This is used in the timer compare unit as extension of the 16 timer
uint8_t fas_skip_A = 0;
uint8_t fas_skip_B = 0;

FastAccelStepper fas_stepperA = FastAccelStepper(true);
FastAccelStepper fas_stepperB = FastAccelStepper(false);

#define StepperA_Toggle       TCCR1A =  (TCCR1A | _BV(COM1A0)) & ~_BV(COM1A1)
#define StepperA_Zero         TCCR1A =  (TCCR1A | _BV(COM1A1)) & ~_BV(COM1A0)
#define StepperA_Disconnect   TCCR1A =  (TCCR1A & ~(_BV(COM1A1) | _BV(COM1A0)))
#define StepperA_IsToggling   ((TCCR1A & (_BV(COM1A0) | _BV(COM1A1))) == _BV(COM1A0))

#define StepperB_Toggle       TCCR1A =  (TCCR1A | _BV(COM1B0)) & ~_BV(COM1B1)
#define StepperB_Zero         TCCR1A =  (TCCR1A | _BV(COM1B1)) & ~_BV(COM1B0)
#define StepperB_Disconnect   TCCR1A =  (TCCR1A & ~(_BV(COM1B1) | _BV(COM1B0)))
#define StepperB_IsToggling   ((TCCR1A & (_BV(COM1B0) | _BV(COM1B1))) == _BV(COM1B0))

void FastAccelStepperEngine::init() {
   noInterrupts();

   // Set WGM13:0 to all zero => Normal mode
   TCCR1A &= ~(_BV(WGM11) | _BV(WGM10));
   TCCR1B &= ~(_BV(WGM13) | _BV(WGM12));

   // Set prescaler to 1
   TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11) | _BV(CS10))) | _BV(CS10);

   // enable OVF interrupt
   TIMSK1 |= _BV(TOIE1);

   interrupts();
}
void FastAccelStepperEngine::setDebugLed(uint8_t ledPin) {
   fas_ledPin = ledPin;
}
inline bool FastAccelStepper::add_queue_entry(uint8_t msb, uint16_t lsw, uint8_t steps, bool dir_high, int16_t change) {
   if (_channelA) {
      uint8_t wp = fas_q_next_writeptr_A;
      uint8_t rp = fas_q_readptr_A;
      uint8_t next_wp = (wp + 1) & QUEUE_LEN_MASK;
      if (next_wp != rp) {
	 steps <<= 1;
         pos_at_queue_end += dir_high ? steps : -steps;
         struct queue_entry *e = &fas_queue_A[wp];
         e->delta_msb = msb;
         e->delta_lsw = lsw;
	 e->delta_change = change;
         e->steps = (dir_high != dir_high_at_queue_end) ? steps | 0x01 : steps;
         dir_high_at_queue_end = dir_high;
         fas_q_next_writeptr_A = next_wp;
	 return true;
      }
   }
   else {
      uint8_t wp = fas_q_next_writeptr_B;
      uint8_t rp = fas_q_readptr_B;
      uint8_t next_wp = (wp + 1) & QUEUE_LEN_MASK;
      if (next_wp != rp) {
	 steps <<= 1;
         pos_at_queue_end += dir_high ? steps : -steps;
         struct queue_entry *e = &fas_queue_B[wp];
         e->delta_msb = msb;
         e->delta_lsw = lsw;
	 e->delta_change = change;
         e->steps = (dir_high != dir_high_at_queue_end) ? steps | 0x01 : steps;
         dir_high_at_queue_end = dir_high;
         fas_q_next_writeptr_B = next_wp;
	 return true;
      }
   }
   return false;
}

inline void FastAccelStepper::isr_fill_queue() {
   if (!isr_speed_control_enabled) {
      return;
   }
   if (isQueueFull()) {
      return;
   }
   if (isQueueEmpty()) {
      if (target_pos == pos_at_queue_end) {
	 _last_ms = 0;
	 isr_speed_control_enabled = false;
	 return;
      }
   }
   long remaining_steps = target_pos - pos_at_queue_end;
   bool accelerating = false;
   bool decelerate_to_stop = false;
   bool reduce_speed = false;
   if (abs(remaining_steps) <= _deceleration_start) {
      decelerate_to_stop = true;
   }
   else if (_curr_speed < _speed) {
      accelerating = true;
   }
   else if (_curr_speed > _speed) {
      reduce_speed = true;
   }
//   long curr_ms = millis();
   long dt_ms = 16000000 / 65536;
//   if (_last_ms != 0) {
//      dt_ms = curr_ms - _last_ms;
//   }
//   _last_ms = curr_ms;
   if (accelerating) {
      _curr_speed += _accel / 1000.0 * dt_ms;
      _curr_speed = min(_curr_speed, _speed);
   }
   if (decelerate_to_stop) {
      _dec_time_ms = max(_dec_time_ms-dt_ms,1.0);
      _curr_speed = 2*abs(remaining_steps) * 1000.0/_dec_time_ms;
   }
   if (reduce_speed) {
      _curr_speed -= _accel / 1000.0 * dt_ms;
      _curr_speed = max(_curr_speed, _speed);
   }
   long delta = round(16000000.0/_curr_speed);

   uint16_t steps = (16000000*8/1000) / delta; // How many steps in 8 ms ?
   steps = min(steps, 127);
   steps = max(steps, 1);

   uint16_t x = delta>>14;
   uint16_t delta_lsw;
   if (x > 1) {
      x--;
      delta_lsw = delta & 16383;
      delta_lsw |= 16384;
   }
   else {
      delta_lsw = delta;
   }
   add_queue_entry(x, delta_lsw, min(steps, abs(remaining_steps)), remaining_steps > 0, 0);
}
ISR(TIMER1_OVF_vect) {
   // disable OVF interrupt to avoid nesting
   TIMSK1 &= ~_BV(TOIE1);

   // enable interrupts for nesting
   interrupts();

   if (fas_ledPin < 255) {
      fas_debug_led_cnt++;
      if (fas_debug_led_cnt == 144) {
        digitalWrite(fas_ledPin, HIGH);
      }
      if (fas_debug_led_cnt == 288) {
        digitalWrite(fas_ledPin, LOW);
	fas_debug_led_cnt = 0;
      }
   }

   fas_stepperA.isr_fill_queue();
   fas_stepperB.isr_fill_queue();

   // enable OVF interrupt again
   TIMSK1 |= _BV(TOIE1);
}

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
         uint8_t pin = fas_stepperA.auto_enablePin();
         if (pin != 255) {
            digitalWrite(pin, HIGH);
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
      digitalWrite(fas_stepperA._dirPin, digitalRead(fas_stepperA._dirPin) == HIGH ? LOW : HIGH);
   }
   uint8_t pin = fas_stepperA.auto_enablePin();
   if (pin != 255) {
      digitalWrite(pin, LOW);
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
         uint8_t pin = fas_stepperB.auto_enablePin();
         if (pin != 255) {
            digitalWrite(pin, HIGH);
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
      digitalWrite(fas_stepperB._dirPin, digitalRead(fas_stepperB._dirPin) == HIGH ? LOW : HIGH);
   }
   uint8_t pin = fas_stepperB.auto_enablePin();
   if (pin != 255) {
      digitalWrite(pin, LOW);
   }
}

FastAccelStepper *FastAccelStepperEngine::stepperA() {
   return &fas_stepperA;
}
FastAccelStepper *FastAccelStepperEngine::stepperB() {
   return &fas_stepperB;
}

FastAccelStepper::FastAccelStepper(bool channelA) {
   _channelA = channelA;
   _auto_enablePin = 255;
   _curr_speed = 0.0;
   pos_at_queue_end = 0;
   dir_high_at_queue_end = true;
   isr_speed_control_enabled = false;

   uint8_t pin = _channelA ? stepPinStepperA : stepPinStepperB;
   digitalWrite(pin, LOW);
   pinMode(pin, OUTPUT);

   // start interrupt
   if (channelA) {
      noInterrupts();
      OCR1A = 32768;         // definite start point
      StepperA_Disconnect;
      TCCR1C = _BV(FOC1A);   // force compare to ensure disconnect
      fas_skip_A = 0;
      TIFR1 = _BV(OCF1A);    // clear interrupt flag
      TIMSK1 |= _BV(OCIE1A); // enable compare A interrupt
      interrupts();
   }
   else {
      noInterrupts();
      OCR1B = 32768;         // definite start point
      StepperB_Disconnect;
      TCCR1C = _BV(FOC1B);   // force compare to ensure disconnect
      fas_skip_B = 0;
      TIFR1 = _BV(OCF1B);    // clear interrupt flag
      TIMSK1 |= _BV(OCIE1B); // enable compare B interrupt
      interrupts();
   }
}
void FastAccelStepper::setDirectionPin(uint8_t dirPin) {
   _dirPin = dirPin;
   digitalWrite(dirPin, HIGH);
   pinMode(dirPin, OUTPUT);
}
void FastAccelStepper::setEnablePin(uint8_t enablePin) {
   _enablePin = enablePin;
   digitalWrite(enablePin, HIGH);
   pinMode(enablePin, OUTPUT);
}
void FastAccelStepper::set_auto_enable(bool auto_enable) {
   if (auto_enable) {
      _auto_enablePin = _enablePin;
   }
   else {
      _auto_enablePin = 255;
   }
}
uint8_t FastAccelStepper::auto_enablePin() {
   return _auto_enablePin;
}
void FastAccelStepper::set_dynamics(float speed, float accel) {
   _speed = speed;
   _accel = accel;
   _min_steps = round(speed*speed/accel);
}
void FastAccelStepper::move(long move) {
   target_pos = pos_at_queue_end + move;
   _calculate_move(move);
}
void FastAccelStepper::moveTo(long position) {
   long move;
   target_pos = position;
   move = position - pos_at_queue_end;
   _calculate_move(move);
}
void FastAccelStepper::_calculate_move(long move) {
   unsigned long steps = abs(move);
   // The movement consists of three phases.
   // 1. Change current speed to constant speed
   // 2. Constant travel speed
   // 3. Decelerate to stop
   //
   // With v_t being travel speed
   //
   // Steps for 3 (no emergency stop):
   //     t_dec = v_t / a
   //     s_3   = 1/2 * a * t_dec² = v_t² / 2a
   //
   // Steps for 1:
   //     if v <= v_t
   //        t_acc_1 = v_t / a
   //        t_acc_2 = v   / a
   //        s_1 = 1/2 * a * t_acc_1² - 1/2 * a * t_acc_2²
   //            = 1/2 * v_t² / a - 1/2 * v² / a
   //            = (v_t² - v²) / 2a
   //            = s_3 - v^2 / 2a
   //
   //     if v > v_t
   //        s_1 = (v^2 - v_t^2) / 2a
   //            = v^2 / 2a - s_3
   //
   // Steps for 2:
   //     s_2 = steps - s_1 - s_3
   //     if v <= v_t
   //        s_2 = steps - 2 s_3 + v^2 / 2a
   //     if v > v_t
   //        s_2 = steps - v^2 / 2a
   //
   // Case 1: Normal operation
   //     steps >= s_1 + s_3 for a proper v_t
   //     if v <= v_t
   //        steps >= 2 s_3 - v^2 / 2a for a proper v_t
   //     if v > v_t
   //        steps >= v^2 / 2a for v_t = v_max
   //
   // Case 2: Emergency stop
   //     steps < v^2 / 2a
   //     this can be covered by a generic step 3, using constant decelaration a_3:
   //         s_remain = 1/2 * v * t_dec
   //         t_dec = 2 s_remain / v
   //         a_3 = v / t_dec = v^2 / 2 s_remain
   //

   // Steps needed to stop from current speed with defined acceleration
   unsigned long new_deceleration_start;
   unsigned long new_dec_time_ms;
   unsigned long s_stop = round(_curr_speed * _curr_speed / 2.0 / _accel);
   if (s_stop > steps) {
      // start deceleration immediately
      new_deceleration_start = steps;
      new_dec_time_ms = round(2000.0 * steps / _curr_speed);
   }
   else if (_curr_speed <= _speed) {
      // add steps to reach current speed to full ramp
      unsigned long s_full_ramp = steps + s_stop;
      unsigned long ramp_steps = min(s_full_ramp, _min_steps);
      new_deceleration_start = ramp_steps/2;
      new_dec_time_ms = round(sqrt(ramp_steps/_accel)*1000.0);
   }
   else {
      // need decelerate first in phase 1, then normal deceleration
      new_deceleration_start = _min_steps/2;
      new_dec_time_ms = round(_speed/_accel*1000.0);
   }
   noInterrupts();
   _deceleration_start = new_deceleration_start;
   _dec_time_ms = new_dec_time_ms;
   interrupts();
   isr_speed_control_enabled = true;
}
void FastAccelStepper::disableOutputs() {
   if (_enablePin != 255) {
     digitalWrite(_enablePin, HIGH);
   }
}
void FastAccelStepper::enableOutputs() {
   if (_enablePin != 255) {
     digitalWrite(_enablePin, LOW);
   }
}
long FastAccelStepper::getCurrentPosition() {
   long pos = pos_at_queue_end;
   bool dir = dir_high_at_queue_end;
   struct queue_entry *q;
   uint8_t rp,wp;
   noInterrupts();
   if (_channelA) {
      q = fas_queue_A;
      rp = fas_q_readptr_A;
      wp = fas_q_next_writeptr_A;
   }
   else {
      q = fas_queue_B;
      rp = fas_q_readptr_B;
      wp = fas_q_next_writeptr_B;
   }
   interrupts();
   if (rp != wp) {
      while (rp != wp) {
         wp = (wp + QUEUE_LEN - 1) & QUEUE_LEN_MASK;
         uint8_t steps = q[wp].steps;
         if (dir) {
	    pos -= steps>>1;
         }
         else {
	    pos += steps>>1;
	 }
         if (steps & 1) {
   	    dir = !dir;
         }
      }
   }
   return pos;
}
bool FastAccelStepper::isQueueFull() {
   bool full;
   if (_channelA) {
      full = (QUEUE_LEN + fas_q_readptr_A - fas_q_next_writeptr_A) & QUEUE_LEN == QUEUE_LEN - 1 ;
   }
   else {
      full = (QUEUE_LEN + fas_q_readptr_B - fas_q_next_writeptr_B) & QUEUE_LEN == QUEUE_LEN - 1 ;
   }
   return full;
}
bool FastAccelStepper::isQueueEmpty() {
   bool empty;
   if (_channelA) {
      empty = (fas_q_readptr_A == fas_q_next_writeptr_A) ;
   }
   else {
      empty = (fas_q_readptr_B == fas_q_next_writeptr_B) ;
   }
   return empty;
}
bool FastAccelStepper::isRunning() {
   return !isQueueEmpty();
}
