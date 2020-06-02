#include "FastAccelStepper.h"

#define stepPinStepperA   9  /* OC1A */
#define stepPinStepperB   10 /* OC1B */

// Here are the global variables to interface with the interrupts

// To realize the 1 Hz debug led
uint8_t fas_ledPin = 255;   // 255 if led blinking off
uint16_t fas_debug_led_cnt = 0;

// These variables control the stepper timing behaviour
#define QUEUE_LEN (1<<4)
#define QUEUE_LEN_MASK 15
uint8_t fas_queue_delta_msb_A[QUEUE_LEN];
uint16_t fas_queue_delta_lsw_A[QUEUE_LEN];
uint8_t fas_queue_steps_A[QUEUE_LEN];
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
// These variables are used to keep track of the stepper position and the target
long fas_target_pos_A = 0;
long fas_target_pos_B = 0;
bool fas_dir_cw_A = true;
bool fas_count_up_B = true;
uint8_t fas_steps_B = 0;
long fas_pos_A = 0;
long fas_pos_B = 0;

// This is used in the timer compare unit as extension of the 16 timer
uint8_t fas_skip_A = 0;
uint8_t fas_skip_B = 0;

FastAccelStepper fas_stepperA = FastAccelStepper(true);
FastAccelStepper fas_stepperB = FastAccelStepper(false);

#define StepperA_Toggle       TCCR1A =  (TCCR1A | _BV(COM1A0)) & ~_BV(COM1A1)
#define StepperA_Zero         TCCR1A =  (TCCR1A | _BV(COM1A1)) & ~_BV(COM1A0)
#define StepperA_Disconnect   TCCR1A =  (TCCR1A & ~(_BV(COM1A1) | _BV(COM1A0)))
#define StepperB_Toggle       TCCR1A =  (TCCR1A | _BV(COM1B0)) & ~_BV(COM1B1)
#define StepperB_Zero         TCCR1A =  (TCCR1A | _BV(COM1B1)) & ~_BV(COM1B0)
#define StepperB_Disconnect   TCCR1A =  (TCCR1A & ~(_BV(COM1B1) | _BV(COM1B0)))

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
      noInterrupts();
      fas_queue_delta_msb_A[0] = msb;
      fas_queue_delta_lsw_A[0] = lsw;
      fas_queue_steps_A[0] = steps;
      interrupts();
   }
   else {
      uint8_t wp = fas_q_next_writeptr_B;
      uint8_t rp = fas_q_readptr_B;
      uint8_t next_wp = (wp + 1) & QUEUE_LEN_MASK;
      if (next_wp != rp) {
         if (wp == rp) {
            pos_at_queue_end = fas_pos_B;
         }
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
      return false;
   }
}

inline void FastAccelStepper::isr_update_move(long remaining_steps) {
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

   long dpA = fas_target_pos_A - fas_pos_A;
   long tpB = fas_target_pos_B;

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
   // Manage stepper A
   if (dpA == 0) {
      fas_stepperA._last_ms = 0;
      fas_stepperA.isr_speed_control = false;
      uint8_t pin =fas_stepperA.auto_enablePin();
      if (pin != 255) {
        digitalWrite(pin, HIGH);
      }
   }
   else {
      if (fas_stepperA.isr_speed_control) {
         fas_stepperA.isr_update_move(abs(dpA));
      }
      if ((TIMSK1 & _BV(OCIE1A)) == 0) {
      // motor is not yet running.
      noInterrupts();
         OCR1A = TCNT1+16000; // delay 1ms for enable to act
      StepperA_Zero;
         TCCR1C = _BV(FOC1A); // force compare to ensure cleared output bits
      StepperA_Toggle;
         fas_skip_A = 0;
         TIFR1 = _BV(OCF1A);    // clear interrupt flag
         TIMSK1 |= _BV(OCIE1A); // enable compare A interrupt
         fas_dir_cw_A = dpA > 0 ? 1 : 0;
         uint8_t pin = fas_stepperA.auto_enablePin();
         if (pin != 255) {
            digitalWrite(pin, LOW);
         }
      interrupts();
      }
   }

   // Manage stepper B
   // if (tpB == fas_stepperB.pos_at_queue_end) {
   if (!fas_stepperB.isRunning() && !fas_stepperB.fill_queue) {
      fas_stepperB._last_ms = 0;
      fas_stepperB.isr_speed_control = false;
      uint8_t pin = fas_stepperB.auto_enablePin();
      if (pin != 255) {
        digitalWrite(pin, HIGH);
      }
   }
   else {
      fas_stepperB.fill_queue = false;
      if (fas_stepperB.isr_speed_control) {
         fas_stepperB.isr_update_move(tpB - fas_stepperB.pos_at_queue_end);
      }
      if ((TIMSK1 & _BV(OCIE1B)) == 0) {
         // motor is not yet running.
         noInterrupts();
         OCR1B = 32768;       // definite start point
         StepperB_Zero;
         TCCR1C = _BV(FOC1B); // force compare to ensure cleared output bits
         StepperB_Toggle;
         fas_skip_B = 0;
         struct queue_entry *e = &fas_queue_B[fas_q_readptr_B];
         uint8_t steps = e->steps;
         fas_steps_B = steps;
         if ((steps & 0x01) != 0) {
            fas_count_up_B = !fas_count_up_B;
            digitalWrite(fas_stepperB._dirPin, fas_count_up_B ? HIGH : LOW);
         }
         TIFR1 = _BV(OCF1B);    // clear interrupt flag
         uint8_t pin = fas_stepperB.auto_enablePin();
         if (pin != 255) {
            digitalWrite(pin, LOW);
         }
         TIMSK1 |= _BV(OCIE1B); // enable compare B interrupt
      interrupts();
      }
   }

   // enable OVF interrupt
   TIMSK1 |= _BV(TOIE1);
}

ISR(TIMER1_COMPA_vect) {
   if (fas_skip_A) {
      if ((--fas_skip_A) == 0) {
         StepperA_Toggle;
      }
      OCR1A += 16384;
   }
   else {
      TCCR1C = _BV(FOC1A); // clear bit
      // count the pulse
      bool res;
      if (fas_dir_cw_A) {
         res = (++fas_pos_A == fas_target_pos_A);
      } else {
         res = (--fas_pos_A == fas_target_pos_A);
      }
      if (res) {
         StepperA_Disconnect;
         TIMSK1 &= ~_BV(OCIE1A);
      }
      else {
         OCR1A += fas_queue_delta_lsw_A[0];
         if (fas_skip_A = fas_queue_delta_msb_A[0]) { // assign to skip and test for not zero
            StepperA_Zero;
         }
      }
   }
}

ISR(TIMER1_COMPB_vect) {
   if (fas_skip_B) {
      if ((--fas_skip_B) == 0) {
	 StepperB_Toggle;
      }
      OCR1B += 16384;
   }
   else {
      TCCR1C = _BV(FOC1B); // clear bit
      // count the pulse
      if (fas_count_up_B) {
         fas_pos_B++;
      } else {
         fas_pos_B--;
      }
      uint8_t rp = fas_q_readptr_B;
      struct queue_entry *e = &fas_queue_B[rp];
      if ((e->steps -= 2) <= 1) {
	 rp = (rp + 1) & QUEUE_LEN_MASK;
	 fas_q_readptr_B = rp;
	 if (rp == fas_q_next_writeptr_B) {
            // queue is empty => set to output zero mode
            StepperB_Disconnect;
            TIMSK1 &= ~_BV(OCIE1B);
	 }
	 else {
            // process next queue entry
            e = &fas_queue_B[rp];
            OCR1B += e->delta_lsw;
            if (fas_skip_B = e->delta_msb) { // assign to skip and test for not zero
               StepperB_Zero;
	    }
	    uint8_t steps = e->steps;
	    fas_steps_B = steps;
	    if ((steps & 0x01) != 0) {
               fas_count_up_B = !fas_count_up_B;
               digitalWrite(fas_stepperB._dirPin, fas_count_up_B ? HIGH : LOW);
	    }
	 }
      }
      else {
         // perform another step with this queue entry
	 OCR1B += (e->delta_lsw += e->delta_change);
         if (fas_skip_B = e->delta_msb) { // assign to skip and test for not zero
            StepperB_Zero;
	 }
      }
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
   isr_speed_control = false;
   fill_queue = false;

   uint8_t pin = _channelA ? stepPinStepperA : stepPinStepperB;
   digitalWrite(pin, LOW);
   pinMode(pin, OUTPUT);
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
   if (_channelA) {
      noInterrupts();
      fas_target_pos_A = fas_pos_A + move;
      interrupts();
   }
   else {
      noInterrupts();
      fas_target_pos_B = fas_stepperB.pos_at_queue_end + move;
      interrupts();
   }
   _calculate_move(move);
}
void FastAccelStepper::moveTo(long position) {
   long move;
   if (_channelA) {
      noInterrupts();
      fas_target_pos_A = position;
      move = position - fas_pos_A;
      interrupts();
   }
   else {
      noInterrupts();
      fas_target_pos_B = position;
      move = position - fas_pos_A;
      interrupts();
   }
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
   isr_speed_control = true;
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
   long pos;
   if (_channelA) {
      noInterrupts();
      pos = fas_pos_A;
      interrupts();
   }
   else {
      noInterrupts();
      pos = fas_pos_B;
      interrupts();
   }
   return pos;
}
bool FastAccelStepper::isRunning() {
   bool is_running;
   if (_channelA) {
      noInterrupts();
      is_running = (fas_pos_A != fas_target_pos_A);
      interrupts();
   }
   else {
      noInterrupts();
      is_running = (fas_q_readptr_B != fas_q_next_writeptr_B) ;
      interrupts();
   }
   return is_running;
}
