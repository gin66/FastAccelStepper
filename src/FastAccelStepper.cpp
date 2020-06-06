#include "FastAccelStepper.h"
#include "StepperISR.h"

#define stepPinStepperA 9  /* OC1A */
#define stepPinStepperB 10 /* OC1B */

// Here are the global variables to interface with the interrupts

// To realize the 1 Hz debug led
uint8_t fas_ledPin = 255;  // 255 if led blinking off
uint16_t fas_debug_led_cnt = 0;

#define TIMER_FREQ 16000000
#define UPM_TIMER_FREQ  ((upm_float)0x97f4)
#define UPM_TIMER_FREQ2 ((upm_float)0xafe8)

FastAccelStepper fas_stepperA = FastAccelStepper(true);
FastAccelStepper fas_stepperB = FastAccelStepper(false);

//*************************************************************************************************
//*************************************************************************************************
//
// Main purpose of FastAccelStepperEngine is timer 1 initialization and access to the steppers.
//
//*************************************************************************************************
//*************************************************************************************************
void FastAccelStepperEngine::init() {
  fas_stepperA.isr_speed_control_enabled = false;
  fas_stepperB.isr_speed_control_enabled = false;
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
//*************************************************************************************************
FastAccelStepper* FastAccelStepperEngine::stepperA() { return &fas_stepperA; }
//*************************************************************************************************
FastAccelStepper* FastAccelStepperEngine::stepperB() { return &fas_stepperB; }
//*************************************************************************************************
void FastAccelStepperEngine::setDebugLed(uint8_t ledPin) {
  fas_ledPin = ledPin;
}


//*************************************************************************************************
//*************************************************************************************************
//
// FastAccelStepper is associated with either stepperA or stepperB and provides:
// - movement control
//       either raw access to the stepper command queue
//       or ramp generator driven by speed/acceleration and move
// - stepper position
//
//*************************************************************************************************
//*************************************************************************************************

//*************************************************************************************************
bool FastAccelStepper::isStopped() { return _ticks_at_queue_end == 0; }
//*************************************************************************************************
void FastAccelStepper::add_queue_stepper_stop() { _ticks_at_queue_end = 0; }
//*************************************************************************************************
inline int FastAccelStepper::add_queue_entry(uint32_t start_delta_ticks,
                                             uint8_t steps, bool dir_high,
                                             int16_t change_ticks) {
  int32_t c_sum = 0;
  if (steps >= 128) {
    return AQE_STEPS_ERROR;
  }
  if (start_delta_ticks > 255L * 16384L + 65535L) {
    return AQE_TOO_HIGH;
  }
  if ((change_ticks != 0) && (steps > 1)) {
    c_sum = change_ticks * (steps - 1);
  }
  if (change_ticks > 0) {
    if (c_sum > 32768) {
      return AQE_CHANGE_TOO_HIGH;
    }
  } else if (change_ticks < 0) {
    if (c_sum < -32768) {
      return AQE_CHANGE_TOO_LOW;
    }
    if (start_delta_ticks + c_sum < min_delta_ticks()) {
      return AQE_CUMULATED_CHANGE_TOO_LOW;
    }
  }

  uint16_t msb = start_delta_ticks >> 14;
  uint16_t lsw;
  if (msb > 1) {
    msb--;
    lsw = start_delta_ticks & 16383;
    lsw |= 16384;
  } else {
    msb = 0;
    lsw = start_delta_ticks;
  }

  uint8_t wp;
  uint8_t rp;
  struct queue_entry* e;
  if (_channelA) {
    wp = fas_q_next_writeptr_A;
    rp = fas_q_readptr_A;
    e = &fas_queue_A[wp];
  } else {
    wp = fas_q_next_writeptr_B;
    rp = fas_q_readptr_B;
    e = &fas_queue_B[wp];
  }
  uint8_t next_wp = (wp + 1) & QUEUE_LEN_MASK;
  if (next_wp != rp) {
    _pos_at_queue_end += dir_high ? steps : -steps;
    _ticks_at_queue_end = change_ticks * (steps - 1) + start_delta_ticks;
    steps <<= 1;
    e->delta_msb = msb;
    e->delta_lsw = lsw;
    e->delta_change = change_ticks;
    e->steps = (dir_high != _dir_high_at_queue_end) ? steps | 0x01 : steps;
    _dir_high_at_queue_end = dir_high;
    if (_channelA) {
      fas_q_next_writeptr_A = next_wp;
    } else {
      fas_q_next_writeptr_B = next_wp;
    }
    return AQE_OK;
  }
  return AQE_FULL;
}

//*************************************************************************************************
// fill_queue generates commands to the stepper for executing a ramp
//
// Plan is to fill the queue with commmands with approx. 10 ms ahead (or more).
// For low speeds, this results in single stepping
// For high speeds (40kSteps/s) approx. 400 Steps to be created using 3 commands
//
// Required steps and time from speed v_1 to v_2 with constant acceleration a is
//
//      v(t) = v_1 + (v_2 - v_1) * t / T = v_1 + a * t
//
//      T = (v_2 - v_1) / a
//
// The steps over time are calculated like this
//
//      s(t) = int_0_t [v_1 + (v_2 - v_1) * t / T] dt
//           = [v_1 * t + (v_2 - v_1)/T * 1/2 * t²] at [0,t]
//           = v_1 * t + (v_2 - v_1)/T * 1/2 * t²
//           = v_1 * t + a * 1/2 * t²
//
// Consequently the required steps S for this speed change is:
//
//		S = s(T) = v_1 * (v_2 - v_1) / a + a * 1/2 * (v_2 - v_1)² /a²
//		         = v_1 * (v_2 - v_1) / a + (v_2 - v_1)² / 2a
//		         = [2 * v_1 + (v_2 - v_1)] * (v_2 - v_1) / 2a
//		         = [2 * v_1 + (v_2 - v_1)] * (v_2 - v_1) / 2a
//		         = [v_1 + v_2] * (v_2 - v_1) / 2a
//		         = [v_1 + v_2] * T / 2
//
// During the speed change the remaining time Tr and steps Sr are updated:
//
//		Sr = S - Δs
//      Tr = T + Δt
//
// At ramp start the ratio S to T is:
//
//		S / T = (v_1 + v_2) / 2
//
// With the factor F this ratio can be made identical to v_1 at ramp_start:
//
//		v_1 = F * S / T
//
//		F = 2 v_1 / (v_1 + v_2)
//
// During speed change the value will be updated as f:
//
//		f = 2 v / (v + v_2)
//
// An demonstration in python is in doc/ramp_calculation.py
//
//
// The formulas in short:
//    Special care for a: this must be negative for v_2 < v_1
//    ... or better use abs() for time difference
//    at ramp start:
//			T = abs(v_2 - v_1) / a
//		    S = [v_1 + v_2] * T / 2
//		    Tr = T
//		    Sr = S
//	  during ramp:
//			f = 2 v / (v + v_2)
//			v = f * Sr / Tr
//			Δs = v * Δt or Δt = Δs/v
//			Sr = Sr - Δs
//			Tr = Tr + Δt
//
// Technical implementation by using delta_ticks in TIMESTEPS and FREQ instead of speed.
//			TIMESTEPS * FREQ = 1
//			FREQ = 16 MHz
//
// Instead of T in s use Tx in TIMESTEPS
//
//			Tx = abs(v_2 - v_1) / a / TIMESTEPS
//			   = abs(FREQ/d_ticks_2 - FREQ/d_ticks_1)/a * FREQ
//			   = abs(FREQ²/(a*d_ticks_2) - FREQ²/(a*d_ticks_1))
//
// a is an uint16_t and d_ticks actually uint24_t. FREQ² is ~48 bits.
// Tx should be 32 bit, so ramp up time is limited to ~8 bits aka 256s.
//
// Similar for S:
//		    S = [v_1 + v_2] * T / 2
//		      = [FREQ/d_ticks_1 + FREQ/d_ticks_2] * Tx * TIMESTEPS / 2
//		      = [Tx/d_ticks_1 + Tx/d_ticks_2] / 2
//
// Tx is 32bits and d_ticks uint24_t. So S with 32bits is ok.
//
// Finally for F:
//		    F = 2 v_1 / (v_1 + v_2)
//		      = 2 (FREQ/d_ticks_1) / ((FREQ/d_ticks_1) + (FREQ/d_ticks_2))
//		      = 2 (1/d_ticks_1) / ((1/d_ticks_1) + (1/d_ticks_2))
//		      = 2 d_ticks_2 / (d_ticks_2 + d_ticks_1)
//
// Finally calculation of v aka d_ticks:
//
//			v = f * Sr / Tr
//			FREQ/d_ticks = F * Sr / Tr
//			d_ticks = FREQ * Tr/(F * Sr)
//			        = FREQ * Txr * TIMESTEPS / (F * Sr)
//			        = Txr / (F * Sr)
//
//	With G = 2 / F:
//		    G = 2 / F = (d_ticks_2 + d_ticks_1) / d_ticks_2
//	or g during ramp:
//		    g = 2 / f = (d_ticks_2 + d_ticks) / d_ticks_2
//			d_ticks_new = Txr * G / 2 / Sr
//
// Perhaps better to replace 32bit multiplication/division in ramp by a
// poor man's float implementation: 8 bit mantissa and 8 bit exponent
// For the stepper control sufficient. Only risk is numeric stability
// aka unwanted speed variations
//
// Updated formulas in short with forward looking speed:
//    at ramp start:
//			Txr = abs(FREQ²/(a*d_ticks_2) - FREQ²/(a*d_ticks_1))
//		    S = [Tx/d_ticks_1 + Tx/d_ticks_2] / 2
//	  during ramp:
//			Δs = Δt / d_ticks or Δt = Δs * d_ticks
//			Sr_scenario = Sr - Δs
//			Txr_scenario = Txr + Δt
//		    g = 1 / f = (d_ticks_2 + d_ticks) / d_ticks_2
//				IF d_ticks_2 equals standstill, then g = 1
//			d_ticks_new = Txr_scenario * g / 2 / Sr_scenario
//			Sr_new = Sr - Δs_executed
//			Txr_new = Txr - Δt_executed
//
// Any movement consists of three phases.
// 1. Change current speed to constant speed
// 2. Constant travel speed
// 3. Decelerate to stop
//
// With v_t being travel speed
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
// Steps for 3 (no emergency stop):
//     t_dec = v_t / a
//     s_3   = 1/2 * a * t_dec² = v_t² / 2a
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
//     this can be covered by a generic step 3, using constant decelaration
//     a_3:
//         s_remain = 1/2 * v * t_dec
//         t_dec = 2 s_remain / v
//         a_3 = v / t_dec = v^2 / 2 s_remain
//
// Assume current speed - represented by curr_ticks, then the next speed shall
// be:
//
// - in case of acceleration:
//		   next_ticks = curr_ticks / ( 1 + accel * dt * curr_ticks)
//
// - in case of deceleration:
//		   next_ticks = curr_ticks / ( 1 - accel * dt * curr_ticks)
//
// Rephrased
//		   next_ticks - curr_ticks = curr_ticks / ( 1 + accel * dt * curr_ticks) - curr_ticks
//		   next_ticks - curr_ticks = - accel * dt * curr_ticks²  / ( 1 + accel * dt * curr_ticks)
//                                 = - curr_ticks * x / (1 + x)
// with x = accel * dt * curr_ticks
//
// dt and curr_ticks are measured in units of 1/16000000s and steps/16000000s respectively.
// accel in steps/s^2. Thus the formula can be written dimensionless:
//
//	  x / (1 + x) = [accel] * [dt] * [curr_ticks] / (256*10^12 + [accel] * [dt] * [curr_ticks])
//
// 256*10^12 is approx 2^48 with 10% error
// accel is up to 16 bit
// dt is approx. 160000 to 4000000  aka 10ms..0.4s
// curr_ticks is approx. 640 to 4000000
//
//
//
//*************************************************************************************************
void FastAccelStepper::_calculate_move(int32_t move) {
  if (move == 0) {
    return;
  }
  uint32_t steps = abs(move);

  uint32_t curr_ticks = _ticks_at_queue_end;
  if (curr_ticks == 0) {  // motor start with minimum speed
    curr_ticks = _starting_ticks;
  }
  upm_float d_ticks_1 = upm_from(curr_ticks);
  upm_float d_ticks_2 = upm_from(_min_travel_ticks);
  upm_float p_2 = divide(UPM_TIMER_FREQ2,multiply(d_ticks_2,_accel));
  upm_float p_1 = divide(UPM_TIMER_FREQ2,multiply(d_ticks_1,_accel));
  upm_float upm_Tx = abs_diff(p_1,p_2);
  upm_float upm_S = shr(sum(divide(upm_Tx,d_ticks_1),divide(upm_Tx,d_ticks_2)),1);

  _ramp_up_Tx = upm_to_u32(upm_Tx);
  _ramp_down_Tx = _ramp_up_Tx;
  _ramp_up_S = upm_to_u32(upm_S);
  _ramp_down_S = _ramp_up_S;
  _deceleration_start = _ramp_down_S;
#ifdef TEST
	printf("Ramp data: starting_ticks = %d curr_ticks = %d travel_ticks = %d accel = %d Tx = %d S = %d\n",
			_starting_ticks,
			curr_ticks,
			_min_travel_ticks,
			upm_to_u32(_accel),
			upm_to_u32(upm_Tx),
			upm_to_u32(upm_S));
#endif
  isr_speed_control_enabled = true;
  return;
#ifdef OLD
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
  //     this can be covered by a generic step 3, using constant decelaration
  //     a_3:
  //         s_remain = 1/2 * v * t_dec
  //         t_dec = 2 s_remain / v
  //         a_3 = v / t_dec = v^2 / 2 s_remain
  //

  // Steps needed to stop from current speed with defined acceleration
  uint32_t new_deceleration_start;
  uint32_t new_dec_time_ms;
  if (_ticks_at_queue_end == 0) {
    // motor start with minimum speed
    _ticks_at_queue_end = round(16000000.0 * sqrt(2.0 / _accel));
  }
  float curr_speed = _ticks_at_queue_end ? 16000000.0 / _ticks_at_queue_end : 0;
  uint32_t s_stop = round(curr_speed * curr_speed / 2.0 / _accel);

  if (s_stop > steps) {
    // start deceleration immediately
    new_deceleration_start = steps;
    new_dec_time_ms = round(2000.0 * steps / curr_speed);
  } else if (_ticks_at_queue_end > _min_travel_ticks) {
    // add steps to reach current speed to full ramp
    uint32_t s_full_ramp = steps + s_stop;
    uint32_t ramp_steps = min(s_full_ramp, _min_steps);
    new_deceleration_start = ramp_steps / 2;
    new_dec_time_ms = round(sqrt(ramp_steps / _accel) * 1000.0);
  } else {
    // need decelerate first in phase 1, then normal deceleration
    new_deceleration_start = _min_steps / 2;
    new_dec_time_ms = round(_speed / _accel * 1000.0);
  }
#ifdef TEST
  printf("deceleration_start=%d  deceleration_time=%d ms\n",
         new_deceleration_start, new_dec_time_ms);
#endif
  noInterrupts();
  _deceleration_start = new_deceleration_start;
  _dec_time_ms = new_dec_time_ms;
  interrupts();
  isr_speed_control_enabled = true;
#endif
}

inline void FastAccelStepper::isr_fill_queue() {
  // Check preconditions to be allowed to fill the queue
  if (target_pos == _pos_at_queue_end) {
    isr_speed_control_enabled = false;
    return;
  }
  if (_min_travel_ticks == 0) {
    return;
  }
  while (!isQueueFull() && isr_speed_control_enabled) {
    isr_single_fill_queue();
  }
}

inline void FastAccelStepper::isr_single_fill_queue() {
  uint32_t runtime_us = micros();

  // preconditions are fulfilled, so create the command(s)

  // check state for acceleration/deceleration or deceleration to stop
  int32_t remaining_steps = target_pos - _pos_at_queue_end;
  bool accelerating = false;
  bool decelerate_to_stop = false;
  bool reduce_speed = false;
  uint32_t curr_ticks = _ticks_at_queue_end;
  if (curr_ticks == 0) {  // motor start with minimum speed
    curr_ticks = _starting_ticks;
  }
  if (abs(remaining_steps) <= _deceleration_start) {
    decelerate_to_stop = true;
  } else if (_min_travel_ticks < curr_ticks) {
    accelerating = true;
  } else if (_min_travel_ticks > curr_ticks) {
    reduce_speed = true;
  }

  // Forward planning of minimum 10ms or more on slow speed.
  uint32_t planning_ticks = max(curr_ticks, 16 * 10000);
#ifdef TEST
  printf("accel=%d  curr_ticks=%d dticks=%d   %s %s %s\n", upm_to_u32(_accel), curr_ticks,
         planning_ticks, accelerating ? "ACC" : "",
         decelerate_to_stop ? "STOP" : "", reduce_speed ? "RED" : "");
#endif

  // Calculate the new speed based on the planning_ticks aka Δt 
  uint32_t next_ticks = curr_ticks;
  if (accelerating) {
    uint32_t Tx_scenario = _ramp_up_Tx - planning_ticks;
	uint32_t S_scenario = _ramp_up_S - planning_ticks / curr_ticks;
    upm_float upm_Tx = upm_from(Tx_scenario);
    upm_float upm_S = upm_from(S_scenario);
	upm_float upm_d_ticks = upm_from(curr_ticks);
	upm_float upm_d_ticks_2 = upm_from(_min_travel_ticks);
    upm_float upm_G = divide(sum(upm_d_ticks,upm_d_ticks_2),upm_d_ticks_2);
	upm_float upm_d_ticks_new = shr(divide(multiply(upm_Tx,upm_G),upm_S),1);
	uint32_t d_ticks_new = upm_to_u32(upm_d_ticks_new);
    
    // avoid overshoot
    next_ticks = max(d_ticks_new, _min_travel_ticks);

    // avoid reduction
    next_ticks = min(next_ticks, curr_ticks);

#ifdef TEST
    printf("accelerate ticks=%d => %d  during %d ticks\n",
           curr_ticks, next_ticks, planning_ticks);
#endif
  }
  if (reduce_speed) {
    float delta =
        1.0 - _accel * planning_ticks * curr_ticks / 16000000.0 / 16000000.0;
    next_ticks = round(curr_ticks / delta);
    // avoid undershoot
    next_ticks = min(next_ticks, _min_travel_ticks);
#ifdef TEST
    printf("reduce ticks=%d => %d  during %d ticks (delta = %f)\n", curr_ticks,
           next_ticks, planning_ticks, delta);
#endif
  }
  if (decelerate_to_stop) {
    uint32_t Tx_scenario = _ramp_down_Tx - planning_ticks;
	uint32_t S_scenario = _ramp_down_S - planning_ticks / curr_ticks;
    upm_float upm_Tx = upm_from(Tx_scenario);
    upm_float upm_S = upm_from(S_scenario);
	upm_float upm_d_ticks = upm_from(curr_ticks);
	upm_float upm_d_ticks_new = shr(divide(upm_Tx,upm_S),1);
	uint32_t d_ticks_new = upm_to_u32(upm_d_ticks_new);

    // avoid undershoot
    next_ticks = max(next_ticks, _min_travel_ticks);
#ifdef TEST
    printf("declerate to stop ticks=%d => %d  during %d ticks\n", curr_ticks,
           next_ticks, planning_ticks);
#endif
//    curr_speed =
//        min(2 * abs(remaining_steps) * 1000.0 / _dec_time_ms, curr_speed);
//    ticks_after_command = round(16000000.0 / curr_speed);
#ifdef TEST
//    printf("towards stop with speed=%f  remaining time=%d\n", curr_speed,
//           _dec_time_ms);
#endif
  }
#ifdef TEST
  printf("=> expected ticks after command(s) = %d\n", next_ticks);
#endif

  // Number of steps to execute with limitation to min 1 and max remaining steps
  uint16_t total_steps = planning_ticks / next_ticks;
  total_steps = max(total_steps, 1);
  total_steps = min(total_steps, abs(remaining_steps));
  uint16_t steps = total_steps;

  // Calculate change per step
  int32_t total_change = (int32_t)next_ticks - (int32_t)curr_ticks;
  int32_t change = total_change;
  if (steps > 1) {
    change /= steps;  // each step will change
  }

  // Number of commands
  uint8_t command_cnt =
      min(steps, max(steps / 128 + 1, (abs(total_change) + 32767) / 32768));

  // Steps per command
  uint16_t steps_per_command = (steps + command_cnt - 1) / command_cnt;

  if (steps_per_command * command_cnt > steps) {
    steps_per_command -= 1;
  }

  if (accelerating) {
     _ramp_up_Tx -= curr_ticks*steps + total_change * steps / 2;
	 _ramp_up_S -= steps;
  }
  if (decelerate_to_stop) {
     _ramp_down_Tx -= curr_ticks*steps + total_change * steps / 2;
	 _ramp_down_S -= steps;
  }

  // Apply change to curr_ticks
  if (change == 0) {
    printf("curr=%d\n", curr_ticks);
    curr_ticks += total_change;
    printf("curr=%d\n", curr_ticks);
  } else {
    curr_ticks += change;
  }

  bool dir = remaining_steps > 0;

#ifdef TEST
  printf(
      "Issue %d commands for %d steps with %d steps per command for total "
      "change %d and %d change per step\n",
      command_cnt, steps, steps_per_command, total_change, change);
#endif

  for (uint16_t c = 1; c < command_cnt; c++) {
    int8_t res = add_queue_entry(curr_ticks, steps_per_command, dir, change);
#ifdef TEST
    printf(
        "add command %d Steps = %d start_ticks = %d  Target "
        "pos = %d "
        "Remaining steps = %d  tick_change=%d"
        " => res=%d   ticks_at_queue_end = %d\n",
        (command_cnt + 1 - c), steps_per_command, curr_ticks, target_pos,
        remaining_steps, change, res, _ticks_at_queue_end);
#endif
    curr_ticks += steps_per_command * change;
    steps -= steps_per_command;
  }
  int8_t res = add_queue_entry(curr_ticks, steps, dir, change);
#ifdef TEST
  printf(
      "add command Steps = %d start_ticks = %d  Target "
      "pos = %d "
      "Remaining steps = %d  tick_change=%d"
      " => res=%d   ticks_at_queue_end = %d\n",
      steps, curr_ticks, target_pos, remaining_steps, change, res,
      _ticks_at_queue_end);
#endif
  curr_ticks += steps_per_command * change;
  if (total_steps == abs(remaining_steps)) {
    add_queue_stepper_stop();
    isr_speed_control_enabled = false;
#ifdef TEST
    puts("Stepper stop");
#endif
  }
#ifdef TEST
  puts("");
#endif
  runtime_us = micros() - runtime_us;
  max_micros = max(max_micros, runtime_us);
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

FastAccelStepper::FastAccelStepper(bool channelA) {
  _channelA = channelA;
  _auto_enablePin = 255;
  target_pos = 0;
  _pos_at_queue_end = 0;
  _dir_high_at_queue_end = true;
  isr_speed_control_enabled = false;
  _min_travel_ticks = 0;
  _ticks_at_queue_end = 0;
  max_micros = 0;

  uint8_t pin = _channelA ? stepPinStepperA : stepPinStepperB;
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);

  // start interrupt
  if (channelA) {
    noInterrupts();
    OCR1A = 32768;  // definite start point
    StepperA_Disconnect;
    TCCR1C = _BV(FOC1A);    // force compare to ensure disconnect
    TIFR1 = _BV(OCF1A);     // clear interrupt flag
    TIMSK1 |= _BV(OCIE1A);  // enable compare A interrupt
    interrupts();
  } else {
    noInterrupts();
    OCR1B = 32768;  // definite start point
    StepperB_Disconnect;
    TCCR1C = _BV(FOC1B);    // force compare to ensure disconnect
    TIFR1 = _BV(OCF1B);     // clear interrupt flag
    TIMSK1 |= _BV(OCIE1B);  // enable compare B interrupt
    interrupts();
  }
}
uint32_t FastAccelStepper::min_delta_ticks() { return TIMER_FREQ / 32000; }
void FastAccelStepper::setDirectionPin(uint8_t dirPin) {
  _dirPin = dirPin;
  digitalWrite(dirPin, HIGH);
  pinMode(dirPin, OUTPUT);
  if (_channelA) {
    fas_dirPin_A = dirPin;
  } else {
    fas_dirPin_B = dirPin;
  }
}
void FastAccelStepper::setEnablePin(uint8_t enablePin) {
  _enablePin = enablePin;
  digitalWrite(enablePin, HIGH);
  pinMode(enablePin, OUTPUT);
}
void FastAccelStepper::set_auto_enable(bool auto_enable) {
  if (auto_enable) {
    _auto_enablePin = _enablePin;
  } else {
    _auto_enablePin = 255;
  }
  if (_channelA) {
    fas_autoEnablePin_A = _auto_enablePin;
  } else {
    fas_autoEnablePin_B = _auto_enablePin;
  }
}
void FastAccelStepper::setSpeed(uint32_t min_step_us) {
  _min_travel_ticks = min_step_us * 16;
//  _min_steps = round(16000000.0 * 16000000.0 / accel / min_travel_ticks /
//                     min_travel_ticks);
//  _starting_ticks = round(16000000.0 * sqrt(2.0 / _accel));
//  if (target_pos != _pos_at_queue_end) {
//    // moveTo(target_pos);
//  }
}
void FastAccelStepper::setAcceleration(uint16_t accel) {
  _starting_ticks = round(16000000.0 * sqrt(2.0 / (1.0*accel)));
  _accel = upm_from(accel);
}
void FastAccelStepper::move(int32_t move) {
  target_pos = _pos_at_queue_end + move;
  _calculate_move(move);
}
void FastAccelStepper::moveTo(int32_t position) {
  int32_t move;
  target_pos = position;
  move = position - _pos_at_queue_end;
  _calculate_move(move);
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
int32_t FastAccelStepper::getPositionAfterCommandsCompleted() {
  return _pos_at_queue_end;
}
int32_t FastAccelStepper::getCurrentPosition() {
  int32_t pos = _pos_at_queue_end;
  bool dir = _dir_high_at_queue_end;
  struct queue_entry* q;
  uint8_t rp, wp;
  noInterrupts();
  if (_channelA) {
    q = fas_queue_A;
    rp = fas_q_readptr_A;
    wp = fas_q_next_writeptr_A;
  } else {
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
        pos -= steps >> 1;
      } else {
        pos += steps >> 1;
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
    full = (((fas_q_next_writeptr_A + 1) & QUEUE_LEN_MASK) == fas_q_readptr_A);
  } else {
    full = (((fas_q_next_writeptr_B + 1) & QUEUE_LEN_MASK) == fas_q_readptr_B);
  }
  return full;
}
bool FastAccelStepper::isQueueEmpty() {
  bool empty;
  if (_channelA) {
    empty = (fas_q_readptr_A == fas_q_next_writeptr_A);
  } else {
    empty = (fas_q_readptr_B == fas_q_next_writeptr_B);
  }
  return empty;
}
bool FastAccelStepper::isRunning() { return !isQueueEmpty(); }
