#include "FastAccelStepper.h"
#include "StepperISR.h"

#define stepPinStepperA 9  /* OC1A */
#define stepPinStepperB 10 /* OC1B */

// Here are the global variables to interface with the interrupts

// To realize the 1 Hz debug led
uint8_t fas_ledPin = 255;  // 255 if led blinking off
uint16_t fas_debug_led_cnt = 0;

FastAccelStepper fas_stepperA = FastAccelStepper(true);
FastAccelStepper fas_stepperB = FastAccelStepper(false);

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
inline int FastAccelStepper::add_queue_entry(uint32_t start_delta_ticks,
                                             uint8_t steps, bool dir_high,
                                             int16_t change_ticks) {
  int32_t c_sum = 0;
  if (steps >= 128) {
    return AQE_STEPS_ERROR;
  }
  if (start_delta_ticks > 255 * 16384 + 65535) {
    return AQE_TOO_HIGH;
  }
  if (change_ticks != 0) {
    c_sum = change_ticks;
    c_sum *= steps;
    c_sum *= (steps - 1);
    c_sum /= 2;
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
      return AQE_CHANGE_TOO_LOW;
    }
  }

  uint16_t msb = start_delta_ticks >> 14;
  uint16_t lsw;
  if (msb > 1) {
    msb--;
    lsw = start_delta_ticks & 16383;
    lsw |= 16384;
  } else {
    lsw = start_delta_ticks;
  }

  if (_channelA) {
    uint8_t wp = fas_q_next_writeptr_A;
    uint8_t rp = fas_q_readptr_A;
    uint8_t next_wp = (wp + 1) & QUEUE_LEN_MASK;
    if (next_wp != rp) {
      _pos_at_queue_end += dir_high ? steps : -steps;
      steps <<= 1;
      struct queue_entry* e = &fas_queue_A[wp];
      e->delta_msb = msb;
      e->delta_lsw = lsw;
      e->delta_change = change_ticks;
      e->steps = (dir_high != _dir_high_at_queue_end) ? steps | 0x01 : steps;
      _ticks_at_queue_end = change_ticks * (steps - 1) + start_delta_ticks;
      _dir_high_at_queue_end = dir_high;
      fas_q_next_writeptr_A = next_wp;
      return AQE_OK;
    }
  } else {
    uint8_t wp = fas_q_next_writeptr_B;
    uint8_t rp = fas_q_readptr_B;
    uint8_t next_wp = (wp + 1) & QUEUE_LEN_MASK;
    if (next_wp != rp) {
      _pos_at_queue_end += dir_high ? steps : -steps;
      steps <<= 1;
      struct queue_entry* e = &fas_queue_B[wp];
      e->delta_msb = msb;
      e->delta_lsw = lsw;
      e->delta_change = change_ticks;
      e->steps = (dir_high != _dir_high_at_queue_end) ? steps | 0x01 : steps;
      _ticks_at_queue_end = change_ticks * (steps - 1) + start_delta_ticks;
      _dir_high_at_queue_end = dir_high;
      fas_q_next_writeptr_B = next_wp;
      return AQE_OK;
    }
  }
  return AQE_FULL;
}

inline void FastAccelStepper::isr_fill_queue() {
  if (!isr_speed_control_enabled) {
    return;
  }
  if (isQueueFull()) {
    return;
  }
  if (isQueueEmpty()) {
    if (target_pos == _pos_at_queue_end) {
      _last_ms = 0;
      isr_speed_control_enabled = false;
      return;
    }
  }
  long remaining_steps = target_pos - _pos_at_queue_end;
  bool accelerating = false;
  bool decelerate_to_stop = false;
  bool reduce_speed = false;
  if (abs(remaining_steps) <= _deceleration_start) {
    decelerate_to_stop = true;
  } else if (_curr_speed < _speed) {
    accelerating = true;
  } else if (_curr_speed > _speed) {
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
    _dec_time_ms = max(_dec_time_ms - dt_ms, 1.0);
    _curr_speed = 2 * abs(remaining_steps) * 1000.0 / _dec_time_ms;
  }
  if (reduce_speed) {
    _curr_speed -= _accel / 1000.0 * dt_ms;
    _curr_speed = max(_curr_speed, _speed);
  }
  unsigned long delta = round(16000000.0 / _curr_speed);

  uint16_t steps = (16000000 * 8 / 1000) / delta;  // How many steps in 8 ms ?
  steps = min(steps, 127);
  steps = max(steps, 1);
  steps = min(steps, abs(remaining_steps));

#ifdef TEST
  int8_t res = add_queue_entry(delta, steps, remaining_steps > 0, 0);
  printf(
      "add Delta = %ld  Steps = %d  Queue End Pos = %ld  Target pos = %ld Remaining steps = %ld "
      " => %d\n",
      delta, steps, _pos_at_queue_end, target_pos, remaining_steps, res);
#else
  add_queue_entry(delta, steps, remaining_steps > 0, 0);
#endif
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

FastAccelStepper* FastAccelStepperEngine::stepperA() { return &fas_stepperA; }
FastAccelStepper* FastAccelStepperEngine::stepperB() { return &fas_stepperB; }

FastAccelStepper::FastAccelStepper(bool channelA) {
  _channelA = channelA;
  _auto_enablePin = 255;
  _curr_speed = 0.0;
  target_pos = 0;
  _pos_at_queue_end = 0;
  _dir_high_at_queue_end = true;
  isr_speed_control_enabled = false;

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
uint32_t FastAccelStepper::min_delta_ticks() { return 16000000 / 32000; }
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
void FastAccelStepper::set_dynamics(float speed, float accel) {
  _speed = speed;
  _accel = accel;
  _min_steps = round(speed * speed / accel);
  if (target_pos != _pos_at_queue_end) {
    moveTo(target_pos);
  }
}
void FastAccelStepper::move(long move) {
  target_pos = _pos_at_queue_end + move;
  _calculate_move(move);
}
void FastAccelStepper::moveTo(long position) {
  long move;
  target_pos = position;
  move = position - _pos_at_queue_end;
  _calculate_move(move);
}
void FastAccelStepper::_calculate_move(long move) {
  if (move == 0) {
    return;
  }
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
  //     this can be covered by a generic step 3, using constant decelaration
  //     a_3:
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
  } else if (_curr_speed <= _speed) {
    // add steps to reach current speed to full ramp
    unsigned long s_full_ramp = steps + s_stop;
    unsigned long ramp_steps = min(s_full_ramp, _min_steps);
    new_deceleration_start = ramp_steps / 2;
    new_dec_time_ms = round(sqrt(ramp_steps / _accel) * 1000.0);
  } else {
    // need decelerate first in phase 1, then normal deceleration
    new_deceleration_start = _min_steps / 2;
    new_dec_time_ms = round(_speed / _accel * 1000.0);
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
long FastAccelStepper::getPositionAfterCommandsCompleted() {
	return _pos_at_queue_end;
}
long FastAccelStepper::getCurrentPosition() {
  long pos = _pos_at_queue_end;
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
