#include "FastAccelStepper.h"

#include "StepperISR.h"

#ifdef TEST
#include <assert.h>
#endif

// This define in order to not shoot myself.
#ifndef TEST
#define printf DO_NOT_USE_PRINTF
#endif

// Here are the global variables to interface with the interrupts

// To realize the 1 Hz debug led
static uint8_t fas_ledPin = PIN_UNDEFINED;
static uint16_t fas_debug_led_cnt = 0;
#if defined(ARDUINO_ARCH_AVR)
#define DEBUG_LED_HALF_PERIOD 144
#elif defined(ARDUINO_ARCH_ESP32)
#define DEBUG_LED_HALF_PERIOD 50
#else
#define DEBUG_LED_HALF_PERIOD 50
#endif

#if (TICKS_PER_S == 16000000)
#define UPM_TICKS_PER_S ((upm_float)0x97f4)
#else
upm_float upm_timer_freq;
#define UPM_TICKS_PER_S upm_timer_freq
#endif

#if defined(ARDUINO_ARCH_AVR)
// this is needed to give the background task isr access to engine
static FastAccelStepperEngine* fas_engine = NULL;

// dynamic allocation seems to not work so well on avr
FastAccelStepper fas_stepper[MAX_STEPPER] = {FastAccelStepper(),
                                             FastAccelStepper()};
#endif
#if defined(ARDUINO_ARCH_ESP32)
FastAccelStepper fas_stepper[MAX_STEPPER] = {
    FastAccelStepper(), FastAccelStepper(), FastAccelStepper(),
    FastAccelStepper(), FastAccelStepper(), FastAccelStepper()};
#endif
#if defined(TEST)
FastAccelStepper fas_stepper[MAX_STEPPER] = {FastAccelStepper(),
                                             FastAccelStepper()};
#endif

//*************************************************************************************************
//*************************************************************************************************
//
// Main purpose of FastAccelStepperEngine is timer 1 initialization and access
// to the steppers.
//
//*************************************************************************************************
#if defined(ARDUINO_ARCH_ESP32)
#define TASK_DELAY_MS 10
void StepperTask(void* parameter) {
  FastAccelStepperEngine* engine = (FastAccelStepperEngine*)parameter;
  const TickType_t delay =
      TASK_DELAY_MS / portTICK_PERIOD_MS;  // block for 10ms
  while (true) {
    engine->manageSteppers();
    vTaskDelay(delay);
  }
}
#endif
//*************************************************************************************************
void FastAccelStepperEngine::init() {
#if (TICKS_PER_S != 16000000)
  upm_timer_freq = upm_from((uint32_t)TICKS_PER_S);
#endif
#if defined(ARDUINO_ARCH_AVR)
  fas_engine = this;

  // Initialize timer for stepper background task and correct time base
  noInterrupts();

  // Set WGM13:0 to all zero => Normal mode
  TCCR1A &= ~(_BV(WGM11) | _BV(WGM10));
  TCCR1B &= ~(_BV(WGM13) | _BV(WGM12));

  // Set prescaler to 1
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11) | _BV(CS10))) | _BV(CS10);

  // enable OVF interrupt
  TIMSK1 |= _BV(TOIE1);

  interrupts();
#endif
#if defined(ARDUINO_ARCH_ESP32)
#define STACK_SIZE 1000
#define PRIORITY 1
  xTaskCreate(StepperTask, "StepperTask", STACK_SIZE, this, PRIORITY, NULL);
#endif
}
//*************************************************************************************************
bool FastAccelStepperEngine::_isValidStepPin(uint8_t step_pin) {
#if defined(ARDUINO_ARCH_AVR)
  return ((step_pin == stepPinStepperA) || (step_pin == stepPinStepperB));
#elif defined(ARDUINO_ARCH_ESP32)
  return true;  // for now
#else
  return true;
#endif
}
//*************************************************************************************************
FastAccelStepper* FastAccelStepperEngine::stepperConnectToPin(
    uint8_t step_pin) {
  // Check if already connected
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    FastAccelStepper* s = _stepper[i];
    if (s) {
      if (s->getStepPin() == step_pin) {
        return NULL;
      }
    }
  }
  if (!_isValidStepPin(step_pin)) {
    return NULL;
  }
  uint8_t fas_stepper_num = 0;
#if defined(ARDUINO_ARCH_AVR)
  // The stepper connection is hardcoded for AVR
  if (step_pin == stepPinStepperA) {
    fas_stepper_num = 0;
  } else {
    fas_stepper_num = 1;
  }
#endif
#if defined(ARDUINO_ARCH_ESP32)
  if (_next_stepper_num >= MAX_STEPPER) {
    return NULL;
  }
  fas_stepper_num = _next_stepper_num;
#endif
  uint8_t stepper_num = _next_stepper_num;
  _next_stepper_num++;

#if defined(ARDUINO_ARCH_AVR) || defined(ESP32) || defined(TEST)
  FastAccelStepper* s = &fas_stepper[fas_stepper_num];
  _stepper[stepper_num] = s;
  s->init(fas_stepper_num, step_pin);
  return s;
#else
  return NULL;
#endif
}
//*************************************************************************************************
void FastAccelStepperEngine::setDebugLed(uint8_t ledPin) {
  fas_ledPin = ledPin;
  pinMode(fas_ledPin, OUTPUT);
  digitalWrite(fas_ledPin, LOW);
}
//*************************************************************************************************
void FastAccelStepperEngine::manageSteppers() {
#ifndef TEST
  if (fas_ledPin != PIN_UNDEFINED) {
    fas_debug_led_cnt++;
    if (fas_debug_led_cnt == DEBUG_LED_HALF_PERIOD) {
      digitalWrite(fas_ledPin, HIGH);
    }
    if (fas_debug_led_cnt == 2 * DEBUG_LED_HALF_PERIOD) {
      digitalWrite(fas_ledPin, LOW);
      fas_debug_led_cnt = 0;
    }
  }
#endif
  for (uint8_t i = 0; i < _next_stepper_num; i++) {
    FastAccelStepper* s = _stepper[i];
    if (s) {
      s->manage();
    }
  }
}

//*************************************************************************************************
//*************************************************************************************************
//
// FastAccelStepper provides:
// - movement control
//       either raw access to the stepper command queue
//       or ramp generator driven by speed/acceleration and move
// - stepper position
//
//*************************************************************************************************
//*************************************************************************************************

//*************************************************************************************************
int FastAccelStepper::addQueueEntry(uint32_t delta_ticks, uint8_t steps,
                                    bool dir_high) {
  uint16_t delay_counter = 0;
  if (_autoEnable) {
    noInterrupts();
    delay_counter = _auto_disable_delay_counter;
    interrupts();
    enableOutputs();
  }
  int res = fas_queue[_queue_num].addQueueEntry(delta_ticks, steps, dir_high);
  if (_autoEnable) {
    if (res == AQE_OK) {
      delay_counter = _off_delay_count;
    }
    noInterrupts();
    _auto_disable_delay_counter = delay_counter;
    interrupts();
  }

  return res;
}

//*************************************************************************************************
// fill_queue generates commands to the stepper for executing a ramp
//
// Plan is to fill the queue with commmands with approx. 10 ms ahead (or more).
// For low speeds, this results in single stepping
// For high speeds (40kSteps/s) approx. 400 Steps to be created using 3 commands
//
// Basis of the calculation is the relation between steps and time via
// acceleration a:
//
//		s = 1/2 * a * t²
//
// With v = a * t for the acceleration case, then v can be deducted:
//
//		s = 1/2 * v² / a
//
//	    v = sqrt(2 * s * a)
//
//*************************************************************************************************
void FastAccelStepper::_calculate_move(int32_t move) {
  inject_fill_interrupt(1);
  if (move == 0) {
    return;
  }
  if ((move < 0) && (_dirPin == PIN_UNDEFINED)) {
    return;
  }
  if (_min_step_us == 0) {
    return;
  }
  if (_accel == 0) {
    return;
  }
  _update_from_speed_acceleration();
  uint32_t steps = abs(move);

  uint32_t curr_ticks = fas_queue[_queue_num].ticks_at_queue_end;
  uint32_t performed_ramp_up_steps;
  uint32_t deceleration_start;
  uint32_t ramp_steps = _ramp_steps;
  if ((curr_ticks == TICKS_FOR_STOPPED_MOTOR) || isQueueEmpty()) {
    // motor is not running
    //
    // ramp starts with s = 0.
    //
    performed_ramp_up_steps = 0;

    // If the maximum speed cannot be reached due to too less steps,
    // then in the isr_single_fill_queue routine the deceleration will be
    // started after move/2 steps.
    deceleration_start = min(ramp_steps, steps / 2);
  } else if (curr_ticks == _min_travel_ticks) {
    // motor is running already at coast speed
    //
    performed_ramp_up_steps = ramp_steps;
    deceleration_start = ramp_steps;
  } else {
    // motor is running
    //
    // Calculate on which step on the speed ramp the current speed is related to
    performed_ramp_up_steps =
        upm_to_u32(divide(_upm_inv_accel2, square(upm_from(curr_ticks))));
    if (curr_ticks >= _min_travel_ticks) {
      // possibly can speed up
      // Full ramp up/down needs 2*ramp_steps
      // => full ramp is possible, if move+performed_ramp_up_steps >
      // 2*ramp_steps
      deceleration_start =
          min(ramp_steps, (move + performed_ramp_up_steps) / 2);
    } else if (curr_ticks < _min_travel_ticks) {
      // speed too high, so need to reduce to _min_travel_ticks speed
      deceleration_start = ramp_steps;
    }
  }

  noInterrupts();
  _deceleration_start = deceleration_start;
  _performed_ramp_up_steps = performed_ramp_up_steps;
  _isr_speed_control_enabled = true;
  interrupts();
  inject_fill_interrupt(2);

#ifdef TEST
  printf(
      "Ramp data: steps to move = %d  curr_ticks = %d travel_ticks = %d "
      "Ramp steps = %d Performed ramp steps = %d deceleration start = %u\n",
      steps, curr_ticks, _min_travel_ticks, ramp_steps, performed_ramp_up_steps,
      deceleration_start);
#endif
#ifdef DEBUG
  char buf[256];
  sprintf(
      buf,
      "Ramp data: steps to move = %ld  curr_ticks = %ld travel_ticks = %ld "
      "Ramp steps = %ld Performed ramp steps = %ld deceleration start = %lu\n",
      steps, curr_ticks, _min_travel_ticks, ramp_steps, performed_ramp_up_steps,
      deceleration_start);
  Serial.println(buf);
#endif
}

inline void FastAccelStepper::isr_fill_queue() {
  // Check preconditions to be allowed to fill the queue
  if (_target_pos == getPositionAfterCommandsCompleted()) {
    _isr_speed_control_enabled = false;
    return;
  }
  if (_min_travel_ticks == 0) {
#ifdef TEST
	  assert(false);
#endif
  	  return;
  }

  // preconditions are fulfilled, so create the command(s)
  while (!isQueueFull() && _isr_speed_control_enabled) {
    isr_single_fill_queue();
  }
}

inline void FastAccelStepper::isr_single_fill_queue() {
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
  // For run time measurement
  uint32_t runtime_us = micros();
#endif

  uint32_t ticks_at_queue_end = fas_queue[_queue_num].ticks_at_queue_end;
  if (ticks_at_queue_end == 0) {
    ticks_at_queue_end = TICKS_FOR_STOPPED_MOTOR;
  }

  // check state for acceleration/deceleration or deceleration to stop
  uint32_t remaining_steps =
      abs(_target_pos - getPositionAfterCommandsCompleted());
  uint32_t planning_steps;
  uint32_t next_ticks;
  if (_rampState == RAMP_STATE_IDLE) {  // motor is stopped. Set to max value
    planning_steps = 1;
    _rampState = RAMP_STATE_ACCELERATE;
  } else {
    // TODO:explain the 16000
    planning_steps = max(16000 / ticks_at_queue_end, 1);
    if (remaining_steps <= _deceleration_start) {
      _rampState = RAMP_STATE_DECELERATE_TO_STOP;
    } else if (_min_travel_ticks < ticks_at_queue_end) {
      _rampState = RAMP_STATE_ACCELERATE;
    } else if (_min_travel_ticks > ticks_at_queue_end) {
      _rampState = RAMP_STATE_DECELERATE;
    } else {
      _rampState = RAMP_STATE_COAST;
    }
  }

  // Forward planning of minimum 10ms or more on slow speed.

#ifdef TEST
  printf(
      "pos@queue_end=%d remaining=%u deceleration_start=%u planning steps=%d  "
      " ",
      getPositionAfterCommandsCompleted(), remaining_steps, _deceleration_start,
      planning_steps);
  switch (_rampState) {
    case RAMP_STATE_COAST:
      printf("COAST");
      break;
    case RAMP_STATE_ACCELERATE:
      printf("ACC");
      break;
    case RAMP_STATE_DECELERATE:
      printf("DEC");
      break;
    case RAMP_STATE_DECELERATE_TO_STOP:
      printf("STOP");
      break;
  }
  printf("\n");
#endif

  uint32_t curr_ticks = ticks_at_queue_end;
#ifdef TEST
  float v2;
#endif
  switch (_rampState) {
    uint32_t d_ticks_new;
    uint32_t upm_rem_steps;
    upm_float upm_d_ticks_new;
    case RAMP_STATE_COAST:
      next_ticks = _min_travel_ticks;
      // do not overshoot ramp down start
      planning_steps =
          min(planning_steps, remaining_steps - _deceleration_start);
      break;
    case RAMP_STATE_ACCELERATE:
      upm_rem_steps = upm_from(_performed_ramp_up_steps + planning_steps);
      upm_d_ticks_new = sqrt(divide(_upm_inv_accel2, upm_rem_steps));

      d_ticks_new = upm_to_u32(upm_d_ticks_new);

      // avoid overshoot
      next_ticks = max(d_ticks_new, _min_travel_ticks);
      if (_performed_ramp_up_steps == 0) {
        curr_ticks = d_ticks_new;
      } else {
        // CLIPPING: avoid increase
        next_ticks = min(next_ticks, curr_ticks);
      }

#ifdef TEST
      printf("accelerate ticks => %d  during %d ticks (d_ticks_new = %u)\n",
             next_ticks, planning_steps, d_ticks_new);
      printf("... %u+%u steps\n", _performed_ramp_up_steps, planning_steps);
#endif
      break;
    case RAMP_STATE_DECELERATE:
      upm_rem_steps = upm_from(_performed_ramp_up_steps + planning_steps);
      upm_d_ticks_new = sqrt(divide(_upm_inv_accel2, upm_rem_steps));

      d_ticks_new = upm_to_u32(upm_d_ticks_new);

      // avoid undershoot
      next_ticks = min(d_ticks_new, _min_travel_ticks);

      // CLIPPING: avoid reduction
      next_ticks = max(next_ticks, curr_ticks);

#ifdef TEST
      printf("decelerate ticks => %d  during %d ticks (d_ticks_new = %u)\n",
             next_ticks, planning_steps, d_ticks_new);
      printf("... %u+%u steps\n", _performed_ramp_up_steps, planning_steps);
#endif
      break;
    case RAMP_STATE_DECELERATE_TO_STOP:
      upm_rem_steps = upm_from(remaining_steps - planning_steps);
      upm_d_ticks_new = sqrt(divide(_upm_inv_accel2, upm_rem_steps));

      d_ticks_new = upm_to_u32(upm_d_ticks_new);

      // avoid undershoot
      next_ticks = max(d_ticks_new, _min_travel_ticks);

      // CLIPPING: avoid reduction
      next_ticks = max(next_ticks, curr_ticks);
#ifdef TEST
      printf("decelerate ticks => %d  during %d ticks (d_ticks_new = %u)\n",
             next_ticks, planning_steps, d_ticks_new);
#endif
      break;
    default:
      // TODO: how to treat this (error) case ?
      next_ticks = curr_ticks;
#ifdef TEST
      assert(false);
#endif
  }

  // CLIPPING: avoid increase
  next_ticks = min(next_ticks, ABSOLUTE_MAX_TICKS);

  // Number of steps to execute with limitation to min 1 and max remaining steps
  uint16_t total_steps = planning_steps;
#ifdef TEST
  printf(
      "total_steps for the command = %d  with planning_steps = %u and "
      "next_ticks = %u\n",
      total_steps, planning_steps, next_ticks);
#endif
  total_steps = max(total_steps, 1);
  total_steps = min(total_steps, abs(remaining_steps));
  uint16_t steps = total_steps;

  // Calculate change per step
  int32_t total_change = (int32_t)next_ticks - (int32_t)curr_ticks;

  // Number of commands
  uint8_t command_cnt = steps / 128 + 1;

  // Steps per command
  uint16_t steps_per_command = (steps + command_cnt - 1) / command_cnt;

  if (steps_per_command * command_cnt > steps) {
    steps_per_command -= 1;
  }

  int32_t change_per_command = total_change / command_cnt;

  if (_rampState == RAMP_STATE_ACCELERATE) {
    _performed_ramp_up_steps += steps;
  }
  if (_rampState == RAMP_STATE_DECELERATE) {
    _performed_ramp_up_steps -= steps;
  }
  // Apply change to curr_ticks
  if (command_cnt > 1) {
    curr_ticks += change_per_command / 2;
  } else {
    curr_ticks = next_ticks;
  }

  bool dir =
      (_target_pos > getPositionAfterCommandsCompleted()) == _dirHighCountsUp;

#ifdef TEST
  if (command_cnt > 1) {
    printf(
        "Issue %d commands for %d steps with %d steps/command for total "
        "change %d and %d change/command and start with %u ticks\n",
        command_cnt, steps, steps_per_command, total_change, change_per_command,
        curr_ticks);
  } else {
    printf(
        "Issue %d command for %d steps "
        "and start with %u ticks\n",
        command_cnt, steps, curr_ticks);
  }
  assert(curr_ticks > 0);
#endif

  for (uint16_t c = 0; c < command_cnt; c++) {
    int8_t res = addQueueEntry(curr_ticks, steps_per_command, dir);
#ifdef TEST
    printf(
        "add command %d Steps = %d start_ticks = %d  Target "
        "pos = %d "
        "Remaining steps = %d  tick_change=%d"
        " => res=%d   ticks_at_queue_end = %d\n",
        (command_cnt + 1 - c), steps_per_command, curr_ticks, _target_pos,
        remaining_steps, change_per_command, res,
        fas_queue[_queue_num].ticks_at_queue_end);
#endif
    if (res != 0) {
      if (res == AQE_FULL) {
        return;
      }
      // Emergency stop on internal error
      _rampState = RAMP_STATE_IDLE;
      _isr_speed_control_enabled = false;
      return;
    }
    steps -= steps_per_command;
    curr_ticks += change_per_command;
  }
  if (total_steps == abs(remaining_steps)) {
    _rampState = RAMP_STATE_IDLE;
    _isr_speed_control_enabled = false;
#ifdef TEST
    puts("Stepper stop");
#endif
  }

#ifdef TEST
  puts("");
#endif

#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
  // For run time measurement
  runtime_us = micros() - runtime_us;
  max_micros = max(max_micros, runtime_us);
#endif
}
#if defined(ARDUINO_ARCH_AVR)
ISR(TIMER1_OVF_vect) {
  // disable OVF interrupt to avoid nesting
  TIMSK1 &= ~_BV(TOIE1);

  // enable interrupts for nesting
  interrupts();

  // manage steppers
  fas_engine->manageSteppers();

  // enable OVF interrupt again
  TIMSK1 |= _BV(TOIE1);
}
#endif

void FastAccelStepper::check_for_auto_disable() {
  if (_auto_disable_delay_counter > 0) {
    if (!isRunning()) {
      _auto_disable_delay_counter--;
      if (_auto_disable_delay_counter == 0) {
        disableOutputs();
      }
    }
  }
}

void FastAccelStepper::init(uint8_t num, uint8_t step_pin) {
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
  // For run time measurement
  max_micros = 0;
#endif
  _target_pos = 0;
  _isr_speed_control_enabled = false;
  _rampState = RAMP_STATE_IDLE;
  _autoEnable = false;
  _off_delay_count = 1;  // ensure to call disableOutputs()
  _auto_disable_delay_counter = 0;
  _min_travel_ticks = 0;
  _stepPin = step_pin;
  _dirHighCountsUp = true;

#if defined(ARDUINO_ARCH_AVR)
  _queue_num = step_pin == stepPinStepperA ? 0 : 1;
#elif defined(ARDUINO_ARCH_ESP32)
  _queue_num = num;
#else
  _queue_num = num;
#endif
  fas_queue[_queue_num].init(_queue_num, step_pin);
}
uint8_t FastAccelStepper::getStepPin() { return _stepPin; }
void FastAccelStepper::setDirectionPin(uint8_t dirPin, bool dirHighCountsUp) {
  _dirPin = dirPin;
  _dirHighCountsUp = dirHighCountsUp;
  digitalWrite(dirPin, HIGH);
  pinMode(dirPin, OUTPUT);
  fas_queue[_queue_num].dirPin = dirPin;
  fas_queue[_queue_num].dirHighCountsUp = dirHighCountsUp;
}
void FastAccelStepper::setEnablePin(uint8_t enablePin,
                                    bool low_active_enables_stepper) {
  if (low_active_enables_stepper) {
    _enablePinLowActive = enablePin;
    digitalWrite(enablePin, HIGH);
    pinMode(enablePin, OUTPUT);
    if (_enablePinHighActive == enablePin) {
      _enablePinHighActive = PIN_UNDEFINED;
    }
  } else {
    _enablePinHighActive = enablePin;
    digitalWrite(enablePin, LOW);
    pinMode(enablePin, OUTPUT);
    if (_enablePinLowActive == enablePin) {
      _enablePinLowActive = PIN_UNDEFINED;
    }
  }
}
void FastAccelStepper::setAutoEnable(bool auto_enable) {
  _autoEnable = auto_enable;
}
int FastAccelStepper::setDelayToEnable(uint32_t delay_us) {
  uint32_t delay_ticks = delay_us * (TICKS_PER_S / 1000L) / 1000L;
  if (delay_us < 1000) {
    return DELAY_TOO_LOW;
  }
  if (delay_ticks > ABSOLUTE_MAX_TICKS) {
    return DELAY_TOO_HIGH;
  }
  fas_queue[_queue_num].on_delay_ticks = delay_ticks;
  return DELAY_OK;
}
void FastAccelStepper::setDelayToDisable(uint16_t delay_ms) {
  uint16_t delay_count = 0;
#if defined(ARDUINO_ARCH_ESP32)
  delay_count = delay_ms / TASK_DELAY_MS;
#endif
#if defined(ARDUINO_ARCH_AVR)
  delay_count = delay_ms / (65536000 / TICKS_PER_S);
#endif
  if ((delay_ms > 0) && (delay_count < 2)) {
    // ensure minimum time
    delay_count = 2;
  }
  _off_delay_count = delay_count;
}
void FastAccelStepper::setSpeed(uint32_t min_step_us) {
  if (min_step_us == 0) {
    return;
  }
  _min_step_us = min_step_us;
}
void FastAccelStepper::setAcceleration(uint32_t accel) {
  if (accel == 0) {
    return;
  }
  _accel = accel;
}
void FastAccelStepper::_update_from_speed_acceleration() {
  _min_travel_ticks = _min_step_us * (TICKS_PER_S / 1000L) / 1000L;
  uint32_t tmp = TICKS_PER_S / 2;
  upm_float upm_inv_accel = upm_from(tmp / _accel);
  _upm_inv_accel2 = multiply(UPM_TICKS_PER_S, upm_inv_accel);
  _ramp_steps =
      upm_to_u32(divide(_upm_inv_accel2, square(upm_from(_min_travel_ticks))));
}
void FastAccelStepper::moveTo(int32_t position) {
  int32_t curr_pos = getPositionAfterCommandsCompleted();
  if (!isrSpeedControlEnabled()) {
    _target_pos = curr_pos;
  }
  int32_t move;
  move = position - curr_pos;
  if (move == 0) {
    return;
  }
  if ((_target_pos > curr_pos) && (move < 0)) {
    return;
  }
  if ((_target_pos < curr_pos) && (move > 0)) {
    return;
  }
  _target_pos = position;
  _calculate_move(move);
}
void FastAccelStepper::move(int32_t move) {
  if (!isrSpeedControlEnabled()) {
    _target_pos = getPositionAfterCommandsCompleted();
  }
  moveTo(_target_pos + move);
}
void FastAccelStepper::stopMove() {
  if (isRunning() && isrSpeedControlEnabled()) {
    int32_t curr_pos = getPositionAfterCommandsCompleted();
    if (_target_pos > curr_pos) {
      moveTo(curr_pos + _performed_ramp_up_steps);
    } else {
      moveTo(curr_pos - _performed_ramp_up_steps);
    }
  }
}
void FastAccelStepper::disableOutputs() {
  if (_enablePinLowActive != PIN_UNDEFINED) {
    digitalWrite(_enablePinLowActive, HIGH);
  }
  if (_enablePinHighActive != PIN_UNDEFINED) {
    digitalWrite(_enablePinHighActive, LOW);
  }
}
void FastAccelStepper::enableOutputs() {
  if (_enablePinLowActive != PIN_UNDEFINED) {
    digitalWrite(_enablePinLowActive, LOW);
  }
  if (_enablePinHighActive != PIN_UNDEFINED) {
    digitalWrite(_enablePinHighActive, HIGH);
  }
}
int32_t FastAccelStepper::getPositionAfterCommandsCompleted() {
  return fas_queue[_queue_num].pos_at_queue_end;
}
void FastAccelStepper::setPositionAfterCommandsCompleted(int32_t new_pos) {
  noInterrupts();
  int32_t delta = new_pos - fas_queue[_queue_num].pos_at_queue_end;
  fas_queue[_queue_num].pos_at_queue_end = new_pos;
  _target_pos += delta;
  interrupts();
}
int32_t FastAccelStepper::getCurrentPosition() {
  struct StepperQueue* q = &fas_queue[_queue_num];
  noInterrupts();
  int32_t pos = q->pos_at_queue_end;
  bool countUp = (q->dir_at_queue_end == q->dirHighCountsUp);
  uint8_t wp = q->next_write_idx;
  uint8_t rp = q->read_idx;
  interrupts();
  while (rp != wp) {
    wp--;
    uint8_t steps_dir = q->entry[wp & QUEUE_LEN_MASK].steps;
    if (countUp) {
      pos -= steps_dir >> 1;
    } else {
      pos += steps_dir >> 1;
    }
    if (steps_dir & 1) {
      countUp = !countUp;
    }
  }
  return pos;
}
void FastAccelStepper::setCurrentPosition(int32_t new_pos) {
  int32_t delta = new_pos - getCurrentPosition();
  noInterrupts();
  fas_queue[_queue_num].pos_at_queue_end += delta;
  _target_pos += delta;
  interrupts();
}
bool FastAccelStepper::isQueueFull() {
  return fas_queue[_queue_num].isQueueFull();
}
bool FastAccelStepper::isQueueEmpty() {
  return fas_queue[_queue_num].isQueueEmpty();
}
bool FastAccelStepper::isrSpeedControlEnabled() {
  return _isr_speed_control_enabled;
};
bool FastAccelStepper::isRunning() { return fas_queue[_queue_num].isRunning; }
int32_t FastAccelStepper::targetPos() { return _target_pos; }
uint8_t FastAccelStepper::rampState() { return _rampState; }
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
uint32_t FastAccelStepper::checksum() { return fas_queue[_queue_num].checksum; }
#endif
