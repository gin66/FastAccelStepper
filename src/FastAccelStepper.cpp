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
#define TASK_DELAY_10MS 10
void StepperTask(void* parameter) {
  FastAccelStepperEngine* engine = (FastAccelStepperEngine*)parameter;
  const TickType_t delay_10ms = TASK_DELAY_10MS / portTICK_PERIOD_MS;
  while (true) {
    engine->manageSteppers();
    vTaskDelay(delay_10ms);
  }
}
#endif
//*************************************************************************************************
void FastAccelStepperEngine::init() {
#if (TICKS_PER_S != 16000000L)
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
#define PRIORITY configMAX_PRIORITIES
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
      s->fill_queue();
    }
  }

  // Check for auto disable
  for (uint8_t i = 0; i < _next_stepper_num; i++) {
    FastAccelStepper* s = _stepper[i];
    if (s) {
      uint8_t high_active_pin = s->getEnablePinHighActive();
      uint8_t low_active_pin = s->getEnablePinLowActive();
      if (s->needAutoDisable()) {
        noInterrupts();
        bool agree = true;
        for (uint8_t j = 0; j < _next_stepper_num; j++) {
          if (i != j) {
            FastAccelStepper* other = _stepper[j];
            if (other) {
              if (other->usesAutoEnablePin(high_active_pin) ||
                  other->usesAutoEnablePin(low_active_pin)) {
                if (!other->agreeWithAutoDisable()) {
                  agree = false;
                  break;
                }
              }
            }
          }
        }
        if (agree) {
          for (uint8_t j = 0; j < _next_stepper_num; j++) {
            FastAccelStepper* other = _stepper[j];
            if (other) {
              if (other->usesAutoEnablePin(high_active_pin) ||
                  other->usesAutoEnablePin(low_active_pin)) {
                other->disableOutputs();
              }
            }
          }
        }
        interrupts();
      }
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
int8_t FastAccelStepper::addQueueEntry(struct stepper_command_s* cmd) {
  if (cmd->steps >= 128) {
    return AQE_STEPS_ERROR;
  }
  if (cmd->ticks < MIN_DELTA_TICKS) {
    return AQE_TOO_LOW;
  }
  StepperQueue* q = &fas_queue[_queue_num];
  int res = AQE_OK;
  if (_autoEnable) {
    noInterrupts();
    uint16_t delay_counter = _auto_disable_delay_counter;
    interrupts();
    if (delay_counter == 0) {
      // outputs are disabled
      enableOutputs();
      // if on delay is defined, perform first step accordingly
      if (_on_delay_ticks > 0) {
        uint32_t delay = _on_delay_ticks;
        while (delay > 0) {
          uint32_t ticks = delay >> 1;
          uint16_t ticks_u16 = ticks;
          if (ticks > 65535) {
            ticks_u16 = 65535;
          } else if (ticks < 32768) {
            ticks_u16 = delay;
          }
          struct stepper_command_s start_cmd = {
              .ticks = ticks_u16, .steps = 0, .count_up = cmd->count_up};
          res = q->addQueueEntry(&start_cmd);
          delay -= ticks_u16;
        }
        if (res != AQE_OK) {
          return res;
        }
      }
    }
  }
  res = q->addQueueEntry(cmd);
  if (_autoEnable) {
    if (res == AQE_OK) {
      noInterrupts();
      _auto_disable_delay_counter = _off_delay_count;
      interrupts();
    }
  }

  return res;
}

//*************************************************************************************************
// fill_queue generates commands to the stepper for executing a ramp
//
// Plan is to fill the queue with commmands with approx. 10 ms ahead (or
// more). For low speeds, this results in single stepping For high speeds
// (40kSteps/s) approx. 400 Steps to be created using 3 commands
//
// Basis of the calculation is the relation between steps and time via
// acceleration a:
//
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

void FastAccelStepper::fill_queue() {
  // Check preconditions to be allowed to fill the queue
  if (!rg.isRampGeneratorActive()) {
    return;
  }
  if (!rg.hasValidConfig()) {
#ifdef TEST
    assert(false);
#endif
    return;
  }
  // preconditions are fulfilled, so create the command(s)
  struct stepper_command_s cmd;
  StepperQueue* q = &fas_queue[_queue_num];
  // Plan ahead for max. 10 ms. Currently hard coded
  while (!isQueueFull() && !q->hasTicksInQueue(TICKS_PER_S / 100) &&
         rg.isRampGeneratorActive()) {
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
    // For run time measurement
    uint32_t runtime_us = micros();
#endif
    int8_t res = AQE_OK;
    uint8_t next_state = rg.getNextCommand(&q->queue_end, &cmd);
    if (cmd.ticks != 0) {
      res = addQueueEntry(&cmd);
      if (res == AQE_OK) {
        rg.commandEnqueued(&cmd, next_state);
        rg.setState(next_state);
      }
    } else {
      rg.setState(RAMP_STATE_IDLE);
    }

#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
    // For run time measurement
    runtime_us = micros() - runtime_us;
    max_micros = max(max_micros, runtime_us);
#endif
    if (cmd.ticks == 0) {
      break;
    }
    if (res == AQE_FULL) {
      break;
    } else if (res != AQE_OK) {
      // TODO: How to deal with these error ?
      rg.stopRamp();
    }
  }
}

#if defined(ARDUINO_ARCH_AVR)
ISR(TIMER1_OVF_vect) {
  // disable OVF interrupt to avoid nesting
  TIMSK1 &= ~_BV(TOIE1);

  // enable interrupts for nesting
  interrupts();

  // manage steppers
  fas_engine->manageSteppers();

  // disable interrupts for exist ISR routine
  noInterrupts();

  // enable OVF interrupt again
  TIMSK1 |= _BV(TOIE1);
}
#endif

bool FastAccelStepper::needAutoDisable() {
  bool need_disable = false;
  // FastAccelStepperEngine will call with interrupts disabled
  // noInterrupts();
  if (_auto_disable_delay_counter > 0) {
    if (!isRunning()) {
      _auto_disable_delay_counter--;
      if (_auto_disable_delay_counter == 0) {
        need_disable = true;
      }
    }
  }
  // interrupts();
  return need_disable;
}

bool FastAccelStepper::usesAutoEnablePin(uint8_t pin) {
  if (pin != PIN_UNDEFINED) {
    if ((pin == _enablePinHighActive) || (pin == _enablePinLowActive)) {
      return true;
    }
  }
  return false;
}

bool FastAccelStepper::agreeWithAutoDisable() {
  bool agree = true;
  noInterrupts();
  if (_auto_disable_delay_counter > 0) {
    agree = false;
  }
  interrupts();
  return agree;
}

void FastAccelStepper::init(uint8_t num, uint8_t step_pin) {
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
  // For run time measurement
  max_micros = 0;
#endif
  _autoEnable = false;
  _on_delay_ticks = 0;
  _off_delay_count = 0;
  _auto_disable_delay_counter = 0;
  _stepPin = step_pin;
  _dirHighCountsUp = true;
  rg.init();

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
  if (auto_enable && (_on_delay_ticks == 0)) {
    _on_delay_ticks = 1;
  }
}
int FastAccelStepper::setDelayToEnable(uint32_t delay_us) {
  uint32_t delay_ticks = US_TO_TICKS(delay_us);
  if (delay_ticks < MIN_DELTA_TICKS) {
    return DELAY_TOO_LOW;
  }
  if (delay_ticks > MAX_ON_DELAY_TICKS) {
    return DELAY_TOO_HIGH;
  }
  _on_delay_ticks = delay_ticks;
  return DELAY_OK;
}
void FastAccelStepper::setDelayToDisable(uint16_t delay_ms) {
  uint16_t delay_count = 0;
#if defined(ARDUINO_ARCH_ESP32)
  delay_count = delay_ms / TASK_DELAY_10MS;
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
  rg.setSpeed(min_step_us);
}
void FastAccelStepper::setAcceleration(uint32_t accel) {
  rg.setAcceleration(accel);
}
int8_t FastAccelStepper::moveTo(int32_t position) {
  return rg.moveTo(position, &fas_queue[_queue_num].queue_end);
}
int8_t FastAccelStepper::move(int32_t move) {
  if ((move < 0) && (_dirPin == PIN_UNDEFINED)) {
    return MOVE_ERR_NO_DIRECTION_PIN;
  }
  return rg.move(move, &fas_queue[_queue_num].queue_end);
}
void FastAccelStepper::keepRunning() { rg.setKeepRunning(); }
void FastAccelStepper::stopMove() { rg.initiate_stop(); }
void FastAccelStepper::applySpeedAcceleration() { rg.applySpeedAcceleration(); }
void FastAccelStepper::forceStopAndNewPosition(uint32_t new_pos) {
  StepperQueue* q = &fas_queue[_queue_num];

  // first stop ramp generator
  rg.stopRamp();

  // stop the stepper interrupt and empty the queue
  q->forceStop();

  // set the new position
  q->queue_end.pos = new_pos;
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
  return fas_queue[_queue_num].queue_end.pos;
}
uint32_t FastAccelStepper::getPeriodAfterCommandsCompleted() {
  uint32_t ticks = fas_queue[_queue_num].queue_end.ticks;
  if (ticks == TICKS_FOR_STOPPED_MOTOR) {
    return 0;
  }
  return TICKS_TO_US(ticks);
}
int32_t FastAccelStepper::getCurrentPosition() {
  struct StepperQueue* q = &fas_queue[_queue_num];
  noInterrupts();
  int32_t pos = q->queue_end.pos;
  bool countUp = (q->queue_end.dir == q->dirHighCountsUp);
  uint8_t wp = q->next_write_idx;
  uint8_t rp = q->read_idx;
  interrupts();
  while (rp != wp) {
    wp--;
    struct queue_entry* e = &q->entry[wp & QUEUE_LEN_MASK];
    if (countUp) {
      pos -= e->steps;
    } else {
      pos += e->steps;
    }
    if (e->toggle_dir) {
      countUp = !countUp;
    }
  }
  return pos;
}
void FastAccelStepper::setCurrentPosition(int32_t new_pos) {
  int32_t delta = new_pos - getCurrentPosition();
  noInterrupts();
  fas_queue[_queue_num].queue_end.pos += delta;
  rg.advanceTargetPositionWithinInterruptDisabledScope(delta);
  interrupts();
}
void FastAccelStepper::setPositionAfterCommandsCompleted(int32_t new_pos) {
  noInterrupts();
  int32_t delta = new_pos - fas_queue[_queue_num].queue_end.pos;
  fas_queue[_queue_num].queue_end.pos = new_pos;
  rg.advanceTargetPositionWithinInterruptDisabledScope(delta);
  interrupts();
}
bool FastAccelStepper::isQueueFull() {
  return fas_queue[_queue_num].isQueueFull();
}
bool FastAccelStepper::isQueueEmpty() {
  return fas_queue[_queue_num].isQueueEmpty();
}
bool FastAccelStepper::isRunning() {
  return fas_queue[_queue_num].isRunning || rg.isRampGeneratorActive();
}
void FastAccelStepper::forwardStep(bool blocking) {
  if (!isRunning()) {
    struct stepper_command_s cmd = {
        .ticks = MIN_DELTA_TICKS, .steps = 1, .count_up = true};
    addQueueEntry(&cmd);
    if (blocking) {
      while (isRunning()) {
        // busy wait
      }
    }
  }
}
void FastAccelStepper::backwardStep(bool blocking) {
  if (!isRunning()) {
    if (_dirPin != PIN_UNDEFINED) {
      struct stepper_command_s cmd = {
          .ticks = MIN_DELTA_TICKS, .steps = 1, .count_up = false};
      addQueueEntry(&cmd);
      if (blocking) {
        while (isRunning()) {
          // busy wait
        }
      }
    }
  }
}
void FastAccelStepper::detachFromPin() { fas_queue[_queue_num].disconnect(); }
void FastAccelStepper::reAttachToPin() { fas_queue[_queue_num].connect(); }
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
uint32_t FastAccelStepper::checksum() { return fas_queue[_queue_num].checksum; }
#endif
