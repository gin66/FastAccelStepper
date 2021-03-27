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
#define DEBUG_LED_HALF_PERIOD (TICKS_PER_S / 65536 / 2)
#elif defined(ARDUINO_ARCH_ESP32)
#define DEBUG_LED_HALF_PERIOD 50
#else
#define DEBUG_LED_HALF_PERIOD 50
#define LOW 0
#define HIGH 1
#endif

#if defined(ARDUINO_ARCH_AVR)
// this is needed to give the background task isr access to engine
FastAccelStepperEngine* fas_engine = NULL;

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
#define TASK_DELAY_4MS 4
void StepperTask(void* parameter) {
  FastAccelStepperEngine* engine = (FastAccelStepperEngine*)parameter;
  const TickType_t delay_4ms = TASK_DELAY_4MS / portTICK_PERIOD_MS;
  while (true) {
    engine->manageSteppers();
    vTaskDelay(delay_4ms);
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
#endif
#if defined(ARDUINO_ARCH_ESP32)
#define STACK_SIZE 1000
#define PRIORITY configMAX_PRIORITIES
  xTaskCreate(StepperTask, "StepperTask", STACK_SIZE, this, PRIORITY, NULL);
#endif
}
//*************************************************************************************************
bool FastAccelStepperEngine::_isValidStepPin(uint8_t step_pin) {
  // ask just first queue entry....
  return StepperQueue::isValidStepPin(step_pin);
}
//*************************************************************************************************
bool FastAccelStepperEngine::isDirPinBusy(uint8_t dir_pin,
                                          uint8_t except_stepper) {
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    if (i != except_stepper) {
      FastAccelStepper* s = _stepper[i];
      if (s) {
        if (s->getDirectionPin() == dir_pin) {
          if (s->isMotorRunning()) {
            return true;
          }
        }
      }
    }
  }
  return false;
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
  int8_t fas_stepper_num = StepperQueue::queueNumForStepPin(step_pin);
  if (fas_stepper_num < 0) {  // flexible, so just choose next
    if (_next_stepper_num >= MAX_STEPPER) {
      return NULL;
    }
    fas_stepper_num = _next_stepper_num;
  }
  uint8_t stepper_num = _next_stepper_num;
  _next_stepper_num++;

#if defined(ARDUINO_ARCH_AVR) || defined(ESP32) || defined(TEST)
  FastAccelStepper* s = &fas_stepper[fas_stepper_num];
  _stepper[stepper_num] = s;
  s->init(this, fas_stepper_num, step_pin);
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
      if (s->needAutoDisable()) {
        uint8_t high_active_pin = s->getEnablePinHighActive();
        uint8_t low_active_pin = s->getEnablePinLowActive();

        // noInterrupts(); // TODO
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
            FastAccelStepper* current = _stepper[j];
            if (current) {
              if (current->usesAutoEnablePin(high_active_pin) ||
                  current->usesAutoEnablePin(low_active_pin)) {
                // if successful, then the _auto_disable_delay_counter is zero
                // Otherwise in next loop will be checked for auto disable again
                current->disableOutputs();
              }
            }
          }
        }
        // interrupts();
      }
    }
  }

  // Update the auto disable counters
  for (uint8_t i = 0; i < _next_stepper_num; i++) {
    FastAccelStepper* s = _stepper[i];
    if (s) {
      noInterrupts();
      // update the counters down to 1
      s->updateAutoDisable();
      interrupts();
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
int8_t FastAccelStepper::addQueueEntry(const struct stepper_command_s* cmd,
                                       bool start) {
  StepperQueue* q = &fas_queue[_queue_num];
  if (cmd == NULL) {
    return q->addQueueEntry(NULL, start);
  }
  if (cmd->ticks < MIN_DELTA_TICKS) {
    return AQE_ERROR_TICKS_TOO_LOW;
  }

  if (_dirPin != PIN_UNDEFINED) {
    if (!isMotorRunning()) {
      if (_engine != NULL) {
        if (_engine->isDirPinBusy(_dirPin, _queue_num)) {
          return AQE_DIR_PIN_IS_BUSY;
        }
      }
    }
  } else {
    if (!cmd->count_up) {
      return AQE_ERROR_NO_DIR_PIN_TO_TOGGLE;
    }
  }

  int res = AQE_OK;
  if (_autoEnable) {
    noInterrupts();
    uint16_t delay_counter = _auto_disable_delay_counter;
    interrupts();
    if (delay_counter == 0) {
      // outputs are disabled
      if (!enableOutputs()) {
        return AQE_WAIT_FOR_ENABLE_PIN_ACTIVE;
      }
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
          res = q->addQueueEntry(&start_cmd, start);
          delay -= ticks_u16;
        }
        if (res != AQE_OK) {
          return res;
        }
      }
    }
  }
  res = q->addQueueEntry(cmd, start);
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
  if (!_rg.isRampGeneratorActive()) {
    return;
  }
  if (!_rg.hasValidConfig()) {
#ifdef TEST
    assert(false);
#endif
    return;
  }
  // preconditions are fulfilled, so create the command(s)
  NextCommand cmd;
  StepperQueue* q = &fas_queue[_queue_num];
  // Plan ahead for max. 20 ms. Currently hard coded
  bool delayed_start = !q->isRunning();
  bool need_delayed_start = false;
  uint32_t ticksPrepared = q->ticksInQueue();
  while (!isQueueFull() &&
         ((ticksPrepared < TICKS_PER_S / 50) || q->queueEntries() <= 1) &&
         _rg.isRampGeneratorActive()) {
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
    // For run time measurement
    uint32_t runtime_us = micros();
#endif
    int8_t res = AQE_OK;
    _rg.getNextCommand(&q->queue_end, &cmd);
    if (cmd.command.ticks != 0) {
      res = addQueueEntry(&cmd.command, !delayed_start);
    }
    if (res == AQE_OK) {
      _rg.afterCommandEnqueued(&cmd);
      need_delayed_start = delayed_start;
      if (cmd.command.steps <= 1) {
        ticksPrepared += cmd.command.ticks;
      } else {
        uint32_t tmp = cmd.command.ticks;
        tmp *= cmd.command.steps;
        ticksPrepared += tmp;
      }
    }

#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
    // For run time measurement
    runtime_us = micros() - runtime_us;
    max_micros = max(max_micros, runtime_us);
#endif
    if (cmd.command.ticks == 0) {
      break;
    }
    if (res != AQE_OK) {
      if (res > 0) {
        // try later again
        break;
      } else {
#ifdef SIM_TEST_INPUT
        Serial.println("Abort ramp due to queue error");
        Serial.print("Steps=");
        Serial.print(cmd.command.steps);
        Serial.print(" ticks=");
        Serial.print(cmd.command.ticks);
        Serial.print(" min_cmd_ticks=");
        Serial.println(MIN_CMD_TICKS);
#endif
#ifdef TEST
        printf("ERROR: Abort ramp due to queue error (%d)\n", res);
        printf("steps=%d ticks=%d limit=%ld state=%d\n", cmd.command.steps,
               cmd.command.ticks, MIN_CMD_TICKS, cmd.rw.ramp_state);
        assert(false);
#endif
        _rg.stopRamp();
        delayed_start = false;
      }
    }
  }
  if (need_delayed_start) {
    addQueueEntry(NULL, true);
  }
}

void FastAccelStepper::updateAutoDisable() {
  // FastAccelStepperEngine will call with interrupts disabled
  // noInterrupts();
  if (_auto_disable_delay_counter > 1) {
    if (!isRunning()) {
      _auto_disable_delay_counter--;
    }
  }
  // interrupts();
}

bool FastAccelStepper::agreeWithAutoDisable() {
  bool agree = true;
  // FastAccelStepperEngine will call with interrupts disabled
  // noInterrupts();
  if (isRunning()) {
    agree = false;
  }
  if (_auto_disable_delay_counter > 1) {
    agree = false;
  }
  // interrupts();
  return agree;
}

bool FastAccelStepper::needAutoDisable() {
  bool need_disable = false;
  // FastAccelStepperEngine will call with interrupts disabled
  // noInterrupts();
  if (_auto_disable_delay_counter == 1) {
    if (!isRunning()) {
      need_disable = true;
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

void FastAccelStepper::init(FastAccelStepperEngine* engine, uint8_t num,
                            uint8_t step_pin) {
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
  // For run time measurement
  max_micros = 0;
#endif
  _engine = engine;
  _autoEnable = false;
  _on_delay_ticks = 0;
  _off_delay_count = 1;
  _auto_disable_delay_counter = 0;
  _stepPin = step_pin;
  _dirHighCountsUp = true;
  _rg.init();
  _externalEnableCall = NULL;

  _queue_num = num;
  fas_queue[_queue_num].init(_queue_num, step_pin);
#if defined(ARDUINO_ARCH_ESP32)
  _attached_pulse_cnt_unit = -1;
#endif
}
uint8_t FastAccelStepper::getStepPin() { return _stepPin; }
void FastAccelStepper::setDirectionPin(uint8_t dirPin, bool dirHighCountsUp) {
  _dirPin = dirPin;
  _dirHighCountsUp = dirHighCountsUp;
  if (_dirPin != PIN_UNDEFINED) {
    digitalWrite(dirPin, dirHighCountsUp ? HIGH : LOW);
    pinMode(dirPin, OUTPUT);
  }
  fas_queue[_queue_num].setDirPin(dirPin, dirHighCountsUp);
}
void FastAccelStepper::setEnablePin(uint8_t enablePin,
                                    bool low_active_enables_stepper) {
  if (low_active_enables_stepper) {
    _enablePinLowActive = enablePin;
    if (enablePin != PIN_UNDEFINED) {
      digitalWrite(enablePin, HIGH);
      pinMode(enablePin, OUTPUT);
      if (_enablePinHighActive == enablePin) {
        _enablePinHighActive = PIN_UNDEFINED;
      }
    }
  } else {
    if (enablePin != PIN_UNDEFINED) {
      _enablePinHighActive = enablePin;
      digitalWrite(enablePin, LOW);
      pinMode(enablePin, OUTPUT);
      if (_enablePinLowActive == enablePin) {
        _enablePinLowActive = PIN_UNDEFINED;
      }
    }
  }
  if ((_enablePinHighActive == PIN_UNDEFINED) &&
      (_enablePinLowActive == PIN_UNDEFINED)) {
    _externalEnableCall = NULL;
  }
}
void FastAccelStepper::setExternalEnableCall(bool (*func)(uint8_t enablePin,
                                                          uint8_t value)) {
  _externalEnableCall = func;
}
void FastAccelStepper::setAutoEnable(bool auto_enable) {
  _autoEnable = auto_enable;
  if (auto_enable && (_off_delay_count == 0)) {
    _off_delay_count = 1;
  }
}
int8_t FastAccelStepper::setDelayToEnable(uint32_t delay_us) {
  uint32_t delay_ticks = US_TO_TICKS(delay_us);
  if (delay_ticks > 0) {
    if (delay_ticks < MIN_CMD_TICKS) {
      return DELAY_TOO_LOW;
    }
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
  delay_count = delay_ms / TASK_DELAY_4MS;
#endif
#if defined(ARDUINO_ARCH_AVR)
  delay_count = delay_ms / (65536000 / TICKS_PER_S);
#endif
  if ((delay_ms > 0) && (delay_count < 2)) {
    // ensure minimum time
    delay_count = 2;
  }
  _off_delay_count = max(delay_count, 1);
}
int8_t FastAccelStepper::moveTo(int32_t position) {
  return _rg.moveTo(position, &fas_queue[_queue_num].queue_end);
}
int8_t FastAccelStepper::move(int32_t move) {
  if ((move < 0) && (_dirPin == PIN_UNDEFINED)) {
    return MOVE_ERR_NO_DIRECTION_PIN;
  }
  return _rg.move(move, &fas_queue[_queue_num].queue_end);
}
void FastAccelStepper::keepRunning() { _rg.setKeepRunning(); }
void FastAccelStepper::stopMove() { _rg.initiate_stop(); }
void FastAccelStepper::applySpeedAcceleration() {
  _rg.applySpeedAcceleration();
}
int8_t FastAccelStepper::moveByAcceleration(int32_t acceleration,
                                            bool allow_reverse) {
  int8_t res = MOVE_OK;
  if (acceleration > 0) {
    setAcceleration(acceleration);
    res = runForward();
  } else if (acceleration < 0) {
    setAcceleration(-acceleration);
    if (allow_reverse && (_dirPin != PIN_UNDEFINED)) {
      res = runBackward();
    } else {
      applySpeedAcceleration();
      stopMove();
    }
  } else {
    uint32_t max_speed = _rg.getSpeedInTicks();
    setSpeedInTicks(getPeriodInTicksAfterCommandsCompleted());
    setAcceleration(1);  // ensure increase, so the speed is kept
    applySpeedAcceleration();
    setSpeedInTicks(max_speed);
  }
  return res;
}
void FastAccelStepper::forceStopAndNewPosition(uint32_t new_pos) {
  StepperQueue* q = &fas_queue[_queue_num];

  // first stop ramp generator
  _rg.stopRamp();

  // stop the stepper interrupt and empty the queue
  q->forceStop();

  // set the new position
  q->queue_end.pos = new_pos;
}
bool FastAccelStepper::disableOutputs() {
  if (isRunning() && _autoEnable) {
    return false;
  }
  bool disabled = true;
  if (_externalEnableCall == NULL) {
    if (_enablePinLowActive != PIN_UNDEFINED) {
      digitalWrite(_enablePinLowActive, HIGH);
    }
    if (_enablePinHighActive != PIN_UNDEFINED) {
      digitalWrite(_enablePinHighActive, LOW);
    }
  } else {
    if (_enablePinLowActive != PIN_UNDEFINED) {
      disabled &= (_externalEnableCall(_enablePinLowActive, HIGH) == HIGH);
    }
    if (_enablePinHighActive != PIN_UNDEFINED) {
      disabled &= (_externalEnableCall(_enablePinHighActive, LOW) == LOW);
    }
  }
  if (disabled) {
    _auto_disable_delay_counter = 0;
  }
  return disabled;
}
bool FastAccelStepper::enableOutputs() {
  bool enabled = true;
  if (_externalEnableCall == NULL) {
    if (_enablePinLowActive != PIN_UNDEFINED) {
      digitalWrite(_enablePinLowActive, LOW);
    }
    if (_enablePinHighActive != PIN_UNDEFINED) {
      digitalWrite(_enablePinHighActive, HIGH);
    }
  } else {
    if (_enablePinLowActive != PIN_UNDEFINED) {
      enabled &= (_externalEnableCall(_enablePinLowActive, LOW) == LOW);
    }
    if (_enablePinHighActive != PIN_UNDEFINED) {
      enabled &= (_externalEnableCall(_enablePinHighActive, HIGH) == HIGH);
    }
  }
  return enabled;
}
int32_t FastAccelStepper::getPositionAfterCommandsCompleted() {
  return fas_queue[_queue_num].queue_end.pos;
}
uint32_t FastAccelStepper::getPeriodInTicksAfterCommandsCompleted() {
  if (_rg.isRampGeneratorActive()) {
    return _rg.getCurrentPeriodInTicks();
  }
  return 0;
}
uint32_t FastAccelStepper::getPeriodInUsAfterCommandsCompleted() {
  if (_rg.isRampGeneratorActive()) {
    return _rg.getCurrentPeriodInUs();
  }
  return 0;
}
int32_t FastAccelStepper::getCurrentSpeedInUs() {
  uint32_t ticks = fas_queue[_queue_num].getActualTicks();
  if (ticks == 0) {
    ticks = getPeriodInTicksAfterCommandsCompleted();
  }
  int32_t speed_in_us = ticks / (TICKS_PER_S / 1000000);
  switch (_rg.rampState() & RAMP_DIRECTION_MASK) {
    case RAMP_DIRECTION_COUNT_UP:
      return speed_in_us;
    case RAMP_DIRECTION_COUNT_DOWN:
      return -speed_in_us;
  }
  return 0;
}
int32_t FastAccelStepper::getCurrentSpeedInMilliHz() {
  uint32_t ticks = fas_queue[_queue_num].getActualTicks();
  if (ticks == 0) {
    ticks = getPeriodInTicksAfterCommandsCompleted();
  }
  int32_t speed_in_milli_hz = 0;
  if (ticks > 0) {
    speed_in_milli_hz = ((uint32_t)250 * TICKS_PER_S) / ticks * 4;
  }
  switch (_rg.rampState() & RAMP_DIRECTION_MASK) {
    case RAMP_DIRECTION_COUNT_UP:
      return speed_in_milli_hz;
    case RAMP_DIRECTION_COUNT_DOWN:
      return -speed_in_milli_hz;
  }
  return 0;
}
void FastAccelStepper::setCurrentPosition(int32_t new_pos) {
  int32_t delta = new_pos - getCurrentPosition();
  noInterrupts();
  fas_queue[_queue_num].queue_end.pos += delta;
  _rg.advanceTargetPositionWithinInterruptDisabledScope(delta);
  interrupts();
}
void FastAccelStepper::setPositionAfterCommandsCompleted(int32_t new_pos) {
  noInterrupts();
  int32_t delta = new_pos - fas_queue[_queue_num].queue_end.pos;
  fas_queue[_queue_num].queue_end.pos = new_pos;
  _rg.advanceTargetPositionWithinInterruptDisabledScope(delta);
  interrupts();
}
bool FastAccelStepper::isQueueFull() {
  return fas_queue[_queue_num].isQueueFull();
}
bool FastAccelStepper::isQueueEmpty() {
  return fas_queue[_queue_num].isQueueEmpty();
}
bool FastAccelStepper::isMotorRunning() {
  return fas_queue[_queue_num].isRunning();
}
bool FastAccelStepper::isRunning() {
  return fas_queue[_queue_num].isRunning() || _rg.isRampGeneratorActive();
}
void FastAccelStepper::forwardStep(bool blocking) {
  if (!isRunning()) {
    struct stepper_command_s cmd = {
        .ticks = MIN_CMD_TICKS, .steps = 1, .count_up = true};
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
          .ticks = MIN_CMD_TICKS, .steps = 1, .count_up = false};
      addQueueEntry(&cmd);
      if (blocking) {
        while (isRunning()) {
          // busy wait
        }
      }
    }
  }
}
int32_t FastAccelStepper::getCurrentPosition() {
  return fas_queue[_queue_num].getCurrentPosition();
}
void FastAccelStepper::detachFromPin() { fas_queue[_queue_num].disconnect(); }
void FastAccelStepper::reAttachToPin() { fas_queue[_queue_num].connect(); }
#if defined(ARDUINO_ARCH_ESP32)
bool FastAccelStepper::attachToPulseCounter(uint8_t pcnt_unit,
                                            int16_t low_value,
                                            int16_t high_value) {
  if (pcnt_unit < 8) {
    if (_esp32_attachToPulseCounter(pcnt_unit, this, low_value, high_value)) {
      _attached_pulse_cnt_unit = pcnt_unit;
      return true;
    }
  }
  return false;
}
void FastAccelStepper::clearPulseCounter() {
  if (_attached_pulse_cnt_unit >= 0) {
    _esp32_clearPulseCounter(_attached_pulse_cnt_unit);
  }
}
int16_t FastAccelStepper::readPulseCounter() {
  if (_attached_pulse_cnt_unit >= 0) {
    return _esp32_readPulseCounter(_attached_pulse_cnt_unit);
  }
  return 0;
}
#endif

#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
uint32_t FastAccelStepper::checksum() { return fas_queue[_queue_num].checksum; }
#endif
