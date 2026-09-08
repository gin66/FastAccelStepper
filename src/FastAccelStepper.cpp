#include "FastAccelStepper.h"
#include "fas_queue/stepper_queue.h"
#include "fas_member/move_timed.h"

// This define in order to not shoot myself.
#ifndef TEST
#define printf DO_NOT_USE_PRINTF
#define puts DO_NOT_USE_PUTS
#endif

// Here are the global variables to interface with the interrupts

StepperQueue* FastAccelStepper::_queue() const {
  return FAS_QUEUE_PTR(_queue_num);
}

// To realize the 1 Hz debug led

//*************************************************************************************************
//*************************************************************************************************
//
// FastAccelStepper provides:
// - movement control
//       either raw access to the stepper command queue
//       or ramp generator driven by speed/acceleration and move
// - stepper position
//
// This implements auto enable and delay from direction change to first step
//
//*************************************************************************************************
//*************************************************************************************************

//*************************************************************************************************
bool FastAccelStepper::handleExternalDirectionPin(StepperQueue* q,
                                                  bool count_up) {
  if (_pendingExternalDirState != ExtDirPendingState::None) {
    if (_engine->_externalCallForPin) {
      uint8_t desiredPinState =
          (_pendingExternalDirState == ExtDirPendingState::High) ? HIGH : LOW;
      bool newState = _engine->_externalCallForPin(_dirPin, desiredPinState);
      if (newState == (desiredPinState == HIGH)) {
        _pendingExternalDirState = ExtDirPendingState::None;
#if defined(SUPPORT_PAUSE_CMD_COUNTING)
        q->clear_pause_stats();
#endif
      }
    }
    if (_pendingExternalDirState != ExtDirPendingState::None) {
      return false;
    }
  }
  if (q->queue_end.count_up != count_up) {
    if (q->hasStepsInQueue()) {
      return false;
    }
    if (_engine->_externalCallForPin) {
      uint8_t desiredPinState = (count_up == _dirHighCountsUp) ? HIGH : LOW;
      bool newState = _engine->_externalCallForPin(_dirPin, desiredPinState);
      if (newState != (desiredPinState == HIGH)) {
        _pendingExternalDirState = (desiredPinState == HIGH)
                                       ? ExtDirPendingState::High
                                       : ExtDirPendingState::Low;
        return false;
      }
#if defined(SUPPORT_PAUSE_CMD_COUNTING)
      q->clear_pause_stats();
#endif
    }
  }
  return true;
}

//*************************************************************************************************
#include "fas_member/add_queue_entry.h"

//*************************************************************************************************
// fill_queue generates commands to the stepper for executing a ramp
//
// Plan is to fill the queue with commmands summing up to approx. 10 ms in the
// future (or more). For low speeds, this results in single stepping For high
// speeds (40kSteps/s) approx. 400 Steps to be created using 3 commands
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
  // check if addition of commands is suspended (due to forceStopAndNewPosition)
  StepperQueue* q = _queue();
  // if force stop has been called, then ignore_commands is true and ramp
  // stopped. So the ramp generator will not create a new command, unless new
  // move command has been given after forceStop..(). So we just clear the flag
  q->ignore_commands = false;

  // preconditions are fulfilled, so create the command(s)
  NextCommand cmd;
  // Plan ahead for max. 20 ms and minimum two commands.
  // This is now configurable using _forward_planning_in_ticks.
  bool delayed_start = !q->isRunning();
  bool need_delayed_start = false;
  uint32_t ticksPrepared = q->ticksInQueue();
  while (!isQueueFull() &&
         ((ticksPrepared < _forward_planning_in_ticks) ||
          (q->queueEntries() <= 1)) &&
         _rg.isRampGeneratorActive()) {
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
    // For run time measurement
    uint32_t runtime_us = micros();
#endif
    AqeResultCode res = AQE_OK;
    _rg.getNextCommand(&q->queue_end, &cmd);
    if (cmd.command.ticks != 0) {
      do {
        res = addQueueEntry(&cmd.command, !delayed_start);
      } while (aqeRetryImmediately(res));
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
    max_micros = fas_max(max_micros, runtime_us);
#endif
    if (cmd.command.ticks == 0) {
      break;
    }
    if (res != AQE_OK) {
      if (aqeRetry(res)) {
        // try later again
        break;
      } else {
#ifdef SIM_TEST_INPUT
        Serial.println("Abort ramp due to queue error res=");
        Serial.print(static_cast<int8_t>(res));
        Serial.print(" Steps=");
        Serial.print(cmd.command.steps);
        Serial.print(" ticks=");
        Serial.print(cmd.command.ticks);
        Serial.print(" min_cmd_ticks=");
        Serial.println(MIN_CMD_TICKS);
#endif
#ifdef TEST
        printf("ERROR: Abort ramp due to queue error: %s\n", toString(res));
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
  // fasDisableInterrupts();
  if (_auto_disable_delay_counter > 1) {
    if (!isRunning()) {
      _auto_disable_delay_counter--;
    }
  }
  // fasEnableInterrupts();
}

bool FastAccelStepper::agreeWithAutoDisable() {
  bool agree = true;
  // FastAccelStepperEngine will call with interrupts disabled
  // fasDisableInterrupts();
  if (isRunning()) {
    agree = false;
  }
  if (_auto_disable_delay_counter > 1) {
    agree = false;
  }
  // fasEnableInterrupts();
  return agree;
}

bool FastAccelStepper::needAutoDisable() {
  bool need_disable = false;
  // FastAccelStepperEngine will call with interrupts disabled
  // fasDisableInterrupts();
  if (_auto_disable_delay_counter == 1) {
    if (!isRunning()) {
      need_disable = true;
    }
  }
  // fasEnableInterrupts();
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
  __builtin_memset(this, 0, sizeof(*this));
  _engine = engine;
  _off_delay_count = 1;
  _stepPin = step_pin;
  _dirHighCountsUp = true;
  _dirPin = PIN_UNDEFINED;
  _enablePinHighActive = PIN_UNDEFINED;
  _enablePinLowActive = PIN_UNDEFINED;
  _forward_planning_in_ticks = TICKS_PER_S / 50;
  _pendingExternalDirState = ExtDirPendingState::None;
  _rg.init();
  _queue_num = num;
#if defined(SUPPORT_ESP32_PULSE_COUNTER) && (ESP_IDF_VERSION_MAJOR == 5)
  _attached_pulse_unit = NULL;
#endif
#if defined(SUPPORT_ESP32_PULSE_COUNTER) && (ESP_IDF_VERSION_MAJOR == 4)
  _attached_pulse_cnt_unit = -1;
#endif
}
uint8_t FastAccelStepper::getStepPin() const { return _stepPin; }
void FastAccelStepper::setDirectionPin(uint8_t dirPin, bool dirHighCountsUp,
                                       uint16_t dir_change_delay_us) {
  _dirPin = dirPin;
  _dirHighCountsUp = dirHighCountsUp;
  if (_dirPin != PIN_UNDEFINED) {
    if (_dirPin & PIN_EXTERNAL_FLAG) {
      if (_engine->_externalCallForPin) {
        _engine->_externalCallForPin(_dirPin, dirHighCountsUp ? HIGH : LOW);
      }
    } else {
      PIN_OUTPUT(dirPin, dirHighCountsUp ? HIGH : LOW);
    }
  }
  _queue()->setDirPin(dirPin, dirHighCountsUp);
  if (dir_change_delay_us != 0) {
    if (dir_change_delay_us > MAX_DIR_DELAY_US) {
      dir_change_delay_us = MAX_DIR_DELAY_US;
    }
    if (dir_change_delay_us < MIN_DIR_DELAY_US) {
      dir_change_delay_us = MIN_DIR_DELAY_US;
    }
    _dir_change_delay_ticks = US_TO_TICKS(dir_change_delay_us);
  } else {
    _dir_change_delay_ticks = 0;
  }
}
void FastAccelStepper::setEnablePin(uint8_t enablePin,
                                    bool low_active_enables_stepper) {
  if (low_active_enables_stepper) {
    _enablePinLowActive = enablePin;
    if (enablePin != PIN_UNDEFINED) {
      if (enablePin & PIN_EXTERNAL_FLAG) {
        if (_engine->_externalCallForPin) {
          _engine->_externalCallForPin(enablePin, HIGH);
        }
      } else {
        PIN_OUTPUT(enablePin, HIGH);
        if (_enablePinHighActive == enablePin) {
          _enablePinHighActive = PIN_UNDEFINED;
        }
      }
    }
  } else {
    _enablePinHighActive = enablePin;
    if (enablePin != PIN_UNDEFINED) {
      if (enablePin & PIN_EXTERNAL_FLAG) {
        if (_engine->_externalCallForPin) {
          _engine->_externalCallForPin(enablePin, LOW);
        }
      } else {
        PIN_OUTPUT(enablePin, LOW);
        if (_enablePinLowActive == enablePin) {
          _enablePinLowActive = PIN_UNDEFINED;
        }
      }
    }
  }
}
void FastAccelStepper::setAutoEnable(bool auto_enable) {
  _autoEnable = auto_enable;
  if (auto_enable && (_off_delay_count == 0)) {
    _off_delay_count = 1;
  }
}
DelayResultCode FastAccelStepper::setDelayToEnable(uint32_t delay_us) {
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
  uint16_t delay_count = delay_ms / DELAY_MS_BASE;
  if ((delay_ms > 0) && (delay_count < 2)) {
    // ensure minimum time
    delay_count = 2;
  }
  _off_delay_count = fas_max(delay_count, (uint16_t)1);
}
MoveResultCode FastAccelStepper::runForward() { return _rg.startRun(true); }
MoveResultCode FastAccelStepper::runBackward() { return _rg.startRun(false); }
MoveResultCode FastAccelStepper::moveTo(int32_t position, bool blocking) {
  MoveResultCode res = _rg.moveTo(position, &_queue()->queue_end);
  if ((res == MOVE_OK) && blocking) {
    while (isRunning()) {
      noop_or_wait;
    }
  }
  return res;
}
MoveResultCode FastAccelStepper::move(int32_t move, bool blocking) {
  if ((move < 0) && (_dirPin == PIN_UNDEFINED)) {
    return MOVE_ERR_NO_DIRECTION_PIN;
  }
  MoveResultCode res = _rg.move(move);
  if ((res == MOVE_OK) && blocking) {
    while (isRunning()) {
      noop_or_wait;
    }
  }
  return res;
}
void FastAccelStepper::keepRunning() { _rg.setKeepRunning(); }
void FastAccelStepper::stopMove() { _rg.initiateStop(); }
void FastAccelStepper::applySpeedAcceleration() {
  _rg.applySpeedAcceleration();
}
MoveResultCode FastAccelStepper::moveByAcceleration(int32_t acceleration,
                                                    bool allow_reverse) {
  MoveResultCode res = MOVE_OK;
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
void FastAccelStepper::forceStop() {
  StepperQueue* q = _queue();

  // ensure no more commands are added to the queue
  q->ignore_commands = true;

  // inform ramp generator to force stop
  _rg.forceStop();
}
void FastAccelStepper::forceStopAndNewPosition(int32_t new_pos) {
  StepperQueue* q = _queue();

  // ensure no more commands are added to the queue
  q->ignore_commands = true;

  // stop ramp generator
  _rg.stopRamp();

  // stop the stepper interrupt and empty the queue
  q->forceStop();

  // set the new position. This should be safe
  q->queue_end.pos = new_pos;
  _rg.setTargetPosition(new_pos);
}
bool FastAccelStepper::setEnablePinState(uint8_t pin, uint8_t active_state) {
  if (pin == PIN_UNDEFINED) {
    return true;
  }
  if (pin & PIN_EXTERNAL_FLAG) {
    if (_engine->_externalCallForPin != NULL) {
      return _engine->_externalCallForPin(pin, active_state) == active_state;
    }
    return true;
  }
  SET_ENABLE_PIN_STATE(_queue(), pin, active_state);
  return true;
}
bool FastAccelStepper::disableOutputs() {
  if (isRunning() && _autoEnable) {
    return false;
  }
  bool disabled = setEnablePinState(_enablePinLowActive, HIGH) &
                  setEnablePinState(_enablePinHighActive, LOW);
  if (disabled) {
    _auto_disable_delay_counter = 0;
  }
  return disabled;
}
bool FastAccelStepper::enableOutputs() {
  return setEnablePinState(_enablePinLowActive, LOW) &
         setEnablePinState(_enablePinHighActive, HIGH);
}
int32_t FastAccelStepper::getPositionAfterCommandsCompleted() const {
  return _queue()->queue_end.pos;
}
uint32_t FastAccelStepper::getPeriodInTicksAfterCommandsCompleted() const {
  if (_rg.isRampGeneratorActive()) {
    return _rg.getCurrentPeriodInTicks();
  }
  return 0;
}
uint32_t FastAccelStepper::getPeriodInUsAfterCommandsCompleted() const {
  if (_rg.isRampGeneratorActive()) {
    return _rg.getCurrentPeriodInUs();
  }
  return 0;
}
void FastAccelStepper::getCurrentSpeedInTicks(struct actual_ticks_s* speed,
                                              bool realtime) const {
  bool valid;
  if (realtime) {
    valid = _queue()->getActualTicksWithDirection(speed);
  } else {
    valid = false;
  }
  if (!valid) {
    if (_rg.isRampGeneratorActive()) {
      _rg.getCurrentSpeedInTicks(speed);
    } else {
      speed->ticks = 0;
    }
  }
}
int32_t FastAccelStepper::getCurrentSpeedInUs(bool realtime) const {
  struct actual_ticks_s speed;
  getCurrentSpeedInTicks(&speed, realtime);
  int32_t speed_in_us = speed.ticks / (TICKS_PER_S / 1000000);
  if (speed.count_up) {
    return speed_in_us;
  }
  return -speed_in_us;
}
int32_t FastAccelStepper::getCurrentSpeedInMilliHz(bool realtime) const {
  struct actual_ticks_s speed;
  getCurrentSpeedInTicks(&speed, realtime);
  if (speed.ticks > 0) {
    int32_t speed_in_mhz = ((uint32_t)250 * TICKS_PER_S) / speed.ticks * 4;
    if (speed.count_up) {
      return speed_in_mhz;
    }
    return -speed_in_mhz;
  }
  return 0;
}
uint16_t FastAccelStepper::getMaxSpeedInTicks() const {
  return _queue()->getMaxSpeedInTicks();
}
uint16_t FastAccelStepper::getMaxSpeedInUs() const {
  uint16_t ticks = getMaxSpeedInTicks();
  uint16_t speed_in_us = ticks / (TICKS_PER_S / 1000000);
  return speed_in_us;
}
uint32_t FastAccelStepper::getMaxSpeedInHz() const {
  uint16_t ticks = getMaxSpeedInTicks();
  uint32_t speed_in_hz = TICKS_PER_S / ticks;
  return speed_in_hz;
}
uint32_t FastAccelStepper::getMaxSpeedInMilliHz() const {
  uint16_t ticks = getMaxSpeedInTicks();
  uint32_t speed_in_milli_hz = ((uint32_t)250 * TICKS_PER_S) / ticks * 4;
  return speed_in_milli_hz;
}
#if defined(SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING)
void FastAccelStepper::setAbsoluteSpeedLimit(uint16_t max_speed_in_ticks) {
  _queue()->setAbsoluteSpeedLimit(max_speed_in_ticks);
}
#endif
int8_t FastAccelStepper::setSpeedInTicks(uint32_t min_step_ticks) {
  if (min_step_ticks < getMaxSpeedInTicks()) {
    return -1;
  }
  if (min_step_ticks == TICKS_FOR_STOPPED_MOTOR) {
    return -1;
  }
  _rg.setSpeedInTicks(min_step_ticks);
  return 0;
}
int8_t FastAccelStepper::setSpeedInUs(uint32_t min_step_us) {
  if (min_step_us >= TICKS_TO_US(0xffffffff)) {
    return -1;
  }
  uint32_t min_step_ticks = US_TO_TICKS(min_step_us);
  return setSpeedInTicks(min_step_ticks);
}
int8_t FastAccelStepper::setSpeedInHz(uint32_t speed_hz) {
  if (speed_hz == 0) {
    return -1;
  }
  uint32_t ticks = _rg.divForHz(speed_hz);
  return setSpeedInTicks(ticks);
}
int8_t FastAccelStepper::setSpeedInMilliHz(uint32_t speed_mhz) {
  if (speed_mhz <= (1000LL * TICKS_PER_S / 0xffffffff + 1)) {
    return -1;
  }
  uint32_t ticks = _rg.divForMilliHz(speed_mhz);
  return setSpeedInTicks(ticks);
}
void FastAccelStepper::setCurrentPosition(int32_t new_pos) {
  int32_t delta = new_pos - getCurrentPosition();
  if (delta != 0) {
    StepperQueue* q = _queue();
#if defined(SUPPORT_RP_PICO)
    q->pos_offset += delta;
#endif
    struct queue_end_s* queue_end = &q->queue_end;
    fasDisableInterrupts();
    queue_end->pos = queue_end->pos + delta;
    _rg.advanceTargetPosition(delta);
    fasEnableInterrupts();
  }
}
void FastAccelStepper::setPositionAfterCommandsCompleted(int32_t new_pos) {
  StepperQueue* q = _queue();
  struct queue_end_s* queue_end = &q->queue_end;
  fasDisableInterrupts();
  int32_t delta = new_pos - q->queue_end.pos;
  queue_end->pos = new_pos;
  if (delta != 0) {
#if defined(SUPPORT_RP_PICO)
    q->pos_offset += delta;
#endif
    _rg.advanceTargetPosition(delta);
  }
  fasEnableInterrupts();
}
uint8_t FastAccelStepper::queueEntries() const {
  return _queue()->queueEntries();
}
uint32_t FastAccelStepper::ticksInQueue() const {
  return _queue()->ticksInQueue();
}
bool FastAccelStepper::hasTicksInQueue(uint32_t min_ticks) const {
  return _queue()->hasTicksInQueue(min_ticks);
}
bool FastAccelStepper::isQueueFull() const { return _queue()->isQueueFull(); }
bool FastAccelStepper::isQueueEmpty() const { return _queue()->isQueueEmpty(); }
bool FastAccelStepper::isQueueRunning() const { return _queue()->isRunning(); }
bool FastAccelStepper::isRunning() const {
  StepperQueue* q = _queue();
  return q->isRunning() || _rg.isRampGeneratorActive() || !isQueueEmpty();
}
void FastAccelStepper::performOneStep(bool count_up, bool blocking) {
  if (!isRunning()) {
    if (count_up || (_dirPin != PIN_UNDEFINED)) {
      StepperQueue* q = _queue();
      q->ignore_commands = false;
      struct stepper_command_s cmd = {
          .ticks = MIN_CMD_TICKS, .steps = 1, .count_up = count_up};
      addQueueEntry(&cmd);
      if (blocking) {
        while (isRunning()) {
        }
      }
    }
  }
}
void FastAccelStepper::forwardStep(bool blocking) {
  performOneStep(true, blocking);
}
void FastAccelStepper::backwardStep(bool blocking) {
  performOneStep(false, blocking);
}
int32_t FastAccelStepper::getCurrentPosition() const {
  return _queue()->getCurrentPosition();
}
void FastAccelStepper::detachFromPin() { _queue()->disconnect(); }
void FastAccelStepper::reAttachToPin() { _queue()->connect(); }

#if defined(SUPPORT_SELECT_DRIVER_TYPE)
FasDriver FastAccelStepper::driverType() const {
  return _queue()->_driver_type;
}
const char* FastAccelStepper::driverTypeString() const {
  switch (_queue()->_driver_type) {
#if defined(SUPPORT_ESP32_MCPWM_PCNT)
    case FasDriver::MCPWM_PCNT:
      return "MCPWM_PCNT";
#endif
    case FasDriver::RMT:
      return "RMT";
#if defined(SUPPORT_ESP32_I2S)
    case FasDriver::I2S_DIRECT:
      return "I2S_DIRECT";
    case FasDriver::I2S_MUX:
      return "I2S_MUX";
#endif
    default:
      return "UNSPECIFIED";
  }
}
#endif
