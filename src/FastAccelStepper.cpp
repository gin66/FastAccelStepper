#include "FastAccelStepper.h"
#include "fas_queue/stepper_queue.h"

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
    }
  }
  return true;
}

//*************************************************************************************************
AqeResultCode FastAccelStepper::addQueueEntry(
    const struct stepper_command_s* cmd, bool start) {
  StepperQueue* q = _queue();
  if (cmd == NULL) {
    return q->addQueueEntry(NULL, start);
  }
  if (cmd->ticks < q->max_speed_in_ticks) {
    return AQE_ERROR_TICKS_TOO_LOW;
  }

  if (_dirPin != PIN_UNDEFINED) {
    if (!isQueueRunning()) {
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

  AqeResultCode res = AQE_OK;
  if (_autoEnable) {
    fasDisableInterrupts();
    uint16_t delay_counter = _auto_disable_delay_counter;
    fasEnableInterrupts();
    if (delay_counter == 0) {
      // outputs are disabled
      if (!enableOutputs()) {
        return AQE_WAIT_FOR_ENABLE_PIN_ACTIVE;
      }
      // if on delay is defined, fill queue if required amount of pauses before
      // the first step
      if (_on_delay_ticks > 0) {
        uint32_t delay = _on_delay_ticks;
        // this delay sets count_up appropriately. If this is shorter than
        // dir_change_delay_ticks, then extend accordingly
        if ((delay < _dir_change_delay_ticks) &&
            (q->queue_end.count_up != cmd->count_up)) {
          delay = _dir_change_delay_ticks;
        }
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
          q->addQueueEntry(&start_cmd, false);
          delay -= ticks_u16;
        }
        res = q->addQueueEntry(NULL, start);
        if (res != AQE_OK) {
          return res;
        }
      }
    }
  }
  bool dir_change_needed =
      (_dirPin != PIN_UNDEFINED) && (q->queue_end.count_up != cmd->count_up);

  if (_dirPin & PIN_EXTERNAL_FLAG) {
    if (dir_change_needed) {
      if (!handleExternalDirectionPin(q, cmd->count_up)) {
        struct stepper_command_s pause_cmd = {
            .ticks = US_TO_TICKS((uint16_t)2000),
            .steps = 0,
            .count_up = cmd->count_up};
        res = q->addQueueEntry(&pause_cmd, start);
        if (res == AQE_OK) {
          res = AQE_DIR_PIN_2MS_PAUSE_ADDED;
        }
        return res;
      }
    }
  } else if (dir_change_needed && (cmd->steps != 0)) {
#if defined(BEFORE_DIR_CHANGE_DELAY_TICKS)
    uint16_t before_delay = BEFORE_DIR_CHANGE_DELAY_TICKS(q);
#else
    uint16_t before_delay = 0;
#endif
    uint16_t after_delay = _dir_change_delay_ticks;

#if defined(AFTER_DIR_CHANGE_DELAY_TICKS)
    after_delay = fas_max(AFTER_DIR_CHANGE_DELAY_TICKS(q), after_delay);
#endif

    if (q->_last_command_ticks >= before_delay) {
      before_delay = 0;
    }

    uint8_t commands_needed = 1;
    if (before_delay > 0) {
      commands_needed++;
    }
    if (after_delay > 0) {
      commands_needed++;
    }
    if (q->queueEntries() >= QUEUE_LEN - commands_needed) {
      return AQE_DIR_PIN_IS_BUSY;
    }

    if (before_delay > 0) {
      struct stepper_command_s before_cmd = {
          .ticks = (uint16_t)fas_max(before_delay, MIN_CMD_TICKS),
          .steps = 0,
          .count_up = cmd->count_up};
      res = q->addQueueEntry(&before_cmd, start);
      if (res != AQE_OK) {
        return res;
      }
    }

    if (after_delay > 0) {
      struct stepper_command_s after_cmd = {
          .ticks = (uint16_t)fas_max(after_delay, MIN_CMD_TICKS),
          .steps = 0,
          .count_up = cmd->count_up};
      res = q->addQueueEntry(&after_cmd, start);
      if (res != AQE_OK) {
        return res;
      }
    }
  }
  res = q->addQueueEntry(cmd, start);
  if (res == AQE_OK) {
    if (_autoEnable) {
      fasDisableInterrupts();
      _auto_disable_delay_counter = _off_delay_count;
      fasEnableInterrupts();
    }
  }

  return res;
}

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

bool FastAccelStepper::init(FastAccelStepperEngine* engine, uint8_t num,
                            uint8_t step_pin) {
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
  // For run time measurement
  max_micros = 0;
#endif
  _engine = engine;
  _autoEnable = false;
  _dir_change_delay_ticks = 0;
  _on_delay_ticks = 0;
  _off_delay_count = 1;
  _auto_disable_delay_counter = 0;
  _stepPin = step_pin;
  _dirHighCountsUp = true;
  _dirPin = PIN_UNDEFINED;
  _enablePinHighActive = PIN_UNDEFINED;
  _enablePinLowActive = PIN_UNDEFINED;
  _forward_planning_in_ticks = TICKS_PER_S / 50;
  _pendingExternalDirState = ExtDirPendingState::None;
  _rg.init();

  _queue_num = num;
  _queue()->init(_queue_num, step_pin);
#if defined(SUPPORT_ESP32_PULSE_COUNTER) && (ESP_IDF_VERSION_MAJOR == 5)
  _attached_pulse_unit = NULL;
#endif
#if defined(SUPPORT_ESP32_PULSE_COUNTER) && (ESP_IDF_VERSION_MAJOR == 4)
  _attached_pulse_cnt_unit = -1;
#endif
  return true;
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
MoveTimedResultCode FastAccelStepper::moveTimed(int16_t steps,
                                                uint32_t duration,
                                                uint32_t* actual_duration,
                                                bool start) {
  MoveTimedResultCode ret_ok =
      isQueueEmpty() ? MOVE_TIMED_EMPTY : MOVE_TIMED_OK;
  if ((steps == 0) && (duration == 0)) {
    if (start) {
      addQueueEntry(NULL, true);  // start the queue
    }
    return ret_ok;
  }
  uint8_t freeEntries = QUEUE_LEN - queueEntries();
  if (actual_duration) {
    *actual_duration = 0;
  }
  struct stepper_command_s cmd = {.ticks = 0, .steps = 0, .count_up = true};
  if (steps == 0) {
    if ((duration >> 16) >= QUEUE_LEN) {
      return MOVE_TIMED_TOO_LARGE_ERROR;
    }
    if ((duration >> 16) >= freeEntries) {
      return MOVE_TIMED_BUSY;
    }
    // Should fit
    while (duration > 0) {
      if (duration <= 65535) {
        // done using one command
        cmd.ticks = duration;
      } else if (duration >= 131072) {
        // need more than one command
        cmd.ticks = 65535;
      } else {
        // just use half of the duration now, and the other half in the next
        // cmd.
        cmd.ticks = duration >> 1;
      }
      AqeResultCode ret = addQueueEntry(&cmd, start);
      if (ret != AQE_OK) {
        // unexpected
        return tmrFrom(ret);
      }
      if (actual_duration) {
        *actual_duration += cmd.ticks;
      }
      duration -= cmd.ticks;
    }
    return ret_ok;
  }

  // let's evaluate the direction
  if (steps < 0) {
    cmd.count_up = false;
    steps = -steps;
  }

  // There are steps to execute
  // Let's first calculate the step rate
  uint32_t rate = duration;
  rate /= steps;
  if (rate > 65535) {
    // we need pauses, so only few steps can be executed
    uint16_t cmds_per_step = (rate >> 16) + 1;  // bit too small
    if (cmds_per_step >= QUEUE_LEN) {
      return MOVE_TIMED_TOO_LARGE_ERROR;
    }
    if (steps >= QUEUE_LEN) {
      return MOVE_TIMED_TOO_LARGE_ERROR;
    }
    uint8_t cmds = steps * cmds_per_step;
    if (cmds >= QUEUE_LEN) {
      return MOVE_TIMED_TOO_LARGE_ERROR;
    }
    if (cmds > freeEntries) {
      return MOVE_TIMED_BUSY;
    }
    // Should fit into the queue.
    for (uint8_t s = 0; s < steps; s++) {
      uint32_t this_duration = rate;
      cmd.steps = 1;
      while (this_duration) {
        if (this_duration > 131072) {
          cmd.ticks = 65535;
        } else if (this_duration > 65535) {
          cmd.ticks = this_duration / 2;
        } else {
          cmd.ticks = this_duration;
        }
        this_duration -= cmd.ticks;

        AqeResultCode ret = addQueueEntry(&cmd, start);
        if (ret != AQE_OK) {
          // unexpected
          return tmrFrom(ret);
        }
        if (actual_duration) {
          *actual_duration += cmd.ticks;
        }
        // remaining are pauses
        cmd.steps = 0;
      }
    }
    return ret_ok;
  }
  // Now we need to run steps at "high" speed.
  if (steps > QUEUE_LEN * 255) {
    return MOVE_TIMED_TOO_LARGE_ERROR;
  }
  if (steps > freeEntries * 255) {
    return MOVE_TIMED_BUSY;
  }
  // The steps should fit in
  cmd.ticks = rate;
  uint32_t expected_duration = rate;
  expected_duration *= steps;
  // duration must be larger than expected_duration
  int16_t missing = duration - expected_duration;
#ifdef TEST
  assert(duration >= expected_duration);
#endif
  while (steps > 0) {
    if (steps > 510) {
      cmd.steps = 255;
    } else if (steps > 255) {
      cmd.steps = steps / 2;
    } else {
      cmd.steps = steps;
    }
    if (steps <= missing) {
      // run the remaining steps bit slower to adjust for missing ticks
      cmd.ticks++;
      missing = 0;  // only increase once
#ifdef TEST
      printf("increase ticks for %d steps\n", steps);
#endif
    }
    AqeResultCode ret = addQueueEntry(&cmd, start);
    if (ret != AQE_OK) {
      // unexpected
      return tmrFrom(ret);
    }
    // Why has this been calculated before and actual_duration is used ?
    // uint32_t cmd_duration = cmd.ticks;
    // cmd_duration *= cmd.steps;
    if (actual_duration) {
      uint32_t d = cmd.ticks;
      d *= steps;
      *actual_duration += d;
    }
    steps -= cmd.steps;
  }
  return ret_ok;
}
void FastAccelStepper::detachFromPin() { _queue()->disconnect(); }
void FastAccelStepper::reAttachToPin() { _queue()->connect(); }
