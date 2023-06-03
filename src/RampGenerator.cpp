#include <stdint.h>

#include "FastAccelStepper.h"
#include "RampGenerator.h"
#include "StepperISR.h"

// This define in order to not shoot myself.
#ifndef TEST
#define printf DO_NOT_USE_PRINTF
#define puts DO_NOT_USE_PRINTF
#endif

void RampGenerator::init() {
  _parameters.init();
  _ro.init();
  _rw.init();
  init_ramp_module();
}
int8_t RampGenerator::setAcceleration(int32_t accel) {
  if (accel <= 0) {
    return -1;
  }
  acceleration = (uint32_t)accel;
  _parameters.setAcceleration(accel);
  return 0;
}
void RampGenerator::applySpeedAcceleration() {
  if (!_ro.isImmediateStopInitiated()) {
    _parameters.applyParameters();
  }
}
int8_t RampGenerator::startRun(bool countUp) {
  uint8_t res = _parameters.checkValidConfig();
  if (res != MOVE_OK) {
    return res;
  }
  _ro.force_stop = false;
  _parameters.setRunning(countUp);
  _rw.startRampIfNotRunning(_parameters.s_jump);
#ifdef DEBUG
  char buf[256];
  sprintf(buf, "Ramp data: curr_ticks = %lu travel_ticks = %lu\n",
          _rw.curr_ticks, _config->min_travel_ticks);
  Serial.println(buf);
#endif
  return MOVE_OK;
}

void RampGenerator::_startMove(bool position_changed) {
  _ro.force_stop = false;

  if (position_changed) {
    // Only start the ramp generator, if the target position is different
    _rw.startRampIfNotRunning(_parameters.s_jump);
  }

#ifdef TEST
  printf("Ramp data: go to %s %d  curr_ticks = %u travel_ticks = %u prus=%u\n",
         _parameters.move_absolute ? "ABS" : "REL", _parameters.move_value,
         _rw.curr_ticks, _ro.config.parameters.min_travel_ticks,
         _rw.performed_ramp_up_steps);
#endif
#ifdef DEBUG
  char buf[256];
  sprintf(buf,
          "Ramp data: go to = %s %ld  curr_ticks = %lu travel_ticks = %lu "
          "prus=%lu\n",
          _parameters.move_absolute ? "ABS" : "REL", _parameters.move_value,
          _rw.curr_ticks, _ro.config.parameters.min_travel_ticks,
          _rw.performed_ramp_up_steps);
  Serial.println(buf);
#endif
}

int8_t RampGenerator::moveTo(int32_t position,
                             const struct queue_end_s *queue_end) {
  uint8_t res = _parameters.checkValidConfig();
  if (res != MOVE_OK) {
    return res;
  }
  int32_t curr_target;
  if (isRampGeneratorActive() && !_ro.config.parameters.keep_running) {
    curr_target = _ro.target_pos;
  } else {
    curr_target = queue_end->pos;
  }
  inject_fill_interrupt(1);
  _parameters.setTargetPosition(position);
  _startMove(curr_target != position);
  inject_fill_interrupt(2);
  return MOVE_OK;
}
int8_t RampGenerator::move(int32_t move, const struct queue_end_s *queue_end) {
  uint8_t res = _parameters.checkValidConfig();
  if (res != MOVE_OK) {
    return res;
  }
  _parameters.setTargetRelativePosition(move);
  _startMove(move != 0);
  return MOVE_OK;
}
void RampGenerator::advanceTargetPosition(int32_t delta,
                                          const struct queue_end_s *queue) {
  // called with interrupts disabled
  _ro.target_pos += delta;
}

void RampGenerator::afterCommandEnqueued(NextCommand *command) {
#ifdef TEST
  printf(
      "after Command Enqueued: performed ramp up steps = %u, pause left = %u, "
      "curr_ticks = %u\n",
      command->rw.performed_ramp_up_steps, command->rw.pause_ticks_left,
      command->rw.curr_ticks);
#endif
  _rw = command->rw;
}
void RampGenerator::getNextCommand(const struct queue_end_s *queue_end,
                                   NextCommand *command) {
  // we are running in higher priority than the application
  // so we can just read the config without disable interrupts
  // copy consistent ramp state
  if (_parameters.apply) {
    _ro.config.parameters = _parameters;
    _parameters.apply = false;
    _parameters.any_change = false;
    _parameters.move_value = 0;
    _parameters.move_absolute = false;
    _parameters.recalc_ramp_steps = false;
    _ro.config.update();
    // if new move command,then reset any immediate stop flag
    if (_ro.isImmediateStopInitiated()) {
      if (_ro.config.parameters.move_absolute) {
        if (_ro.target_pos != (uint32_t)_ro.config.parameters.move_value) {
          _ro.clearImmediateStop();
        }
      } else if (_ro.config.parameters.move_value != 0) {
        _ro.clearImmediateStop();
      }
    }
  }

  fasDisableInterrupts();
  struct queue_end_s qe = *queue_end;
  fasEnableInterrupts();

  uint32_t curr_ticks = _rw.curr_ticks;
  if (curr_ticks == TICKS_FOR_STOPPED_MOTOR) {
    // just started
    uint32_t s_jump = _ro.config.parameters.s_jump;
    if (s_jump != 0) {
      uint32_t ticks = _ro.config.calculate_ticks(s_jump);
      if (ticks < _ro.config.parameters.min_travel_ticks) {
        s_jump = _ro.config.calculate_ramp_steps(
            _ro.config.parameters.min_travel_ticks);
      }
    }
    _rw.performed_ramp_up_steps = s_jump;
    _ro.config.parameters.recalc_ramp_steps = false;
  }

  // If the acceleration has changed, recalculate the ramp up/down steps,
  // which is the equivalent to the current speed.
  // Even if the acceleration value is constant, the calculated value
  // can deviate due to precision or clipping effect
  if (_ro.config.parameters.recalc_ramp_steps) {
    uint32_t performed_ramp_up_steps =
        _ro.config.calculate_ramp_steps(curr_ticks);
#ifdef TEST
    printf("Recalculate performed_ramp_up_steps from %d to %d from %d ticks\n",
           _rw.performed_ramp_up_steps, performed_ramp_up_steps, curr_ticks);
#endif
    _rw.performed_ramp_up_steps = performed_ramp_up_steps;
  }

  if (_ro.force_stop) {
    _ro.config.parameters.keep_running = false;
    uint32_t target_pos = qe.pos;
    if (qe.count_up) {
      target_pos += _rw.performed_ramp_up_steps;
    } else {
      target_pos -= _rw.performed_ramp_up_steps;
    }
#ifdef TEST
    printf("Force stop: adjust target position from %d to %d\n", _ro.target_pos,
           target_pos);
#endif
    _ro.target_pos = target_pos;
  } else if (_ro.config.parameters.any_change) {
    // calculate new target position
    _ro.config.parameters.any_change = false;
    if (_ro.config.parameters.keep_running) {
    } else if (_ro.config.parameters.move_absolute) {
      _ro.target_pos = _ro.config.parameters.move_value;
    } else {
      _ro.target_pos = _ro.target_pos + _ro.config.parameters.move_value;
    }
  }

  _ro.force_stop = false;

  // clear recalc flag
  _ro.config.parameters.recalc_ramp_steps = false;

  if (_ro.isImmediateStopInitiated()) {
    // no more commands
    command->command.ticks = 0;
    _ro.clearImmediateStop();
    _ro.target_pos = qe.pos;
    command->rw.stopRamp();
    return;
  }
  _getNextCommand(&_ro, &_rw, &qe, command);
}
int32_t RampGenerator::getCurrentAcceleration() {
  switch (_rw.rampState() &
          (RAMP_STATE_ACCELERATING_FLAG | RAMP_STATE_DECELERATING_FLAG |
           RAMP_DIRECTION_MASK)) {
    case RAMP_STATE_ACCELERATING_FLAG | RAMP_DIRECTION_COUNT_UP:
    case RAMP_STATE_DECELERATING_FLAG | RAMP_DIRECTION_COUNT_DOWN:
      return acceleration;
    case RAMP_STATE_DECELERATING_FLAG | RAMP_DIRECTION_COUNT_UP:
    case RAMP_STATE_ACCELERATING_FLAG | RAMP_DIRECTION_COUNT_DOWN:
      return -acceleration;
  }
  return 0;
}
