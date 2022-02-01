#include <stdint.h>

#include "FastAccelStepper.h"
#include "StepperISR.h"
#include "RampGenerator.h"

// This define in order to not shoot myself.
#ifndef TEST
#define printf DO_NOT_USE_PRINTF
#define puts DO_NOT_USE_PRINTF
#endif

#ifdef SUPPORT_UPM_TIMER_FREQ_VARIABLES
  upm_float upm_timer_freq;
  upm_float upm_timer_freq_div_500;
#endif

void RampGenerator::init() {
  speed_in_ticks = 0;
  _config.init();
  _ro.init();
  _rw.init();
#ifdef SUPPORT_UPM_TIMER_FREQ_VARIABLES
  upm_timer_freq = upm_from((uint32_t)TICKS_PER_S);
  upm_timer_freq_div_500 = upm_divide(upm_timer_freq, UPM_CONST_500);
#endif
}
int8_t RampGenerator::setSpeedInTicks(uint32_t min_step_ticks) {
  if (min_step_ticks < MIN_DELTA_TICKS) {
    return -1;
  }
  if (min_step_ticks == TICKS_FOR_STOPPED_MOTOR) {
    return -1;
  }
  speed_in_ticks = min_step_ticks;
  _config.setSpeedInTicks(min_step_ticks);
  return 0;
}
int8_t RampGenerator::setSpeedInUs(uint32_t min_step_us) {
  if (min_step_us >= TICKS_TO_US(0xffffffff)) {
    return -1;
  }
  uint32_t min_step_ticks = US_TO_TICKS(min_step_us);
  return setSpeedInTicks(min_step_ticks);
}
int8_t RampGenerator::setAcceleration(int32_t accel) {
  if (accel <= 0) {
    return -1;
  }
  acceleration = (uint32_t)accel;
  _config.setAcceleration(accel);
  return 0;
}
void RampGenerator::applySpeedAcceleration() {
  fasDisableInterrupts();
  _ro.config = _config;
  fasEnableInterrupts();
}
int8_t RampGenerator::startRun(bool countUp) {
  uint8_t res = _config.checkValidConfig();
  if (res != MOVE_OK) {
    return res;
  }
  struct ramp_ro_s new_ramp;
  new_ramp.keepRunning(&_config, countUp);

  fasDisableInterrupts();
  _rw.startRampIfNotRunning();
  _ro = new_ramp;
  fasEnableInterrupts();
  return MOVE_OK;
}

int8_t RampGenerator::_startMove(int32_t target_pos, bool position_changed) {
  uint8_t res = _config.checkValidConfig();
  if (res != MOVE_OK) {
    return res;
  }

  struct ramp_ro_s new_ramp;
  new_ramp.runToPosition(&_config, target_pos);

  fasDisableInterrupts();
  if (position_changed) {
    // Only start the ramp generator, if the target position is different
    _rw.startRampIfNotRunning();
  }
  _ro = new_ramp;
  fasEnableInterrupts();

#ifdef TEST
  printf("Ramp data: go to %d  curr_ticks = %u travel_ticks = %u\n", target_pos,
         _rw.curr_ticks, _config.min_travel_ticks);
#endif
#ifdef DEBUG
  char buf[256];
  sprintf(buf, "Ramp data: go to = %ld  curr_ticks = %lu travel_ticks = %lu\n",
          target_pos, _rw.curr_ticks, _min_travel_ticks);
  Serial.println(buf);
#endif
  return MOVE_OK;
}

int8_t RampGenerator::moveTo(int32_t position,
                             const struct queue_end_s *queue_end) {
  int32_t curr_pos;
  if (isRampGeneratorActive() && !_ro.keep_running) {
    curr_pos = _ro.target_pos;
  } else {
    curr_pos = queue_end->pos;
  }
  inject_fill_interrupt(1);
  int res = _startMove(position, curr_pos != position);
  inject_fill_interrupt(2);
  return res;
}
int8_t RampGenerator::move(int32_t move, const struct queue_end_s *queue_end) {
  int32_t curr_pos;
  if (isRampGeneratorActive() && !_ro.keep_running) {
    curr_pos = _ro.target_pos;
  } else {
    curr_pos = queue_end->pos;
  }
  int32_t new_pos = curr_pos + move;
  return _startMove(new_pos, curr_pos != new_pos);
}

void RampGenerator::afterCommandEnqueued(NextCommand *command) {
#ifdef TEST
  printf(
      "after Command Enqueued: performed ramp up steps = %d, pause left = %d, "
      "curr_ticks = %d\n",
      command->rw.performed_ramp_up_steps, command->rw.pause_ticks_left,
      command->rw.curr_ticks);
#endif
  _rw = command->rw;
}
void RampGenerator::getNextCommand(const struct queue_end_s *queue_end,
                                   NextCommand *command) {
  fasDisableInterrupts();
  // copy consistent ramp state
  struct ramp_ro_s ramp = _ro;
  fasEnableInterrupts();

  return _getNextCommand(&ramp, &_rw, queue_end, command);
}
void RampGenerator::stopRamp() {
  _rw.stopRamp();
}
int32_t RampGenerator::getCurrentAcceleration() {
  switch (_rw.ramp_state &
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
