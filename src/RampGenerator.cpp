#include <stdint.h>

#include "FastAccelStepper.h"
#include "StepperISR.h"

#ifdef TEST
#include <assert.h>
#endif

#include "RampCalculator.h"
#include "RampGenerator.h"

// This define in order to not shoot myself.
#ifndef TEST
#define printf DO_NOT_USE_PRINTF
#endif

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

void RampGenerator::init() {
  speed_in_ticks = 0;
  _config.accel_change_cnt = 0;
  _config.min_travel_ticks = 0;
  _config.upm_inv_accel2 = 0;
  _ro.target_pos = 0;
  _rw.pause_ticks_left = 0;
  _rw.performed_ramp_up_steps = 0;
  _rw.accel_change_cnt = 0xff;
  _rw.ramp_state = RAMP_STATE_IDLE;
  _rw.curr_ticks = TICKS_FOR_STOPPED_MOTOR;
#if (TICKS_PER_S != 16000000L)
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
  _config.min_travel_ticks = min_step_ticks;
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
#ifdef UPM_ACCEL_FACTOR
  upm_float upm_inv_accel2 =
      upm_divide(UPM_ACCEL_FACTOR, upm_from((uint32_t)accel));
#else
  upm_float upm_inv_accel =
      upm_divide(upm_shr(UPM_TICKS_PER_S, 1), upm_from(accel));
  upm_float upm_inv_accel2 = upm_multiply(UPM_TICKS_PER_S, upm_inv_accel);
#endif
  if (_config.upm_inv_accel2 != upm_inv_accel2) {
    _config.upm_inv_accel2 = upm_inv_accel2;

    // This is A = f / sqrt(2*a) = (f/sqrt(2))*rsqrt(a)
    _config.upm_sqrt_inv_accel = upm_multiply(
        upm_rsqrt(upm_from((uint32_t)accel)), UPM_TICKS_PER_S_DIV_SQRT_OF_2);
    _config.accel_change_cnt = _rw.accel_change_cnt + 1;
  }
  return 0;
}
void RampGenerator::applySpeedAcceleration() {
  noInterrupts();
  _ro.config = _config;
  interrupts();
}
int8_t RampGenerator::startRun(bool countUp) {
  if (_config.min_travel_ticks == 0) {
    return MOVE_ERR_SPEED_IS_UNDEFINED;
  }
  if (_config.upm_inv_accel2 == 0) {
    return MOVE_ERR_ACCELERATION_IS_UNDEFINED;
  }

  struct ramp_ro_s new_ramp = {.config = _config,
                               .target_pos = 0,
                               .force_stop = false,
                               .keep_running = true,
                               .keep_running_count_up = countUp};

  noInterrupts();
  if (_rw.ramp_state == RAMP_STATE_IDLE) {
    _rw.ramp_state = RAMP_STATE_ACCELERATE;
    _rw.curr_ticks = TICKS_FOR_STOPPED_MOTOR;
    _rw.performed_ramp_up_steps = 0;
  }
  _ro = new_ramp;
  interrupts();
  return MOVE_OK;
}

int8_t RampGenerator::_startMove(int32_t target_pos, int32_t curr_target_pos) {
  if (_config.min_travel_ticks == 0) {
    return MOVE_ERR_SPEED_IS_UNDEFINED;
  }
  if (_config.upm_inv_accel2 == 0) {
    return MOVE_ERR_ACCELERATION_IS_UNDEFINED;
  }

  struct ramp_ro_s new_ramp = {.config = _config,
                               .target_pos = target_pos,
                               .force_stop = false,
                               .keep_running = false,
                               .keep_running_count_up = true};

  noInterrupts();
  if ((_rw.ramp_state == RAMP_STATE_IDLE) && (target_pos != curr_target_pos)) {
    // Only start the ramp generator, if the target position is different
    _rw.ramp_state = RAMP_STATE_ACCELERATE;
    _rw.curr_ticks = TICKS_FOR_STOPPED_MOTOR;
    _rw.performed_ramp_up_steps = 0;
  }
  _ro = new_ramp;
  interrupts();

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
  int res = _startMove(position, curr_pos);
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
  return _startMove(new_pos, curr_pos);
}

//*************************************************************************************************
static void _getNextCommand(const struct ramp_ro_s *ramp,
                            const struct ramp_rw_s *rw,
                            const struct queue_end_s *queue_end,
                            NextCommand *command) {
  {
    // If there is a pause from last step, then just output a pause
    uint32_t pause_ticks = rw->pause_ticks_left;
    if (pause_ticks > 0) {
      if (pause_ticks > 65535) {
        pause_ticks >>= 1;
        pause_ticks = min(pause_ticks, 65535);
      }
      command->command.ticks = pause_ticks;
      command->command.steps = 0;
      command->command.count_up = queue_end->count_up;
      command->rw = *rw;
      command->rw.pause_ticks_left -= pause_ticks;
#ifdef TEST
      printf("add command pause ticks = %d  remaining pause = %d\n",
             pause_ticks, command->rw.pause_ticks_left);
#endif
      return;
    }
  }

  // If the acceleration has changed, recalculate the ramp up/down steps,
  // which is the equivalent to the current speed.
  // Even if the acceleration value is constant, the calculated value
  // can deviate due to precision or clipping effect
  uint32_t curr_ticks = rw->curr_ticks;
  uint8_t accel_change_cnt = ramp->config.accel_change_cnt;
  uint32_t performed_ramp_up_steps;
  if (accel_change_cnt != rw->accel_change_cnt) {
    if (curr_ticks == TICKS_FOR_STOPPED_MOTOR) {
      performed_ramp_up_steps = 0;
    } else {
      performed_ramp_up_steps = upm_to_u32(upm_multiply(
          ramp->config.upm_inv_accel2, upm_rsquare(upm_from(curr_ticks))));
#ifdef TEST
      printf("Recalculate performed_ramp_up_steps to %d from %d ticks\n",
             performed_ramp_up_steps, curr_ticks);
#endif
    }
  } else {
    performed_ramp_up_steps = rw->performed_ramp_up_steps;
  }

  bool count_up = queue_end->count_up;

  // check state for acceleration/deceleration or deceleration to stop
  uint8_t this_state;
  uint32_t remaining_steps;
  bool need_count_up;
  if (ramp->keep_running) {
    need_count_up = ramp->keep_running_count_up;
    remaining_steps = 0xfffffff;
  } else {
    // this can overflow, which is legal
    int32_t delta = ramp->target_pos - queue_end->pos;

    if (delta == 0) {
      // this case can happen on overshoot. So reverse current direction
      need_count_up = !count_up;
    } else {
      need_count_up = delta > 0;
    }
    remaining_steps = abs(delta);
  }

  // If not moving, then use requested direction
  if (performed_ramp_up_steps == 0) {
    count_up = need_count_up;
  }

  if ((remaining_steps == 0) && (performed_ramp_up_steps <= 1)) {
    command->command.ticks = 0;
    command->rw.pause_ticks_left = 0;
    command->rw.ramp_state = RAMP_STATE_IDLE;
    command->rw.curr_ticks = TICKS_FOR_STOPPED_MOTOR;
#ifdef TEST
    puts("ramp complete");
#endif
    return;
  }

  // Forward planning of 2ms or more on slow speed.
  uint16_t planning_steps;
  if (curr_ticks < TICKS_PER_S / 1000) {
    upm_float upm_ps =
        upm_divide(UPM_TICKS_PER_S_DIV_500, upm_from(curr_ticks));
    planning_steps = upm_to_u16(upm_ps);
  } else {
    planning_steps = 1;
  }

  // In case of force stop just run down the ramp
  uint32_t coast_speed = rw->curr_ticks;
  if (ramp->force_stop) {
    this_state = RAMP_STATE_DECELERATE_TO_STOP;
    remaining_steps = performed_ramp_up_steps;
  } else if (count_up != need_count_up) {
    // On direction change, do reversing
    this_state = RAMP_STATE_REVERSE;
    remaining_steps = performed_ramp_up_steps;
  } else {
    // If come here, then direction is same as current movement
    if (remaining_steps == performed_ramp_up_steps) {
      this_state = RAMP_STATE_DECELERATE;
      remaining_steps = performed_ramp_up_steps;
    } else if (remaining_steps < performed_ramp_up_steps) {
      // We will overshoot
      this_state = RAMP_STATE_REVERSE;
      remaining_steps = performed_ramp_up_steps;
    } else if (ramp->config.min_travel_ticks < rw->curr_ticks) {
      this_state = RAMP_STATE_ACCELERATE;
      if (rw->curr_ticks < 2 * MIN_CMD_TICKS) {
        // special consideration needed, that invalid commands are not generated
        //
        // possible coast steps is divided by 4: 1 part acc, 2 part coast, 1
        // part dec
        uint32_t possible_coast_steps =
            (remaining_steps - performed_ramp_up_steps) >> 2;
        // curr_ticks is not necessarily correct due to speed increase
        uint32_t coast_time = possible_coast_steps * rw->curr_ticks;
        if (coast_time < 2 * MIN_CMD_TICKS) {
          this_state = RAMP_STATE_COAST;
#ifdef TEST
          printf("low speed coast %d %d\n", possible_coast_steps,
                 remaining_steps - performed_ramp_up_steps);
#endif
        }
        if (planning_steps > remaining_steps - performed_ramp_up_steps) {
          this_state = RAMP_STATE_DECELERATE;
        }
      }
      if (remaining_steps - performed_ramp_up_steps < 2 * planning_steps) {
        if (curr_ticks != TICKS_FOR_STOPPED_MOTOR) {
          this_state = RAMP_STATE_COAST;
          planning_steps = remaining_steps - performed_ramp_up_steps;
        }
      }
    } else if (ramp->config.min_travel_ticks > rw->curr_ticks) {
      this_state = RAMP_STATE_DECELERATE;
      if (performed_ramp_up_steps <= planning_steps) {
        if (performed_ramp_up_steps > 0) {
          planning_steps = performed_ramp_up_steps;
        } else {
          planning_steps = 1;
        }
      }
    } else {
      this_state = RAMP_STATE_COAST;
      coast_speed = ramp->config.min_travel_ticks;
    }
  }
  if (remaining_steps == 0) {  // This implies performed_ramp_up_steps == 0
    command->command.ticks = 0;
    command->rw.pause_ticks_left = 0;
    command->rw.ramp_state = RAMP_STATE_IDLE;
    command->rw.curr_ticks = TICKS_FOR_STOPPED_MOTOR;
#ifdef TEST
    puts("ramp complete");
#endif
    return;
  }

  // Guarantee here:
  //	remaining_steps > 0
  //	remaining_steps >= performed_ramp_up_steps
  //	remaining_steps >  performed_ramp_up_steps, in COAST
  //	performed_ramp_up_steps can be 0
  //	planning_steps >= 1
#ifdef TEST
  assert(remaining_steps > 0);
  assert(remaining_steps >= performed_ramp_up_steps);
  assert((remaining_steps > performed_ramp_up_steps) ||
         (this_state != RAMP_STATE_COAST));
  assert(planning_steps > 0);
#endif

#ifdef TEST
  printf("prus=%d planning_steps=%d remaining_steps=%d force_stop=%d\n",
         performed_ramp_up_steps, planning_steps, remaining_steps,
         ramp->force_stop);
#endif
  uint32_t d_ticks_new;
  {
    uint32_t coast_steps;
    if (this_state & RAMP_STATE_ACCELERATING_FLAG) {
      // do not overshoot ramp down start
      planning_steps =
          min(planning_steps, (remaining_steps - performed_ramp_up_steps) >> 1);

      uint32_t rs = performed_ramp_up_steps + planning_steps;
      d_ticks_new = calculate_ticks_v8(rs, ramp->config.upm_sqrt_inv_accel);
#ifdef TEST
      printf("Calculate d_ticks_new=%d from ramp steps=%d\n", d_ticks_new, rs);
#endif

      // if acceleration is very high, then d_ticks_new can be lower than
      // min_travel_ticks
      if (d_ticks_new < ramp->config.min_travel_ticks) {
        d_ticks_new = ramp->config.min_travel_ticks;
      }
    } else if (this_state & RAMP_STATE_DECELERATING_FLAG) {
      uint32_t rs;
      if (performed_ramp_up_steps == 0) {
        d_ticks_new = ramp->config.min_travel_ticks;
      } else {
        if (performed_ramp_up_steps <= planning_steps) {
          rs = planning_steps;
        } else {
          rs = performed_ramp_up_steps - planning_steps;
        }
        d_ticks_new = calculate_ticks_v8(rs, ramp->config.upm_sqrt_inv_accel);
      }
#ifdef TEST
      printf("Calculate d_ticks_new=%d from ramp steps=%d\n", d_ticks_new, rs);
#endif
    } else {
      d_ticks_new = coast_speed;
      // do not overshoot ramp down start
      coast_steps = remaining_steps - performed_ramp_up_steps;
      if (coast_steps < planning_steps * 2) {
        planning_steps = coast_steps;
      }
#ifdef TEST
      printf("planning steps=%d remaining steps=%d coast steps=%d\n",
             planning_steps, remaining_steps, coast_steps);
#endif
    }
  }

  if (d_ticks_new < MIN_CMD_TICKS) {
    uint32_t cmd_ticks = d_ticks_new * planning_steps;
    if (cmd_ticks < MIN_CMD_TICKS) {
      // planning_steps = MIN_CMD_TICKS / next_ticks;

      uint32_t new_planning_steps = upm_to_u32(upm_divide(
          upm_from((uint32_t)(REF_CMD_TICKS)), upm_from(d_ticks_new)));
#ifdef TEST
      printf("Increase planning steps %d => %d\n", planning_steps,
             new_planning_steps);
#endif
      planning_steps = new_planning_steps;
    }
  }

  // perform clipping with current ticks
  uint32_t next_ticks = d_ticks_new;
  if (curr_ticks != TICKS_FOR_STOPPED_MOTOR) {
    if (this_state & RAMP_STATE_ACCELERATING_FLAG) {
      next_ticks = min(next_ticks, curr_ticks);
    } else if (this_state & RAMP_STATE_DECELERATING_FLAG) {
      // CLIPPING: avoid reduction unless curr_ticks indicates stopped motor
      // Issue #25: root cause is, that curr_ticks can be
      // TICKS_FOR_STOPPED_MOTOR for the case, that queue is emptied before
      // the next command is issued
      next_ticks = max(next_ticks, curr_ticks);
      //      if (this_state != RAMP_STATE_DECELERATE) {
      //        next_ticks = max(next_ticks, ramp->config.min_travel_ticks);
      //      }
    }
  }
#ifdef TEST
  assert(next_ticks > 0);
#endif
#ifdef TEST
  if (next_ticks != d_ticks_new) {
    printf(
        "Clipping result d_ticks_new=%d => next_ticks=%d  with curr_ticks=%d  "
        "state=%d\n",
        d_ticks_new, next_ticks, curr_ticks, this_state);
  }
#endif

#ifdef TEST
  printf("planning steps=%d remaining steps=%d\n", planning_steps,
         remaining_steps);
#endif
  // Number of steps to execute with limitation to min 1 and max remaining steps
  uint16_t steps = planning_steps;
  steps = min(steps, abs(remaining_steps));  // This could be problematic
  steps = max(steps, 1);
  steps = min(255, steps);

  // Check if pauses need to be added. If yes, reduce next_ticks and calculate
  // pause_ticks_left
  uint32_t pause_ticks_left;
  if (next_ticks > 65535) {
    steps = 1;
    pause_ticks_left = next_ticks;
    next_ticks >>= 1;
    next_ticks = min(next_ticks, 65535);
    pause_ticks_left -= next_ticks;
  } else {
    pause_ticks_left = 0;
  }

  // determine performed_ramp_up_steps after command enqueued
  if (this_state & RAMP_STATE_ACCELERATING_FLAG) {
    performed_ramp_up_steps += steps;
  } else if (this_state & RAMP_STATE_DECELERATING_FLAG) {
    if (performed_ramp_up_steps < steps) {
      // This can occur with performed_ramp_up_steps = 0 and steps = 1
#ifdef TEST
      assert((performed_ramp_up_steps == 0) && (steps == 1));
#endif
      // based on above assumption actually obsolete
      performed_ramp_up_steps = 0;
    } else {
      performed_ramp_up_steps -= steps;
    }
  }

  if (count_up) {
    this_state |= RAMP_DIRECTION_COUNT_UP;
  } else {
    this_state |= RAMP_DIRECTION_COUNT_DOWN;
  }

  command->command.ticks = next_ticks;
  command->command.steps = steps;
  command->command.count_up = count_up;

  command->rw.ramp_state = this_state;
  command->rw.accel_change_cnt = accel_change_cnt;
  command->rw.performed_ramp_up_steps = performed_ramp_up_steps;
  command->rw.pause_ticks_left = pause_ticks_left;
  command->rw.curr_ticks = pause_ticks_left + next_ticks;

#ifdef TEST
  printf(
      "pos@queue_end=%d remaining=%u ramp steps=%u planning steps=%d "
      "last_ticks=%u travel_ticks=%u ",
      queue_end->pos, remaining_steps, performed_ramp_up_steps, planning_steps,
      rw->curr_ticks, ramp->config.min_travel_ticks);
  switch (this_state & RAMP_DIRECTION_MASK) {
    case RAMP_DIRECTION_COUNT_UP:
      printf("+");
      break;
    case RAMP_DIRECTION_COUNT_DOWN:
      printf("-");
      break;
  }
  switch (this_state & RAMP_STATE_MASK) {
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
    case RAMP_STATE_REVERSE:
      printf("REVERSE");
      break;
  }
  printf("\n");
  printf(
      "add command Steps=%u ticks=%u  Target pos=%u "
      "Remaining steps=%u, planning_steps=%u, "
      "d_ticks_new=%u, pause_left=%u\n",
      steps, next_ticks, ramp->target_pos, remaining_steps, planning_steps,
      d_ticks_new, pause_ticks_left);
  if ((this_state & RAMP_STATE_MASK) == RAMP_STATE_ACCELERATE) {
    assert(pause_ticks_left + next_ticks >= ramp->config.min_travel_ticks);
  }
#endif
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
  noInterrupts();
  // copy consistent ramp state
  struct ramp_ro_s ramp = _ro;
  interrupts();

  return _getNextCommand(&ramp, &_rw, queue_end, command);
}
void RampGenerator::stopRamp() {
  // Should be safe on avr and on esp32 due to task prio
  _rw.ramp_state = RAMP_STATE_IDLE;
  _rw.curr_ticks = TICKS_FOR_STOPPED_MOTOR;
  _rw.performed_ramp_up_steps = 0;
}
bool RampGenerator::isRampGeneratorActive() {
  return (_rw.ramp_state != RAMP_STATE_IDLE);
}
int32_t RampGenerator::getCurrentAcceleration() {
  switch (_rw.ramp_state &
          (RAMP_STATE_ACCELERATING_FLAG | RAMP_STATE_DECELERATING_FLAG |
           RAMP_DIRECTION_MASK)) {
    case RAMP_STATE_ACCELERATING_FLAG | RAMP_DIRECTION_COUNT_UP:
      return acceleration;
    case RAMP_STATE_DECELERATING_FLAG | RAMP_DIRECTION_COUNT_UP:
      return -acceleration;
    case RAMP_STATE_ACCELERATING_FLAG | RAMP_DIRECTION_COUNT_DOWN:
      return -acceleration;
    case RAMP_STATE_DECELERATING_FLAG | RAMP_DIRECTION_COUNT_DOWN:
      return acceleration;
  }
  return 0;
}
