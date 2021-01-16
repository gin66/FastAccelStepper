#include <stdint.h>

#include "FastAccelStepper.h"
#include "StepperISR.h"

#ifdef TEST
#include <assert.h>
#endif

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
  speed_in_us = 0;
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
#endif
}
void RampGenerator::setSpeed(uint32_t min_step_us) {
  if (min_step_us == 0) {
    return;
  }
  speed_in_us = min_step_us;
  uint32_t min_travel_ticks = US_TO_TICKS(min_step_us);
  if (min_travel_ticks < MIN_DELTA_TICKS) {
    min_travel_ticks = MIN_DELTA_TICKS;  // set to lower limit
  }
  _config.min_travel_ticks = min_travel_ticks;
}
void RampGenerator::setAcceleration(uint32_t accel) {
  if (accel == 0) {
    return;
  }
  upm_float upm_inv_accel = upm_divide(UPM_TICKS_PER_S, upm_from(2 * accel));
  _config.upm_inv_accel2 = upm_multiply(UPM_TICKS_PER_S, upm_inv_accel);
  _config.accel_change_cnt = _rw.accel_change_cnt + 1;
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
  }
  _ro = new_ramp;
  interrupts();
  return MOVE_OK;
}

int8_t RampGenerator::_startMove(int32_t target_pos,
                                 const struct queue_end_s *queue_end) {
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
  if (_rw.ramp_state == RAMP_STATE_IDLE) {
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
  inject_fill_interrupt(1);
  int res = _startMove(position, queue_end);
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
  return moveTo(new_pos, queue_end);
}

//*************************************************************************************************
static void _getNextCommand(const struct ramp_ro_s *ramp,
                            const struct ramp_rw_s *rw,
                            const struct queue_end_s *queue_end,
                            NextCommand *command) {
  // If there is a pause from last step, then just output a pause
  if (rw->pause_ticks_left > 0) {
    uint32_t ticks = rw->pause_ticks_left;
    if (ticks > 65535) {
      ticks >>= 1;
      ticks = min(ticks, 65535);
    }
    command->command.ticks = ticks;
    command->command.steps = 0;
    command->command.count_up = queue_end->count_up;
    command->rw.ramp_state = rw->ramp_state;
    command->rw.accel_change_cnt = rw->accel_change_cnt;
    command->rw.performed_ramp_up_steps = rw->performed_ramp_up_steps;
    command->rw.pause_ticks_left = rw->pause_ticks_left - ticks;
    command->rw.curr_ticks = rw->curr_ticks;
#ifdef TEST
    printf(
        "add command pause ticks = %d  remaining pause = %d  Target pos = %d\n",
        ticks, command->rw.pause_ticks_left, ramp->target_pos);
#endif
    return;
  }

  // If the acceleration has changed, recalculate the ramp up/down steps,
  // which is the equivalent to the current speed.
  // Even if the accelration value is constant, the calculated value
  // can deviated due to precision or clipping effect
  uint8_t accel_change_cnt = ramp->config.accel_change_cnt;
  uint32_t performed_ramp_up_steps;
  if (accel_change_cnt != rw->accel_change_cnt) {
    performed_ramp_up_steps = upm_to_u32(upm_divide(
        ramp->config.upm_inv_accel2, upm_square(upm_from(rw->curr_ticks))));
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
  uint32_t curr_ticks = rw->curr_ticks;
  uint16_t planning_steps;
  if (curr_ticks < TICKS_PER_S/1000) {
      upm_float upm_ps = upm_divide(UPM_TICKS_PER_S, upm_from(curr_ticks));
      upm_ps = upm_divide(upm_ps, UPM_CONST_500);
	  planning_steps = upm_to_u16(upm_ps);
  }
  else {
	  planning_steps = 1;
  }

  // In case of force stop just run down the ramp
  uint32_t coast_speed = rw->curr_ticks;
  if (ramp->force_stop) {
    this_state = RAMP_STATE_DECELERATE_TO_STOP;
    remaining_steps = performed_ramp_up_steps;
  } else if (count_up != need_count_up) {
    // On direcction change, do reversing
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
  //	performed_ramp_up_steps can be 0

  uint32_t next_ticks;

#ifdef TEST
  printf(
      "pos@queue_end=%d remaining=%u ramp steps=%u planning steps=%d "
      "last_ticks=%u travel_ticks=%u ",
      queue_end->pos, remaining_steps, performed_ramp_up_steps, planning_steps,
      rw->curr_ticks, ramp->config.min_travel_ticks);
  if (count_up) {
    printf("+");
  } else {
    printf("-");
  }
  switch (this_state) {
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
#endif

  // Guarantee here:
  //	remaining_steps > 0
  //	remaining_steps >= performed_ramp_up_steps
  //	remaining_steps >  performed_ramp_up_steps, in COAST
  //	performed_ramp_up_steps can be 0
  //	planning_steps >= 1

  // TODO:
  // In case of high speed, need to stop acceleration, if the steps in high
  // speed too low otherwise esp32 can reject a command ! During COAST need to
  // make sure the steps are not 1

  switch (this_state) {
    uint32_t d_ticks_new;
    uint32_t upm_rem_steps;
    upm_float upm_d_ticks_new;
    uint32_t coast_steps;
    case RAMP_STATE_COAST:
      next_ticks = coast_speed;
      // do not overshoot ramp down start
      coast_steps = remaining_steps - performed_ramp_up_steps;
      if (coast_steps < planning_steps * 2) {
        planning_steps = coast_steps;
      }
      break;
    case RAMP_STATE_ACCELERATE:
      // do not overshoot ramp down start
      planning_steps =
          min(planning_steps, (remaining_steps - performed_ramp_up_steps) >> 1);

      upm_rem_steps = upm_from(performed_ramp_up_steps + planning_steps);
      upm_d_ticks_new =
          upm_sqrt(upm_divide(ramp->config.upm_inv_accel2, upm_rem_steps));

      d_ticks_new = upm_to_u32(upm_d_ticks_new);

      // if acceleration is very high, then d_ticks_new can be lower than
      // min_travel_ticks
      next_ticks = max(d_ticks_new, ramp->config.min_travel_ticks);

      if (curr_ticks != TICKS_FOR_STOPPED_MOTOR) {
        // CLIPPING: avoid increase
        next_ticks = min(next_ticks, curr_ticks);
      }

#ifdef TEST
      printf("accelerate ticks => %d  during %d steps (d_ticks_new = %u)",
             next_ticks, planning_steps, d_ticks_new);
      printf("... %u+%u steps\n", performed_ramp_up_steps, planning_steps);
#endif
      break;
    case RAMP_STATE_DECELERATE:

      if (performed_ramp_up_steps > planning_steps) {
        upm_rem_steps = upm_from(performed_ramp_up_steps - planning_steps);
        upm_d_ticks_new =
            upm_sqrt(upm_divide(ramp->config.upm_inv_accel2, upm_rem_steps));
        d_ticks_new = upm_to_u32(upm_d_ticks_new);
      } else {
        planning_steps = max(1, performed_ramp_up_steps);
        d_ticks_new = curr_ticks;
      }

      next_ticks = d_ticks_new;

      // CLIPPING: avoid reduction unless curr_ticks indicates stopped motor
      // Issue #25: root cause is, that curr_ticks can be
      // TICKS_FOR_STOPPED_MOTOR for the case, that queue is emptied before the
      // next command is issued
      if (curr_ticks != TICKS_FOR_STOPPED_MOTOR) {
        next_ticks = max(next_ticks, curr_ticks);
      }

#ifdef TEST
      printf("decelerate ticks => %d  during %d steps (d_ticks_new = %u)",
             next_ticks, planning_steps, d_ticks_new);
      printf("... %u+%u steps\n", performed_ramp_up_steps, planning_steps);
#endif
      break;
    case RAMP_STATE_DECELERATE_TO_STOP:
      upm_rem_steps = upm_from(remaining_steps - planning_steps);
      upm_d_ticks_new =
          upm_sqrt(upm_divide(ramp->config.upm_inv_accel2, upm_rem_steps));

      d_ticks_new = upm_to_u32(upm_d_ticks_new);

      // CLIPPING: avoid reduction
      next_ticks = max(d_ticks_new, curr_ticks);
#ifdef TEST
      printf("decelerate ticks => %d  during %d steps (d_ticks_new = %u)\n",
             next_ticks, planning_steps, d_ticks_new);
#endif
      break;
    case RAMP_STATE_REVERSE:
      upm_rem_steps = upm_from(remaining_steps - planning_steps);
      upm_d_ticks_new =
          upm_sqrt(upm_divide(ramp->config.upm_inv_accel2, upm_rem_steps));

      d_ticks_new = upm_to_u32(upm_d_ticks_new);

      next_ticks = d_ticks_new;

      // CLIPPING: avoid reduction
      if (curr_ticks != TICKS_FOR_STOPPED_MOTOR) {
        next_ticks = max(d_ticks_new, curr_ticks);
      }
#ifdef TEST
      printf("decelerate ticks => %d  during %d steps (d_ticks_new = %u)\n",
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

  // Number of steps to execute with limitation to min 1 and max remaining steps
  uint16_t steps = planning_steps;

#ifdef TEST
  printf(
      "steps for the command = %d  with planning_steps = %u and "
      "next_ticks = %u\n",
      steps, planning_steps, next_ticks);
#endif
  steps = min(steps, abs(remaining_steps));  // This could be problematic
  steps = max(steps, 1);
  steps = min(255, steps);

#ifdef TEST
  assert(next_ticks > 0);
#endif

  uint32_t pause_ticks_left;
  if (next_ticks > 65535) {
    steps = 1;
    pause_ticks_left = next_ticks;
    next_ticks >>= 1;
    next_ticks = min(next_ticks, 65535);
    pause_ticks_left -= next_ticks;
  } else {
    pause_ticks_left = 0;
    if (steps == remaining_steps) {
      if (count_up == need_count_up) {
        if (performed_ramp_up_steps == steps) {
          this_state = RAMP_STATE_IDLE;
        }
      }
    }
  }

  switch (this_state & RAMP_STATE_MASK) {
    case RAMP_STATE_COAST:
      break;
    case RAMP_STATE_ACCELERATE:
      performed_ramp_up_steps += steps;
      break;
    case RAMP_STATE_REVERSE:
    case RAMP_STATE_DECELERATE:
    case RAMP_STATE_DECELERATE_TO_STOP:
      if (performed_ramp_up_steps < steps) {
        performed_ramp_up_steps = 0;  // TODO: should be obsolete
      } else {
        performed_ramp_up_steps -= steps;
      }
      break;
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
      "add command Steps = %d ticks = %d  Target pos = %d "
      "Remaining steps = %d\n",
      steps, next_ticks, ramp->target_pos, remaining_steps);
#endif
}
void RampGenerator::afterCommandEnqueued(NextCommand *command) {
#ifdef TEST
  puts("after Command Enqueued");
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
