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
  _config.change_cnt = 0;
  _config.min_travel_ticks = 0;
  _config.upm_inv_accel2 = 0;
  _ro.target_pos = 0;
  _wo.ramp_state = RAMP_STATE_IDLE;
#if (TICKS_PER_S != 16000000L)
  upm_timer_freq = upm_from((uint32_t)TICKS_PER_S);
#endif
}
void RampGenerator::setSpeed(uint32_t min_step_us) {
  if (min_step_us == 0) {
    return;
  }
  uint32_t min_travel_ticks = US_TO_TICKS(min_step_us);
  if (min_travel_ticks < MIN_DELTA_TICKS) {
    min_travel_ticks = MIN_DELTA_TICKS;  // set to lower limit
  }
  _config.min_travel_ticks = min_travel_ticks;
  _config.change_cnt++;
}
void RampGenerator::setAcceleration(uint32_t accel) {
  if (accel == 0) {
    return;
  }
  upm_float upm_inv_accel = upm_divide(UPM_TICKS_PER_S, upm_from(2 * accel));
  _config.upm_inv_accel2 = upm_multiply(UPM_TICKS_PER_S, upm_inv_accel);
  _config.change_cnt++;
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

  if (_wo.ramp_state == RAMP_STATE_IDLE) {
    noInterrupts();
    _ro = new_ramp;
    _wo.ramp_state = RAMP_STATE_ACCELERATE;
    interrupts();
  } else {
    noInterrupts();
    _ro = new_ramp;
    interrupts();
  }
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

  if (_wo.ramp_state == RAMP_STATE_IDLE) {
    if (target_pos == queue_end->pos) {
      return MOVE_OK;
    }
    noInterrupts();
    _ro = new_ramp;
    _wo.ramp_state = RAMP_STATE_ACCELERATE;
    interrupts();
  } else {
    noInterrupts();
    _ro = new_ramp;
    interrupts();
  }

#ifdef TEST
  printf("Ramp data: go to %d  curr_ticks = %u travel_ticks = %u\n", target_pos,
         queue_end->ticks, _config.min_travel_ticks);
#endif
#ifdef DEBUG
  char buf[256];
  sprintf(buf, "Ramp data: go to = %ld  curr_ticks = %lu travel_ticks = %lu\n",
          target_pos, queue_end->ticks, _min_travel_ticks);
  Serial.println(buf);
#endif
  return MOVE_OK;
}

int8_t RampGenerator::moveTo(int32_t position,
                             const struct queue_end_s *queue_end) {
  if (isStopping()) {
    return MOVE_ERR_STOP_ONGOING;
  }
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
static uint8_t _getNextCommand(const struct ramp_ro_s *ramp,
                               struct ramp_rw_s *rw,
                               const struct queue_end_s *queue_end,
                               struct stepper_command_s *command) {
  uint32_t qe_ticks = queue_end->ticks;
  if (ramp->config.change_cnt != rw->change_cnt) {
    uint32_t performed_ramp_up_steps = upm_to_u32(upm_divide(
        ramp->config.upm_inv_accel2, upm_square(upm_from(qe_ticks))));
    noInterrupts();
    rw->change_cnt = ramp->config.change_cnt;
    rw->performed_ramp_up_steps = performed_ramp_up_steps;
    interrupts();
  }

  bool count_up = queue_end->count_up;

  // check state for acceleration/deceleration or deceleration to stop
  uint8_t next_state;
  uint32_t remaining_steps;
  bool need_count_up;
  if (ramp->keep_running) {
    need_count_up = ramp->keep_running_count_up;
    remaining_steps = 0xfffffff;
  } else {
    int32_t delta =
        ramp->target_pos - queue_end->pos;  // this can overflow, which is legal
    if (delta == 0) {  // This case should actually never happen
      command->ticks = 0;
      return RAMP_STATE_IDLE;
    }
    need_count_up = delta > 0;
    remaining_steps = abs(delta);
  }

  if (queue_end->ticks == TICKS_FOR_STOPPED_MOTOR) {
    count_up = need_count_up;
  }

  if (ramp->force_stop) {
    next_state = RAMP_STATE_DECELERATE_TO_STOP;
    remaining_steps = rw->performed_ramp_up_steps;
  }
  // Detect change in direction and if so, initiate deceleration to stop
  else if (count_up != need_count_up) {
    next_state = RAMP_STATE_REVERSE;
    uint32_t prus = rw->performed_ramp_up_steps;
    if (prus != 0) {
      remaining_steps = prus;
    } else {
      count_up = need_count_up;
      next_state = RAMP_STATE_ACCELERATE;
    }
  } else {
    // If come here, then direction is same as current movement
    if (remaining_steps <= rw->performed_ramp_up_steps) {
      next_state = RAMP_STATE_DECELERATE_TO_STOP;
    } else if (ramp->config.min_travel_ticks < qe_ticks) {
      next_state = RAMP_STATE_ACCELERATE;
    } else if (ramp->config.min_travel_ticks > qe_ticks) {
      next_state = RAMP_STATE_DECELERATE;
    } else {
      next_state = RAMP_STATE_COAST;
    }
  }
  // Forward planning of 1ms or more on slow speed.
  uint32_t planning_steps = max((TICKS_PER_S / 1000) / qe_ticks, 1);
  uint32_t next_ticks;

#ifdef TEST
  printf(
      "pos@queue_end=%d remaining=%u ramp steps=%u planning steps=%d "
      "queue_end.ticks=%u ",
      queue_end->pos, remaining_steps, rw->performed_ramp_up_steps,
      planning_steps, qe_ticks);
  if (count_up) {
    printf("+");
  } else {
    printf("-");
  }
  switch (next_state) {
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

  uint32_t curr_ticks = qe_ticks;
  switch (next_state) {
    uint32_t d_ticks_new;
    uint32_t upm_rem_steps;
    upm_float upm_d_ticks_new;
    case RAMP_STATE_COAST:
      next_ticks = ramp->config.min_travel_ticks;
      // do not overshoot ramp down start
      planning_steps =
          min(planning_steps, remaining_steps - rw->performed_ramp_up_steps);
      break;
    case RAMP_STATE_ACCELERATE:
      upm_rem_steps = upm_from(rw->performed_ramp_up_steps + planning_steps);
      upm_d_ticks_new =
          upm_sqrt(upm_divide(ramp->config.upm_inv_accel2, upm_rem_steps));

      d_ticks_new = upm_to_u32(upm_d_ticks_new);

      // avoid overshoot
      next_ticks = max(d_ticks_new, ramp->config.min_travel_ticks);
      if (rw->performed_ramp_up_steps == 0) {
        curr_ticks = d_ticks_new;
      } else {
        // CLIPPING: avoid increase
        next_ticks = min(next_ticks, curr_ticks);
      }

#ifdef TEST
      printf("accelerate ticks => %d  during %d steps (d_ticks_new = %u)",
             next_ticks, planning_steps, d_ticks_new);
      printf("... %u+%u steps\n", rw->performed_ramp_up_steps, planning_steps);
#endif
      break;
    case RAMP_STATE_DECELERATE:
      upm_rem_steps = upm_from(rw->performed_ramp_up_steps + planning_steps);
      upm_d_ticks_new =
          upm_sqrt(upm_divide(ramp->config.upm_inv_accel2, upm_rem_steps));

      d_ticks_new = upm_to_u32(upm_d_ticks_new);

      // avoid undershoot
      next_ticks = min(d_ticks_new, ramp->config.min_travel_ticks);

      // CLIPPING: avoid reduction
      next_ticks = max(next_ticks, curr_ticks);

#ifdef TEST
      printf("decelerate ticks => %d  during %d steps (d_ticks_new = %u)",
             next_ticks, planning_steps, d_ticks_new);
      printf("... %u+%u steps\n", rw->performed_ramp_up_steps, planning_steps);
#endif
      break;
    case RAMP_STATE_DECELERATE_TO_STOP:
    case RAMP_STATE_REVERSE:
      upm_rem_steps = upm_from(remaining_steps - planning_steps);
      upm_d_ticks_new =
          upm_sqrt(upm_divide(ramp->config.upm_inv_accel2, upm_rem_steps));

      d_ticks_new = upm_to_u32(upm_d_ticks_new);

      // avoid undershoot
      next_ticks = max(d_ticks_new, ramp->config.min_travel_ticks);

      // CLIPPING: avoid reduction
      next_ticks = max(next_ticks, curr_ticks);
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
  steps = max(steps, 1);
  steps = min(steps, abs(remaining_steps));
  steps = min(255, steps);

#ifdef TEST
  assert(next_ticks > 0);
#endif

  if (next_ticks > 65535) {
    steps = 1;
    // insert a pause
    if (queue_end->ticks_from_last_step < next_ticks) {
      next_ticks -= queue_end->ticks_from_last_step;
      if (next_ticks > 65535) {
        // insert a pause
        next_ticks >>= 1;
        next_ticks = min(next_ticks, 65535);
        steps = 0;
      }
    } else {
      next_ticks = 32768;
    }
  }

  if (steps == abs(remaining_steps)) {
    if (count_up == need_count_up) {
      next_state = RAMP_STATE_IDLE;
    }
  }

  command->ticks = next_ticks;
  command->steps = steps;
  command->count_up = count_up;

#ifdef TEST
  printf(
      "add command Steps = %d ticks = %d  Target pos = %d "
      "Remaining steps = %d\n",
      steps, next_ticks, ramp->target_pos, remaining_steps);
#endif
  return next_state;
}
void RampGenerator::commandEnqueued(struct stepper_command_s *command,
                                    uint8_t state) {
  noInterrupts();
  switch (state & RAMP_STATE_MASK) {
    case RAMP_STATE_COAST:
      break;
    case RAMP_STATE_ACCELERATE:
      _rw.performed_ramp_up_steps += command->steps;
      break;
    case RAMP_STATE_REVERSE:
    case RAMP_STATE_DECELERATE:
    case RAMP_STATE_DECELERATE_TO_STOP:
      _rw.performed_ramp_up_steps -= command->steps;
      break;
  }
  interrupts();
}
uint8_t RampGenerator::getNextCommand(const struct queue_end_s *queue_end,
                                      struct stepper_command_s *command) {
  noInterrupts();
  // copy consistent ramp state
  struct ramp_ro_s ramp = _ro;
  interrupts();

  return _getNextCommand(&ramp, &_rw, queue_end, command);
}
void RampGenerator::stopRamp() { _wo.ramp_state = RAMP_STATE_IDLE; }
bool RampGenerator::isRampGeneratorActive() {
  return (_wo.ramp_state != RAMP_STATE_IDLE);
}
