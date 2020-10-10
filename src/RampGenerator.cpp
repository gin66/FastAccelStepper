
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
  _config.min_travel_ticks = 0;
  _config.upm_inv_accel2 = 0;
  _ro.target_pos = 0;
  _rw.ramp_state = RAMP_STATE_IDLE;
#ifdef F_CPU
#if (F_CPU != 16000000)
  upm_timer_freq = upm_from((uint32_t)F_CPU);
#endif
#endif
}
void RampGenerator::update_ramp_steps() {
  _config.ramp_steps = upm_to_u32(divide(
      _config.upm_inv_accel2, square(upm_from(_config.min_travel_ticks))));
}
void RampGenerator::setSpeed(uint32_t min_step_us) {
  _config.min_travel_ticks = min_step_us * (TICKS_PER_S / 1000L) / 1000L;
  update_ramp_steps();
}
void RampGenerator::setAcceleration(uint32_t accel) {
  uint32_t tmp = TICKS_PER_S / 2;
  upm_float upm_inv_accel = upm_from(tmp / accel);
  _config.upm_inv_accel2 = multiply(UPM_TICKS_PER_S, upm_inv_accel);
  update_ramp_steps();
}
int RampGenerator::calculate_moveTo(int32_t target_pos,
                                    const struct ramp_config_s *config,
                                    uint32_t ticks_at_queue_end,
                                    int32_t position_at_queue_end) {
  if (config->min_travel_ticks == 0) {
    return MOVE_ERR_SPEED_IS_UNDEFINED;
  }
  if (config->upm_inv_accel2 == 0) {
    return MOVE_ERR_ACCELERATION_IS_UNDEFINED;
  }

  uint32_t performed_ramp_up_steps = upm_to_u32(
      divide(config->upm_inv_accel2, square(upm_from(ticks_at_queue_end))));

  uint8_t start_state;
  if (target_pos > position_at_queue_end) {
    start_state = RAMP_STATE_ACCELERATE | RAMP_MOVE_UP;
  } else if (target_pos < position_at_queue_end) {
    start_state = RAMP_STATE_ACCELERATE | RAMP_MOVE_DOWN;
  } else {
    start_state = RAMP_STATE_IDLE;
  }

  noInterrupts();
  _ro.target_pos = target_pos;
  _ro.min_travel_ticks = config->min_travel_ticks;
  _ro.upm_inv_accel2 = config->upm_inv_accel2;
  _ro.force_stop = false;
  _rw.performed_ramp_up_steps = performed_ramp_up_steps;
  if (_rw.ramp_state == RAMP_STATE_IDLE) {
    _rw.ramp_state = start_state;
  }
  interrupts();

#ifdef TEST
  printf(
      "Ramp data: go to %d  curr_ticks = %u travel_ticks = %u "
      "Ramp steps = %u Performed ramp steps = %u\n",
      target_pos, ticks_at_queue_end, config->min_travel_ticks,
      config->ramp_steps, performed_ramp_up_steps);
#endif
#ifdef DEBUG
  char buf[256];
  sprintf(buf,
          "Ramp data: go to = %ld  curr_ticks = %lu travel_ticks = %lu "
          "Ramp steps = %lu Performed ramp steps = %lu\n",
          target_pos, ticks_at_queue_end, _min_travel_ticks, config->ramp_steps,
          performed_ramp_up_steps);
  Serial.println(buf);
#endif
  return MOVE_OK;
}

//*************************************************************************************************
void RampGenerator::single_fill_queue(const struct ramp_ro_s *ro,
                                      struct ramp_rw_s *rw,
                                      uint32_t ticks_at_queue_end,
                                      int32_t position_at_queue_end,
                                      struct ramp_command_s *command) {
  // This should never be true
  if (ticks_at_queue_end == 0) {
    ticks_at_queue_end = TICKS_FOR_STOPPED_MOTOR;
  }

  // check state for acceleration/deceleration or deceleration to stop
  bool need_count_up = ro->target_pos > position_at_queue_end;
  uint32_t remaining_steps = abs(ro->target_pos - position_at_queue_end);
  uint8_t next_state = rw->ramp_state;
  uint8_t move_state = next_state & RAMP_MOVE_MASK;
  bool countUp = (move_state == RAMP_MOVE_UP);

  if (ro->force_stop) {
    next_state = RAMP_STATE_DECELERATE_TO_STOP | move_state;
    remaining_steps = rw->performed_ramp_up_steps;
  }
  // Detect change in direction and if so, initiate deceleration to stop
  else if (countUp != need_count_up) {
    next_state = RAMP_STATE_DECELERATE_TO_STOP | move_state;
    remaining_steps = rw->performed_ramp_up_steps;
  } else {
    // If come here, then direction is same as current movement
    if (remaining_steps <= rw->performed_ramp_up_steps) {
      next_state = RAMP_STATE_DECELERATE_TO_STOP;
    } else if (ro->min_travel_ticks < ticks_at_queue_end) {
      next_state = RAMP_STATE_ACCELERATE;
    } else if (ro->min_travel_ticks > ticks_at_queue_end) {
      next_state = RAMP_STATE_DECELERATE;
    } else {
      next_state = RAMP_STATE_COAST;
    }
    next_state |= move_state;
  }

  switch (rw->ramp_state & RAMP_STATE_MASK) {
    case RAMP_STATE_COAST:
      break;
    case RAMP_STATE_ACCELERATE:
      break;
    case RAMP_STATE_DECELERATE:
      break;
    case RAMP_STATE_DECELERATE_TO_STOP:
      break;
  }

  // TODO:explain the 16000
  uint32_t planning_steps = max(16000 / ticks_at_queue_end, 1);
  uint32_t next_ticks;

  rw->ramp_state = next_state;

  // Forward planning of minimum 10ms or more on slow speed.

#ifdef TEST
  printf("pos@queue_end=%d remaining=%u ramp steps=%u planning steps=%d  ",
         position_at_queue_end, remaining_steps, rw->performed_ramp_up_steps,
         planning_steps);
  switch (next_state & RAMP_MOVE_MASK) {
    case RAMP_MOVE_UP:
      printf("+");
      break;
    case RAMP_MOVE_DOWN:
      printf("-");
      break;
    case 0:
      printf("=");
      break;
    default:
      printf("ERR");
      break;
  }
  switch (next_state & RAMP_STATE_MASK) {
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
  switch (next_state & RAMP_STATE_MASK) {
    uint32_t d_ticks_new;
    uint32_t upm_rem_steps;
    upm_float upm_d_ticks_new;
    case RAMP_STATE_COAST:
      next_ticks = ro->min_travel_ticks;
      // do not overshoot ramp down start
      planning_steps =
          min(planning_steps, remaining_steps - rw->performed_ramp_up_steps);
      break;
    case RAMP_STATE_ACCELERATE:
      upm_rem_steps = upm_from(rw->performed_ramp_up_steps + planning_steps);
      upm_d_ticks_new = sqrt(divide(ro->upm_inv_accel2, upm_rem_steps));

      d_ticks_new = upm_to_u32(upm_d_ticks_new);

      // avoid overshoot
      next_ticks = max(d_ticks_new, ro->min_travel_ticks);
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
      upm_d_ticks_new = sqrt(divide(ro->upm_inv_accel2, upm_rem_steps));

      d_ticks_new = upm_to_u32(upm_d_ticks_new);

      // avoid undershoot
      next_ticks = min(d_ticks_new, ro->min_travel_ticks);

      // CLIPPING: avoid reduction
      next_ticks = max(next_ticks, curr_ticks);

#ifdef TEST
      printf("decelerate ticks => %d  during %d steps (d_ticks_new = %u)",
             next_ticks, planning_steps, d_ticks_new);
      printf("... %u+%u steps\n", rw->performed_ramp_up_steps, planning_steps);
#endif
      break;
    case RAMP_STATE_DECELERATE_TO_STOP:
      upm_rem_steps = upm_from(remaining_steps - planning_steps);
      upm_d_ticks_new = sqrt(divide(ro->upm_inv_accel2, upm_rem_steps));

      d_ticks_new = upm_to_u32(upm_d_ticks_new);

      // avoid undershoot
      next_ticks = max(d_ticks_new, ro->min_travel_ticks);

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

  // CLIPPING: avoid increase
  next_ticks = min(next_ticks, ABSOLUTE_MAX_TICKS);

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
  steps = min(127, steps);

  switch (next_state & RAMP_STATE_MASK) {
    case RAMP_STATE_COAST:
      break;
    case RAMP_STATE_ACCELERATE:
      rw->performed_ramp_up_steps += steps;
      break;
    case RAMP_STATE_DECELERATE:
    case RAMP_STATE_DECELERATE_TO_STOP:
      rw->performed_ramp_up_steps -= steps;
      break;
  }

#ifdef TEST
  assert(next_ticks > 0);
#endif

  command->ticks = next_ticks;
  command->steps = steps;
  command->count_up = countUp;

  if (steps == abs(remaining_steps)) {
    if (countUp != need_count_up) {
      rw->ramp_state = RAMP_STATE_ACCELERATE | (move_state ^ RAMP_MOVE_MASK);
#ifdef TEST
      puts("Stepper reverse");
#endif
    } else {
      rw->ramp_state = RAMP_STATE_IDLE;
#ifdef TEST
      puts("Stepper stop");
#endif
    }
  }

#ifdef TEST
  printf(
      "add command Steps = %d ticks = %d  Target pos = %d "
      "Remaining steps = %d\n",
      steps, next_ticks, ro->target_pos, remaining_steps);
#endif
}
