
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
#if (TICKS_PER_S == 16000000)
#define UPM_TICKS_PER_S ((upm_float)0x97f4)
#else
upm_float upm_timer_freq;
#define UPM_TICKS_PER_S upm_timer_freq
#endif

void RampGenerator::init() {
  _config.min_travel_ticks = 0;
  _config.upm_inv_accel2 = 0;
  _ro.target_pos = 0;
  _rw.ramp_state = RAMP_STATE_IDLE;
}
void RampGenerator::setSpeed(uint32_t min_step_us) {
  _config.min_travel_ticks = min_step_us * (TICKS_PER_S / 1000L) / 1000L;
}
void RampGenerator::setAcceleration(uint32_t accel) {
  uint32_t tmp = TICKS_PER_S / 2;
  upm_float upm_inv_accel = upm_from(tmp / accel);
  _config.upm_inv_accel2 = multiply(UPM_TICKS_PER_S, upm_inv_accel);
}
int RampGenerator::calculate_move(int32_t move,
                                  const struct ramp_config_s *config,
                                  uint32_t ticks_at_queue_end,
                                  bool queue_empty) {
  if (config->min_travel_ticks == 0) {
    return MOVE_ERR_SPEED_IS_UNDEFINED;
  }
  // if (_accel == 0) {
  //  return MOVE_ERR_ACCELERATION_IS_UNDEFINED;
  //}

  uint32_t ramp_steps = upm_to_u32(divide(
      config->upm_inv_accel2, square(upm_from(config->min_travel_ticks))));

  uint32_t steps = abs(move);

  uint32_t curr_ticks = ticks_at_queue_end;
  uint32_t performed_ramp_up_steps;
  uint32_t deceleration_start;
  if ((curr_ticks == TICKS_FOR_STOPPED_MOTOR) || queue_empty) {
    // motor is not running
    //
    // ramp starts with s = 0.
    //
    performed_ramp_up_steps = 0;

    // If the maximum speed cannot be reached due to too less steps,
    // then in the single_fill_queue routine the deceleration will be
    // started after move/2 steps.
    deceleration_start = min(ramp_steps, steps / 2);
  } else if (curr_ticks == config->min_travel_ticks) {
    // motor is running already at coast speed
    //
    performed_ramp_up_steps = ramp_steps;
    deceleration_start = ramp_steps;
  } else {
    // motor is running
    //
    // Calculate on which step on the speed ramp the current speed is related to
    performed_ramp_up_steps = upm_to_u32(
        divide(config->upm_inv_accel2, square(upm_from(curr_ticks))));
    if (curr_ticks >= config->min_travel_ticks) {
      // possibly can speed up
      // Full ramp up/down needs 2*ramp_steps
      // => full ramp is possible, if move+performed_ramp_up_steps >
      // 2*ramp_steps
      deceleration_start =
          min(ramp_steps, (move + performed_ramp_up_steps) / 2);
    } else if (curr_ticks < config->min_travel_ticks) {
      // speed too high, so need to reduce to _min_travel_ticks speed
      deceleration_start = ramp_steps;
    }
  }

  noInterrupts();
  _ro.target_pos += move;
  _ro.deceleration_start = deceleration_start;
  _ro.min_travel_ticks = config->min_travel_ticks;
  _ro.upm_inv_accel2 = config->upm_inv_accel2;
  _rw.performed_ramp_up_steps = performed_ramp_up_steps;
  interrupts();
  inject_fill_interrupt(2);

#ifdef TEST
  printf(
      "Ramp data: steps to move = %d  curr_ticks = %u travel_ticks = %u "
      "Ramp steps = %u Performed ramp steps = %u deceleration start = %u\n",
      steps, curr_ticks, config->min_travel_ticks, ramp_steps,
      performed_ramp_up_steps, deceleration_start);
#endif
#ifdef DEBUG
  char buf[256];
  sprintf(
      buf,
      "Ramp data: steps to move = %ld  curr_ticks = %lu travel_ticks = %lu "
      "Ramp steps = %lu Performed ramp steps = %lu deceleration start = %lu\n",
      steps, curr_ticks, _min_travel_ticks, ramp_steps, performed_ramp_up_steps,
      deceleration_start);
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
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
  // For run time measurement
  uint32_t runtime_us = micros();
#endif

  if (ticks_at_queue_end == 0) {
    ticks_at_queue_end = TICKS_FOR_STOPPED_MOTOR;
  }

  // check state for acceleration/deceleration or deceleration to stop
  uint32_t remaining_steps = abs(ro->target_pos - position_at_queue_end);
  uint32_t planning_steps;
  uint32_t next_ticks;
  if (rw->ramp_state ==
      RAMP_STATE_IDLE) {  // motor is stopped. Set to max value
    planning_steps = 1;
    rw->ramp_state = RAMP_STATE_ACCELERATE;
  } else {
    // TODO:explain the 16000
    planning_steps = max(16000 / ticks_at_queue_end, 1);
    if (remaining_steps <= ro->deceleration_start) {
      rw->ramp_state = RAMP_STATE_DECELERATE_TO_STOP;
    } else if (ro->min_travel_ticks < ticks_at_queue_end) {
      rw->ramp_state = RAMP_STATE_ACCELERATE;
    } else if (ro->min_travel_ticks > ticks_at_queue_end) {
      rw->ramp_state = RAMP_STATE_DECELERATE;
    } else {
      rw->ramp_state = RAMP_STATE_COAST;
    }
  }

  // Forward planning of minimum 10ms or more on slow speed.

#ifdef TEST
  printf(
      "pos@queue_end=%d remaining=%u deceleration_start=%u planning steps=%d  "
      " ",
      position_at_queue_end, remaining_steps, ro->deceleration_start,
      planning_steps);
  switch (rw->ramp_state) {
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
#ifdef TEST
  float v2;
#endif
  switch (rw->ramp_state) {
    uint32_t d_ticks_new;
    uint32_t upm_rem_steps;
    upm_float upm_d_ticks_new;
    case RAMP_STATE_COAST:
      next_ticks = ro->min_travel_ticks;
      // do not overshoot ramp down start
      planning_steps =
          min(planning_steps, remaining_steps - ro->deceleration_start);
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
      printf("accelerate ticks => %d  during %d ticks (d_ticks_new = %u)\n",
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
      printf("decelerate ticks => %d  during %d ticks (d_ticks_new = %u)\n",
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
      printf("decelerate ticks => %d  during %d ticks (d_ticks_new = %u)\n",
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

  if (rw->ramp_state == RAMP_STATE_ACCELERATE) {
    rw->performed_ramp_up_steps += steps;
  }
  if (rw->ramp_state == RAMP_STATE_DECELERATE) {
    rw->performed_ramp_up_steps -= steps;
  }
  bool countUp = (ro->target_pos > position_at_queue_end);

#ifdef TEST
  assert(next_ticks > 0);
#endif

  command->ticks = next_ticks;
  command->steps = steps;
  command->count_up = countUp;

  if (steps == abs(remaining_steps)) {
    rw->ramp_state = RAMP_STATE_IDLE;
#ifdef TEST
    puts("Stepper stop");
#endif
  }

#ifdef TEST
  printf(
      "add command Steps = %d ticks = %d  Target pos = %d "
      "Remaining steps = %d\n",
      steps, next_ticks, ro->target_pos, remaining_steps);
#endif

#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
  // For run time measurement
  runtime_us = micros() - runtime_us;
  max_micros = max(max_micros, runtime_us);
#endif
}
