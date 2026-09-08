#ifndef FAS_MOVE_TIMED_H
#define FAS_MOVE_TIMED_H

#include <stdint.h>

#include "fas_arch/result_codes.h"

// This header contains the core command generation logic of
// FastAccelStepper::moveTimed(). It is kept separate so it can be unit
// tested on the pc_based platform without any real hardware dependencies.
//
// The test provides its own FastAccelStepper/StepperQueue definitions and
// includes this header to compile the production moveTimed() body verbatim,
// the same pattern as fas_member/fas_add_queue_entry.h.

// Up to two queue entries are consumed by the direction-change machinery in
// FastAccelStepper::addQueueEntry() (a before and an after pause command) in
// addition to the step command it wraps. moveTimed() must therefore treat
// those two slots as reserved for every timed move, so that a move is only
// admitted when the whole move plus the direction pauses fits into the queue.
// Otherwise a direction change could run the queue short between the separate
// pause/step appends and silently drop steps (Issue 370). The application
// should keep the number of queue commands a move generates well below
// QUEUE_LEN/2 so that splitting large moves on the application side stays
// feasible.
inline MoveTimedResultCode FastAccelStepper::moveTimed(
    int16_t steps, uint32_t duration, uint32_t* actual_duration, bool start) {
  MoveTimedResultCode ret_ok =
      isQueueEmpty() ? MOVE_TIMED_EMPTY : MOVE_TIMED_OK;
  if ((steps == 0) && (duration == 0)) {
    if (start) {
      addQueueEntry(NULL, true);  // start the queue
    }
    return ret_ok;
  }
  uint8_t freeEntries = QUEUE_LEN - queueEntries();
  if (freeEntries > 2) {
    freeEntries -= 2;
  } else {
    freeEntries = 0;
  }
  if (actual_duration != NULL) {
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
      if (actual_duration != NULL) {
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
    for (uint8_t steps_i = 0; steps_i < steps; steps_i++) {
      uint32_t this_duration = rate;
      cmd.steps = 1;
      while (this_duration) {
        if (this_duration >= 131072) {
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
        if (actual_duration != NULL) {
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
    }
    AqeResultCode ret = addQueueEntry(&cmd, start);
    if (ret != AQE_OK) {
      // unexpected
      return tmrFrom(ret);
    }
    if (actual_duration != NULL) {
      uint32_t d = cmd.ticks;
      d *= cmd.steps;
      *actual_duration += d;
    }
    steps -= cmd.steps;
  }
  return ret_ok;
}

#endif  // FAS_MOVE_TIMED_H