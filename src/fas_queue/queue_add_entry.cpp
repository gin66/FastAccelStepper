#include <stdint.h>

#include "fas_queue/stepper_queue.h"

AqeResultCode StepperQueue::addQueueEntry(const struct stepper_command_s* cmd,
                                          bool start) {
  // Just to check if, if the struct has the correct size
  // if (sizeof(entry) != 6 * QUEUE_LEN) {
  //  return -1;
  //}
  if (!isReadyForCommands()) {
    return AQE_DEVICE_NOT_READY;
  }
  if (cmd == NULL) {
    if (start && !isRunning()) {
      if (next_write_idx == read_idx) {
        return AQE_ERROR_EMPTY_QUEUE_TO_START;
      }
      startQueue();
    }
    return AQE_OK;
  }
  if (isQueueFull()) {
    return AQE_QUEUE_FULL;
  }
  uint16_t period = cmd->ticks;
  uint8_t steps = cmd->steps;

// #define TRACE
#ifdef TRACE
  Serial.print(':');
  Serial.print(start ? "START" : "PUSH");
  Serial.print(':');
  Serial.print(cmd->count_up ? 'U' : 'D');
  Serial.print(':');
  Serial.print(steps);
  Serial.print(':');
  Serial.print(period);
  Serial.print('X');
#endif

  uint32_t command_rate_ticks = period;
  if (steps > 1) {
    command_rate_ticks *= steps;
  }
  if (command_rate_ticks < MIN_CMD_TICKS) {
    return AQE_ERROR_TICKS_TOO_LOW;
  }

  uint8_t wp = next_write_idx;
  struct queue_entry* e = &entry[wp & QUEUE_LEN_MASK];
  bool dir = (cmd->count_up == dirHighCountsUp);
  bool toggle_dir = false;
  bool repeat_entry = false;
#if defined(SUPPORT_RP_PICO)
  if (isQueueEmpty() && !isRunning()) {
    // store the offset from pico sm's step count and position
    pos_offset = queue_end.pos - getCurrentStepCount();
  }
#endif
  if (dirPin != PIN_UNDEFINED) {
    if ((isQueueEmpty() && !isRunning()) &&
        ((dirPin & PIN_EXTERNAL_FLAG) == 0)) {
      // set the dirPin here. Necessary with shared direction pins
      SET_DIRECTION_PIN_STATE(this, dir);
#if defined(AFTER_SET_DIR_PIN_DELAY_US) && AFTER_SET_DIR_PIN_DELAY_US > 0
      delayMicroseconds(AFTER_SET_DIR_PIN_DELAY_US);
#endif
      queue_end.dir = dir;
    } else {
      toggle_dir = (dir != queue_end.dir);
      if (toggle_dir && (dirPin & PIN_EXTERNAL_FLAG)) {
        repeat_entry = toggle_dir;
        toggle_dir = false;
      }
    }
  }

#if defined(BEFORE_DIR_CHANGE_DELAY_TICKS) || \
    defined(AFTER_DIR_CHANGE_DELAY_TICKS)
  if (toggle_dir) {
    uint16_t before_delay = 0;
    uint16_t after_delay = 0;
#if defined(BEFORE_DIR_CHANGE_DELAY_TICKS)
    before_delay = BEFORE_DIR_CHANGE_DELAY_TICKS(this);
#endif
#if defined(AFTER_DIR_CHANGE_DELAY_TICKS)
    after_delay = AFTER_DIR_CHANGE_DELAY_TICKS(this);
#endif
    if (before_delay > 0) {
      // This is a bit tricky. We need to look at the previous command.
      // If the previous command is a pause, then this may be sufficient.
      struct queue_entry* prev = &entry[(wp - 1) & QUEUE_LEN_MASK];
      if (prev->steps == 0) {
        if (prev->ticks >= before_delay) {
          before_delay = 0;
        }
      }
    }
    if (after_delay > 0) {
      if (steps == 0) {
        period = fas_max(after_delay, MIN_CMD_TICKS);
        after_delay = 0;
      }
    }
    uint8_t commands_needed = 1;
    if (before_delay > 0) {
      commands_needed++;
    }
    if (after_delay > 0) {
      commands_needed++;
    }
    if (queueEntries() >= QUEUE_LEN - commands_needed) {
      return AQE_DIR_PIN_IS_BUSY;
    }
    if (before_delay > 0) {
      struct queue_entry* pe = &entry[wp & QUEUE_LEN_MASK];
      pe->steps = 0;
      pe->toggle_dir = 0;
      pe->countUp = cmd->count_up ? 1 : 0;
      pe->moreThanOneStep = 0;
      pe->hasSteps = 0;
      pe->ticks = fas_max(before_delay, MIN_CMD_TICKS);
      pe->repeat_entry = 0;
      pe->dirPinState = dir;
#if defined(SUPPORT_QUEUE_ENTRY_END_POS_U16)
      pe->end_pos_last16 = (uint32_t)queue_end.pos & 0xffff;
#endif
#if defined(SUPPORT_QUEUE_ENTRY_START_POS_U16)
      pe->start_pos_last16 = (uint32_t)queue_end.pos & 0xffff;
#endif
      wp = wp + 1;
    }

    if (after_delay > 0) {
      struct queue_entry* pe = &entry[wp & QUEUE_LEN_MASK];
      pe->steps = 0;
      pe->toggle_dir = 1;
      pe->countUp = cmd->count_up ? 1 : 0;
      pe->moreThanOneStep = 0;
      pe->hasSteps = 0;
      pe->ticks = fas_max(after_delay, MIN_CMD_TICKS);
      pe->repeat_entry = 0;
      pe->dirPinState = dir;
#if defined(SUPPORT_QUEUE_ENTRY_END_POS_U16)
      pe->end_pos_last16 = (uint32_t)queue_end.pos & 0xffff;
#endif
#if defined(SUPPORT_QUEUE_ENTRY_START_POS_U16)
      pe->start_pos_last16 = (uint32_t)queue_end.pos & 0xffff;
#endif
      wp = wp + 1;

      toggle_dir = false;
    }
  }
#endif

  e = &entry[wp & QUEUE_LEN_MASK];
  e->steps = steps;
  e->repeat_entry = repeat_entry;
  e->dirPinState = dir;
  e->toggle_dir = toggle_dir;
  e->countUp = cmd->count_up ? 1 : 0;
  e->moreThanOneStep = steps > 1 ? 1 : 0;
  e->hasSteps = steps > 0 ? 1 : 0;
  e->ticks = period;
  struct queue_end_s next_queue_end = queue_end;
#if defined(SUPPORT_QUEUE_ENTRY_START_POS_U16)
  e->start_pos_last16 = (uint32_t)next_queue_end.pos & 0xffff;
#endif
  next_queue_end.pos = next_queue_end.pos + (cmd->count_up ? steps : -steps);
#if defined(SUPPORT_QUEUE_ENTRY_END_POS_U16)
  e->end_pos_last16 = (uint32_t)next_queue_end.pos & 0xffff;
#endif
  next_queue_end.dir = dir;
  next_queue_end.count_up = cmd->count_up;

  // Advance write pointer (wp may have been incremented for a pause entry)
  fasDisableInterrupts();
  if (!ignore_commands) {
    if (isReadyForCommands()) {
      next_write_idx = wp + 1;
      queue_end = next_queue_end;
    } else {
      fasEnableInterrupts();
      return AQE_DEVICE_NOT_READY;
    }
  }
  fasEnableInterrupts();

  if (!isRunning() && start) {
    // stepper is not yet running and start is requested
#ifdef TRACE
    Serial.print('S');
#endif
    startQueue();
  } else {
#if defined(SUPPORT_RP_PICO)
    // with pio using interrupts, the queue may not be full
    // and then the interrupts are disabled.
    startQueue();
#endif
#ifdef TRACE
    // WHY IS start 0 in seq_01c
    Serial.print(isRunning() ? 'R' : 'T');
    Serial.print(start ? '1' : '0');
    Serial.println('N');
#endif
  }
  return AQE_OK;
}
