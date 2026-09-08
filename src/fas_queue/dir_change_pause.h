#ifndef FAS_QUEUE_DIR_CHANGE_PAUSE_H
#define FAS_QUEUE_DIR_CHANGE_PAUSE_H

// Inline default implementation of the StepperQueue::addDirChangePauseToQueue()
// protocol method (declared in fas_queue/protocol.h). Every pulse driver gets
// its implementation from this header, so the driver translation units only
// need to include it.
//
// The before/after delays are taken from the driver's own macros, mirroring
// the legacy addQueueEntry() behavior:
//   - BEFORE_DIR_CHANGE_DELAY_TICKS(q)   driver ticks to pause BEFORE the
//     direction change (undefined/absent on command-synced drivers -> 0)
//   - AFTER_DIR_CHANGE_DELAY_TICKS(q)    minimum ticks of the direction-change
//     command itself (undefined -> 0)
// Buffered drivers (I2S, RMT) define these; command-synced drivers (AVR, SAM,
// SAMD, PICO, MCPWM/PCNT) leave them undefined and get the plain default.
//
// The pauses are inserted through StepperQueue::addQueueEntry() so queue_end /
// toggle_dir stay on the same path as every other command.
//
// StepperQueue must be a complete type at the include site and the driver
// macros must be defined before this header is included. The real platform
// queue headers include this file after their class and macro definitions; the
// pd_test platform includes it from its driver translation unit
// (StepperISR_test.cpp) because the TEST target #defines inline away.
//
// The pause insertion only applies to step commands: for a pure pause command
// (steps == 0) the method returns AQE_OK without enqueuing anything. If one or
// both pauses were inserted, AqeResultCode::DirChangePauseInjected is
// returned; otherwise (no pause needed) the return value is AQE_OK. A busy
// queue is reported as AQE_DIR_PIN_IS_BUSY before anything is enqueued.
inline AqeResultCode StepperQueue::addDirChangePauseToQueue(
    const struct stepper_command_s* cmd, bool start,
    uint16_t dir_change_delay_ticks) {
  if (cmd->steps == 0) {
    return AQE_OK;
  }
  uint16_t before_delay = 0;
#if defined(BEFORE_DIR_CHANGE_DELAY_TICKS)
  before_delay = BEFORE_DIR_CHANGE_DELAY_TICKS(this);
#endif
  uint16_t after_delay = dir_change_delay_ticks;
#if defined(AFTER_DIR_CHANGE_DELAY_TICKS)
  after_delay = fas_max(AFTER_DIR_CHANGE_DELAY_TICKS(this), after_delay);
#endif
#if defined(SUPPORT_PAUSE_CMD_COUNTING)
  if (_nr_of_pauses != 0 && _last_pause_ticks >= before_delay) {
    before_delay = 0;
  }
#endif
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
  bool pause_injected = false;
  if (before_delay > 0) {
    struct stepper_command_s before_cmd = {
        .ticks = (uint16_t)fas_max(before_delay, MIN_CMD_TICKS),
        .steps = 0,
        .count_up = queue_end.count_up};  // delay with old value
    AqeResultCode res = addQueueEntry(&before_cmd, start);
    if (res != AQE_OK) {
      return res;
    }
    pause_injected = true;
  }
  if (after_delay > 0) {
    struct stepper_command_s after_cmd = {
        .ticks = (uint16_t)fas_max(after_delay, MIN_CMD_TICKS),
        .steps = 0,
        .count_up = cmd->count_up};  // delay with new value
    AqeResultCode res = addQueueEntry(&after_cmd, start);
    if (res != AQE_OK) {
      return res;
    }
    pause_injected = true;
  }
  return pause_injected ? AQE_DIR_CHANGE_PAUSE_INJECTED : AQE_OK;
}

#endif  // FAS_QUEUE_DIR_CHANGE_PAUSE_H
