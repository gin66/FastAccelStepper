#ifndef PD_TEST_DIR_CHANGE_PAUSE_H
#define PD_TEST_DIR_CHANGE_PAUSE_H

// Default implementation of the StepperQueue::addDirChangePauseToQueue() protocol
// method (declared in fas_queue/protocol.h) for the pd_test platform. Real pulse
// drivers (AVR, SAM, SAMD, PICO, ESP32) provide their own, platform specific
// implementation in their <arch>_queue.h and do not use this default.
//
// The before/after delays come from the BEFORE/AFTER_DIR_CHANGE_DELAY_TICKS
// macros in test_queue.h. The pauses are inserted through
// StepperQueue::addQueueEntry() so queue_end / toggle_dir stay on the same path
// as every other command. Only applies to step commands (steps > 0); a pure
// pause command (steps == 0) returns AQE_OK. Returns
// AqeResultCode::DirChangePauseInjected if a pause was inserted, AQE_OK if none
// was needed, or AQE_DIR_PIN_IS_BUSY on a busy queue (checked before enqueuing).
//
// The pd_test target #defines inline away (see fas_arch/test_pc.h), so this
// out-of-line definition would emit a real symbol in every TU that includes it.
// It must therefore be included in exactly one TU per linked binary (the
// StepperISR_test.cpp driver TU for the LIB_O binaries, and test_24.cpp for its
// own self-contained binary), never from a shared header such as test_queue.h.
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
          .count_up = queue_end.count_up};    // delay with old value
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
          .count_up = cmd->count_up};    // delay with new value
    AqeResultCode res = addQueueEntry(&after_cmd, start);
    if (res != AQE_OK) {
      return res;
      }
    pause_injected = true;
    }
  return pause_injected ? AQE_DIR_CHANGE_PAUSE_INJECTED : AQE_OK;
}

#endif    // PD_TEST_DIR_CHANGE_PAUSE_H
