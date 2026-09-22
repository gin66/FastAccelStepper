#ifndef FAS_NAXIS_SIM_PORT_H
#define FAS_NAXIS_SIM_PORT_H

#include <stdint.h>

#include "fas_arch/common.h"
#include "fas_arch/result_codes.h"

// SimPort — a duck-typed stand-in for the platform StepperQueue that the
// FasNAxis feeder talks to through addQueueEntry() (whitepaper section 4.1).
//
// It reproduces, WITHOUT a running ISR, the parts of the real addQueueEntry /
// StepperQueue protocol the n-axis feeder depends on:
//    - a ring of pending commands, queue_end tracking the last count_up
//    - the start-flag / kick-off contract (addQueueEntry(NULL, true))
//    - the pd_test "no inject for a pause, inject for a reversing step" rule
//    - a test hook that injects a before-pause (old DIR) then an after-pause
//      (new DIR) on a reversing step, flipping queue_end on the after-pause
//    - drain() that advances position and a simulated clock
//    - isRampGeneratorActive() false unless the test forces it
//
// It is a test helper: it is NOT included from src/FasNAxis.h and never links
// the real pulse drivers.
class SimPort {
 public:
  // Inject hook: how the port reacts to a reversing step command.
  //   InjectNone — default pd_test: a reversing step (or a reverting pause)
  //      enqueues with no injected pause; the DIR is simply the new count_up.
  //   InjectDirPauses — buffered-driver behaviour (RMT/MCPWM/I2S): on a
  //      reversing step the port injects a before-pause with the OLD count_up,
  //      then an after-pause with the NEW count_up (which flips queue_end),
  //      before the step itself enqueues. Each call returns
  //      DirChangePauseInjected and the command is NOT enqueued until the
  //      sequence completes.
  enum InjectMode { InjectNone, InjectDirPauses };

  SimPort(uint16_t max_speed_in_ticks = 80, int queue_len = 64)
      : max_speed_in_ticks_(max_speed_in_ticks),
        accel_(2000),
        queue_len_(queue_len),
        inject_(InjectNone),
        inject_ticks_(8000),
        ramp_generator_active_(false) {
    reset();
  }

  // Reset to a fresh, empty, un-kicked-off port at rest.
  void reset() {
    read_idx_ = 0;
    next_write_idx_ = 0;
    position_ = 0;
    clock_ = 0;
    queue_end_count_up_ = true;
    kicked_off_ = false;
    injected_pause_ticks_ = 0;
    before_done_ = false;
    after_done_ = false;
    fail_rc_ = AQE_OK;
  }

  // --- inject / driver hook configuration ---
  void setInjectMode(InjectMode m) { inject_ = m; }
  void setInjectTicks(uint16_t ticks) { inject_ticks_ = ticks; }

  // One-shot retryable fault for Step 8. The next addQueueEntry(non-NULL)
  // returns `rc` once, enqueues nothing, and leaves isQueueFull() (and
  // queueEntries()) unchanged. Used to prove the FasNAxis feeder holds a
  // command that returned a retryable code and re-sends it on the next pump
  // instead of dropping it or re-planning the partner axis.
  void failNext(AqeResultCode rc) { fail_rc_ = rc; }

  // --- ramp generator idle contract (whitepaper section 4.6) ---
  // The feeder requires the FAS ramp generator to be idle; SimPort models
  // that as a plain flag the test may force.
  bool isRampGeneratorActive() const { return ramp_generator_active_; }
  void setRampGeneratorActive(bool on) { ramp_generator_active_ = on; }

  // --- query surface used by the feeder ---
  // queue_end.count_up: the direction the last enqueued (or, for a reversing
  // step in inject mode, the last injected) command left the port in.
  bool queueEndCountUp() const { return queue_end_count_up_; }
  int32_t position() const { return position_; }
  // Duck-type alias for the real FastAccelStepper::getCurrentPosition(): the
  // FasNAxis feeder / syncFromSteppers() talks to every stepper through one
  // name, so SimPort exposes the same getter the production stepper uses.
  int32_t getCurrentPosition() const { return position_; }
  // Duck-type alias for the real FastAccelStepper::getMaxSpeedInTicks():
  // addAxis / setLimitsFromSteppers read the configured period through one
  // name, so SimPort exposes its max_speed_in_ticks under that name.
  uint16_t getMaxSpeedInTicks() const { return max_speed_in_ticks_; }
  // Duck-type alias for the real FastAccelStepper::getAcceleration(): addAxis
  // reads the per-axis acceleration through the same name (section 6.3).
  uint32_t getAcceleration() const { return accel_; }
  void setAcceleration(uint32_t a) { accel_ = a; }
  uint32_t clock() const { return clock_; }
  // Ticks of the pause the last call injected (0 for AQE_OK). An inject the
  // plan did not carve is a planner failure (whitepaper section 4.4.3).
  uint16_t injectedPauseTicks() const { return injected_pause_ticks_; }
  bool isQueueEmpty() const { return read_idx_ == next_write_idx_; }
  // Pending command count, matching FastAccelStepper::queueEntries().
  uint32_t queueEntries() const {
    return (uint32_t)(next_write_idx_ - read_idx_);
  }
  // Duck-type alias for FastAccelStepper::isQueueFull(). One slot is reserved
  // so a caller that checks this before a coordinated slice always has room
  // for two entries (a pause-stuffed step, section 4.2).
  bool isQueueFull() const {
    return queueEntries() + 1 >= (uint32_t)queue_len_;
  }
  // After a kick-off the queue running with an empty queue while the plan is
  // still open is an underrun; before a kick-off an empty queue is expected
  // (prefill) and is NOT an underrun.
  bool hasUnderrun() const { return kicked_off_ && isQueueEmpty(); }
  // Modelled from FastAccelStepper::isRunning(): a kick-off started the queue
  // and it has not settled to a no-motion state.
  bool isRunning() const { return kicked_off_; }

  // --- the feeder's only entry point (whitepaper section 4.1) ---
  AqeResultCode addQueueEntry(const struct stepper_command_s* cmd, bool start) {
    injected_pause_ticks_ = 0;
    // One-shot injected fault (Step 8): fire once on a command, enqueue
    // nothing, and leave the queue length untouched.
    if (cmd != NULL && fail_rc_ != AQE_OK) {
      AqeResultCode rc = fail_rc_;
      fail_rc_ = AQE_OK;
      return rc;
    }
    // Kick-off: addQueueEntry(NULL, true) starts the queue. On an empty queue
    // it is an error (there is nothing to start).
    if (cmd == NULL) {
      if (isQueueEmpty()) {
        return AQE_ERROR_EMPTY_QUEUE_TO_START;
      }
      kicked_off_ = true;
      return AQE_OK;
    }
    // The max-speed floor is a planner contract for step timing: a step
    // command faster than ticks_cfg is ErrorTicksTooLow. A pause (steps == 0)
    // is a delay, not a step; the real queue only checks MIN_CMD_TICKS on it,
    // so a slow axis's 65535 split may use a shorter pause than max_speed.
    if (cmd->steps > 0 && cmd->ticks < max_speed_in_ticks_) {
      return AQE_ERROR_TICKS_TOO_LOW;
    }
    if (isQueueFull()) {
      return AQE_QUEUE_FULL;
    }

    // In inject mode a reversing step (steps > 0 and a new DIR) drives the
    // buffered-driver before/after pause sequence; the step itself enqueues
    // only once the sequence has completed.
    bool reversing_step =
        cmd->steps > 0 && cmd->count_up != queue_end_count_up_;
    if (inject_ == InjectDirPauses && reversing_step) {
      if (!before_done_) {
        // Before-pause: drain the pipeline, keep the OLD count_up.
        enqueue(inject_ticks_, 0, queue_end_count_up_);
        injected_pause_ticks_ = inject_ticks_;
        before_done_ = true;
        return AQE_DIR_CHANGE_PAUSE_INJECTED;
      }
      if (!after_done_) {
        // After-pause: the toggle command, NEW count_up, flips queue_end.
        enqueue(inject_ticks_, 0, cmd->count_up);
        queue_end_count_up_ = cmd->count_up;
        injected_pause_ticks_ = inject_ticks_;
        after_done_ = true;
        return AQE_DIR_CHANGE_PAUSE_INJECTED;
      }
      // Sequence done: enqueue the reversing step itself.
      before_done_ = false;
      after_done_ = false;
    } else if (reversing_step) {
      // Default pd_test: no injected pause; the step just flips the DIR.
      before_done_ = false;
      after_done_ = false;
    }

    // Pause (steps == 0) or non-reversing step: enqueue as given. A pause
    // uses the count_up the caller passes with no implicit flip, so a
    // reverting pause (count_up = !old) simply leaves the port in the new DIR.
    enqueue(cmd->ticks, cmd->steps, cmd->count_up);
    queue_end_count_up_ = cmd->count_up;
    if (start) {
      kicked_off_ = true;
    }
    return AQE_OK;
  }

  // Consume exactly one pending command (the front of the queue). Returns the
  // tick sum of that command (0 if the queue is empty) and, optionally, its
  // steps and count_up. Used by the FasNAxis feeder tests to drain the axes in
  // lockstep, one coordinated command at a time.
  uint32_t drain_one(int64_t* steps_out = NULL, bool* count_up_out = NULL) {
    if (read_idx_ == next_write_idx_) {
      return 0;
    }
    struct queue_entry& e = entry_[read_idx_ & queue_mask()];
    uint32_t ticks = e.steps == 0 ? e.ticks : (uint32_t)e.steps * e.ticks;
    if (e.steps != 0) {
      position_ += e.count_up ? e.steps : -(int32_t)e.steps;
    }
    clock_ += ticks;
    if (steps_out != NULL) {
      *steps_out = e.steps;
    }
    if (count_up_out != NULL) {
      *count_up_out = e.count_up;
    }
    read_idx_++;
    return ticks;
  }

  // Consume the whole pending queue: advance position (signed by count_up) and
  // the simulated clock (pause ticks + steps*ticks for step commands).
  uint32_t drain() { return drain_all(); }

  // Consume pending commands up to a tick budget. Returns the number of ticks
  // drained from this call. When a command straddles the budget the remainder
  // is left in place and the call returns.
  uint32_t drain(uint32_t budget) {
    uint32_t drained = 0;
    while (read_idx_ != next_write_idx_ && drained < budget) {
      struct queue_entry& e = entry_[read_idx_ & queue_mask()];
      uint32_t ticks = e.steps == 0 ? e.ticks : (uint32_t)e.steps * e.ticks;
      if (e.steps != 0) {
        position_ += e.count_up ? e.steps : -(int32_t)e.steps;
      }
      if (drained + ticks > budget) {
        uint32_t room = budget - drained;
        if (room == 0) {
          break;
        }
        clock_ += room;
        drained += room;
        e.ticks = (uint16_t)room;  // leave the remainder in the command
        return drained;
      }
      clock_ += ticks;
      drained += ticks;
      read_idx_++;
    }
    return drained;
  }

 private:
  struct queue_entry {
    uint16_t ticks;
    uint8_t steps;
    bool count_up;
  };

  uint16_t max_speed_in_ticks_;
  uint32_t accel_;
  int queue_len_;
  InjectMode inject_;
  uint16_t inject_ticks_;
  bool ramp_generator_active_;

  queue_entry entry_[64];
  int read_idx_;
  int next_write_idx_;
  int32_t position_;
  uint32_t clock_;
  bool queue_end_count_up_;
  bool kicked_off_;
  uint16_t injected_pause_ticks_;
  bool before_done_;
  bool after_done_;
  AqeResultCode fail_rc_;

  inline uint32_t queue_mask() const { return (uint32_t)queue_len_ - 1; }

  // Drain the whole pending queue (see drain()).
  uint32_t drain_all() {
    uint32_t drained = 0;
    while (read_idx_ != next_write_idx_) {
      struct queue_entry& e = entry_[read_idx_ & queue_mask()];
      uint32_t ticks = e.steps == 0 ? e.ticks : (uint32_t)e.steps * e.ticks;
      if (e.steps != 0) {
        position_ += e.count_up ? e.steps : -(int32_t)e.steps;
      }
      clock_ += ticks;
      drained += ticks;
      read_idx_++;
    }
    return drained;
  }

  // Append one entry to the ring. The port never fills the ring in these
  // small contract tests; a full ring would surface as a retryable Busy.
  void enqueue(uint16_t ticks, uint8_t steps, bool count_up) {
    entry_[next_write_idx_ & queue_mask()] =
        queue_entry{ticks, steps, count_up};
    next_write_idx_++;
  }
};

#endif  // FAS_NAXIS_SIM_PORT_H
