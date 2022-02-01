#ifndef RAMP_GENERATOR_H
#define RAMP_GENERATOR_H

#include "common.h"
#include "FastAccelStepper.h"
#include "RampConstAcceleration.h"
#include "RampCalculator.h"

class FastAccelStepper;

class RampGenerator {
 private:
  // Latest configuration for acceleration/speed calculate_move, only
  struct ramp_config_s _config;

  // The ro variables are those, which are only read from _getNextCommand().
  // The rw variables are only read and written by _getNextCommand() and
  // commandEnqueued() Reading ro variables is safe in application
  struct ramp_ro_s _ro;
  struct ramp_rw_s _rw;

 public:
  uint32_t speed_in_ticks;
  uint32_t acceleration;
  inline uint8_t rampState() {
    // reading one byte is atomic
    return _rw.ramp_state;
  }
  void init();
  inline int32_t targetPosition() { return _ro.target_pos; }
  inline void advanceTargetPositionWithinInterruptDisabledScope(int32_t delta) {
    _ro.target_pos += delta;
  }
  int8_t setSpeedInTicks(uint32_t min_step_ticks);
  int8_t setSpeedInUs(uint32_t min_step_us);
  uint32_t getSpeedInUs() { return speed_in_ticks / (TICKS_PER_S / 1000000); }
  uint32_t getSpeedInTicks() { return speed_in_ticks; }
  uint32_t divForMilliHz(uint32_t f) {
    uint32_t base = (uint32_t)250 * TICKS_PER_S;
    uint32_t res = base / f;
    base -= res * f;
    base <<= 2;
    res <<= 2;
    base += f / 2;  // add rounding
    res += base / f;
    return res;
  }
  uint32_t divForHz(uint32_t f) {
    uint32_t base = TICKS_PER_S;
    base += f / 2;  // add rounding
    uint32_t res = base / f;
    return res;
  }
  uint32_t getSpeedInMilliHz() {
    if (speed_in_ticks == 0) {
      return 0;
    }
    return divForMilliHz(speed_in_ticks);
  }
  int8_t setSpeedInMilliHz(uint32_t speed_mhz) {
    if (speed_mhz <= (1000LL * TICKS_PER_S / 0xffffffff + 1)) {
      return -1;
    }
    return setSpeedInTicks(divForMilliHz(speed_mhz));
  }
  int8_t setSpeedInHz(uint32_t speed_hz) {
    if (speed_hz == 0) {
      return -1;
    }
    return setSpeedInTicks(divForHz(speed_hz));
  }
  int8_t setAcceleration(int32_t accel);
  uint32_t getAcceleration() { return acceleration; }
  int32_t getCurrentAcceleration();
  inline bool hasValidConfig() {
	return _config.checkValidConfig() == MOVE_OK;
  }
  void applySpeedAcceleration();
  int8_t move(int32_t move, const struct queue_end_s *queue);
  int8_t moveTo(int32_t position, const struct queue_end_s *queue);
  int8_t startRun(bool countUp);
  inline void initiate_stop() { _ro.force_stop = true; }
  inline bool isStopping() { return _ro.force_stop && isRampGeneratorActive(); }
  bool isRampGeneratorActive();

  void stopRamp();
  inline void setKeepRunning() { _ro.keep_running = true; }
  inline bool isRunningContinuously() { return _ro.keep_running; }
  void getNextCommand(const struct queue_end_s *queue_end,
                      NextCommand *cmd_out);
  void afterCommandEnqueued(NextCommand *cmd_in);
  uint32_t getCurrentPeriodInTicks() {
    fasDisableInterrupts();
    uint32_t ticks = _rw.curr_ticks;
    fasEnableInterrupts();
    return ticks;
  }
  uint32_t getCurrentPeriodInUs() {
    fasDisableInterrupts();
    uint32_t ticks = _rw.curr_ticks;
    fasEnableInterrupts();
    return TICKS_TO_US(ticks);
  }

 private:
  int8_t _startMove(int32_t target_pos, int32_t current_target_pos);
#if (TICKS_PER_S != 16000000L)
  upm_float upm_timer_freq;
  upm_float upm_timer_freq_div_500;
#endif
};

#define fas_abs(x) ((x) >= 0 ? (x) : (-x))

#endif
