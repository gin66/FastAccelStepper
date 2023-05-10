#ifndef RAMP_GENERATOR_H
#define RAMP_GENERATOR_H

#include "FastAccelStepper.h"
#include "RampCalculator.h"
#include "RampConstAcceleration.h"
#include "common.h"

class FastAccelStepper;

#ifdef SUPPORT_PMF_TIMER_FREQ_VARIABLES
extern pmf_logarithmic pmfl_timer_freq;
extern pmf_logarithmic pmfl_timer_freq_div_sqrt_of_2;
extern pmf_logarithmic pmfl_timer_freq_square_div_2;
#endif

class RampGenerator {
 private:
  struct ramp_parameters_s _parameters;

  // Latest configuration for acceleration/speed calculate_move, only
  struct ramp_config_s *_config;

  struct ramp_config_s _config1;
  struct ramp_config_s _config2;

  // The ro variables are those, which are only read from _getNextCommand().
  // The rw variables are only read and written by _getNextCommand() and
  // commandEnqueued() Reading ro variables is safe in application
  struct ramp_ro_s _ro;
  struct ramp_rw_s _rw;

 public:
  uint32_t acceleration;
  inline uint8_t rampState() { return _rw.rampState(); }
  void init();
  inline int32_t targetPosition() { return _ro.targetPosition(); }
  void advanceTargetPosition(int32_t delta, const struct queue_end_s *queue);
  inline void setSpeedInTicks(uint32_t min_step_ticks) {
	struct ramp_config_s *c = get_new_config();
	c->setSpeedInTicks(min_step_ticks);
  }
  inline uint32_t getSpeedInUs() {
    return getSpeedInTicks() / (TICKS_PER_S / 1000000);
  }
  inline uint32_t getSpeedInTicks() { 
	struct ramp_config_s *c = get_new_config();
	return c->min_travel_ticks;
  }
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
  inline uint32_t getSpeedInMilliHz() {
    if (_config->min_travel_ticks == 0) {
      return 0;
    }
    return divForMilliHz(getSpeedInTicks());
  }
  int8_t setAcceleration(int32_t accel);
  inline uint32_t getAcceleration() { return acceleration; }
  void setLinearAcceleration(uint32_t linear_acceleration_steps) {
    struct ramp_config_s *c = get_new_config();
    c->setCubicAccelerationSteps(linear_acceleration_steps);
  }
  int32_t getCurrentAcceleration();
  inline bool hasValidConfig() { 
    struct ramp_config_s *c = get_new_config();
	 return c->checkValidConfig() == MOVE_OK;
  }
  void applySpeedAcceleration();
  int8_t move(int32_t move, const struct queue_end_s *queue);
  int8_t moveTo(int32_t position, const struct queue_end_s *queue);
  int8_t startRun(bool countUp);
  inline void forceStop() { _ro.immediateStop(); }
  inline void initiateStop() { _ro.initiateStop(); }
  inline bool isStopping() {
    return _ro.isStopInitiated() && isRampGeneratorActive();
  }
  inline bool isRampGeneratorActive() { return rampState() != RAMP_STATE_IDLE; }

  void stopRamp();
  inline void setKeepRunning() { _ro.setKeepRunning(); }
  inline bool isRunningContinuously() { return _ro.isRunningContinuously(); }
  void getNextCommand(const struct queue_end_s *queue_end,
                      NextCommand *cmd_out);
  void afterCommandEnqueued(NextCommand *cmd_in);
  uint32_t getCurrentPeriodInTicks() {
    fasDisableInterrupts();
    uint32_t ticks = _rw.curr_ticks;
    fasEnableInterrupts();
    return ticks;
  }
  inline uint32_t getCurrentPeriodInUs() {
    return TICKS_TO_US(getCurrentPeriodInTicks());
  }

 private:
  int8_t _startMove(int32_t target_pos, bool position_changed);
  struct ramp_config_s *get_new_config() {
	// _config points to the last valid config
	struct ramp_config_s *new_config = (_config == &_config1) ? &_config2 : &_config1;
	return new_config;
  }
  void apply_modification() {
	// _config points to the last valid config
	struct ramp_config_s *old_config = _config;
	struct ramp_config_s *new_config = (_config == &_config1) ? &_config2 : &_config1;

	// make new config visible to interrupt
    fasDisableInterrupts();
	new_config->change_cnt = _ro.config.change_cnt + 1;
	_config = new_config;
    fasEnableInterrupts();

	// copy over the configuration
	*old_config = *new_config;
	old_config->recalc_ramp_steps = false;
	old_config->any_change = false;
  }
};

#endif
