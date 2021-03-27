#ifndef RAMP_GENERATOR_H
#define RAMP_GENERATOR_H

#if defined(TEST)
#define MAX_STEPPER 2
#define TICKS_PER_S 16000000L
#elif defined(ARDUINO_ARCH_AVR)
#if defined(__AVR_ATmega328P__)
#define MAX_STEPPER 2
#elif defined(__AVR_ATmega2560__)
#define MAX_STEPPER 3
#endif
#define TICKS_PER_S F_CPU
#elif defined(ARDUINO_ARCH_ESP32)
#define MAX_STEPPER 6
#define TICKS_PER_S 16000000L
#else
#define MAX_STEPPER 6
#define TICKS_PER_S 16000000L
#endif

#include "common.h"

class FastAccelStepper;

#if (TICKS_PER_S == 16000000L)
#define UPM_TICKS_PER_S UPM_CONST_16E6
#define UPM_TICKS_PER_S_DIV_500 UPM_CONST_32000
#define UPM_TICKS_PER_S_DIV_SQRT_OF_2 UPM_CONST_16E6_DIV_SQRT_OF_2
#define UPM_ACCEL_FACTOR UPM_CONST_128E12
#define US_TO_TICKS(u32) (u32 * 16)
#define TICKS_TO_US(u32) (u32 / 16)
#else
#define UPM_TICKS_PER_S upm_timer_freq
#define UPM_TICKS_PER_S_DIV_500 upm_timer_freq_div_500

// This overflows for approx. 1s at 40 MHz, only
#define US_TO_TICKS(u32) \
  ((uint32_t)((((uint32_t)((u32) * (TICKS_PER_S / 10000L))) / 100L)))

// This calculation needs more work
#define TICKS_TO_US(u32) \
  ((uint32_t)((((uint32_t)((u32) / (TICKS_PER_S / 1000000L))) / 1L)))
#endif

struct ramp_config_s {
  uint32_t min_travel_ticks;
  upm_float upm_inv_accel2;
  upm_float upm_sqrt_inv_accel;
  uint8_t accel_change_cnt;
};
struct ramp_ro_s {
  struct ramp_config_s config;
  int32_t target_pos;
  bool force_stop;
  bool keep_running;
  bool keep_running_count_up;
};

struct ramp_rw_s {
  uint8_t ramp_state;
  // if accel_change_cnt does not match config.accel_change_cnt, then
  // performed_ramp_up_steps to be recalculated
  uint8_t accel_change_cnt;
  // the speed is linked on both ramp slopes to this variable as per
  //       s = v²/2a   =>   v = sqrt(2*a*s)
  uint32_t performed_ramp_up_steps;
  // Are the ticks stored of the last previous step, if pulse time requires
  // more than one command
  uint32_t pause_ticks_left;
  // Current ticks for ongoing step
  uint32_t curr_ticks;
};

class NextCommand {
 public:
  struct stepper_command_s command;
  struct ramp_rw_s rw;
};

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
    return ((_config.min_travel_ticks != 0) && (_config.upm_inv_accel2 != 0));
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
    noInterrupts();
    uint32_t ticks = _rw.curr_ticks;
    interrupts();
    return ticks;
  }
  uint32_t getCurrentPeriodInUs() {
    noInterrupts();
    uint32_t ticks = _rw.curr_ticks;
    interrupts();
    return TICKS_TO_US(ticks);
  }

 private:
  int8_t _startMove(int32_t target_pos, int32_t current_target_pos);
#if (TICKS_PER_S != 16000000L)
  upm_float upm_timer_freq;
  upm_float upm_timer_freq_div_500;
#endif
};
#endif
