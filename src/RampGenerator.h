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
#define UPM_TICKS_PER_S ((upm_float)0x97f4)
#define US_TO_TICKS(u32) (u32 * 16)
#define TICKS_TO_US(u32) (u32 / 16)
#else
#define UPM_TICKS_PER_S upm_timer_freq

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
  uint8_t accel_change_cnt;
};
struct ramp_ro_s {
  struct ramp_config_s config;
  int32_t target_pos;
  bool force_stop;
  bool keep_running;
  bool keep_running_count_up;
  uint8_t ramp_state;
};

struct ramp_rw_s {
  // the speed is linked on both ramp slopes to this variable as per
  //       s = v²/2a   =>   v = sqrt(2*a*s)
  uint32_t performed_ramp_up_steps;
  // if accel_change_cnt does not match config.accel_change_cnt, then
  // performed_ramp_up_steps to be recalculated
  uint8_t accel_change_cnt;
  // Are the ticks stored of the last previous step, if pulse time requires
  // more than one command
  uint32_t pause_ticks_left;
};

class RampGenerator {
 private:
  // This latest configuration for acceleration/speed calculate_move, only
  struct ramp_config_s _config;

  // The ro variables are those, which are only read from _getNextCommand().
  // The rw variables are only read and written by _getNextCommand() and
  // commandEnqueued() Reading ro variables is safe in application
  struct ramp_ro_s _ro;
  struct ramp_rw_s _rw;

 public:
  uint32_t speed_in_us;
  inline uint8_t rampState() {
    // reading one byte is atomic
    return _ro.ramp_state;
  }
  void init();
  inline int32_t targetPosition() { return _ro.target_pos; }
  inline void advanceTargetPositionWithinInterruptDisabledScope(int32_t delta) {
    _ro.target_pos += delta;
  }
  void setSpeed(uint32_t min_step_us);
  void setAcceleration(uint32_t accel);
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
  inline void setState(uint8_t state) { _ro.ramp_state = state; }

  void stopRamp();
  inline void setKeepRunning() { _ro.keep_running = true; }
  inline bool isRunningContinuously() { return _ro.keep_running; }
  uint8_t getNextCommand(const struct queue_end_s *queue_end,
                         struct stepper_command_s *command);
  void commandEnqueued(struct stepper_command_s *command, uint8_t state);

 private:
  int8_t _startMove(int32_t target_pos, const struct queue_end_s *queue_end);
#if (TICKS_PER_S != 16000000L)
  upm_float upm_timer_freq;
#endif
};
#endif
