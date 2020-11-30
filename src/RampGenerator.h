#ifndef RAMP_GENERATOR_H
#define RAMP_GENERATOR_H

#if defined(TEST)
#define MAX_STEPPER 2
#define TICKS_PER_S 16000000L
#elif defined(ARDUINO_ARCH_AVR)
#define MAX_STEPPER 2
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
  uint32_t ramp_steps;
};
struct ramp_ro_s {
  int32_t target_pos;
  uint32_t min_travel_ticks;
  upm_float upm_inv_accel2;
  bool force_stop;
  bool keep_running;
};
struct ramp_rw_s {
  uint8_t ramp_state;
  // the speed is linked on both ramp slopes to this variable as per
  //       s = v²/2a   =>   v = sqrt(2*a*s)
  uint32_t performed_ramp_up_steps;
};

class RampGenerator {
 public:
  // private:
  // The following variables are configuration input to
  // calculate_move, only
  struct ramp_config_s _config;
  // The ro variables are those, which are only read from single_fill_queue.
  // Reading ro variables is safe.
  // Writing has to be protected with noInterrupts/interrupts-calls and
  // together (!) with relevant rw variables, if needed
 private:
  struct ramp_ro_s _ro;
  struct ramp_rw_s _rw;

 public:
  inline uint8_t rampState() {
    // reading one byte is atomic
    return _rw.ramp_state;
  }
  void init();
  inline int32_t targetPosition() { return _ro.target_pos; }
  void advanceTargetPositionWithinInterruptDisabledScope(int32_t delta) {
    _ro.target_pos += delta;
  }
  void setSpeed(uint32_t min_step_us);
  void setAcceleration(uint32_t accel);

 private:
  void _applySpeedAcceleration(uint32_t ticks_at_queue_end, int32_t target_pos);

 public:
  void applySpeedAcceleration(uint32_t ticks_at_queue_end);
  int8_t move(int32_t move, const struct queue_end_s *queue);
  int8_t moveTo(int32_t position, const struct queue_end_s *queue);
  void initiate_stop() { _ro.force_stop = true; }
  bool isStopping() { return _ro.force_stop && isRampGeneratorActive(); }
  bool isRampGeneratorActive();
  void stopRamp();
  void setKeepRunning() { _ro.keep_running = true; }
  bool isRunningContinuously() { return _ro.keep_running; }
  bool getNextCommand(const struct queue_end_s *queue_end,
                      struct stepper_command_s *command);
  void commandEnqueued(struct stepper_command_s *command);

 private:
  int calculateMoveTo(int32_t target_pos, const struct queue_end_s *queue_end);

 private:
#if (TICKS_PER_S != 16000000L)
  upm_float upm_timer_freq;
#endif
  void update_ramp_steps();
};
#endif
