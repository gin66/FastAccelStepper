#ifndef RAMP_GENERATOR_H
#define RAMP_GENERATOR_H

class FastAccelStepper;

struct ramp_command_s {
  uint32_t ticks;
  uint8_t steps;
  bool count_up;
};

#if (F_CPU == 16000000)
#define UPM_TICKS_PER_S ((upm_float)0x97f4)
#else
#define UPM_TICKS_PER_S upm_timer_freq
#endif

class RampGenerator {
 public:
  // The following variables are configuration input to
  // calculate_move, only
  struct ramp_config_s {
    uint32_t min_travel_ticks;
    upm_float upm_inv_accel2;
    uint32_t ramp_steps;
  } _config;
  // The ro variables are those, which are only read from single_fill_queue.
  // Reading ro variables is safe.
  // Writing has to be protected with noInterrupts/interrupts-calls and
  // together (!) with relevant rw variables, if needed
  struct ramp_ro_s {
    int32_t target_pos;
    uint32_t min_travel_ticks;
    upm_float upm_inv_accel2;
    bool force_stop;
  } _ro;
  struct ramp_rw_s {
    uint8_t ramp_state;
    // the speed is linked on both ramp slopes to this variable as per
    //       s = v²/2a   =>   v = sqrt(2*a*s)
    uint32_t performed_ramp_up_steps;
  } _rw;
  void init();
  inline uint8_t rampState() {
    // reading one byte is atomic
    return _rw.ramp_state;
  }
  inline int32_t targetPosition() { return _ro.target_pos; }
  void setSpeed(uint32_t min_step_us);
  void setAcceleration(uint32_t accel);
  void initiate_stop() { _ro.force_stop = true; }
  bool is_stopping() { return _ro.force_stop; }
  int calculate_moveTo(int32_t target_pos, const struct ramp_config_s *config,
                       uint32_t ticks_at_queue_end,
                       int32_t position_at_queue_end);
  void single_fill_queue(const struct ramp_ro_s *ro, struct ramp_rw_s *rw,
                         uint32_t ticks_at_queue_end,
                         int32_t position_at_queue_end,
                         struct ramp_command_s *command);

 private:
#if (F_CPU != 16000000)
  upm_float upm_timer_freq;
#endif
  void update_ramp_steps();
};
#endif
