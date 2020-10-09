#ifndef RAMP_GENERATOR_H
#define RAMP_GENERATOR_H

class FastAccelStepper;

struct ramp_command_s {
  uint32_t ticks;
  uint8_t steps;
  bool count_up;
};

class RampGenerator {
 public:
  struct ramp_config_s {
    uint32_t min_travel_ticks;
    upm_float upm_inv_accel2;
  } _config;
  struct ramp_ro_s {
    int32_t target_pos;
    uint32_t deceleration_start;
    uint32_t min_travel_ticks;
    upm_float upm_inv_accel2;
  } _ro;
  struct ramp_rw_s {
    uint8_t ramp_state;
    uint32_t performed_ramp_up_steps;
  } _rw;
  void init();
  void setSpeed(uint32_t min_step_us);
  void setAcceleration(uint32_t accel);
  int calculate_move(int32_t move, const struct ramp_config_s *config,
                     uint32_t ticks_at_queue_end, bool queue_empty);
  void single_fill_queue(const struct ramp_ro_s *ro, struct ramp_rw_s *rw,
                         uint32_t ticks_at_queue_end,
                         int32_t position_at_queue_end,
                         struct ramp_command_s *command);
};
#endif
