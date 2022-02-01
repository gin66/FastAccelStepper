#ifndef RAMP_CONST_ACCELERATION_H
#define RAMP_CONST_ACCELERATION_H

struct ramp_config_s {
  uint32_t min_travel_ticks;
  upm_float upm_inv_accel2;
  upm_float upm_sqrt_inv_accel;
  uint8_t accel_change_cnt;

  void init() {
  }
  inline bool hasValidConfig() {
    return ((min_travel_ticks != 0) && (upm_inv_accel2 != 0));
  }
};

struct ramp_ro_s {
  struct ramp_config_s config;
  int32_t target_pos;
  bool force_stop;
  bool keep_running;
  bool keep_running_count_up;
};

struct ramp_rw_s {
  volatile uint8_t ramp_state;
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

void _getNextCommand(const struct ramp_ro_s *ramp,
                            const struct ramp_rw_s *rw,
                            const struct queue_end_s *queue_end,
                            NextCommand *command);
#endif

