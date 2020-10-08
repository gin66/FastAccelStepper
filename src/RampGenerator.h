#ifndef RAMP_GENERATOR_H
#define RAMP_GENERATOR_H

class FastAccelStepper;

class RampGenerator {
  public:
	struct ramp_ro_s {
		int32_t target_pos;
		uint32_t deceleration_start;
		uint32_t min_travel_ticks;
		upm_float upm_inv_accel2;
		bool dirHighCountsUp;
	} _ro;
	struct ramp_rw_s {
		uint8_t ramp_state;
		uint32_t performed_ramp_up_steps;
	} _rw;
    void single_fill_queue(const struct ramp_ro_s *ro, struct ramp_rw_s *rw,FastAccelStepper *stepper, uint32_t ticks_at_queue_end,
			int32_t position_at_queue_end);
};
#endif
