#ifndef FAS_NAXIS_RAMP_MAP_H
#define FAS_NAXIS_RAMP_MAP_H

#include "fas_ramp/RampCalculator.h"

// FasNAxis F1 kernel: the single-axis period/step law.
//
// The white paper (section 7.1) says FasNAxis must reuse
// ramp_config_s::calculate_ticks / calculate_ramp_steps and must never
// re-derive v = sqrt(2 a s). This wrapper is a thin forwarder: it builds a
// ramp_config_s from a configured period (ticks_cfg, from getSpeedInTicks)
// and an acceleration (stored as log2_accel via log2_from, as the library
// does), then forwards. Periods are log2_value_t math inside ramp_config_s;
// this header performs no float, double, or integer division on the hot path.
//
// P = 0 is "stopped": FAS starts ramps at P >= 1, so calculate_ticks(0) is
// never called. Callers pass P in [1, P_coast].

class RampMap {
 public:
  // ticks_cfg: configured period in ticks/step. accel: acceleration in
  // step/s^2 (converted to log2 here, exactly as ramp_parameters_s does).
  RampMap(uint32_t ticks_cfg, uint32_t accel) {
    config.init();
    config.parameters.setSpeedInTicks(ticks_cfg);
    config.parameters.setAcceleration(accel);
    config.update();
  }

  // Period (ticks) at ramp position P. P must be >= 1.
  uint32_t calculate_ticks(uint32_t P) const {
    return config.calculate_ticks(P);
  }

  // Inverse: performed ramp steps for a given period.
  uint32_t calculate_ramp_steps(uint32_t ticks) const {
    return config.calculate_ramp_steps(ticks);
  }

  // Coast position P_coast = performed ramp steps at the configured period.
  uint32_t P_coast() const { return config.max_ramp_up_steps; }

  // Configured period (ticks/step).
  uint32_t ticks_cfg() const { return config.parameters.min_travel_ticks; }

  const ramp_config_s& get_config() const { return config; }

 private:
  ramp_config_s config;
};

#endif /* FAS_NAXIS_RAMP_MAP_H */
