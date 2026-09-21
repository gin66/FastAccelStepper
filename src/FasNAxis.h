#ifndef FAS_NAXIS_H
#define FAS_NAXIS_H

#include <stdint.h>

// The default stepper type. Only its name is needed to parse the template
// (tests supply SimPort; production supplies the real FastAccelStepper from
// <FastAccelStepper.h>, which this header deliberately does not include).
class FastAccelStepper;

// FasNAxis — a multi-axis planner that drives N FastAccelStepper queues from
// one polyline so the axes stay time-synchronized (whitepaper
// extras/doc/n_axes_whitepaper.md). Header-only: no new src/*.cpp, and it is
// NOT included from FastAccelStepper.h (the stepper has no dependency on the
// planner). The hot path performs no float, double, or integer division; ramp
// math lives in fas_naxis/ramp_map.h (log2_value_t) and RampCalculator.
//
// Step 5 (this file) is the skeleton: config, pump status, axis registration,
// and current position. No motion is planned yet (that is Step 6+); the block
// buffer exists but addLine only records the current position and the length
// of a committed segment.

// PumpStatus is the result of a pump() tick. Deliberately NO LookaheadTooShort:
// a short lookahead slows the track (speed cap, G4/F11/F19) instead of
// erroring.
enum class PumpStatus : int {
  Idle = 0,      // no block pending, queue settled
  Running = 1,   // a block is being planned / fed
  Underrun = 2,  // queue ran dry after a kick-off (F13)
  Error = 3      // a contract violation the caller must fix
};

// Configuration. Default member initializers make FasNAxisConfig{} a valid
// Linear config. The constructor recovers 0 to the default so a raw zeroed
// struct still means "default slice / default threshold", not a zero-duration
// slice or a zero diagnostic threshold.
struct FasNAxisConfig {
  enum Mode {
    Linear,    // snaps to every vertex; path speed 0 at a non-collinear vertex
    Overshoot  // may leave the chord within overshoot_max (Step 11+)
  };

  uint32_t dt_ticks = 32000;      // 2 ms at 16 MHz; 0 means this default
  uint16_t kappa_stop_q8 = 320;   // 1.25 in Q8; 0 means this default
  uint16_t overshoot_max = 8;     // steps; ignored in Linear
  uint16_t dir_before_ticks = 0;  // 0 = use stepper getDirChangeBeforeTicks()
  uint16_t dir_after_ticks = 0;   // 0 = use stepper getDirChangeAfterTicks()
  Mode mode = Linear;
};

// Per-axis frozen limits, read from the stepper at addAxis (and on
// setLimitsFromSteppers). ticks_cfg is the configured period; P_stop is the
// performed ramp-up steps (calculate_ramp_steps(ticks_cfg)). No ramp math is
// run here in Step 5; the fields exist for the planner in Step 6+.
struct AxisLimits {
  uint32_t ticks_cfg = 0;
  uint32_t P_stop = 0;
};

template <uint8_t NAXES, uint16_t HORIZON = 64,
          typename Stepper = FastAccelStepper>
class FasNAxis {
 public:
  explicit FasNAxis(const FasNAxisConfig& cfg) {
    FasNAxisConfig c = cfg;  // copy so the default recovery is observable
    if (c.dt_ticks == 0) {
      c.dt_ticks = 32000;
    }
    if (c.kappa_stop_q8 == 0) {
      c.kappa_stop_q8 = 320;
    }
    for (uint8_t i = 0; i < NAXES; i++) {
      _s[i] = NULL;
      _registered[i] = false;
      _lim[i].ticks_cfg = 0;
      _lim[i].P_stop = 0;
      _p[i] = 0;
    }
    _cfg = c;
    _position_synced = false;
    _block_count = 0;
  }

  // Read-back of the recovered config defaults.
  uint32_t dt_ticks() const { return _cfg.dt_ticks; }
  uint16_t kappa_stop_q8() const { return _cfg.kappa_stop_q8; }

  // Register axis i to stepper s. Fails (returns false, no state change) when
  // i is out of range, the pointer is null, or the stepper's ramp generator is
  // active or the stepper is running — the feeder must never race a prior
  // moveTo / manageSteppers. A small HORIZON relative to P_stop is NOT a
  // failure: HORIZON only caps the planned ramp later (F19).
  bool addAxis(uint8_t i, Stepper* s) {
    if (i >= NAXES) {
      return false;
    }
    if (s == NULL) {
      return false;
    }
    if (s->isRampGeneratorActive() || s->isRunning()) {
      return false;
    }
    _s[i] = s;
    _registered[i] = true;
    _lim[i].ticks_cfg = s->getMaxSpeedInTicks();
    return true;
  }

  // Re-read ticks_cfg from every registered stepper. (P_stop ramp math is
  // Step 6+; the field is refreshed here.)
  void setLimitsFromSteppers() {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_s[i] != NULL) {
        _lim[i].ticks_cfg = _s[i]->getMaxSpeedInTicks();
      }
    }
  }

  // Set the current position from the steppers and open addLine.
  void syncFromSteppers() {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_s[i] != NULL) {
        _p[i] = _s[i]->getCurrentPosition();
      }
    }
    _position_synced = true;
  }

  // Set the current position from a caller-supplied array and open addLine.
  void setCurrentPosition(const int32_t p[NAXES]) {
    for (uint8_t i = 0; i < NAXES; i++) {
      _p[i] = p[i];
    }
    _position_synced = true;
  }

  // Queue an absolute target position (steps). Illegal (returns false) before
  // the position is synced. A target equal to the current position (every
  // delta 0) is a dwell of 0 ticks: a no-op that records no block. Returns
  // true when the call is accepted (including the no-op case).
  bool addLine(const int32_t p[NAXES]) {
    if (!_position_synced) {
      return false;
    }
    bool any = false;
    for (uint8_t i = 0; i < NAXES; i++) {
      if (p[i] != _p[i]) {
        any = true;
      }
    }
    if (!any) {
      return true;  // L = 0: dwell of 0 ticks, no block recorded
    }
    for (uint8_t i = 0; i < NAXES; i++) {
      _p[i] = p[i];
    }
    _block_count++;
    return true;
  }

  // Number of motion blocks recorded by addLine (0 after a no-op). Test hook
  // for the "addLine to current position is a no-op" contract.
  uint32_t block_count() const { return _block_count; }

 private:
  Stepper* _s[NAXES];
  AxisLimits _lim[NAXES];
  int32_t _p[NAXES];
  FasNAxisConfig _cfg;
  bool _registered[NAXES];
  bool _position_synced;
  uint32_t _block_count;
};

#endif /* FAS_NAXIS_H */
