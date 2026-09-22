#ifndef FAS_NAXIS_H
#define FAS_NAXIS_H

#include <stdint.h>

#include "fas_arch/common.h"
#include "fas_naxis/ramp_law.h"
#include "fas_naxis/ramp_map.h"
#include "fas_naxis/remaining.h"

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
  uint32_t accel = 0;  // steps/s^2, read from the stepper at addAxis
};

template <uint8_t NAXES, uint16_t HORIZON = 64,
          typename Stepper = FastAccelStepper>
class FasNAxis {
 public:
  explicit FasNAxis(const FasNAxisConfig& cfg) : _law(1, 1, 0) {
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
      _lim[i].accel = 0;
      _tick_cfg[i] = 0;
      _p[i] = 0;
      _seg[i] = 0;
      _fed[i] = 0;
      _err[i] = 0;
      _dir[i] = true;
    }
    _cfg = c;
    _position_synced = false;
    _block_count = 0;
    _have_seg = false;
    _path_closed = false;
    _feeding = false;
    _done = false;
    _kicked_off = false;
    _underrun = false;
    _master = 0;
    _abs_master = 0;
    _pause_left = 0;
    _P = 0;
    _R = 0;
    _ticks_last = 0;
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
    _lim[i].accel = s->getAcceleration();
    _tick_cfg[i] = _lim[i].ticks_cfg;
    return true;
  }

  // Re-read ticks_cfg / accel from every registered stepper. (P_stop ramp math
  // is Step 6+; the fields are refreshed here.)
  void setLimitsFromSteppers() {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_s[i] != NULL) {
        _lim[i].ticks_cfg = _s[i]->getMaxSpeedInTicks();
        _lim[i].accel = _s[i]->getAcceleration();
        _tick_cfg[i] = _lim[i].ticks_cfg;
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
      _seg[i] = p[i] - _p[i];
      _p[i] = p[i];
    }
    _have_seg = true;
    _feeding = false;
    _done = false;
    _kicked_off = false;
    _underrun = false;
    _pause_left = 0;
    _block_count++;
    return true;
  }

  // Number of motion blocks recorded by addLine (0 after a no-op). Test hook
  // for the "addLine to current position is a no-op" contract.
  uint32_t block_count() const { return _block_count; }

  // Close the path: the last committed point is rest (section 10.3). The
  // one-block Step 6 feeder commits a rest-to-rest segment, so this only
  // records that no further block is expected.
  void endPath() { _path_closed = true; }

  // Plan and feed one committed Linear segment (section 10.4). Prefill every
  // axis with start=false, then kick off with addQueueEntry(NULL, true); later
  // commands use start=true. An empty queue during prefill is expected and is
  // not underrun; after kick-off an empty queue while the plan still moves is
  // underrun (section 10.5).
  PumpStatus pump() {
    if (_have_seg && !_feeding) {
      feeder_start();
      while (!_done && all_have_room()) {
        feed_one();
      }
      bool started = false;
      for (uint8_t i = 0; i < NAXES; i++) {
        if (_registered[i] && !_s[i]->isQueueEmpty()) {
          _s[i]->addQueueEntry(NULL, true);
          started = true;
        }
      }
      _kicked_off = started;
    }
    if (_kicked_off && !_done) {
      for (uint8_t i = 0; i < NAXES; i++) {
        if (_registered[i] && _s[i]->isQueueEmpty()) {
          _underrun = true;
        }
      }
    }
    while (!_done && all_have_room()) {
      feed_one();
    }
    if (_underrun) {
      return PumpStatus::Underrun;
    }
    if (_done && !any_queue_nonempty()) {
      _have_seg = false;
      return PumpStatus::Idle;
    }
    if (!_have_seg && !any_queue_nonempty()) {
      return PumpStatus::Idle;
    }
    return PumpStatus::Running;
  }

  bool isBusy() const {
    if (_have_seg) {
      return true;
    }
    return any_queue_nonempty();
  }

  bool hasUnderrun() const { return _underrun; }

  // Performed ramp-up steps of the current segment's DDA master (section 7.1).
  uint32_t performedRampUp() const { return _P; }
  // Live remaining-to-stop of the master in master steps (section 8.2).
  uint32_t remainingToStop() const { return _R; }
  // Period (ticks) of the last issued step entry.
  uint32_t lastTicks() const { return _ticks_last; }
  // DDA master axis of the current segment (longest |delta|).
  uint8_t masterAxis() const { return (uint8_t)_master; }

 private:
  static int64_t abs_i64(int32_t d) { return d > 0 ? (int64_t)d : -(int64_t)d; }

  bool any_queue_nonempty() const {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_registered[i] && !_s[i]->isQueueEmpty()) {
        return true;
      }
    }
    return false;
  }

  bool all_have_room() const {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_registered[i] && _s[i]->isQueueFull()) {
        return false;
      }
    }
    return true;
  }

  void send_to(uint8_t i, uint16_t ticks, uint8_t steps, bool count_up) {
    struct stepper_command_s cmd = {ticks, steps, count_up};
    _s[i]->addQueueEntry(&cmd, _kicked_off);
  }

  // Initialize the one-block Linear ramp: DDA master is the longest |delta|
  // (Remaining::longest_axis), the period law is RampLaw over master steps at
  // ticks_floor (section 6.3).
  void feeder_start() {
    for (uint8_t i = 0; i < NAXES; i++) {
      _fed[i] = _p[i] - _seg[i];
      _err[i] = 0;
      _dir[i] = true;
    }
    _master = Remaining::longest_axis(_seg, _tick_cfg, NAXES);
    _abs_master = abs_i64(_seg[_master]);
    uint32_t t_law = Remaining::ticks_floor(_seg, _tick_cfg, NAXES);
    if (t_law == 0) {
      t_law = _tick_cfg[_master] != 0 ? _tick_cfg[_master] : 1;
    }
    _law = RampLaw(t_law, _lim[_master].accel, (uint32_t)_abs_master);
    _pause_left = 0;
    _P = 0;
    _R = _law.R;
    _ticks_last = 0;
    _done = (_abs_master == 0);
    _feeding = true;
  }

  // Emit at most one queue entry per registered axis per call, so the axes stay
  // in lockstep. A master step with a period above 65535 is represented the FAS
  // way: a half-period step entry followed by pause entries covering the
  // remainder (sections 4.2 / 9.3).
  void feed_one() {
    if (_done) {
      return;
    }
    if (_pause_left > 0) {
      uint32_t chunk = _pause_left;
      if (chunk > 65535) {
        chunk >>= 1;
        if (chunk > 65535) {
          chunk = 65535;
        }
      }
      for (uint8_t i = 0; i < NAXES; i++) {
        if (_registered[i]) {
          send_to(i, (uint16_t)chunk, 0, _dir[i]);
        }
      }
      _pause_left -= chunk;
      if (_law.done() && _pause_left == 0) {
        _done = true;
      }
      return;
    }

    uint32_t T = _law.step();
    _P = _law.P;
    _R = _law.R;
    _ticks_last = T;

    int32_t st[NAXES];
    for (uint8_t i = 0; i < NAXES; i++) {
      st[i] = 0;
    }
    st[_master] = _seg[_master] > 0 ? 1 : -1;
    for (uint8_t i = 0; i < NAXES; i++) {
      if (i == _master) {
        continue;
      }
      int64_t ad = abs_i64(_seg[i]);
      if (ad == 0) {
        continue;
      }
      _err[i] += ad;
      if (2 * _err[i] >= _abs_master) {
        st[i] = _seg[i] > 0 ? 1 : -1;
        _err[i] -= _abs_master;
      }
    }

    uint32_t t_step = T;
    if (T > 65535) {
      t_step = T >> 1;
      if (t_step > 65535) {
        t_step = 65535;
      }
      _pause_left = T - t_step;
    }
    for (uint8_t i = 0; i < NAXES; i++) {
      if (!_registered[i]) {
        continue;
      }
      uint8_t steps = st[i] != 0 ? 1 : 0;
      bool up = steps != 0 ? (st[i] > 0) : _dir[i];
      if (steps != 0) {
        _dir[i] = up;
      }
      send_to(i, (uint16_t)t_step, steps, up);
      _fed[i] += st[i];
    }
    if (_law.done() && _pause_left == 0) {
      _done = true;
    }
  }

  Stepper* _s[NAXES];
  AxisLimits _lim[NAXES];
  uint32_t _tick_cfg[NAXES];
  int32_t _p[NAXES];
  int32_t _seg[NAXES];
  int32_t _fed[NAXES];
  int64_t _err[NAXES];
  bool _dir[NAXES];
  FasNAxisConfig _cfg;
  bool _registered[NAXES];
  bool _position_synced;
  uint32_t _block_count;
  bool _have_seg;
  bool _path_closed;
  bool _feeding;
  bool _done;
  bool _kicked_off;
  bool _underrun;
  int _master;
  int64_t _abs_master;
  uint32_t _pause_left;
  uint32_t _P;
  uint32_t _R;
  uint32_t _ticks_last;
  RampLaw _law;
};

#endif /* FAS_NAXIS_H */
