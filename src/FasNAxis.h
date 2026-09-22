#ifndef FAS_NAXIS_H
#define FAS_NAXIS_H

#include <stdint.h>

#include "fas_arch/common.h"
#include "fas_naxis/overshoot.h"
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
// Step 7 is the Linear lookahead planner: addLine() commits points into a block
// ring of up to HORIZON n-dim points, and pump() feeds the committed Linear
// path through addQueueEntry(). The DDA master is the longest |delta| of the
// current block; R is Remaining-style remaining master steps to the next Linear
// path-stop (section 8.1 + 8.5). A non-collinear vertex resets P; a collinear
// joint carries it. The last buffered point of an open path is rest, so the
// ramp always stops there.
//
// Step 8 makes the feeder fault-tolerant: feed_one() stores a held command per
// axis (one slice) and flush_held() sends it, retrying on a retryable
// addQueueEntry result on the next pump() without re-planning the slice, and
// reserving QUEUE_LEN - 2 slots so a pause-stuffed entry always fits.

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
// performed ramp-up steps (calculate_ramp_steps(ticks_cfg)).
struct AxisLimits {
  uint32_t ticks_cfg = 0;
  uint32_t P_stop = 0;
  uint32_t accel = 0;  // steps/s^2, read from the stepper at addAxis
  // DIR pause budget (whitepaper section 4.4), read from the stepper:
  // n_before entries of dir_before ticks (old DIR), then dir_after (new DIR).
  uint16_t dir_before = 0;
  uint8_t dir_n_before = 0;
  uint16_t dir_after = 0;
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
      _lim[i].dir_before = 0;
      _lim[i].dir_n_before = 0;
      _lim[i].dir_after = 0;
      _tick_cfg[i] = 0;
      _p[i] = 0;
      _dir[i] = true;
      _err[i] = 0;
      _held[i].waiting = false;
      _held[i].ticks = 0;
      _held[i].steps = 0;
      _held[i].count_up = true;
      _carve_axis[i].active = false;
      _carve_axis[i].phase = 0;
      _carve_axis[i].step_left = 0;
      _carve_axis[i].n_before = 0;
      _carve_axis[i].before = 0;
      _carve_axis[i].after = 0;
      _carve_axis[i].old_up = true;
      _carve_axis[i].new_up = true;
      for (uint16_t b = 0; b < HORIZON; b++) {
        _blk[b][i] = 0;
      }
    }
    _cfg = c;
    _position_synced = false;
    _block_count = 0;
    _n_blk = 0;
    _head = 0;
    _path_closed = false;
    _feeding = false;
    _done = true;
    _kicked_off = false;
    _underrun = false;
    _slice_open = false;
    _error = false;
    _carve_then_advance = false;
    _master = 0;
    _abs_master = 0;
    _block_left = 0;
    _pause_left = 0;
    _ticks_law = 1;
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
    read_dir_budget(i);
    _tick_cfg[i] = _lim[i].ticks_cfg;
    RampMap map(_lim[i].ticks_cfg, _lim[i].accel);
    _lim[i].P_stop = map.P_coast();
    return true;
  }

  // Re-read ticks_cfg / accel from every registered stepper and refresh P_stop.
  void setLimitsFromSteppers() {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_s[i] != NULL) {
        _lim[i].ticks_cfg = _s[i]->getMaxSpeedInTicks();
        _lim[i].accel = _s[i]->getAcceleration();
        read_dir_budget(i);
        _tick_cfg[i] = _lim[i].ticks_cfg;
        RampMap map(_lim[i].ticks_cfg, _lim[i].accel);
        _lim[i].P_stop = map.P_coast();
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
    clear_path();
    _position_synced = true;
  }

  // Set the current position from a caller-supplied array and open addLine.
  void setCurrentPosition(const int32_t p[NAXES]) {
    for (uint8_t i = 0; i < NAXES; i++) {
      _p[i] = p[i];
    }
    clear_path();
    _position_synced = true;
  }

  // Queue an absolute target position (steps) by appending its delta to the
  // block ring. Illegal (returns false) before the position is synced or when
  // the ring is full (HORIZON points committed but not yet executed). A target
  // equal to the current position (every delta 0) is a no-op that records no
  // block and returns true. Appending after the plan already caught up with
  // the buffer re-opens the plan (R may grow; section 8.7).
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
    if (_n_blk >= (int)HORIZON) {
      return false;  // ring full: backpressure until pump() drains
    }
    for (uint8_t i = 0; i < NAXES; i++) {
      _blk[_n_blk][i] = p[i] - _p[i];
      _p[i] = p[i];
    }
    _n_blk++;
    _block_count++;
    if (_done) {
      // A plan that had caught up to the buffer now has more path: feed the
      // unexecuted tail on the next pump() (replan, section 8.7).
      _done = false;
    }
    return true;
  }

  // Number of motion blocks recorded by addLine (0 after a no-op / a sync).
  uint32_t block_count() const { return _block_count; }

  // Close the path: the last committed point is rest and R will not grow
  // (section 10.3).
  void endPath() { _path_closed = true; }

  // Plan and feed the committed Linear path (section 10.4). Prefill every axis
  // with start=false, then kick off with addQueueEntry(NULL, true); later
  // commands use start=true. An empty queue during prefill is expected and is
  // not underrun; after kick-off an empty queue while the plan still moves is
  // underrun (section 10.5).
  PumpStatus pump() {
    if (!_feeding && _head < _n_blk) {
      feeder_start();
      feed_loop();
      if (!_error) {
        bool started = false;
        for (uint8_t i = 0; i < NAXES; i++) {
          if (_registered[i] && !_s[i]->isQueueEmpty()) {
            _s[i]->addQueueEntry(NULL, true);
            started = true;
          }
        }
        _kicked_off = started;
      }
    }
    if (!_error && _kicked_off && !_done) {
      for (uint8_t i = 0; i < NAXES; i++) {
        if (_registered[i] && _s[i]->isQueueEmpty()) {
          _underrun = true;
        }
      }
    }
    feed_loop();
    if (_error) {
      return PumpStatus::Error;
    }
    if (_underrun) {
      return PumpStatus::Underrun;
    }
    if (_done && !any_queue_nonempty()) {
      _feeding = false;
      _n_blk = 0;
      _head = 0;
      _path_closed = false;
      _slice_open = false;
      return PumpStatus::Idle;
    }
    if (!_feeding && !any_queue_nonempty()) {
      return PumpStatus::Idle;
    }
    return PumpStatus::Running;
  }

  bool isBusy() const {
    if (_head < _n_blk || !_done) {
      return true;
    }
    return any_queue_nonempty();
  }

  bool hasUnderrun() const { return _underrun; }

  // Performed ramp-up steps of the current segment's DDA master (section 7.1).
  uint32_t performedRampUp() const { return _P; }
  // Live remaining-to-stop of the master in path steps (section 8.2).
  uint32_t remainingToStop() const { return _R; }
  // Period (ticks) of the last issued step entry.
  uint32_t lastTicks() const { return _ticks_last; }
  // DDA master axis of the current block (longest |delta|).
  uint8_t masterAxis() const { return (uint8_t)_master; }
  // Number of blocks committed but not yet fully executed.
  int pendingBlocks() const { return _n_blk - _head; }

 private:
  static int64_t abs_i64(int32_t d) { return d > 0 ? (int64_t)d : -(int64_t)d; }

  static uint32_t abs_u32(int32_t d) {
    return d > 0 ? (uint32_t)d : (uint32_t)(-(int64_t)d);
  }

  bool any_queue_nonempty() const {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_registered[i] && !_s[i]->isQueueEmpty()) {
        return true;
      }
    }
    return false;
  }

  // Reserve two slots on every queue (QUEUE_LEN - 2) so a coordinated slice --
  // and the pause-stuffed entry a long period may split into -- always fits.
  // isQueueFull() alone allows QUEUE_LEN - 1 and cannot express this reserve.
  bool all_have_room() const {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_registered[i] && _s[i]->queueEntries() + 2 >= (uint32_t)QUEUE_LEN) {
        return false;
      }
    }
    return true;
  }

  // Read the DIR pause budget from the stepper (whitepaper section 4.4). The
  // config override is applied at use time in reverse_budget(), not here.
  void read_dir_budget(uint8_t i) {
    _lim[i].dir_before = _s[i]->getDirChangeBeforeTicks();
    _lim[i].dir_n_before = _s[i]->getDirChangeBeforePauseCount();
    _lim[i].dir_after = _s[i]->getDirChangeAfterTicks();
  }

  // Resolve the reverse budget for axis i: config override when non-zero, else
  // the stepper getter. n_before defaults to 1 when the config supplies a
  // before period but the getter count is 0.
  void reverse_budget(uint8_t i, uint16_t* before, uint8_t* n_before,
                      uint16_t* after) const {
    if (_cfg.dir_before_ticks != 0) {
      *before = _cfg.dir_before_ticks;
      *n_before = _lim[i].dir_n_before != 0 ? _lim[i].dir_n_before : 1;
    } else {
      *before = _lim[i].dir_before;
      *n_before = _lim[i].dir_n_before;
    }
    *after =
        _cfg.dir_after_ticks != 0 ? _cfg.dir_after_ticks : _lim[i].dir_after;
  }

  uint32_t reverse_tau(uint8_t i) const {
    uint16_t before = 0;
    uint8_t n = 0;
    uint16_t after = 0;
    reverse_budget(i, &before, &n, &after);
    return (uint32_t)n * before + (uint32_t)after;
  }

  // Axis i reverses between block b and the next buffered block.
  bool reverses_at_end(int b, uint8_t i) const {
    if (b + 1 >= _n_blk) {
      return false;
    }
    int32_t cur = _blk[b][i];
    int32_t nxt = _blk[b + 1][i];
    if (cur == 0 || nxt == 0) {
      return false;
    }
    return (cur > 0) != (nxt > 0);
  }

  // One axis's pending DIR carve: a shortened last step (old DIR) plus n_before
  // before-pauses (old DIR) plus one after-pause (new DIR), tick sum unchanged.
  struct Carve {
    bool active;
    uint8_t phase;       // 0 shortened step, 1 before-pauses, 2 after-pause
    uint32_t step_left;  // remaining wait of the shortened step
    uint8_t n_before;    // remaining before-pauses
    uint16_t before;
    uint16_t after;
    bool old_up;
    bool new_up;
  };

  bool any_carve() const {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_carve_axis[i].active) {
        return true;
      }
    }
    return false;
  }

  // Arm the carve on axis i for the last old-direction step of period T. The
  // shortened step is T - tau when that stays a legal slow step; otherwise the
  // tail was too short and the step is held at the floor (the slice grows).
  void start_carve(uint8_t i, uint32_t T, bool new_up) {
    uint16_t before = 0;
    uint8_t n_before = 0;
    uint16_t after = 0;
    reverse_budget(i, &before, &n_before, &after);
    uint32_t tau = (uint32_t)n_before * before + (uint32_t)after;
    if (tau == 0) {
      return;
    }
    uint32_t floor = _tick_cfg[i] > (uint32_t)MIN_CMD_TICKS
                         ? _tick_cfg[i]
                         : (uint32_t)MIN_CMD_TICKS;
    uint32_t reduced;
    if (T > tau && T - tau >= floor) {
      reduced = T - tau;
    } else {
      reduced = floor;  // short tail: the carve lengthens the slice
    }
    Carve& c = _carve_axis[i];
    c.active = true;
    c.phase = 0;
    c.step_left = reduced;
    c.n_before = n_before;
    c.before = before;
    c.after = after;
    c.old_up = _dir[i];
    c.new_up = new_up;
  }

  // Emit one command of axis i's carve (at most one per call). `held` reports
  // whether a command was placed in the slice.
  void carve_emit(uint8_t i, bool* held) {
    Carve& c = _carve_axis[i];
    if (!c.active) {
      return;
    }
    if (c.phase == 0) {
      uint32_t left = c.step_left;
      uint16_t t;
      if (left > 65535) {
        uint32_t half = left >> 1;
        if (half > 65535) {
          half = 65535;
        }
        if (half < _ticks_law) {
          half = _ticks_law;
        }
        if (half > 65535) {
          half = 65535;
        }
        t = (uint16_t)half;
      } else {
        t = (uint16_t)left;
      }
      hold(i, t, 1, c.old_up);
      c.step_left -= t;
      if (c.step_left == 0) {
        c.phase = 1;
      }
      *held = true;
      return;
    }
    if (c.phase == 1) {
      if (c.n_before > 0) {
        hold(i, c.before, 0, c.old_up);
        c.n_before--;
        *held = true;
        return;
      }
      c.phase = 2;
    }
    if (c.phase == 2) {
      if (c.after > 0) {
        hold(i, c.after, 0, c.new_up);
        c.after = 0;
        *held = true;
        return;
      }
      c.phase = 3;
    }
    if (c.phase >= 3) {
      _dir[i] = c.new_up;
      c.active = false;
    }
  }

  // Drive every active carve one command per pass until at least one command is
  // held. Only the carving axes are sent; every other axis keeps the command it
  // was already given for those ticks.
  void feed_carve() {
    bool held = false;
    for (uint8_t guard = 0; guard < 8; guard++) {
      bool any = false;
      for (uint8_t i = 0; i < NAXES; i++) {
        if (_carve_axis[i].active) {
          any = true;
          carve_emit(i, &held);
        }
      }
      if (!any || held) {
        break;
      }
    }
    if (!any_carve() && _carve_then_advance) {
      _carve_then_advance = false;
      advance_block();
    }
    _slice_open = held;
  }

  void clear_path() {
    _n_blk = 0;
    _head = 0;
    _path_closed = false;
    _feeding = false;
    _done = true;
    _kicked_off = false;
    _underrun = false;
    _block_count = 0;
    _block_left = 0;
    _pause_left = 0;
    _P = 0;
    _R = 0;
    _ticks_last = 0;
    _slice_open = false;
    _error = false;
    _carve_then_advance = false;
    for (uint8_t i = 0; i < NAXES; i++) {
      _held[i].waiting = false;
      _carve_axis[i].active = false;
    }
  }

  // Store one axis's slot of the slice currently being emitted (Step 8). The
  // command is not sent until flush_held(); that lets the paired axes re-send a
  // held command without re-planning the slice.
  void hold(uint8_t i, uint16_t ticks, uint8_t steps, bool count_up) {
    _held[i].ticks = ticks;
    _held[i].steps = steps;
    _held[i].count_up = count_up;
    _held[i].waiting = true;
  }

  bool any_waiting() const {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_registered[i] && _held[i].waiting) {
        return true;
      }
    }
    return false;
  }

  // Send the held slice to every axis still waiting. AQE_OK clears waiting; a
  // retryable code (QueueFull / DirPinIsBusy / WaitForEnablePinActive /
  // DeviceNotReady) leaves it set so the next pump() re-sends the same command.
  // The error codes are terminal: set _error and report it.
  AqeResultCode flush_held() {
    AqeResultCode retry = AqeResultCode::OK;
    for (uint8_t i = 0; i < NAXES; i++) {
      if (!_registered[i] || !_held[i].waiting) {
        continue;
      }
      struct stepper_command_s cmd = {_held[i].ticks, _held[i].steps,
                                      _held[i].count_up};
      AqeResultCode rc = _s[i]->addQueueEntry(&cmd, _kicked_off);
      if (rc == AqeResultCode::OK) {
        _held[i].waiting = false;
      } else if (aqeIsPauseInjected(rc)) {
        // Injected DIR pause: planner failure (whitepaper §4.4.3).
        _error = true;
      } else if (aqeRetry(rc)) {
        retry = rc;
      } else {
        _error = true;  // TicksTooLow and any other terminal code
      }
    }
    if (!any_waiting()) {
      _slice_open = false;
    }
    return retry;
  }

  // Emit slices while there is room and the path is live. Building is one
  // feed_one() per open slice; flushing is one flush_held() per slice. A
  // retryable flush stops this pump() call (the held slice is retried on the
  // next call) so a single pump never plans past an unaccepted command.
  void feed_loop() {
    while (!_error) {
      if (!_slice_open) {
        if (_done || !all_have_room()) {
          break;
        }
        feed_one();
        if (!_slice_open) {
          break;
        }
      }
      AqeResultCode rc = flush_held();
      if (rc != AqeResultCode::OK || _done) {
        break;
      }
    }
  }

  // Section 8.5 collinear, same sense between two full path directions.
  //   (dot(d, d'))^2 * 100000  >=  99878 * |d|^2 * |d'|^2
  //   (cos^2(2deg) ~= 0.99878). Integer mul/compare, no division, no sqrt.
  bool collinear_same_sense(int a, int b) const {
    int64_t dot = 0;
    int64_t mag_a = 0;
    int64_t mag_b = 0;
    for (uint8_t i = 0; i < NAXES; i++) {
      int64_t da = _blk[a][i];
      int64_t db = _blk[b][i];
      dot += da * db;
      mag_a += da * da;
      mag_b += db * db;
    }
    if (dot <= 0 || mag_a == 0 || mag_b == 0) {
      return false;  // opposite sense or a zero vector
    }
    return dot * dot * 100000 >= 99878 * mag_a * mag_b;
  }

  // Remaining master steps from `head` to the next Linear path-stop: end of
  // the buffer, or the first non-collinear vertex (sections 8.1 / 8.5). P and
  // R live in these path-step units so a collinear run may rebind the DDA
  // master without changing the ramp-step currency.
  uint32_t remaining_path_steps(int head) const {
    uint32_t s = 0;
    int started = 0;
    for (int b = head; b < _n_blk; b++) {
      if (started && !collinear_same_sense(b - 1, b)) {
        break;
      }
      int m = Remaining::longest_axis(_blk[b], _tick_cfg, NAXES);
      s += abs_u32(_blk[b][m]);
      started = 1;
    }
    return s;
  }

  // Set up the ramp law and DDA state for block `b`. `reset_P` is false at a
  // collinear joint (P carries over); R is recomputed to the next path-stop.
  void start_block(int b, bool reset_P) {
    _head = b;
    _master = Remaining::longest_axis(_blk[b], _tick_cfg, NAXES);
    uint32_t t_law = Remaining::ticks_floor(_blk[b], _tick_cfg, NAXES);
    if (t_law == 0) {
      t_law = _tick_cfg[_master] != 0 ? _tick_cfg[_master] : 1;
    }
    _ticks_law = t_law;
    int binder = Remaining::binder_axis(_blk[b], _tick_cfg, NAXES);
    uint32_t accel = _lim[binder].accel;
    // If this block ends at a reversal with a DIR budget, the approach must
    // reach a slow enough last period (§4.4.1). Cap the acceleration so the
    // first-step period calculate_ticks(1) holds tau + the legal step floor.
    for (uint8_t i = 0; i < NAXES; i++) {
      if (!_registered[i] || !reverses_at_end(b, i)) {
        continue;
      }
      uint32_t tau = reverse_tau(i);
      if (tau == 0) {
        continue;
      }
      uint32_t floor = _tick_cfg[i] > (uint32_t)MIN_CMD_TICKS
                           ? _tick_cfg[i]
                           : (uint32_t)MIN_CMD_TICKS;
      uint32_t need = tau + floor;
      uint32_t a = accel;
      while (a > 1) {
        RampMap probe(t_law, a);
        if (probe.calculate_ticks(1) >= need) {
          break;
        }
        a >>= 1;
      }
      accel = a;
    }
    uint32_t R_new = remaining_path_steps(b);
    if (R_new == 0) {
      R_new = abs_u32(_blk[b][_master]);
    }
    uint32_t carry = reset_P ? 0 : _law.P;
    if (carry > R_new) {
      carry = R_new;
    }
    _law = RampLaw(t_law, accel, R_new);
    _law.P = carry;
    _abs_master = abs_i64(_blk[b][_master]);
    _block_left = abs_u32(_blk[b][_master]);
    for (uint8_t i = 0; i < NAXES; i++) {
      _err[i] = 0;
    }
    _pause_left = 0;
    _P = _law.P;
    _R = _law.R;
    _ticks_last = 0;
  }

  // Overshoot is selected only for the explicit mode with a non-zero cap
  // (overshoot_max == 0 is the Linear limit, whitepaper section 6.5).
  bool overshoot_mode() const {
    return _cfg.mode == FasNAxisConfig::Overshoot && _cfg.overshoot_max != 0;
  }

  // Set up an Overshoot segment from block `b`: per-axis ramps decide the
  // binding axis (largest T_opt) and every command lasts one of its periods.
  // `_block_left` counts the binding commands; the non-binding axes ride along
  // (whitepaper sections 6.4 / 7.3).
  void start_overshoot(int b) {
    int32_t d[NAXES] = {0};
    uint32_t ac[NAXES] = {0};
    for (uint8_t i = 0; i < NAXES; i++) {
      d[i] = _blk[b][i];
      ac[i] = _lim[i].accel;
    }
    _ovs.init(NAXES, d, _tick_cfg, ac, _cfg.overshoot_max);
    _head = b;
    _master = _ovs.binder;
    _ticks_law = Remaining::ticks_floor(_blk[b], _tick_cfg, NAXES);
    if (_ticks_law == 0) {
      _ticks_law = _tick_cfg[_master] != 0 ? _tick_cfg[_master] : 1;
    }
    _block_left = _ovs.total[_master];
    _pause_left = 0;
    _P = 0;
    _R = _block_left;
    _ticks_last = 0;
    for (uint8_t i = 0; i < NAXES; i++) {
      _err[i] = 0;
    }
  }

  // The current block's DDA walk is exhausted.
  bool block_done() const { return _block_left == 0; }

  // Move to the next committed block: path-stop resets P, collinear carries.
  void advance_block() {
    int prev = _head;
    _head++;
    if (_head >= _n_blk) {
      _done = true;  // last buffered point is rest
      return;
    }
    bool stop = !collinear_same_sense(prev, _head);
    start_block(_head, stop);
  }

  void feeder_start() {
    _feeding = true;
    _done = false;
    _underrun = false;
    for (uint8_t i = 0; i < NAXES; i++) {
      _dir[i] = true;
    }
    if (overshoot_mode()) {
      start_overshoot(_head);
      return;
    }
    start_block(_head, true);
  }

  // Emit at most one queue entry per registered axis per call, so the axes stay
  // in lockstep. A master step with a period above 65535 is represented the FAS
  // way: a half-period step entry followed by pause entries covering the
  // remainder (sections 4.2 / 9.3).
  void feed_one() {
    if (_done || _slice_open) {
      return;
    }
    if (any_carve()) {
      feed_carve();
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
          hold(i, (uint16_t)chunk, 0, _dir[i]);
        }
      }
      _slice_open = true;
      _pause_left -= chunk;
      if (_pause_left == 0 && block_done()) {
        advance_block();
      }
      return;
    }
    if (block_done()) {
      advance_block();
      if (_done) {
        return;
      }
    }

    uint32_t T;
    int32_t st[NAXES];
    for (uint8_t i = 0; i < NAXES; i++) {
      st[i] = 0;
    }
    if (overshoot_mode()) {
      // Overshoot: one command per binding RampLaw step; the non-binding axes
      // ride along on the same tick sum (whitepaper sections 6.4 / 7.3).
      T = _ovs.step(st);
      if (T == 0) {
        _done = true;
        return;
      }
      _P = _ovs.law.P;
      _R = _block_left > 0 ? _block_left - 1 : 0;
      _ticks_last = T;
    } else {
      T = _law.step();
      _P = _law.P;
      _R = _law.R;
      _ticks_last = T;

      st[_master] = _blk[_head][_master] > 0 ? 1 : -1;
      for (uint8_t i = 0; i < NAXES; i++) {
        if (i == (uint8_t)_master) {
          continue;
        }
        int64_t ad = abs_i64(_blk[_head][i]);
        if (ad == 0) {
          continue;
        }
        _err[i] += ad;
        if (2 * _err[i] >= _abs_master) {
          st[i] = _blk[_head][i] > 0 ? 1 : -1;
          _err[i] -= _abs_master;
        }
      }
    }

    uint32_t t_step = T;
    if (T > 65535) {
      t_step = T >> 1;
      if (t_step > 65535) {
        t_step = 65535;
      }
      if (t_step < _ticks_law) {
        t_step = _ticks_law;  // keep the step entry at or above the envelope
        if (t_step > 65535) {
          t_step = 65535;
        }
      }
      _pause_left = T - t_step;
    }

    // At the last binder step before a reversal with a DIR budget, carve the
    // pause out of the reversing axis's own last step (whitepaper section 4.4).
    // Only the 16-bit-representable tail is carved here; a longer tail keeps
    // the §9.3 stuffing path unchanged.
    bool carving = false;
    if (!overshoot_mode() && _block_left == 1 && T <= 65535) {
      for (uint8_t i = 0; i < NAXES; i++) {
        if (!_registered[i] || st[i] == 0 || !reverses_at_end(_head, i)) {
          continue;
        }
        if (reverse_tau(i) == 0) {
          continue;
        }
        start_carve(i, T, _blk[_head + 1][i] > 0);
        carving = true;
      }
    }

    for (uint8_t i = 0; i < NAXES; i++) {
      if (!_registered[i]) {
        continue;
      }
      if (_carve_axis[i].active) {
        bool h = false;
        carve_emit(i, &h);
        continue;
      }
      uint8_t steps = st[i] != 0 ? 1 : 0;
      bool up = steps != 0 ? (st[i] > 0) : _dir[i];
      if (steps != 0) {
        _dir[i] = up;
      }
      hold(i, (uint16_t)t_step, steps, up);
    }
    _slice_open = true;
    _block_left--;
    if (carving) {
      // The carve sequence is flushed over the next feed_one() calls; the next
      // block only starts once every carving axis has finished.
      _carve_then_advance = true;
      return;
    }
    if (_pause_left == 0 && block_done()) {
      advance_block();
    }
  }

  // One held command per axis (Step 8): the slice being emitted. waiting stays
  // set until addQueueEntry returns AQE_OK, so a retryable fault re-sends the
  // same command on the next pump without re-planning the slice.
  struct Held {
    bool waiting;
    uint16_t ticks;
    uint8_t steps;
    bool count_up;
  };

  Stepper* _s[NAXES];
  AxisLimits _lim[NAXES];
  uint32_t _tick_cfg[NAXES];
  int32_t _p[NAXES];
  int64_t _err[NAXES];
  bool _dir[NAXES];
  int32_t _blk[HORIZON][NAXES];
  FasNAxisConfig _cfg;
  bool _registered[NAXES];
  bool _position_synced;
  uint32_t _block_count;
  int _n_blk;
  int _head;
  bool _path_closed;
  bool _feeding;
  bool _done;
  bool _kicked_off;
  bool _underrun;
  bool _slice_open;
  bool _error;
  Held _held[NAXES];
  Carve _carve_axis[NAXES];
  bool _carve_then_advance;
  int _master;
  int64_t _abs_master;
  uint32_t _block_left;
  uint32_t _pause_left;
  uint32_t _ticks_law;
  uint32_t _P;
  uint32_t _R;
  uint32_t _ticks_last;
  RampLaw _law;
  OvershootBlock<NAXES> _ovs;
};

#endif /* FAS_NAXIS_H */
