#ifndef FAS_TIMED_H
#define FAS_TIMED_H

#include <stdint.h>

#include "fas_arch/common.h"
#include "fas_naxis/ramp_map.h"

// Default template arguments. The header does not include FastAccelStepper.h;
// production callers pass the real types, PC tests pass SimPort.
class FastAccelStepper;
class FastAccelStepperEngine;

// Result of FasTimed::addDelta. Rejected is a bad call (not synced, ring
// full, |steps| > 128, ticks < MIN_CMD_TICKS). TimingNotAchievable is a
// feasible-rate failure: faster than the axis limit, a ramp-step jump the
// chunk cannot cover, or a duration the queue cannot emit within |delta|
// ticks of the request. A too-slow request is not rewritten into a faster
// one.
enum class TimedAdd : uint8_t { Ok = 0, Rejected = 1, TimingNotAchievable = 2 };

// Result of FasTimed::pump. Underrun latches until syncFromSteppers /
// setCurrentPosition. Stopped latches the same way after an external stop.
enum class TimedStatus : uint8_t {
  Idle = 0,
  Running = 1,
  Underrun = 2,
  Error = 3,
  Stopped = 4
};

// Faithful timed trajectory (whitepaper section 3.3 problem 2). Not a
// FasNAxis mode: each waypoint is one constant rate, and the planner does
// not invent a ramp between waypoints.
//
//   TimedAdd addDelta(const int16_t steps[NAXES], uint16_t ticks);
//
// steps[i] is in [-128, 128]. ticks is the shared duration of those steps,
// in driver ticks, and must lie in [MIN_CMD_TICKS, 65535]. All-zero steps
// are a dwell on that common clock. One call has one sign per axis.
//
// The feeder emits one or two addQueueEntry commands per axis whose tick
// sums are equal. When the requested duration is not an exact legal command
// split, the shared sum moves to the nearest duration that is, and no
// farther than the longest |delta| of the call. actualTicks() is that sum.
//
// HORIZON is only the number of chunks waiting to be queued. Each chunk is
// already one command, so the ring stays small (default 8). A longer path
// streams: addDelta until it returns Rejected, pump, and add again.
template <uint8_t NAXES, uint16_t HORIZON = 8,
          typename Stepper = FastAccelStepper,
          typename Engine = FastAccelStepperEngine>
class FasTimed {
 public:
  explicit FasTimed(Engine& engine) : _engine(&engine), _chunk{} {
    for (uint8_t i = 0; i < NAXES; i++) {
      _s[i] = NULL;
      _min_period[i] = 0;
      _accel[i] = 0;
      _p[i] = 0;
      _ramp_p[i] = 0;
      _sign[i] = 0;
      _count_up[i] = true;
      _held[i].waiting = false;
      _held[i].ticks = 0;
      _held[i].steps = 0;
      _held[i].count_up = true;
    }
    _count = 0;
    _actual = 0;
    _synced = false;
    _kicked = false;
    _underrun = false;
    _error = false;
    _fault = false;
  }

  // Register axis i. Fails when i is out of range, s is null, or the
  // stepper's ramp generator is active or the queue is already running.
  // Speed and acceleration are read here and stay until the next addAxis.
  bool addAxis(uint8_t i, Stepper* s) {
    if (i >= NAXES || s == NULL) {
      return false;
    }
    if (s->isRampGeneratorActive() || s->isRunning()) {
      return false;
    }
    s->takeStopCause();
    _s[i] = s;
    _min_period[i] = s->getMaxSpeedInTicks();
    _accel[i] = s->getAcceleration();
    return true;
  }

  void syncFromSteppers() {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_s[i] != NULL) {
        _p[i] = _s[i]->getCurrentPosition();
      }
    }
    clear_plan();
    _synced = true;
  }

  void setCurrentPosition(const int32_t p[NAXES]) {
    for (uint8_t i = 0; i < NAXES; i++) {
      _p[i] = p[i];
    }
    clear_plan();
    _synced = true;
  }

  // Queue one constant-rate chunk. See the file comment for the waypoint
  // shape. On Ok the planned position advances by steps and actualTicks()
  // grows by the duration that will be issued.
  TimedAdd addDelta(const int16_t steps[NAXES], uint16_t ticks) {
    if (_fault || _error || !_synced || !all_registered()) {
      return TimedAdd::Rejected;
    }
    if (ticks < MIN_CMD_TICKS) {
      return TimedAdd::Rejected;
    }
    uint16_t mag[NAXES];
    int sign[NAXES];
    for (uint8_t i = 0; i < NAXES; i++) {
      if (steps[i] > 128 || steps[i] < -128) {
        return TimedAdd::Rejected;
      }
      if (steps[i] < 0) {
        mag[i] = (uint16_t)(-steps[i]);
        sign[i] = -1;
      } else if (steps[i] > 0) {
        mag[i] = (uint16_t)steps[i];
        sign[i] = 1;
      } else {
        mag[i] = 0;
        sign[i] = 0;
      }
    }
    if (_count >= HORIZON) {
      return TimedAdd::Rejected;
    }
    if (!speed_ok(mag, ticks)) {
      return TimedAdd::TimingNotAchievable;
    }
    uint16_t issued = 0;
    if (!find_duration(mag, ticks, &issued)) {
      return TimedAdd::TimingNotAchievable;
    }
    uint32_t p_act[NAXES];
    for (uint8_t i = 0; i < NAXES; i++) {
      p_act[i] = 0;
      if (mag[i] == 0) {
        if (!change_ok(i, 0, 0, 0, &p_act[i])) {
          return TimedAdd::TimingNotAchievable;
        }
        continue;
      }
      uint16_t tau_req = 0;
      uint16_t tau_act = 0;
      uint16_t rem = 0;
      udiv_u16(ticks, mag[i], &tau_req, &rem);
      udiv_u16(issued, mag[i], &tau_act, &rem);
      uint32_t ignore = 0;
      if (!change_ok(i, mag[i], sign[i], tau_req, &ignore)) {
        return TimedAdd::TimingNotAchievable;
      }
      if (tau_act != tau_req &&
          !change_ok(i, mag[i], sign[i], tau_act, &p_act[i])) {
        return TimedAdd::TimingNotAchievable;
      }
      if (tau_act == tau_req) {
        p_act[i] = ignore;
      }
    }

    Chunk chunk;
    clear_chunk(&chunk);
    for (uint8_t i = 0; i < NAXES; i++) {
      fill_axis(&chunk, i, steps[i], mag[i], issued);
    }
    _chunk[_count] = chunk;
    _count++;
    _actual += issued;
    for (uint8_t i = 0; i < NAXES; i++) {
      _p[i] += steps[i];
      _ramp_p[i] = p_act[i];
      if (mag[i] != 0) {
        _sign[i] = (int8_t)sign[i];
        _count_up[i] = steps[i] > 0;
      }
    }
    return TimedAdd::Ok;
  }

  int32_t plannedPosition(uint8_t i) const {
    if (i >= NAXES) {
      return 0;
    }
    return _p[i];
  }

  // Sum of the durations actually committed, in driver ticks.
  uint32_t actualTicks() const { return _actual; }

  bool isBusy() const {
    if (_count > 0 || any_waiting()) {
      return true;
    }
    return any_queue_nonempty();
  }

  // Prefill with start=false, then one engine synchronizedStart. A retryable
  // addQueueEntry stops this call; the held command is sent again on the
  // next pump. An injected DIR pause the plan did not emit is Error.
  TimedStatus pump() {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_s[i] != NULL && _s[i]->takeStopCause() != StepperStopCause::None) {
        _fault = true;
      }
    }
    if (_fault) {
      return TimedStatus::Stopped;
    }
    if (_kicked && !any_waiting() && _count > 0 && any_queue_empty()) {
      _underrun = true;
    }
    feed_loop();
    if (!_error && !_kicked && any_queue_nonempty()) {
      Stepper* active[NAXES];
      uint8_t n_active = 0;
      for (uint8_t i = 0; i < NAXES; i++) {
        if (_s[i] != NULL && !_s[i]->isQueueEmpty()) {
          active[n_active++] = _s[i];
        }
      }
      AqeResultCode rc = _engine->synchronizedStart(active, n_active);
      _kicked = n_active > 0;
      if (rc != AqeResultCode::OK) {
        _error = true;
      }
    }
    if (_error) {
      return TimedStatus::Error;
    }
    if (_underrun) {
      return TimedStatus::Underrun;
    }
    if (_count == 0 && !any_waiting() && !any_queue_nonempty()) {
      _kicked = false;
      return TimedStatus::Idle;
    }
    return TimedStatus::Running;
  }

 private:
  struct Qcmd {
    uint16_t ticks;
    uint8_t steps;
    bool count_up;
  };
  struct Chunk {
    Qcmd cmd[NAXES][2];
    uint8_t ncmd[NAXES];
    uint8_t sent[NAXES];
  };
  struct Held {
    bool waiting;
    uint16_t ticks;
    uint8_t steps;
    bool count_up;
  };

  Engine* _engine;
  Stepper* _s[NAXES];
  uint32_t _min_period[NAXES];
  uint32_t _accel[NAXES];
  int32_t _p[NAXES];
  uint32_t _ramp_p[NAXES];
  int8_t _sign[NAXES];
  bool _count_up[NAXES];
  Chunk _chunk[HORIZON];
  uint16_t _count;
  uint32_t _actual;
  bool _synced;
  bool _kicked;
  bool _underrun;
  bool _error;
  bool _fault;
  Held _held[NAXES];

  static void udiv_u16(uint16_t n, uint16_t d, uint16_t* q_out,
                       uint16_t* r_out) {
    uint16_t q = 0;
    uint16_t r = 0;
    if (d == 0) {
      *q_out = 0;
      *r_out = 0;
      return;
    }
    for (int8_t bit = 15; bit >= 0; bit--) {
      r = (uint16_t)(r << 1);
      if ((n & (uint16_t)(1u << bit)) != 0) {
        r = (uint16_t)(r | 1u);
      }
      if (r >= d) {
        r = (uint16_t)(r - d);
        q = (uint16_t)(q | (uint16_t)(1u << bit));
      }
    }
    *q_out = q;
    *r_out = r;
  }

  // A queue command is legal when its span is at least MIN_CMD_TICKS.
  // steps == 1 uses the period alone; steps > 1 uses period * steps.
  static bool command_ok(uint16_t period, uint16_t n) {
    if (n == 0) {
      return true;
    }
    if (period == 0 || n > 255) {
      return false;
    }
    uint32_t span = (n > 1) ? (uint32_t)period * n : (uint32_t)period;
    return span >= (uint32_t)MIN_CMD_TICKS;
  }

  static bool realizable(uint16_t steps, uint16_t dur, uint32_t min_period) {
    if (dur < MIN_CMD_TICKS) {
      return false;
    }
    if (steps == 0) {
      return true;
    }
    if (min_period > 65535u) {
      return false;
    }
    if ((uint32_t)steps * min_period > dur) {
      return false;
    }
    uint16_t q = 0;
    uint16_t r = 0;
    udiv_u16(dur, steps, &q, &r);
    if ((uint32_t)q < min_period) {
      return false;
    }
    if (r == 0) {
      return command_ok(q, steps);
    }
    if (q == 65535) {
      return false;
    }
    return command_ok((uint16_t)(q + 1), r) &&
           command_ok(q, (uint16_t)(steps - r));
  }

  bool realizable_all(const uint16_t mag[NAXES], uint16_t dur) const {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (!realizable(mag[i], dur, _min_period[i])) {
        return false;
      }
    }
    return true;
  }

  // Nearest duration every axis can emit, searched downward first so a tie
  // keeps the shorter sum. The search stops once it is farther than the
  // longest |delta|: a bigger stretch would be a different rate.
  bool find_duration(const uint16_t mag[NAXES], uint16_t requested,
                     uint16_t* issued) const {
    if (realizable_all(mag, requested)) {
      *issued = requested;
      return true;
    }
    uint16_t cap = 0;
    uint32_t lo = MIN_CMD_TICKS;
    for (uint8_t i = 0; i < NAXES; i++) {
      if (mag[i] > cap) {
        cap = mag[i];
      }
      uint32_t need = (uint32_t)mag[i] * _min_period[i];
      if (need > lo) {
        lo = need;
      }
    }
    if (cap == 0 || lo > 65535u) {
      return false;
    }
    for (uint16_t dist = 1; dist != 0; dist++) {
      if (dist > cap) {
        return false;
      }
      bool down = requested >= dist;
      bool up = (uint32_t)requested + dist <= 65535u;
      if (down) {
        uint16_t a = (uint16_t)(requested - dist);
        if ((uint32_t)a >= lo && realizable_all(mag, a)) {
          *issued = a;
          return true;
        }
      }
      if (up) {
        uint16_t a = (uint16_t)(requested + dist);
        if ((uint32_t)a >= lo && realizable_all(mag, a)) {
          *issued = a;
          return true;
        }
      }
      if (!down && !up) {
        return false;
      }
    }
    return false;
  }

  bool speed_ok(const uint16_t mag[NAXES], uint16_t ticks) const {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (mag[i] == 0) {
        continue;
      }
      if ((uint32_t)mag[i] * _min_period[i] > ticks) {
        return false;
      }
    }
    return true;
  }

  uint32_t ramp_at(uint8_t i, uint16_t tau) const {
    RampMap map(_min_period[i], _accel[i]);
    return map.calculate_ramp_steps(tau);
  }

  // |P(tau) - P_prev| must fit in this chunk's steps. A sign change, and a
  // zero-step axis, are legal only from ramp-step 0 (already at rest).
  bool change_ok(uint8_t i, uint16_t steps, int sign, uint16_t tau,
                 uint32_t* p_out) const {
    if (steps == 0) {
      if (_ramp_p[i] != 0) {
        return false;
      }
      *p_out = 0;
      return true;
    }
    uint32_t p_new = ramp_at(i, tau);
    uint32_t prev = _ramp_p[i];
    uint32_t diff = (p_new > prev) ? (p_new - prev) : (prev - p_new);
    if (diff > steps) {
      return false;
    }
    if (_sign[i] != 0 && sign != _sign[i] && prev != 0) {
      return false;
    }
    *p_out = p_new;
    return true;
  }

  static void clear_chunk(Chunk* c) {
    for (uint8_t i = 0; i < NAXES; i++) {
      c->ncmd[i] = 0;
      c->sent[i] = 0;
      for (uint8_t k = 0; k < 2; k++) {
        c->cmd[i][k].ticks = 0;
        c->cmd[i][k].steps = 0;
        c->cmd[i][k].count_up = true;
      }
    }
  }

  void fill_axis(Chunk* c, uint8_t i, int16_t steps, uint16_t mag,
                 uint16_t dur) const {
    if (mag == 0) {
      c->cmd[i][0].ticks = dur;
      c->cmd[i][0].steps = 0;
      c->cmd[i][0].count_up = _count_up[i];
      c->ncmd[i] = 1;
      return;
    }
    bool up = steps > 0;
    uint16_t q = 0;
    uint16_t r = 0;
    udiv_u16(dur, mag, &q, &r);
    if (r == 0) {
      c->cmd[i][0].ticks = q;
      c->cmd[i][0].steps = (uint8_t)mag;
      c->cmd[i][0].count_up = up;
      c->ncmd[i] = 1;
      return;
    }
    c->cmd[i][0].ticks = (uint16_t)(q + 1);
    c->cmd[i][0].steps = (uint8_t)r;
    c->cmd[i][0].count_up = up;
    c->cmd[i][1].ticks = q;
    c->cmd[i][1].steps = (uint8_t)(mag - r);
    c->cmd[i][1].count_up = up;
    c->ncmd[i] = 2;
  }

  bool all_registered() const {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_s[i] == NULL) {
        return false;
      }
    }
    return true;
  }

  bool any_waiting() const {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_held[i].waiting) {
        return true;
      }
    }
    return false;
  }

  bool any_queue_nonempty() const {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_s[i] != NULL && !_s[i]->isQueueEmpty()) {
        return true;
      }
    }
    return false;
  }

  bool any_queue_empty() const {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_s[i] != NULL && _s[i]->isQueueEmpty()) {
        return true;
      }
    }
    return false;
  }

  bool all_have_room() const {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (_s[i] != NULL && _s[i]->isQueueFull()) {
        return false;
      }
    }
    return true;
  }

  static bool chunk_done(const Chunk* c) {
    for (uint8_t i = 0; i < NAXES; i++) {
      if (c->sent[i] < c->ncmd[i]) {
        return false;
      }
    }
    return true;
  }

  void pop_front() {
    if (_count == 0) {
      return;
    }
    for (uint16_t i = 1; i < _count; i++) {
      _chunk[i - 1] = _chunk[i];
    }
    _count--;
  }

  void clear_plan() {
    _count = 0;
    _actual = 0;
    _kicked = false;
    _underrun = false;
    _error = false;
    _fault = false;
    for (uint8_t i = 0; i < NAXES; i++) {
      _ramp_p[i] = 0;
      _sign[i] = 0;
      _count_up[i] = true;
      _held[i].waiting = false;
    }
  }

  // Hold the next unsent command of chunk 0 on every axis that still has
  // one. Returns false when there is nothing left to send.
  bool prepare_round() {
    while (_count > 0 && chunk_done(&_chunk[0])) {
      pop_front();
    }
    if (_count == 0 || !all_have_room()) {
      return false;
    }
    Chunk* c = &_chunk[0];
    bool any = false;
    for (uint8_t i = 0; i < NAXES; i++) {
      if (c->sent[i] >= c->ncmd[i]) {
        continue;
      }
      uint8_t k = c->sent[i];
      _held[i].ticks = c->cmd[i][k].ticks;
      _held[i].steps = c->cmd[i][k].steps;
      _held[i].count_up = c->cmd[i][k].count_up;
      _held[i].waiting = true;
      any = true;
    }
    return any;
  }

  // False means this pump must stop (retry or hard error).
  bool flush() {
    bool retry = false;
    for (uint8_t i = 0; i < NAXES; i++) {
      if (!_held[i].waiting || _s[i] == NULL) {
        continue;
      }
      struct stepper_command_s cmd = {_held[i].ticks, _held[i].steps,
                                      _held[i].count_up};
      AqeResultCode rc = _s[i]->addQueueEntry(&cmd, _kicked);
      if (rc == AqeResultCode::OK) {
        _held[i].waiting = false;
        _chunk[0].sent[i]++;
      } else if (aqeIsPauseInjected(rc)) {
        _error = true;
      } else if (aqeRetry(rc)) {
        retry = true;
      } else {
        _error = true;
      }
    }
    return !_error && !retry;
  }

  void feed_loop() {
    while (!_error) {
      if (!any_waiting()) {
        if (!prepare_round()) {
          break;
        }
      }
      if (!flush()) {
        break;
      }
    }
  }
};

#endif  // FAS_TIMED_H
