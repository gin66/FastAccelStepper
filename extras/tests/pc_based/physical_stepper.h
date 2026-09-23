// physical_stepper.h — a rotordynamic plant that stands in for a real stepper.
//
// The ideal stepper in the PC test suite is an *exact* integrator: it counts
// pulses and never lags, stalls, or rings. That is perfect for kinematics, but
// it cannot answer the practical question "what does the motor actually do".
//
// PhysicalStepper is a lightweight rotor model with a single degree of freedom
// — the rotor position x. It is driven by the only two things a real driver can
// hand it:
//
//     how many steps, in which direction, and how long each step lasts (ticks).
//
// Between the commanded position x_c and the actual rotor x sits a
// rotordynamics model:
//
//      J * dw/dt  =  tau_magnetic(delta)  -  B * w
//      dx/dt      =  w
//
// with delta = x - x_c (actual minus target) and a bounded, sinusoidal
// magnetic coupling — the holding/detent torque curve:
//
//      tau_magnetic(delta) = -Fmax * sin(pi * delta / D)   while |delta| <  D
//                          =  0                            while |delta| >= D
//
// The force is zero when the rotor is exactly on its command, peaks in the
// opposing direction half a full step away, and returns to zero one full step
// away. D is the number of steps in one full step; it is private to the plant.
//
// Step loss is the domain boundary of that curve, not a bolt-on detector: once
// the lag reaches one full step (|delta| >= D) the coupling loses grip. A
// stalled rotor is then *inert*: further high-speed pulses keep advancing the
// commanded position while the actual rotor does not move, until the command
// is walked back within D. See the whitepaper §5.5.
//
// ---- opt-in, never the default -------------------------------------------
//
// This plant NEVER replaces the ideal stepper by default. Every existing test
// keeps the ideal stepper. Turn the plant on by request, per test (e.g.
// test_26), with the gate at the top of the file:
//
//     #define FAS_PHYSICAL_STEPPER_ENABLED 1
//     #include "physical_stepper.h"
//
// While FAS_PHYSICAL_STEPPER_ENABLED == 0 the header is an inert stub: a test
// that forgets to opt in keeps the ideal stepper and loses nothing.
//
// Full model, derivation and gnuplot figures:
// extras/doc/physical_stepper_whitepaper.md
#ifndef FAS_PHYSICAL_STEPPER_H
#define FAS_PHYSICAL_STEPPER_H

#if !defined(FAS_PHYSICAL_STEPPER_ENABLED)
#define FAS_PHYSICAL_STEPPER_ENABLED 0
#endif

#include <math.h>
#include <stdint.h>

#include <cstdio>
#include <cstdlib>
#if FAS_PHYSICAL_STEPPER_ENABLED
#include <vector>
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Self-contained: provide a default TICKS_PER_S if the host test does not pull
// in fas_arch/common.h. (FastAccelStepper uses 16 MHz on the PC target.)
#ifndef TICKS_PER_S
#define TICKS_PER_S 16000000L
#endif

// One row of the simulation time series — the six gnuplot panels (pos, speed,
// accel, force, steploss, stall) all derive from these fields. Position and
// error are in steps; a step quantum is the plant's private business.
struct observed_s {
  uint32_t t;    // accumulated wall clock (ticks since start)
  double x;      // rotor position, steps (the physical, reported count)
  double x_c;    // commanded position, steps
  double delta;  // position error x - x_c, steps
  double w;      // rotor speed, steps / s
  double a;      // rotor acceleration, steps / s^2
  double tau;    // magnetic force -Fmax * sin(pi * delta / D)
  bool stall;    // |delta| >= D fired on this command
};

// Simple clamp so we don't pull in <algorithm> (which collides with the
// test suite's test(x, msg) macro).
static inline double fas_clamp(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

#if FAS_PHYSICAL_STEPPER_ENABLED

// Rotordynamic stepper plant.
//
// Interface: the only inputs are steps, count_up and ticks. There is no
// queue_entry, no hardware — just a command of N steps, each lasting `ticks`
// (TICKS_PER_S units), and a direction.
class PhysicalStepper {
 public:
  // units_per_full_step = D, the steps in one full step (the period of the
  // force curve). J rotor moment of inertia, B viscous drag, Fmax peak torque.
  PhysicalStepper(uint8_t units_per_full_step = 64, double inertia = 1.0e-6,
                  double drag = 1.0e-6, double force_gain = 1.0e-1)
      : D_(units_per_full_step),
        J_(inertia),
        B_(drag),
        Fmax_(force_gain),
        ticks_(0),
        stalls_(0) {
    reset();
  }

  // Reset the rotor to rest exactly at the commanded position.
  void reset() {
    state_.x = 0.0;
    state_.x_c = 0.0;
    state_.w = 0.0;
    ticks_ = 0;
    stalls_ = 0;
    sim_time_ = 0.0;
    cur_f_ = 0.0;
    cur_gate_ = 0.0;
    cur_disp_ = 0.0;
    phase_ = 0.0;
    audio_.clear();
    audio_written_ = 0;
    last_ = observed_s{0, 0, 0, 0, 0, 0, 0, false};
  }

  // Advance the plant by `steps` steps, each lasting `ticks` (in TICKS_PER_S
  // units), in the given direction. A direction flip makes the spring force
  // flip, producing the reversal behaviour of a real motor.
  observed_s step(int steps, bool count_up, uint16_t ticks) {
    const double dir = count_up ? 1.0 : -1.0;
    const double dt = (double)ticks / (double)TICKS_PER_S;

    double delta = state_.x - state_.x_c;
    double tau = 0.0;
    double a = 0.0;
    uint32_t lost = 0;  // step-losses in this command

    for (int k = 0; k < steps; k++) {
      state_.x_c += dir;
      delta = state_.x - state_.x_c;

      if (fabs(delta) >= (double)D_) {
        // grip lost: a full step of error. The coupling can no longer pull
        // the rotor; it is subject to drag alone. This is the stall.
        lost++;
        tau = 0.0;
      } else {
        tau = -Fmax_ * sin(M_PI * delta / (double)D_);
      }
      a = (tau - B_ * state_.w) / J_;
      state_.w += a * dt;
      state_.x += state_.w * dt;
      sim_time_ += dt;
      update_audio_state(a, tau);
      record_audio();
    }

    if (steps == 0) {
      // pause (ticks): no commanded motion, but the holding torque is still
      // active toward the frozen command — the rotor is held, not released.
      const double dtp = (double)ticks / (double)TICKS_PER_S;
      delta = state_.x - state_.x_c;
      if (fabs(delta) >= (double)D_) {
        tau = 0.0;
      } else {
        tau = -Fmax_ * sin(M_PI * delta / (double)D_);
      }
      a = (tau - B_ * state_.w) / J_;
      state_.w += a * dtp;
      state_.x += state_.w * dtp;
      delta = state_.x - state_.x_c;
      sim_time_ += dtp;
      update_audio_state(a, tau);
      record_audio();
    }

    stalls_ += lost;

    // advance the simulated wall clock by the real command duration.
    ticks_ += (steps == 0) ? ticks : (uint32_t)steps * ticks;

    last_.t = ticks_;
    last_.x = state_.x;
    last_.x_c = state_.x_c;
    last_.delta = delta;
    last_.w = state_.w;
    last_.a = a;
    last_.tau = tau;
    last_.stall = (lost > 0);

    return last_;
  }

  // Run a whole command list; returns the last observed_s.
  //   cmd[i] = {steps, count_up, ticks}
  struct Cmd {
    int steps;
    bool count_up;
    uint16_t ticks;
  };
  observed_s run(const Cmd* cmds, int len) {
    observed_s last;
    for (int i = 0; i < len; i++) {
      last = step(cmds[i].steps, cmds[i].count_up, cmds[i].ticks);
    }
    return last;
  }

  // The rotor's *physical* step count — what the queue base would report,
  // diverging from the commanded count whenever the rotor lags or stalls.
  int32_t getCurrentPosition() const { return (int32_t)llround(state_.x); }

  uint32_t stalls() const { return stalls_; }
  uint32_t total_ticks() const { return ticks_; }

  // Emit one row to a gnuplot writer (see naxis_plot.h), one of the six panels:
  // position, speed, acceleration, force, steploss, stall-event.
  void plot_row(FILE* g, uint8_t panel, const observed_s& o) {
    fprintf(g, "%llu %g %g %g %g %g\n", (unsigned long long)o.t, o.x, o.w, o.a,
            o.tau, (double)o.stall);
    (void)panel;
  }

  const observed_s& last() const { return last_; }

  // Raw state for the gnuplot traces.
  double x() const { return state_.x; }
  double x_c() const { return state_.x_c; }
  double delta() const { return state_.x - state_.x_c; }
  double speed() const { return state_.w; }

  // Acoustic emission (see whitepaper §5.8): a hybrid source driven by the
  // actual rotor motion and gated by the magnetic force curve, plus a
  // displacement (acceleration) term. The plant records one 44.1 kHz sample per
  // output interval while it steps, so to_wav() renders the *whole* simulated
  // move (e.g. the 10 s canonical trapezoid -> ~882000 bytes of PCM) rather
  // than a single capped note. Returns true on success.
  bool to_wav(const char* path, uint32_t sr = 44100) {
    (void)sr;
    const uint32_t rate = kAudioSr;
    // Guarantee a valid, non-empty data chunk even for a zero-length move.
    if (audio_.empty()) audio_.push_back(0);
    const uint32_t n = (uint32_t)audio_.size();

    FILE* fp = std::fopen(path, "wb");
    if (!fp) return false;
    const uint16_t bps = 16, ch = 1;
    const uint32_t byteRate = rate * ch * bps / 8;
    const uint16_t blockAlign = ch * bps / 8;
    const uint32_t dataSize = n * ch * bps / 8;
    const uint32_t chunkSize = 36 + dataSize;
    const char riff[] = {'R', 'I', 'F', 'F'};
    const char wave[] = {'W', 'A', 'V', 'E'};
    const char fmt[] = {'f', 'm', 't', ' '};
    const char data[] = {'d', 'a', 't', 'a'};
    // RIFF / fmt / data chunk headers.
    fwrite(riff, 1, 4, fp);
    write_u32le(fp, chunkSize);
    fwrite(wave, 1, 4, fp);
    fwrite(fmt, 1, 4, fp);
    write_u32le(fp, 16);  // fmt subchunk size (PCM)
    write_u16le(fp, 1);   // audio format = PCM
    write_u16le(fp, ch);
    write_u32le(fp, rate);
    write_u32le(fp, byteRate);
    write_u16le(fp, blockAlign);
    write_u16le(fp, bps);
    fwrite(data, 1, 4, fp);
    write_u32le(fp, dataSize);
    for (uint32_t i = 0; i < n; i++) write_u16le(fp, (uint16_t)audio_[i]);
    std::fclose(fp);
    return true;
  }

  // Number of recorded audio samples (for tests that check the rendered
  // length).
  uint32_t audio_samples() const { return (uint32_t)audio_.size(); }

 private:
  // Fixed audio sample rate for the recording grid.
  enum { kAudioSr = 44100 };

  // Cache the hybrid source parameters after each integration sub-step.
  void update_audio_state(double a, double tau) {
    // Hum pitch is the full-step (electrical) rate, |w|/D Hz — the rate the
    // rotor's magnetic field actually turns — not the finest step rate.
    cur_f_ = fabs(state_.w) / (double)D_;
    cur_gate_ = (Fmax_ > 0.0) ? fabs(tau) / Fmax_ : 0.0;
    cur_disp_ = (Fmax_ > 0.0) ? (a * J_) / Fmax_ : 0.0;
  }

  // Emit every audio sample whose time has now been reached.
  void record_audio() {
    double want_d = sim_time_ * (double)kAudioSr;
    if (want_d > 1.0e9) return;  // safety: never allocate unboundedly
    uint32_t want = (uint32_t)floor(want_d);
    static const double amp[] = {1.0, 0.5, 0.33, 0.25, 0.2};
    const double dphase = 2.0 * M_PI * cur_f_ / (double)kAudioSr;
    while (audio_written_ < want) {
      // Accumulate phase so a changing pitch never clicks.
      phase_ += dphase;
      if (phase_ > 2.0 * M_PI) phase_ -= 2.0 * M_PI;
      double s = 0.0;
      for (int k = 0; k < 5; k++) {
        s += amp[k] * sin((double)(k + 1) * phase_);
      }
      s = cur_gate_ * s + 0.1 * cur_disp_;
      // The gated hum is small (gate ~ |delta|/D), so scale it up out of the
      // int16 rounding floor before clipping.
      s *= 3.0;
      double q = fas_clamp(s, -1.0, 1.0) * 32767.0;
      audio_.push_back((int16_t)lround(q));
      audio_written_++;
    }
  }

  static void write_u16le(FILE* f, uint16_t v) {
    fputc((int)(v & 0xff), f);
    fputc((int)((v >> 8) & 0xff), f);
  }
  static void write_u32le(FILE* f, uint32_t v) {
    fputc((int)(v & 0xff), f);
    fputc((int)((v >> 8) & 0xff), f);
    fputc((int)((v >> 16) & 0xff), f);
    fputc((int)((v >> 24) & 0xff), f);
  }

  struct {
    double x, x_c, w;
  } state_;    // x (rotor), x_c (cmd), w
  uint8_t D_;  // steps per full step — the force-curve period
  double J_, B_, Fmax_;
  uint32_t ticks_;                      // accumulated wall clock
  uint32_t stalls_;                     // cumulative step-loss events
  double sim_time_;                     // accumulated simulated time (s)
  double cur_f_, cur_gate_, cur_disp_;  // audio source at the current step
  double phase_;                        // continuous hum phase (rad)
  std::vector<int16_t> audio_;          // recorded PCM samples
  uint32_t audio_written_;              // samples already emitted
  observed_s last_;
};

#else  // !FAS_PHYSICAL_STEPPER_ENABLED — inert stub, keeps the ideal stepper

// Placeholder so callers compile unchanged when the plant is not opted in.
class PhysicalStepper {
 public:
  PhysicalStepper(uint8_t = 64, double = 0, double = 0, double = 0) { reset(); }
  void reset() {}
  observed_s step(int, bool, uint16_t) { return zero(); }
  struct Cmd {
    int steps;
    bool count_up;
    uint16_t ticks;
  };
  observed_s run(const Cmd*, int) { return zero(); }
  int32_t getCurrentPosition() const { return 0; }
  uint32_t stalls() const { return 0; }
  uint32_t total_ticks() const { return 0; }
  void plot_row(FILE*, uint8_t, const observed_s&) {}
  const observed_s& last() const { return zero(); }
  double x() const { return 0; }
  double x_c() const { return 0; }
  double delta() const { return 0; }
  double speed() const { return 0; }
  bool to_wav(const char*, uint32_t) { return false; }

 private:
  static observed_s& zero() {
    static observed_s s = {0, 0, 0, 0, 0, 0, 0, false};
    return s;
  }
};

#endif  // FAS_PHYSICAL_STEPPER_ENABLED
#endif  // FAS_PHYSICAL_STEPPER_H
