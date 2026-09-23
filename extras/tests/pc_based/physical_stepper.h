// physical_stepper.h — a rotordynamic plant that stands in for a real stepper.
//
// The ideal stepper in the PC test suite is an *exact* integrator: it counts
// pulses and never lags, stalls, or rings. That is perfect for kinematics, but
// it cannot answer the practical question "what does the motor actually do".
//
// PhysicalStepper is a lightweight rotor model with a single degree of freedom
// — the rotor angle θ. It is driven by the only two things a real driver can
// hand it:
//
//     how many microsteps (steps), in which direction (count_up),
//     and how long each step lasts (ticks).
//
// Between the commanded microstep angle θc and the actual rotor θ sits a
// rotordynamics model:
//
//      J * dω/dt  =  τ_magnetic(Δθ)  −  B * ω
//      dθ/dt      =  ω
//
// with a deliberately quadratic, sign-following magnetic spring
//
//      τ_magnetic(Δθ) = K * Δθ² · sgn(Δθ)      while |Δθ| < pull_in_miss
//                     = 0                       otherwise (grip lost → stall)
//
// and a hard ½-full-step step-loss rule: a command that would move the rotor
// by more than half a full step in one update is impossible — it is a 100 %
// error / stall event.
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
// Full model, derivation and gnuplot figures: extras/doc/physical_stepper_whitepaper.md
#ifndef FAS_PHYSICAL_STEPPER_H
#define FAS_PHYSICAL_STEPPER_H

#if !defined(FAS_PHYSICAL_STEPPER_ENABLED)
#define FAS_PHYSICAL_STEPPER_ENABLED 0
#endif

#include <math.h>
#include <stdint.h>

#include <cstdio>
#include <cstdlib>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Self-contained: provide a default TICKS_PER_S if the host test does not pull
// in fas_arch/common.h. (FastAccelStepper uses 16 MHz on the PC target.)
#ifndef TICKS_PER_S
#define TICKS_PER_S 16000000L
#endif

#if FAS_PHYSICAL_STEPPER_ENABLED

// One row of the simulation time series — the six gnuplot panels (pos, speed,
// accel, force, steploss, stall) all derive from these fields.
struct observed_s {
  uint32_t t;      // accumulated wall clock (ticks since start)
  double theta;    // rotor position, microsteps (the physical, reported count)
  double thetac;   // commanded microstep angle (microsteps)
  double mis;      // step error ΔN = thetac − theta, microsteps
  double w;        // rotor speed, microsteps / s
  double a;        // rotor acceleration, microsteps / s^2
  double tau;      // magnetic torque K * Δθ²  (SI-ish, scaled)
  bool stall;      // the ½ full-step rule fired on this command
};

// Rotordynamic stepper plant.
//
// Interface: the only inputs are steps, count_up and ticks. There is no
// queue_entry, no hardware — just a command of N microsteps, each lasting
// `ticks` (TICKS_PER_S units), and a direction.
class PhysicalStepper {
 public:
  // microsteps = microsteps per full step (16 as requested).
  // J  rotor moment of inertia   (kg m^2)
  // B  viscous drag coefficient  (N m s)
  // K  magnetic spring gain      (N m / rad^2) — quadratic coupling
  // dtheta_pi  pull-in miss      (rad) — |Δθ| beyond which grip is lost
  PhysicalStepper(uint8_t microsteps      = 16,
                  double inertia           = 1.0e-6,
                  double drag              = 2.0e-5,
                  double magnetic_gain     = 2.0e-3,
                  double dtheta_pi         = M_PI / 6.0)  // 30°
      : microsteps_(microsteps),
        J_(inertia),
        B_(drag),
        K_(magnetic_gain),
        dtheta_pi_(dtheta_pi),
        step_rad_((2.0 * M_PI) / microsteps),
        ticks_(0),
        stalls_(0) {
    reset();
  }

  // Reset the rotor to rest exactly at the commanded angle.
  void reset() {
    state_.theta = 0.0;
    state_.theta_cmnd = 0.0;
    state_.w = 0.0;
    ticks_ = 0;
    stalls_ = 0;
    last_ = observed_s{0, 0, 0, 0, 0, 0, 0, false};
  }

  // Advance the plant by `steps` microsteps, each lasting `ticks` (in
  // TICKS_PER_S units), in the given direction. A direction flip makes the
  // spring force flip, producing the reversal behaviour of a real motor.
  //
  // Returns the last observed_s snapshot; `stall` is true iff the ½-full-step
  // rule fired on the last microstep of this command.
  observed_s step(int steps, bool count_up, uint16_t ticks) {
    const int32_t dir = count_up ? 1 : -1;
    const double step_per_rad = (2.0 * M_PI) / step_rad_;

    // Minimum catchable period: below this the rotor cannot move a full
    // microstep within one update, so every step past it is a stall. This is
    // the ½-full-step rule applied at the *update boundary* — a command whose
    // required microstep delta per update exceeds half a step is uncatchable.
    const double dtau_min = 1.0 / 120.0;  // ~3 ms minimum catchable period

    double mis_micro = 0.0;
    double a_micro   = 0.0;
    double tau_micro = 0.0;
    bool this_stall  = false;

    for (int k = 0; k < steps; k++) {
      const double thec_next =
          state_.theta_cmnd + (double)dir * (double)microsteps_ *
                                  step_rad_ / (2.0 * M_PI);

      // the hard ½-full-step rule: a command delta beyond half a full step in
      // one update is uncatchable by definition.
      const double dN_cmd = fabs(thec_next - state_.theta_cmnd) / step_per_rad /
                            (double)microsteps_;
      this_stall = (dN_cmd > 0.5);

      const double dt = (double)ticks / (double)TICKS_PER_S;
      const double dt_safe = (dt < dtau_min) ? dtau_min : dt;

      if (this_stall) {
        // grip lost — rotor holds its angle, magnetic torque vanishes.
        state_.w = 0.0;
        tau_micro = 0.0;
        mis_micro = fabs(thec_next - state_.theta) * step_per_rad;
      } else {
        // Δθ in microstep units; the quadratic spring sees the microstep
        // level error between commanded and rotor angle.
        const double theta_cmnd_micro =
            state_.theta_cmnd + (double)dir * (double)microsteps_;
        const double dtheta_micro = theta_cmnd_micro - state_.theta;  // ΔN
        // pull-in clamp: above the pull-in miss the motor can no longer grip.
        const double dtheta_pi_micro = dtheta_pi_ * step_per_rad;
        double torque =
            K_ * dtheta_micro * dtheta_micro * (dtheta_micro > 0.0 ? 1.0 : -1.0);
        if (fabs(dtheta_micro) > dtheta_pi_micro) {
          torque = 0.0;  // lost grip → step loss on the next overshoot
        }
        // Euler-Cromer: ω += (τ − Bω)/J·dt ; then θ += ω·dt.
        a_micro = (torque - B_ * state_.w) / J_;
        state_.w += a_micro * dt_safe;
        state_.theta += state_.w * dt_safe;
        mis_micro = dtheta_micro;
        tau_micro = torque;
      }

      // advance the commanded angle by one microstep.
      state_.theta_cmnd = thec_next;
    }

    if (steps == 0) {
      // pause (ticks): no commanded motion, rotor only decays under drag.
      const double dt = (double)ticks / (double)TICKS_PER_S;
      a_micro = 0.0;
      tau_micro = 0.0;
      state_.w *= exp(-(B_ / J_) * dt);  // ω(t) ∝ e^(-(B/J)t)
      state_.theta += state_.w * dt;
      mis_micro = state_.theta_cmnd - state_.theta;
    }

    if (this_stall) {
      stalls_++;
    }

    // advance the simulated wall clock by the real command duration.
    ticks_ += (steps == 0) ? ticks : (uint32_t)steps * ticks;

    last_.t = ticks_;
    last_.theta  = state_.theta;
    last_.thetac = state_.theta_cmnd;
    last_.mis    = mis_micro;
    last_.w      = state_.w;
    last_.a      = a_micro;
    last_.tau    = tau_micro;
    last_.stall  = this_stall;

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

  // The rotor's *physical* microstep count — what the queue base would report,
  // diverging from the commanded count whenever the rotor lags or stalls.
  int32_t getCurrentPosition() const { return (int32_t)llround(state_.theta); }

  uint32_t stalls() const { return stalls_; }
  uint32_t total_ticks() const { return ticks_; }

  // Emit one row to a gnuplot writer (see naxis_plot.h), one of the six panels:
  // position, speed, acceleration, force, steploss, stall-event.
  void plot_row(FILE* g, uint8_t panel, const observed_s& o) {
    fprintf(g, "%llu %g %g %g %g %g\n",
            (unsigned long long)o.t, o.theta, o.w, o.a, o.tau,
            (double)o.stall);
    (void)panel;
  }

  const observed_s& last() const { return last_; }

  // The rotor angle and speed as raw values for the gnuplot traces.
  double theta() const { return state_.theta; }
  double theta_cmnd() const { return state_.theta_cmnd; }
  double speed() const { return state_.w; }

  // Acoustic emission (see whitepaper §5.8): turn the last simulated command
  // into a 16-bit PCM .wav, so a test can *play back* the motor's hum and hear
  // it drop when the rotor stalls. Sampled at a fixed audio rate; the per-sample
  // signal is the first few harmonics of the step frequency f = ν·|ω|,
  // amplitudes A_k ∝ 1/k (electromagnetic hum model). Returns true on success.
  bool to_wav(const char* path, uint32_t sr = 44100) {
    static const double amp[] = {1.0, 0.5, 0.33, 0.25, 0.2};
    static const int kHarm = 5;
    const double f = (double)microsteps_ * std::fabs(speed());
    const double dt = 1.0 / (double)sr;
    // one second of note, or the move's own length, whichever is shorter;
    // capped so a test file stays small.
    double dur = (double)ticks_ / (double)TICKS_PER_S;
    if (dur > 1.0) dur = 1.0;
    if (dur < 0.001) dur = 0.001;
    uint32_t n = (uint32_t)(dur * sr);
    if (n == 0) n = 1;

    FILE* f = std::fopen(path, "wb");
    if (!f) return false;
    const uint16_t bps = 16, ch = 1;
    const uint32_t byteRate = sr * ch * bps / 8;
    const uint16_t blockAlign = ch * bps / 8;
    const uint32_t dataSize = n * ch * bps / 8;
    const uint32_t chunkSize = 36 + dataSize;
    const char riff[] = {'R','I','F','F'};
    const char wave[] = {'W','A','V','E'};
    const char fmt[]  = {'f','m','t',' '};
    const char data[] = {'d','a','t','a'};
    // RIFF / fmt / data chunk headers.
    fwrite(riff, 1, 4, f); write_u32le(f, chunkSize);
    fwrite(wave, 1, 4, f); fwrite(fmt, 1, 4, f); write_u16le(f, 1); // PCM
    write_u16le(f, ch);      write_u32le(f, sr);
    write_u32le(f, byteRate); write_u16le(f, blockAlign);
    write_u16le(f, bps);     fwrite(data, 1, 4, f);
    write_u32le(f, dataSize);
    // samples.
    for (uint32_t i = 0; i < n; i++) {
      double t = (double)i * dt;
      double s = 0.0;
      for (int k = 0; k < kHarm; k++) {
        s += amp[k] * std::sin(2.0 * M_PI * (k + 1) * f * t);
      }
      s *= 0.25;  // headroom
      int16_t out = (int16_t)std::lround(std::clamp(s, -1.0, 1.0));
      write_u16le(f, (uint16_t)out);
    }
    std::fclose(f);
    return true;
  }

 private:
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

  struct { double theta, theta_cmnd, w; } state_;  // θ, θc (cmd), ω
  uint8_t microsteps_;
  double J_, B_, K_, dtheta_pi_;
  double step_rad_;   // 2π / microsteps — radians per commanded microstep
  uint32_t ticks_;    // accumulated wall clock
  uint32_t stalls_;   // cumulative ½-full-step stall events
  observed_s last_;
};

#else  // !FAS_PHYSICAL_STEPPER_ENABLED — inert stub, keeps the ideal stepper

// Placeholder so callers compile unchanged when the plant is not opted in.
struct observed_s {
  uint32_t t;
  double theta, thetac, mis, w, a, tau;
  bool stall;
};
class PhysicalStepper {
 public:
  PhysicalStepper(int, double = 0, double = 0, double = 0, double = 0) { reset(); }
  void reset() {}
  observed_s step(int, bool, uint16_t) {
    static observed_s s = {0, 0, 0, 0, 0, 0, 0, false};
    return s;
  }
  struct Cmd { int steps; bool count_up; uint16_t ticks; };
  observed_s run(const Cmd*, int) {
    static observed_s s = {0, 0, 0, 0, 0, 0, 0, false};
    return s;
  }
  int32_t getCurrentPosition() const { return 0; }
  uint32_t stalls() const { return 0; }
  uint32_t total_ticks() const { return 0; }
  void plot_row(FILE*, uint8_t, const observed_s&) {}
  const observed_s& last() const {
    static observed_s s = {0, 0, 0, 0, 0, 0, 0, false};
    return s;
  }
  bool to_wav(const char*, uint32_t) { return false; }
};

#endif  // FAS_PHYSICAL_STEPPER_ENABLED
#endif  // FAS_PHYSICAL_STEPPER_H
