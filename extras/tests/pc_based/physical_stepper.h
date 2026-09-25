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
//      J * dw/dt  =  tau_magnetic(delta)  -  F_friction(w)
//      dx/dt      =  w
//
// with delta = x - x_c (actual minus target) and a bounded, sinusoidal
// magnetic coupling — the holding/detent torque curve, with a stable detent
// once per full step:
//
//      tau_magnetic(delta) = -Fmax * sin(2*pi * delta / D)   for any delta
//
// The magnetic field is the only torque the driver commands. It is zero at the
// stable detents (delta = k*D), reaches its peak Fmax a quarter step away, and
// crosses zero again at the half-step separatrix (the unstable point between
// two detents). It is periodic in delta, so it is evaluated for *any* delta —
// no “grip lost” branch (§5.4).
//
// The only inverse force to the motor is friction, an *independent* mechanical
// load (it does not scale with Fmax):
//
//      F_friction(w) = friction_static + friction_viscous * |w|
//
// opposite to w, capped at Fmax. At low speed it is the Coulomb floor; it rises
// with speed until it equals Fmax at the pull-out speed
// v_max = (Fmax - friction_static) / friction_viscous. There is deliberately no
// separate magnetic drag term (§5.6).
//
// A one-shot rotor displacement (`rotate`) stands in for a mechanical hold:
// nudging the rotor back half a full step parks it on the separatrix, so the
// field out-runs it and it stalls; nudging back a full step parks it on the
// next stable detent and it only loses one step. A command pause does not
// release the motor either: the detent force keeps acting on the frozen
// command. Stall is not implemented: it is only the *observation* that
// a full step behind *and still slipping* (§5.5). See the whitepaper
// §5.5–§5.6.
//
// ---- opt-in, never the default -------------------------------------------
//
// This plant NEVER replaces the ideal stepper by default. Every existing test
// keeps the ideal stepper. Turn the plant on by request, per test (e.g.
// test_27), with the gate at the top of the file:
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
  bool stall;    // 1 while the rotor is slipping (dynamic, §5.5)
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
  // force curve). J rotor moment of inertia, Fmax peak magnetic torque.
  // Friction is an *independent* mechanical load: `friction_static` is its
  // Coulomb floor and `friction_viscous` its increase per step/s. It does NOT
  // scale with Fmax — a stronger motor does not have more bearing friction.
  // The pull-out speed is where the two balance: v_max = (Fmax - static)/visc.
  PhysicalStepper(uint8_t units_per_full_step = 64, double inertia = 1.0e-6,
                  double force_gain = 1.0, double friction_static = 0.1,
                  double friction_viscous = 2.25e-5)
      : D_(units_per_full_step),
        J_(inertia),
        Fmax_(force_gain),
        friction_static_(friction_static),
        friction_viscous_(friction_viscous),
        ticks_(0),
        slip_f_(0.0),
        stall_ever_(false),
        trace_decim_(0),
        trace_n_(0),
        trace_phase_(0) {
    reset();
  }

  // Reset the rotor to rest exactly at the commanded position.
  void reset() {
    state_.x = 0.0;
    state_.x_c = 0.0;
    state_.w = 0.0;
    ticks_ = 0;
    sim_time_ = 0.0;
    phase_ = 0.0;
    peak_abs_delta_ = 0.0;
    slip_f_ = 0.0;
    stall_ever_ = false;
    cur_env_ = 0.0;
    audio_.clear();
    audio_written_ = 0;
    trace_.clear();
    trace_n_ = 0;
    last_ = observed_s{0, 0, 0, 0, 0, 0, 0, false};
  }

  // One-shot external displacement of the rotor by `steps` (signed, positive =
  // forward), with velocity preserved. This is the simulation stand-in for
  // "mechanically holding" the rotor: holding it while the field advances by
  // half a full step is the same as rotating the rotor half a step backwards
  // once. Half a step parks it on the separatrix (it stalls); a full step parks
  // it on the next stable detent (it loses exactly one step).
  void rotate(double steps) { state_.x += steps; }

  // The magnetic field: the only commanded torque, zero at each stable detent
  // (delta = k*D), peak Fmax a quarter step away, separatrix at the half step.
  double magnetic_torque(double delta) const {
    return -Fmax_ * sin(2.0 * M_PI * delta / (double)D_);
  }

  // Magnitude of the only inverse force to the motor: a Coulomb floor plus a
  // viscous term, both absolute mechanical parameters independent of Fmax.
  // Capped at Fmax (the motor cannot pull harder than its own torque). Returned
  // unsigned; the integrator applies it against the direction of motion.
  double friction_mag(double w) const {
    double f = friction_static_ + friction_viscous_ * fabs(w);
    if (f > Fmax_) f = Fmax_;
    return f;
  }

  // Pull-out speed: where friction has grown to equal Fmax.
  double v_max() const {
    if (friction_viscous_ <= 0.0) return 0.0;
    double v = (Fmax_ - friction_static_) / friction_viscous_;
    return v > 0.0 ? v : 0.0;
  }

  // Signed friction force (for reporting / the gnuplot trace).
  double friction_force(double w) const {
    double f = friction_mag(w);
    return (w > 0.0) ? -f : (w < 0.0 ? f : 0.0);
  }

  // One Euler-Cromer sub-step under magnetic torque `tau`. Friction removes up
  // to Ff*dt/J of velocity but can never reverse the motion, so a rotor at rest
  // is held while |tau| <= static friction (the mu0*Fmax floor). Returns the
  // effective rotor acceleration (dw/dt) for the trace and the audio model.
  double substep(double tau, double dt) {
    const double w_old = state_.w;
    const double Ff = friction_mag(w_old);
    const double w_free = w_old + (tau / J_) * dt;
    const double dw_fric = Ff * dt / J_;
    if (fabs(w_free) <= dw_fric) {
      state_.w = 0.0;
    } else {
      state_.w = w_free - (w_free > 0.0 ? dw_fric : -dw_fric);
    }
    state_.x += state_.w * dt;
    return (dt > 0.0) ? (state_.w - w_old) / dt : 0.0;
  }

  // Integrate one command's duration `dt` in sub-steps no longer than kMaxDt,
  // so a long commanded period (up to 65535 ticks = 4.1 ms) never out-runs the
  // detent dynamics. Returns the effective rotor acceleration over `dt`.
  double integrate(double dt) {
    const double w_before = state_.w;
    int n = (int)ceil(dt / kMaxDt);
    if (n < 1) n = 1;
    const double h = dt / (double)n;
    for (int i = 0; i < n; i++) {
      substep(magnetic_torque(state_.x - state_.x_c), h);
    }
    return (dt > 0.0) ? (state_.w - w_before) / dt : 0.0;
  }

  // Advance the plant by `steps` steps, each lasting `ticks` (in TICKS_PER_S
  // units), in the given direction. A direction flip makes the spring force
  // flip, producing the reversal behaviour of a real motor.
  observed_s step(int steps, bool count_up, uint16_t ticks) {
    const double dir = count_up ? 1.0 : -1.0;
    const double dt = (double)ticks / (double)TICKS_PER_S;
    const double v_field =
        (steps > 0) ? (double)TICKS_PER_S / (double)ticks : 0.0;

    double delta = state_.x - state_.x_c;
    double tau = 0.0;
    double a = 0.0;

    for (int k = 0; k < steps; k++) {
      state_.x_c += dir;

      // The magnetic field is the only commanded torque: a single periodic
      // curve evaluated for ANY delta (whitepaper §5.4 — no domain boundary,
      // no “grip lost” branch). Friction is the only inverse force (§5.6).
      a = integrate(dt);
      delta = state_.x - state_.x_c;
      tau = magnetic_torque(delta);
      sim_time_ += dt;
      if (fabs(delta) > peak_abs_delta_) peak_abs_delta_ = fabs(delta);
      update_stall(delta, v_field, dt);
      update_audio_state(delta, a, tau);
      record_audio();
      record_trace(delta, a, tau);
    }

    if (steps == 0) {
      // pause (ticks): no commanded motion, but the detent force is still in
      // full effect toward the frozen command — the rotor is braked, not
      // released. This is what stops a coasting rotor (§5.6).
      const double dtp = (double)ticks / (double)TICKS_PER_S;
      a = integrate(dtp);
      delta = state_.x - state_.x_c;
      tau = magnetic_torque(delta);
      sim_time_ += dtp;
      if (fabs(delta) > peak_abs_delta_) peak_abs_delta_ = fabs(delta);
      update_stall(delta, 0.0, dtp);
      update_audio_state(delta, a, tau);
      record_audio();
      record_trace(delta, a, tau);
    }

    // advance the simulated wall clock by the real command duration.
    ticks_ += (steps == 0) ? ticks : (uint32_t)steps * ticks;

    last_.t = ticks_;
    last_.x = state_.x;
    last_.x_c = state_.x_c;
    last_.delta = delta;
    last_.w = state_.w;
    last_.a = a;
    last_.tau = tau;

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

  uint32_t total_ticks() const { return ticks_; }

  // Peak |x - x_c| seen since the last reset — the cleanest scalar to assert
  // "a full step was lost" on, independent of trace decimation.
  double peak_abs_delta() const { return peak_abs_delta_; }

  // Whether the rotor was ever observed slipping since the last reset. The
  // instantaneous `last().stall` clears again once the rotor re-captures, so
  // this is the latch for "a stall happened" assertions.
  bool stall_ever() const { return stall_ever_; }

  // Emit one row to a gnuplot writer (see naxis_plot.h), one of the six panels:
  // position, speed, acceleration, force, steploss, stall-event.
  void plot_row(FILE* g, uint8_t panel, const observed_s& o) {
    fprintf(g, "%llu %g %g %g %g %g\n", (unsigned long long)o.t, o.x, o.w, o.a,
            o.tau, (double)o.stall);
    (void)panel;
  }

  const observed_s& last() const { return last_; }

  // ---- gnuplot time-series recorder -------------------------------------
  // The six-panel figure (§6) is built from the per-step integration. The raw
  // trace has one row per command, which for a 10000 step/s move at 16 MHz is
  // ~16000 rows/s — too dense to plot meaningfully and too big for a stall
  // trace that spans a 75 s settle. So we decimate by a fixed step count
  // `decim` and emit one row per `decim` integration sub-steps. Pass 0 to
  // disable. The row is the raw plant state after that sub-step.
  struct Trace {
    double t;         // simulated time, s
    double x;         // rotor position, steps
    double x_c;       // commanded position, steps
    double delta;     // x - x_c, steps
    double w;         // rotor speed, steps/s
    double a;         // rotor acceleration, steps/s^2 (raw, to the rotor)
    double tau;       // magnetic force -Fmax*sin(pi*delta/D), N*m
    double friction;  // friction force (signed against motion), N*m
    int stall;        // 1 while the rotor is slipping (§5.5)
    int phase;        // 0=accel 1=coast 2=decel 3=pause (test-labelled)
  };

  void trace_begin(unsigned long decim) {
    trace_.clear();
    trace_decim_ = decim ? decim : 1;
    trace_n_ = 0;
    trace_phase_ = 0;
  }

  void trace_set_phase(int phase) { trace_phase_ = phase; }

  // Write the accumulated trace as a whitespace-delimited gnuplot input:
  //   t x x_c delta w a tau friction stall phase
  // Returns the number of rows written (0 if the trace is empty).
  size_t trace_dump(const char* path) {
    FILE* fp = std::fopen(path, "w");
    if (!fp) return 0;
    for (size_t i = 0; i < trace_.size(); i++) {
      const Trace& r = trace_[i];
      std::fprintf(fp, "%.6f %g %g %g %g %g %g %g %d %d\n", r.t, r.x, r.x_c,
                   r.delta, r.w, r.a, r.tau, r.friction, r.stall, r.phase);
    }
    std::fclose(fp);
    return trace_.size();
  }

  size_t trace_rows() const { return trace_.size(); }

  // Raw state for the gnuplot traces.
  double x() const { return state_.x; }
  double x_c() const { return state_.x_c; }
  double delta() const { return state_.x - state_.x_c; }
  double speed() const { return state_.w; }

  // Acoustic emission (see whitepaper §5.8): a hybrid source driven by the
  // actual rotor motion and gated by the magnetic force curve, plus a
  // displacement (acceleration) term. While |delta| >= D the motor has run
  // past its command (the stall observation of §5.5), so it buzzes like a
  // real motor drawing holding current — so the recording never goes silent
  // on a stall.
  // The plant records one 44.1 kHz sample per
  // output interval while it steps, so to_wav() renders the *whole* simulated
  // move (e.g. the 5 s canonical trapezoid -> ~441000 bytes of PCM) rather
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

  // Raw recorded PCM, so a test can mix several axes into one stereo wav.
  // Out-of-range reads are silence.
  uint32_t audio_sample_count() const { return (uint32_t)audio_.size(); }
  int16_t audio_sample(uint32_t i) const {
    return i < audio_.size() ? audio_[i] : (int16_t)0;
  }

 private:
  // Fixed audio sample rate for the recording grid.
  enum { kAudioSr = 44100 };
  // Largest physics sub-step (s). The commanded period can reach 65535 ticks
  // (4.1 ms), which is too coarse for the ~63 ms detent; sub-step below this.
  static constexpr double kMaxDt = 2.5e-4;
  // Stall slip detector: the rotor/field speed mismatch is low-passed with time
  // constant kSlipTau (s); a sustained mismatch above kSlipThreshold (steps/s)
  // is a stall, while sub-threshold ring or a direction-change transient is
  // not.
  static constexpr double kSlipTau = 0.05;
  static constexpr double kSlipThreshold = 100.0;
  // Sum of the humming-partial amplitudes in record_audio (normalization).
  static constexpr double kHarmSum = 2.28;
  // Speed at which the audio envelope is at half amplitude (steps/s): below
  // this the hum fades toward silence, so a near-rest rotor does not rumble.
  static constexpr double kAudioW0 = 400.0;

  // Update the dynamic stall observation from the current rotor/field speed
  // mismatch. The slip is low-pass filtered (time constant kSlipTau) so a
  // commanded direction change or an accel transient does not read as a stall.
  // Stalled = a full step or more behind AND still slipping; it clears once the
  // rotor re-captures (w -> v_field), e.g. when the field slows (§5.5).
  void update_stall(double delta, double v_field, double dt) {
    const double slip = state_.w - v_field;
    slip_f_ += (slip - slip_f_) * (dt / (kSlipTau + dt));
    last_.stall =
        (fabs(delta) > (double)D_) && (fabs(slip_f_) > kSlipThreshold);
    if (last_.stall) stall_ever_ = true;
  }

  // Append one decimated row to the gnuplot trace (if enabled).
  void record_trace(double delta, double a, double tau) {
    if (trace_decim_ == 0) return;
    if ((trace_n_++ % trace_decim_) != 0) return;
    Trace r;
    r.t = sim_time_;
    r.x = state_.x;
    r.x_c = state_.x_c;
    r.delta = delta;
    r.w = state_.w;
    r.a = a;
    r.tau = tau;
    r.friction = friction_force(state_.w);
    r.stall = last_.stall ? 1 : 0;
    r.phase = trace_phase_;
    trace_.push_back(r);
  }

  // Cache the hybrid source parameters after each integration sub-step.
  void update_audio_state(double delta, double a, double tau) {
    // Hum pitch is the full-step (electrical) rate, |w|/D Hz — the rate the
    // rotor's magnetic field actually turns — not the finest step rate.
    cur_f_ = fabs(state_.w) / (double)D_;
    cur_gate_ = (Fmax_ > 0.0) ? fabs(tau) / Fmax_ : 0.0;
    cur_disp_ = (Fmax_ > 0.0) ? (a * J_) / Fmax_ : 0.0;
    // The stall observation (§5.5): |delta| has drifted a full step. There is
    // no “grip lost” branch — the force runs for any delta, and the motor
    // simply cannot keep up, so |delta| grows past D.
    cur_stalled_ = last_.stall ? 1.0 : 0.0;
    // Speed envelope. The hum is gated by the magnetic force, which is *large*
    // at low speed (the rotor lags most while accelerating) — so without this a
    // move fades in and out of a loud low-frequency rumble at the start and end
    // of every ramp ("strange noise"). Fade the source with rotor speed so a
    // near-rest rotor is silent and the tone rises/falls with it.
    const double w = fabs(state_.w);
    cur_env_ = w / (w + kAudioW0);
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
      // Normalize the harmonic sum (Σamp = 2.28) so the hum no longer clips for
      // most of a move; the ×3 lift keeps it out of the int16 rounding floor.
      s = cur_gate_ * (s / kHarmSum) + 0.1 * cur_disp_;
      // Fade the hum with rotor speed: a near-rest rotor is silent, so a ramp
      // does not fade in/out of a loud low-frequency rumble.
      s *= cur_env_;
      // A stalled rotor (§5.5 — |delta| has drifted a full step) draws a lot
      // of holding current and buzzes, so a low-frequency buzz rides on top of
      // the hum. The recording therefore never goes silent on a stall.
      if (cur_stalled_) {
        double buzz = 1.0 + 0.3 * sin(2.0 * M_PI * 120.0 * audio_written_ /
                                      (double)kAudioSr);
        s += 0.4 * buzz;
      }
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
  } state_;                  // x (rotor), x_c (cmd), w
  uint8_t D_;                // steps per full step — the force-curve period
  double J_;                 // rotor moment of inertia
  double Fmax_;              // peak magnetic torque
  double friction_static_;   // Coulomb friction floor (absolute)
  double friction_viscous_;  // friction increase per step/s (absolute)
  uint32_t ticks_;           // accumulated wall clock
  double sim_time_;          // accumulated simulated time (s)
  double peak_abs_delta_;    // peak |x - x_c| since reset
  double slip_f_;            // low-passed rotor/field speed mismatch
  bool stall_ever_;          // slipped at any time since reset
  double cur_f_, cur_gate_, cur_disp_, cur_stalled_;  // audio source
  double cur_env_;                                    // speed envelope [0:1]
  double phase_;                // continuous hum phase (rad)
  std::vector<int16_t> audio_;  // recorded PCM samples
  uint32_t audio_written_;      // samples already emitted
  std::vector<Trace> trace_;    // decimated gnuplot rows
  unsigned long trace_decim_;   // emit 1 row per this many sub-steps
  unsigned long trace_n_;       // sub-steps seen since trace_begin
  int trace_phase_;             // test-labelled phase tag
  observed_s last_;
};

#else  // !FAS_PHYSICAL_STEPPER_ENABLED — inert stub, keeps the ideal stepper

// Placeholder so callers compile unchanged when the plant is not opted in.
class PhysicalStepper {
 public:
  PhysicalStepper(uint8_t = 64, double = 0, double = 0, double = 0,
                  double = 0) {
    reset();
  }
  void reset() {}
  observed_s step(int, bool, uint16_t) { return zero(); }
  struct Cmd {
    int steps;
    bool count_up;
    uint16_t ticks;
  };
  observed_s run(const Cmd*, int) { return zero(); }
  int32_t getCurrentPosition() const { return 0; }
  uint32_t total_ticks() const { return 0; }
  double peak_abs_delta() const { return 0; }
  bool stall_ever() const { return false; }
  double friction_force(double) const { return 0; }
  double magnetic_torque(double) const { return 0; }
  void rotate(double) {}
  void plot_row(FILE*, uint8_t, const observed_s&) {}
  const observed_s& last() const { return zero(); }
  double x() const { return 0; }
  double x_c() const { return 0; }
  double delta() const { return 0; }
  double speed() const { return 0; }
  void trace_begin(unsigned long) {}
  void trace_set_phase(int) {}
  size_t trace_dump(const char*) { return 0; }
  size_t trace_rows() const { return 0; }
  bool to_wav(const char*, uint32_t) { return false; }
  uint32_t audio_samples() const { return 0; }
  uint32_t audio_sample_count() const { return 0; }
  int16_t audio_sample(uint32_t) const { return 0; }

 private:
  static observed_s& zero() {
    static observed_s s = {0, 0, 0, 0, 0, 0, 0, false};
    return s;
  }
};

#endif  // FAS_PHYSICAL_STEPPER_ENABLED
#endif  // FAS_PHYSICAL_STEPPER_H
