// test_28: NEMA-17 / A4988 pull-out benchmark — fitting the plant to hardware
//
// The rotordynamic plant (physical_stepper.h, whitepaper
// extras/doc/physical_stepper_whitepaper.md) was originally parameterised by
// hand as a generic "reference" motor. This test does the opposite: it takes
// the measured pull-out data of a real NEMA-17 driven by an A4988 on an ESP32
// (whitepaper §10.4) and *finds* the plant parameters that reproduce it.
//
// Hardware (see §10.4):
//   200 full steps/rev (1.8 deg), A4988 set to 16x microstepping
//   => 3200 microsteps/rev, D = 16 microsteps per full step
//   ~19 V power supply, ESP32 MCPWM/PCNT, FastAccelStepper stepperdemo
//
// The demo reports `getCurrentSpeedInMilliHz()`; the sweep values in the
// whitepaper are in millisteps/s, so microsteps/s = value / 1000.
//
// Measured pull-out boundary (microsteps/s), by commanded acceleration
// (microsteps/s^2):
//
//   A = 1e4 : ~134 000   (133.33 M works, 134.45 M stalls)
//   A = 1e5 : ~118 000   (111.11 M works, 125.00 M stalls)
//   A = 1e6 : ~123 000   (122.14 M works, 124.03 M stalls)
//   A = 1e7 : ~ 21 000   (47 us/step works, 48 us/step stalls)
//
// The first three are the steady-state pull-out (friction reaches the drive
// torque); the last is dominated by the commanded acceleration exceeding
// Fmax/J_eff, which is why the plant can reproduce it at all.
//
// Two parameters are found:
//   * friction_viscous, so the A = 1e6 pull-out matches ~123 000 usteps/s
//     (the steady-state friction balance v_max = (Fmax - fs)/fv), and
//   * the effective plant inertia J_plant, so the A = 1e7 pull-out matches
//     ~21 000 usteps/s (the acceleration-capability limit Fmax/J_eff).
//
// The plant's coordinate is the microstep; its inertia is therefore the
// mechanical inertance in microstep units, J_plant = J_eff * theta_u, where
// theta_u = 1.8 deg / 16 is the angle of one microstep.
//
// Opt-in gate as usual.

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TICKS_PER_S 16000000L
#define FAS_PHYSICAL_STEPPER_ENABLED 1
#include "physical_stepper.h"

// ---- hardware constants ----------------------------------------------------
static const uint8_t D = 16;  // microsteps per full step (16x)
static const double STEPS_PER_REV = 200.0;
static const double USTEPS_PER_REV = 3200.0;
static const double THETA_U = M_PI / 100.0 / 16.0;  // rad per microstep
static const double FMAX = 0.4;          // N*m, datasheet-like holding torque
static const double FRIC_STATIC = 0.04;  // N*m, Coulomb floor (~10% Fmax)
// Typical bare NEMA-17 rotor inertia. The motor was unloaded (sitting on the
// table), so this is the mechanical inertia to use when reading the fit's
// degeneracy: only Fmax/J_plant is constrained, and J_plant = J*THETA_U.
static const double J_ROTOR = 5.4e-6;  // kg*m^2, bare rotor (typical)

// Fitted parameters, shared with the trace/wav/gnuplot renderers below.
static double g_J_plant = 0.0;
static double g_fv = 0.0;

// Measured pull-out boundary per commanded acceleration.
struct meas_s {
  double accel;    // microsteps/s^2
  double pullout;  // microsteps/s
  const char* why;
};
static const meas_s MEASURED[] = {
    {1.0e4, 134000.0, "steady-state friction balance"},
    {1.0e5, 118000.0, "steady-state friction balance"},
    {1.0e6, 123000.0, "steady-state friction balance"},
    {1.0e7, 21000.0, "acceleration-capability limit (Fmax/J)"},
};
static const int NMEAS = (int)(sizeof(MEASURED) / sizeof(MEASURED[0]));

// The calibration targets used by find_motor_parameters().
static const double FIT_ACCEL_FRICTION = 1.0e6;  // steady pull-out anchor
static const double FIT_PULLOUT_FRICTION = 123000.0;
static const double FIT_ACCEL_INERTIA = 1.0e7;  // accel-limit anchor
static const double FIT_PULLOUT_INERTIA = 21000.0;

// Simulation coast (s) held at top speed before the stop ramp; long enough
// for a slipping rotor to grow |delta| past D.
static const double COAST_S = 0.30;

static int failures = 0;
static void check(int cond, const char* msg) {
  if (!cond) {
    printf("  FAIL: %s\n", msg);
    failures++;
  } else {
    printf("  ok: %s\n", msg);
  }
}

// ---- step-train helpers (same vocabulary as test_27) -----------------------

// Ticks per step for a given speed (microsteps/s), clamped into uint16.
static uint16_t ticks_for_v(double v) {
  double t = (double)TICKS_PER_S / v;
  if (t > 65535.0) t = 65535.0;
  if (t < 1.0) t = 1.0;
  return (uint16_t)llround(t);
}

// One step whose following period may exceed uint16: emit the step, then
// pause(s), exactly as the ramp generator does.
static void issue_single_step(PhysicalStepper& p, bool up, uint32_t period) {
  bool first = true;
  while (period > 65535) {
    if (first) {
      p.step(1, up, 65535);
      first = false;
    } else {
      p.step(0, up, 65535);
    }
    period -= 65535;
  }
  if (first) {
    p.step(1, up, (uint16_t)period);
  } else if (period > 0) {
    p.step(0, up, (uint16_t)period);
  }
}

// Per-step periods of an acceleration from v0 to v1 at rate `a`.
static std::vector<uint32_t> build_ramp(double v0, double v1, double a) {
  std::vector<uint32_t> seg;
  const int n = (int)llround((v1 * v1 - v0 * v0) / (2.0 * a));
  double t_prev = 0.0;
  for (int i = 1; i <= n; i++) {
    double ti = (-v0 + sqrt(v0 * v0 + 2.0 * a * (double)i)) / a;
    uint32_t tk = (uint32_t)llround((ti - t_prev) * (double)TICKS_PER_S);
    if (tk < 1) tk = 1;
    seg.push_back(tk);
    t_prev = ti;
  }
  return seg;
}

// Release from standstill: accelerate 0 -> V at A, then run forward at V for
// COAST_S. Returns whether the plant ever observed a slip. This mirrors the
// hardware protocol exactly (standstill, run forward).
static bool simulate_move(PhysicalStepper& p, double accel, double v) {
  p.reset();
  const std::vector<uint32_t> up = build_ramp(0.0, v, accel);
  for (size_t i = 0; i < up.size(); i++) {
    if (p.stall_ever()) break;
    issue_single_step(p, true, up[i]);
  }
  // Coast at top speed. The hardware measurement simply released the motor
  // from standstill and let it run forward, so there is deliberately no stop
  // ramp here: the slip flag latches the moment the rotor falls behind.
  const uint32_t ct = ticks_for_v(v);
  const int n = (int)llround(COAST_S * v);
  for (int i = 0; i < n; i++) {
    if (p.stall_ever()) break;
    p.step(1, true, ct);
  }
  return p.stall_ever();
}

// Simulated pull-out boundary: the speed at which a move at acceleration
// `accel` flips from tracking to slipping.
static double boundary(PhysicalStepper& p, double accel) {
  double lo = 1.0e3, hi = 4.0e5;
  for (int i = 0; i < 16; i++) {
    double mid = 0.5 * (lo + hi);
    if (simulate_move(p, accel, mid)) {
      hi = mid;
    } else {
      lo = mid;
    }
  }
  return 0.5 * (lo + hi);
}

// ---- parameter search ------------------------------------------------------
//
// The plant's steady pull-out is v_max = (Fmax - fs)/fv, so friction_viscous
// sets the low-acceleration boundary. The acceleration-capability limit is
// Fmax / J_plant, so the plant inertia sets the high-acceleration collapse.
// Both dependencies are monotonic, so a bisection on each converges.

// Find friction_viscous so the A = 1e6 pull-out equals `target`.
static double fit_friction_viscous(double J_plant, double target) {
  double lo = 1.0e-7, hi = 1.0e-4;
  for (int i = 0; i < 22; i++) {
    double fv = 0.5 * (lo + hi);
    PhysicalStepper p(D, J_plant, FMAX, FRIC_STATIC, fv);
    if (boundary(p, FIT_ACCEL_FRICTION) > target) {
      lo = fv;  // too little friction -> boundary too high
    } else {
      hi = fv;
    }
  }
  return 0.5 * (lo + hi);
}

// Find the plant inertia so the A = 1e7 pull-out equals `target`.
static double fit_inertia(double fv, double target) {
  double lo = 1.0e-9, hi = 1.0e-6;
  for (int i = 0; i < 22; i++) {
    double J = 0.5 * (lo + hi);
    PhysicalStepper p(D, J, FMAX, FRIC_STATIC, fv);
    if (boundary(p, FIT_ACCEL_INERTIA) > target) {
      lo = J;  // too little inertia -> accel limit too high
    } else {
      hi = J;
    }
  }
  return 0.5 * (lo + hi);
}

// ---- artifact rendering (traces, wav, gnuplot) -----------------------------

static long file_size(const char* path) {
  FILE* f = fopen(path, "rb");
  if (!f) return -1;
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fclose(f);
  return n;
}

// Peak absolute PCM sample in a 16-bit mono .wav (0 if silent/absent).
static int wav_peak(const char* path) {
  FILE* f = fopen(path, "rb");
  if (!f) return 0;
  char hdr[44];
  if (fread(hdr, 1, 44, f) != 44) {
    fclose(f);
    return 0;
  }
  int peak = 0;
  int16_t s;
  while (fread(&s, 2, 1, f) == 1) {
    int v = s < 0 ? -s : s;
    if (v > peak) peak = v;
  }
  fclose(f);
  return peak;
}

// Release from standstill at `accel` up to `v`, run forward for COAST_S, while
// recording the plant's six-panel trace and its acoustic stream. The whole
// coast is recorded even after the rotor slips, so the divergence is visible
// in the trace and audible in the wav. Writes the trace to `dat` (if non-null)
// and the wav to `wav` (if non-null). Returns whether the rotor slipped.
static bool trace_scenario(double accel, double v, unsigned long decim,
                           const char* dat, const char* wav) {
  PhysicalStepper p(D, g_J_plant, FMAX, FRIC_STATIC, g_fv);
  p.trace_begin(decim);
  const std::vector<uint32_t> up = build_ramp(0.0, v, accel);
  p.trace_set_phase(0);  // accel
  for (size_t i = 0; i < up.size(); i++) {
    issue_single_step(p, true, up[i]);
  }
  const uint32_t ct = ticks_for_v(v);
  const int n = (int)llround(COAST_S * v);
  p.trace_set_phase(1);  // coast
  for (int i = 0; i < n; i++) {
    p.step(1, true, ct);
  }
  if (dat) p.trace_dump(dat);
  if (wav) p.to_wav(wav);
  return p.stall_ever();
}

// Boundary-vs-acceleration table for the plot: A, sim, measured, sim fps.
static void write_boundary_dat() {
  FILE* f = fopen("test_28_boundary.dat", "w");
  if (!f) return;
  PhysicalStepper p(D, g_J_plant, FMAX, FRIC_STATIC, g_fv);
  for (int i = 0; i < NMEAS; i++) {
    double b = boundary(p, MEASURED[i].accel);
    fprintf(f, "%g %g %g %g\n", MEASURED[i].accel, b, MEASURED[i].pullout,
            b / D);
  }
  fclose(f);
}

// The six-panel figure: the three scenarios overlaid (position, error, speed,
// raw rotor accel, magnetic force) plus the simulated-vs-measured pull-out
// boundary. Dat columns: 1:t 2:x 3:x_c 4:delta 5:w 6:a 7:tau 8:friction
// 9:stall 10:phase.
static void write_gnuplot_asset() {
  FILE* g = fopen("test_28.gnuplot", "w");
  if (!g) return;
  fprintf(g,
          "set term pngcairo size 1600,1400\n"
          "set output \"test_28.png\"\n"
          "set multiplot layout 3,2 title "
          "\"NEMA-17/A4988: fitted plant (D=16, 16x) - run vs stall vs "
          "high-accel collapse\"\n"
          "D=16.0\n"
          "set xlabel \"t [s]\"\n");
  // 1: position
  fprintf(g,
          "set title \"position [usteps] (x solid, x_c dashed)\"\n"
          "plot \"test_28_run.dat\" u 1:2 w l t 'X run x', "
          "\"test_28_run.dat\" u 1:3 w l dt 2 t 'X run x_c', "
          "\"test_28_stall.dat\" u 1:2 w l t 'S stall x', "
          "\"test_28_stall.dat\" u 1:3 w l dt 2 t 'S stall x_c', "
          "\"test_28_collapse.dat\" u 1:2 w l t 'C collapse x', "
          "\"test_28_collapse.dat\" u 1:3 w l dt 2 t 'C collapse x_c'\n");
  // 2: error
  fprintf(g,
          "set title \"step error delta [usteps]\"\n"
          "plot \"test_28_run.dat\" u 1:4 w l t 'run', "
          "\"test_28_stall.dat\" u 1:4 w l t 'stall', "
          "\"test_28_collapse.dat\" u 1:4 w l t 'collapse', "
          "D lt 0 title '+D', -D lt 0 notitle\n");
  // 3: speed
  fprintf(g,
          "set title \"rotor speed [usteps/s]\"\n"
          "plot \"test_28_run.dat\" u 1:5 w l t 'run', "
          "\"test_28_stall.dat\" u 1:5 w l t 'stall', "
          "\"test_28_collapse.dat\" u 1:5 w l t 'collapse'\n");
  // 4: accel
  fprintf(g,
          "set title \"raw rotor acceleration [usteps/s^2]\"\n"
          "plot \"test_28_run.dat\" u 1:6 w l t 'run', "
          "\"test_28_stall.dat\" u 1:6 w l t 'stall', "
          "\"test_28_collapse.dat\" u 1:6 w l t 'collapse'\n");
  // 5: force
  fprintf(g,
          "set title \"magnetic force tau and friction\"\n"
          "plot \"test_28_stall.dat\" u 1:7 w l t 'stall tau', "
          "\"test_28_stall.dat\" u 1:8 w l t 'stall friction', "
          "\"test_28_collapse.dat\" u 1:7 w l t 'collapse tau'\n");
  // 6: boundary sweep
  fprintf(g,
          "set title \"pull-out boundary vs acceleration\"\n"
          "set logscale x\n"
          "set xlabel \"acceleration [usteps/s^2]\"\n"
          "set ylabel \"pull-out [usteps/s]\"\n"
          "plot \"test_28_boundary.dat\" u 1:2 w lp pt 7 t 'simulated', "
          "\"test_28_boundary.dat\" u 1:3 w lp pt 5 t 'measured'\n"
          "unset logscale x\n"
          "unset ylabel\n"
          "unset multiplot\n"
          "unset output\n");
  fclose(g);
}

int main() {
  printf("=== test_28: NEMA-17 / A4988 pull-out benchmark ===\n");
  printf("D=%u usteps/full-step, %g full-steps/rev, %g usteps/rev\n", D,
         STEPS_PER_REV, USTEPS_PER_REV);

  // ---- 1. find the motor parameters from the measured boundary ------------
  printf("\n=== P1: fit plant parameters to the measured data ===\n");
  // First the inertia (needs a friction estimate to make the accel-limit
  // boundary meaningful); then friction; then a second inertia pass so the
  // two constraints settle.
  double fv = fit_friction_viscous(3.0e-8, FIT_PULLOUT_FRICTION);
  double J_plant = fit_inertia(fv, FIT_PULLOUT_INERTIA);
  fv = fit_friction_viscous(J_plant, FIT_PULLOUT_FRICTION);
  J_plant = fit_inertia(fv, FIT_PULLOUT_INERTIA);

  const double J_eff = J_plant / THETA_U;  // mechanical inertia (kg m^2)
  const double v_max = (FMAX - FRIC_STATIC) / fv;
  printf("  found: J_plant=%.4g  J_eff=%.4g kg m^2 (%.1f g cm^2)\n", J_plant,
         J_eff, J_eff * 1.0e7);
  printf("         friction_viscous=%.4g N m s  friction_static=%.4g N m\n", fv,
         FRIC_STATIC);
  printf("         v_max=(Fmax-fs)/fv=%.0f usteps/s\n", v_max);

  // The fit pins only Fmax/J_plant. Because the motor was UNLOADED, the
  // physical reading is: anchor the inertia at the bare rotor and solve for
  // the effective torque (much lower than the nominal Fmax).
  const double J_bare_plant = J_ROTOR * THETA_U;
  const double Fmax_eff = FMAX * J_bare_plant / J_plant;
  printf(
      "  unloaded-rotor equivalent: J_plant=%.4g (%.1f g cm^2) => "
      "Fmax_eff=%.4g N m\n",
      J_bare_plant, J_ROTOR * 1.0e7, Fmax_eff);
  printf(
      "  (only Fmax/J is determined; the nominal Fmax anchor gives the "
      "heavier J above)\n");

  check(J_plant > 0.0 && J_plant < 1.0e-6, "plant inertia in a sane range");
  check(Fmax_eff > 0.0 && Fmax_eff < FMAX,
        "bare-rotor anchor implies a lower effective torque");
  check(fv > 0.0, "friction_viscous positive");
  check(fabs(v_max - FIT_PULLOUT_FRICTION) / FIT_PULLOUT_FRICTION < 0.20,
        "v_max within 20% of the measured steady pull-out");

  PhysicalStepper plant(D, J_plant, FMAX, FRIC_STATIC, fv);

  // ---- 2. reproduce every measured boundary ------------------------------
  printf("\n=== P2: simulated vs measured pull-out boundary ===\n");
  printf("  %-10s %-12s %-12s %-8s %-8s\n", "accel", "sim(ustep/s)",
         "meas(ustep/s)", "ratio", "sim fps");
  for (int i = 0; i < NMEAS; i++) {
    double b = boundary(plant, MEASURED[i].accel);
    double ratio = b / MEASURED[i].pullout;
    printf("  %-10.0e %-12.0f %-12.0f %-8.3f %-8.0f\n", MEASURED[i].accel, b,
           MEASURED[i].pullout, ratio, b / D);
    // The two high-acceleration / steady-state regimes are fit directly; the
    // A = 1e5 point carries only 2-point granularity (111k runs, 125k stalls).
    double tol = (MEASURED[i].accel >= 1.0e7) ? 0.35 : 0.20;
    char msg[96];
    snprintf(msg, sizeof(msg), "A=%.0e boundary within %d%% of measured",
             MEASURED[i].accel, (int)(tol * 100));
    check(fabs(ratio - 1.0) < tol, msg);
  }

  // ---- 3. the high-acceleration collapse is captured ---------------------
  printf("\n=== P3: high acceleration collapses the usable speed ===\n");
  double b_1e6 = boundary(plant, 1.0e6);
  double b_1e7 = boundary(plant, 1.0e7);
  printf(
      "  A=1e6 pull-out=%.0f usteps/s (%.0f fps), A=1e7=%.0f usteps/s "
      "(%.0f fps)\n",
      b_1e6, b_1e6 / D, b_1e7, b_1e7 / D);
  check(b_1e7 < 0.45 * b_1e6,
        "A=1e7 pull-out is <45% of the A=1e6 pull-out (measured ~6x drop)");

  // ---- 4. concrete scenario classification -------------------------------
  printf("\n=== P4: scenario runs/stalls at the measured points ===\n");
  // (accel, speed, expect_run, label)
  struct scen_s {
    double accel, speed;
    bool run;
    const char* label;
  };
  const scen_s runs[] = {
      {1.0e6, 110000.0, true, "A=1e6 @110k runs"},
      {1.0e6, 140000.0, false, "A=1e6 @140k stalls"},
      {1.0e7, 10000.0, true, "A=1e7 @10k runs"},
      {1.0e7, 45000.0, false, "A=1e7 @45k stalls"},
      {1.0e4, 120000.0, true, "A=1e4 @120k runs"},
  };
  for (size_t i = 0; i < sizeof(runs) / sizeof(runs[0]); i++) {
    plant.reset();
    bool stalled = simulate_move(plant, runs[i].accel, runs[i].speed);
    printf("  %-22s -> %s (delta=%.1f usteps)\n", runs[i].label,
           stalled ? "STALL" : "run", plant.delta());
    check(stalled != runs[i].run, runs[i].label);
  }

  // ---- 5. full-step / RPM conversion sanity ------------------------------
  printf("\n=== P5: unit conversions ===\n");
  printf("  123000 usteps/s = %.0f full-steps/s = %.0f RPM\n", 123000.0 / D,
         123000.0 / D * 60.0 / STEPS_PER_REV);
  check(fabs(123000.0 / D - 7687.5) < 1.0,
        "123k usteps/s is ~7688 full-steps/s");
  check(fabs(123000.0 / D * 60.0 / STEPS_PER_REV - 2306.25) < 1.0,
        "123k usteps/s is ~2306 RPM");

  // ---- 6. physical artifacts: traces, wav, gnuplot -----------------------
  printf("\n=== P6: trace + wav + gnuplot artifacts ===\n");
  g_J_plant = J_plant;
  g_fv = fv;
  bool run_stall =
      trace_scenario(1.0e6, 110000.0, 16, "test_28_run.dat", "test_28_run.wav");
  bool mid_stall = trace_scenario(1.0e6, 140000.0, 16, "test_28_stall.dat",
                                  "test_28_stall.wav");
  bool acc_stall = trace_scenario(1.0e7, 45000.0, 16, "test_28_collapse.dat",
                                  "test_28_collapse.wav");
  check(!run_stall, "the run scenario trace does not stall");
  check(mid_stall, "the steady-state stall scenario trace stalls");
  check(acc_stall, "the high-acceleration collapse trace stalls");
  write_boundary_dat();
  write_gnuplot_asset();

  const char* dats[] = {"test_28_run.dat", "test_28_stall.dat",
                        "test_28_collapse.dat", "test_28_boundary.dat",
                        "test_28.gnuplot"};
  for (size_t i = 0; i < sizeof(dats) / sizeof(dats[0]); i++) {
    check(file_size(dats[i]) > 0, dats[i]);
  }
  const char* wavs[] = {"test_28_run.wav", "test_28_stall.wav",
                        "test_28_collapse.wav"};
  for (size_t i = 0; i < sizeof(wavs) / sizeof(wavs[0]); i++) {
    long sz = file_size(wavs[i]);
    char msg[64];
    snprintf(msg, sizeof(msg), "%s is a non-empty PCM wav", wavs[i]);
    check(sz > 44 && wav_peak(wavs[i]) > 0, msg);
  }

  printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures,
         failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
