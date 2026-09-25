// test_27: physical_stepper — a rotordynamic plant test drive
//
// This is the "test drive" for the opt-in PhysicalStepper plant (see
// physical_stepper.h and extras/doc/physical_stepper_whitepaper.md). It drives
// the plant with the exact step/direction/ticks vocabulary a real queue hands
// a driver and asserts that a real motor's failure modes show up:
//
//   * it follows a rest-to-rest profile without stalling,
//   * it lags during acceleration and runs ahead during deceleration,
//   * a half-step rotor slip mid-coast is the decisive failure mode: the rotor
//     is parked on the detent separatrix and the field out-runs it, so |delta|
//     runs away — a stall induced while coasting (§5.5),
//   * while a full-step slip parks the rotor on the next detent and it only
//     loses exactly one step,
//   * and — the contract this test exists for — after the stall the profile
//     decelerates to a *small* speed the rotor can follow again: it
//     re-captures and tracks the field there, then the final decel brings both
//     to standstill.
//
// The shared profile (identical for the reference and the stall case):
//
//   accelerate 0 -> 2000 step/s     in 0.5 s
//   coast 2000 step/s               for 1 s
//   accelerate 2000 -> 10000 step/s in 0.5 s
//   coast 10000 step/s              for 1 s
//   [stall case only: half/full-step rotor slip]
//   coast 10000 step/s              for 1 s
//   decelerate 10000 -> 2000 step/s (at 10000 step/s^2)
//   coast 2000 step/s               for 3 s   <- the stalled rotor follows here
//   decelerate 2000 -> 0 step/s
//
// Total 8 s. Both runs are dumped as gnuplot traces (position, error, speed,
// raw rotor acceleration, force/friction, stall).
//
// The plant is opt-in: define FAS_PHYSICAL_STEPPER_ENABLED before including the
// header. The build here always does; every other test keeps the ideal stepper.

#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// <vector> must precede FastAccelStepper.h: the PC test shims define a
// function-like `test` macro that collides with libc++'s <atomic>.
#include <vector>

#include "FastAccelStepper.h"
#include "fas_queue/stepper_queue.h"

#define FAS_PHYSICAL_STEPPER_ENABLED 1
#include "physical_stepper.h"

char TCCR1A;
char TCCR1B;
char TCCR1C;
char TIMSK1;
char TIFR1;
unsigned short OCR1A;
unsigned short OCR1B;

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

// The canonical plant: 64 steps per full step, light inertia, and enough peak
// torque (Fmax) to follow the 10000 step/s ramp. Friction is an independent
// load (Coulomb floor 0.1, 2.25e-5 per step/s) whose pull-out is ~40000 step/s.
// With Fmax = 1.0 the natural frequency is ~50 Hz (a NEMA-17-like mid-band).
static PhysicalStepper g_plant{64, 1.0e-6, 1.0, 0.1, 2.25e-5};

// A dedicated plant for the traced coast-stall run (the trace is per instance),
// with a heavier load (friction reaches Fmax at ~11500 step/s).
static PhysicalStepper g_stall{64, 1.0e-6, 1.0, 0.1, 7.83e-5};

// A deliberately weak motor: peak accel Fmax/J = 2000 step/s^2, which cannot
// follow a 10000 step/s^2 ramp — it must lose grip.
static PhysicalStepper g_weak{64, 1.0e-6, 2.0e-3, 2.0e-4, 2.25e-6};

// Long enough for the rotor to ring down.
static const uint32_t SETTLE_TICKS = 12u * 16000000u;

// The shared move profile.
static const double VMAX = 10000.0;
static const double VSMALL = 2000.0;             // speed the stall can re-join
static const double ACCEL = 10000.0;             // deceleration rate
static const double T_SPIN1 = 0.5;               // 0 -> vsmall
static const double T_SPIN_HOLD = 1.0;           // hold vsmall
static const double T_SPIN2 = 0.5;               // vsmall -> vmax
static const double A_SPIN1 = VSMALL / T_SPIN1;  // 4000
static const double A_SPIN2 = (VMAX - VSMALL) / T_SPIN2;  // 16000
static const double T_COAST = 1.0;
static const double T_COAST_SMALL = 3.0;  // hold the slow speed for 3 s
static const double T_TOTAL = T_SPIN1 + T_SPIN_HOLD + T_SPIN2 + 2.0 * T_COAST +
                              (VMAX - VSMALL) / ACCEL + T_COAST_SMALL +
                              VSMALL / ACCEL;  // = 7.0 s

// Test-labelled trace phases (column 10 of the .dat).
enum { PH_ACCEL = 0, PH_COAST = 1, PH_DECEL = 2, PH_SLIP = 3 };

static int failures = 0;
static void check(int cond, const char* msg) {
  if (!cond) {
    printf("  FAIL: %s\n", msg);
    failures++;
  } else {
    printf("  ok: %s\n", msg);
  }
}

// Ticks per step for a given speed (steps/s), clamped into uint16.
static uint16_t ticks_for_v(double v) {
  double t = (double)TICKS_PER_S / v;
  if (t > 65535.0) t = 65535.0;
  if (t < 1.0) t = 1.0;
  return (uint16_t)llround(t);
}

// A pause longer than uint16, split into chunks the command can carry.
static void dwell(PhysicalStepper& p, uint32_t ticks) {
  while (ticks > 65535) {
    p.step(0, true, 65535);
    ticks -= 65535;
  }
  if (ticks > 0) p.step(0, true, (uint16_t)ticks);
}

// Emit a single physical step whose following dwell is `period` ticks, split
// into a step command plus pause(s) exactly as the ramp generator does when
// the period exceeds uint16.
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

// Per-step periods of an acceleration from v0 to v1 at rate `a`. Step i is
// reached at t_i = (−v0 + sqrt(v0² + 2·a·i))/a, so the period is t_i − t_{i−1}.
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

static void issue_ramp(PhysicalStepper& p, const std::vector<uint32_t>& seg,
                       bool up) {
  for (size_t i = 0; i < seg.size(); i++) issue_single_step(p, up, seg[i]);
}

// Probes captured during the shared profile, for assertions.
struct move_probe {
  double delta_stalled;  // rotor/field error at the end of the stall coast
  double w_stalled;
  int stall;
  double x_small_start;  // rotor position at the start of the small-speed coast
  double x_small_end;    // ... and at its end
  double w_small_end;    // rotor speed at the end of the small-speed coast
};

// Run the full rest-to-rest profile:
//   0 -> 2000 step/s (0.5 s), hold 2000 (1 s), 2000 -> 10000 (0.5 s),
//   coast 10000 (1 s), [slip], coast 10000 (1 s),
//   decelerate 10000 -> 2000, hold 2000 (3 s), decelerate 2000 -> 0.
// `slip_steps` is a one-shot rotor displacement applied after the first coast
// (0 = reference; -D/2 = half-step slip that stalls; -D = full-step slip).
static double run_move(PhysicalStepper& p, double slip_steps, bool trace,
                       move_probe* probe) {
  const std::vector<uint32_t> spin1 = build_ramp(0.0, VSMALL, A_SPIN1);
  const std::vector<uint32_t> spin2 = build_ramp(VSMALL, VMAX, A_SPIN2);
  const std::vector<uint32_t> dec = build_ramp(0.0, VMAX, ACCEL);
  const int nd = (int)dec.size();
  const int ns = (int)llround(VSMALL * VSMALL / (2.0 * ACCEL));
  const uint32_t ct = ticks_for_v(VMAX);
  const uint32_t cts = ticks_for_v(VSMALL);
  const int ncv = (int)llround(T_COAST * VMAX);
  const int nch = (int)llround(T_SPIN_HOLD * VSMALL);
  const int ncs = (int)llround(T_COAST_SMALL * VSMALL);

  if (trace) p.trace_set_phase(PH_ACCEL);
  issue_ramp(p, spin1, true);                          // 0 -> vsmall (0.5 s)
  for (int i = 0; i < nch; i++) p.step(1, true, cts);  // hold vsmall (1 s)
  issue_ramp(p, spin2, true);                          // vsmall -> vmax (0.5 s)

  if (trace) p.trace_set_phase(PH_COAST);
  for (int i = 0; i < ncv; i++) p.step(1, true, ct);

  if (slip_steps != 0.0) {
    p.rotate(slip_steps);
    if (trace) p.trace_set_phase(PH_SLIP);
  }

  if (trace) p.trace_set_phase(PH_COAST);
  for (int i = 0; i < ncv; i++) p.step(1, true, ct);

  if (probe) {
    probe->delta_stalled = p.delta();
    probe->w_stalled = p.speed();
    probe->stall = p.last().stall ? 1 : 0;
  }

  // Decelerate vmax -> vsmall.
  if (trace) p.trace_set_phase(PH_DECEL);
  for (int i = nd - 1; i >= ns; i--) issue_single_step(p, true, dec[i]);

  // Coast at vsmall; the re-captured rotor should follow here.
  if (trace) p.trace_set_phase(PH_COAST);
  if (probe) probe->x_small_start = p.x();
  for (int i = 0; i < ncs; i++) p.step(1, true, cts);
  if (probe) {
    probe->x_small_end = p.x();
    probe->w_small_end = p.speed();
  }

  // Decelerate vsmall -> 0.
  if (trace) p.trace_set_phase(PH_DECEL);
  for (int i = ns - 1; i >= 0; i--) issue_single_step(p, true, dec[i]);

  return p.x_c();
}

// ---- T1: single pause ------------------------------------------------------
static void test_dwell() {
  printf("=== T1: single pause (dwell) ===\n");
  g_plant.reset();
  observed_s o = g_plant.step(0, true, 50000);
  check(o.stall == false, "pause does not stall");
  check(fabs(g_plant.speed()) < 1.0, "speed decays near zero during dwell");
  check(g_plant.getCurrentPosition() == 0, "rotor does not drift during dwell");
}

// ---- T2: single step -------------------------------------------------------
static void test_single_step() {
  printf("=== T2: single step (slow) ===\n");
  g_plant.reset();
  // One step, then a long dwell so the rotor has time to settle onto it.
  issue_single_step(g_plant, true, 200000);
  dwell(g_plant, SETTLE_TICKS);
  check(g_plant.last().stall == false, "single step does not stall");
  check(g_plant.x_c() == 1.0, "commanded position advanced by one step");
  check(fabs(g_plant.delta()) < 1.5,
        "rotor settles onto the command within the friction deadband");
  printf("  rotor=%.4f cmd=%.1f delta=%.4f\n", g_plant.x(), g_plant.x_c(),
         g_plant.delta());
}

// ---- T3: gentle burst ------------------------------------------------------
static void test_gentle_burst() {
  printf("=== T3: gentle burst (64 steps @ 16000 ticks) ===\n");
  g_plant.reset();
  for (int i = 0; i < 64; i++) g_plant.step(1, true, 16000);
  dwell(g_plant, SETTLE_TICKS);
  check(g_plant.last().stall == false, "gentle burst never stalls");
  check(g_plant.x_c() == 64.0, "commanded advance is 64 steps");
  check(fabs(g_plant.delta()) < 1.5, "rotor settles onto the command");
}

// ---- T4: too-fast => step loss ---------------------------------------------
static void test_too_fast_stalls() {
  printf("=== T4: too-fast burst @ 1 tick (16 Msteps/s) ===\n");
  g_plant.reset();
  observed_s last = g_plant.step(80, true, 1);
  check(last.stall, "a too-fast burst is reported as a step loss");
}

// ---- T5: reversal ----------------------------------------------------------
static void test_reversal() {
  printf("=== T5: forward then reversal ===\n");
  g_plant.reset();
  for (int i = 0; i < 200; i++) g_plant.step(1, true, 16000);
  double peak = g_plant.x_c();
  for (int i = 0; i < 100; i++) g_plant.step(1, false, 16000);
  (void)peak;
  int32_t pos = g_plant.getCurrentPosition();
  check(pos < 200, "reversal brings position below the forward peak");
  check(pos >= 90, "reversal does not overshoot the origin");
  check(g_plant.last().stall == false, "no stall on a clean forward-then-back");
  printf("  net position after reversal: %d (fwd=200, back=100)\n", pos);
}

static long file_size(const char* path) {
  FILE* f = fopen(path, "rb");
  if (!f) return -1;
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fclose(f);
  return n;
}

// Peak absolute PCM sample in a .wav's data chunk (0 if silent/absent).
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

// ---- T6: the canonical rest-to-rest profile --------------------------------
static void test_canonical_profile() {
  printf(
      "=== T6: canonical profile (accel 1s, coast, decel, small, stop) ===\n");
  g_plant.reset();
  g_plant.trace_begin(4);
  move_probe probe;
  double cmd = run_move(g_plant, 0.0, true, &probe);
  size_t rows = g_plant.trace_dump("test_27_trapezoid.dat");
  g_plant.to_wav("test_27_trapezoid.wav");
  long wav = file_size("test_27_trapezoid.wav");
  double sim = (double)g_plant.total_ticks() / (double)TICKS_PER_S;
  dwell(g_plant, SETTLE_TICKS);
  int32_t pos = g_plant.getCurrentPosition();
  printf(
      "  sim_time=%.4f s, final pos=%d commanded=%.0f stall=%u, wav=%ld B, "
      "trace rows=%zu\n",
      sim, pos, cmd, (unsigned)g_plant.last().stall, wav, rows);
  check(g_plant.last().stall == false, "the canonical profile never stalls");
  check(fabs(sim - T_TOTAL) < 0.05, "simulated time matches the profile");
  check(fabs((double)pos - cmd) < 2.0, "rotor lands on the commanded position");
  // T_TOTAL at 44100 Hz, 16-bit mono, plus the 44-byte header.
  long wav_expect = (long)llround(T_TOTAL * 44100.0 * 2.0) + 44;
  check(wav > wav_expect - 2000 && wav < wav_expect + 2000,
        "the wav renders the whole move");
  int peak = wav_peak("test_27_trapezoid.wav");
  printf("  wav peak sample: %d\n", peak);
  check(peak > 1000, "the recorded wav carries audible signal, not silence");
  check(rows > 100, "the profile gnuplot trace has rows");
}

// ---- T7: a weak motor on the same profile must lose a step -----------------
static void test_weak_motor_stalls() {
  printf("=== T7: weak motor (Fmax=2e-3) on the same profile ===\n");
  g_weak.reset();
  run_move(g_weak, 0.0, false, nullptr);
  dwell(g_weak, SETTLE_TICKS);
  printf("  stall_ever=%d final pos=%d peak|delta|=%.1f\n", g_weak.stall_ever(),
         g_weak.getCurrentPosition(), g_weak.peak_abs_delta());
  check(g_weak.stall_ever(), "an under-torqued motor loses synchronism");
}

// ---- T8: over-drive outruns the rotor (emergent stall) ---------------------
static void test_overdrive_outruns_rotor() {
  printf("=== T8: over-drive outruns the rotor (emergent stall) ===\n");
  g_plant.reset();
  observed_s o = g_plant.step(80, true, 1);
  check(o.stall, "a too-fast burst is reported as a step loss");
  printf("  stall observation fired, |delta|=%.1f, x_c=%.0f\n",
         fabs(g_plant.delta()), g_plant.x_c());
}

// ---- T9: re-engagement latches onto a whole-step detent --------------------
static void test_reengagement_emerges() {
  printf("=== T9: re-engagement emerges from the model ===\n");
  g_plant.reset();
  observed_s o = g_plant.step(80, true, 1);
  check(o.stall, "over-drive stalls the rotor");
  printf("  stalled: |delta|=%.1f, x=%.1f, x_c=%.0f\n", fabs(g_plant.delta()),
         g_plant.x(), g_plant.x_c());

  for (int i = 0; i < 64; i++) g_plant.step(1, true, 16000);
  dwell(g_plant, SETTLE_TICKS);

  double off = g_plant.delta() / 64.0;
  printf("  re-synced: x=%.1f, x_c=%.0f, delta=%.3f (%.2f full steps)\n",
         g_plant.x(), g_plant.x_c(), g_plant.delta(), off);
  check(fabs(off - llround(off)) < 0.2 || fabs(g_plant.delta()) < 1.0,
        "re-engagement latches to a whole-step detent");

  int32_t before = g_plant.getCurrentPosition();
  for (int i = 0; i < 20; i++) g_plant.step(1, true, 16000);
  dwell(g_plant, SETTLE_TICKS);
  int32_t after = g_plant.getCurrentPosition();
  printf("  fresh move: %d -> %d\n", before, after);
  check(after - before == 20, "the re-synced rotor tracks a fresh slow move");
}

// ---- T12: a half-step slip mid-coast stalls, then re-joins ----
// The rotor is displaced half a full step after the first coast: it is parked
// on the separatrix, the field out-runs it and |delta| runs away. Later the
// profile decelerates to a small speed the rotor can follow again — it
// re-captures there and tracks, then the final decel brings both to rest.
static void test_half_step_slip_stalls() {
  printf("=== T12: half-step slip mid-coast stalls, then re-joins ===\n");
  g_stall.reset();
  g_stall.trace_begin(4);
  move_probe probe;
  run_move(g_stall, -32.0, true, &probe);
  size_t rows = g_stall.trace_dump("test_27_coast_stall.dat");
  g_stall.to_wav("test_27_coast_stall.wav");
  long wav = file_size("test_27_coast_stall.wav");
  int peak_sample = wav_peak("test_27_coast_stall.wav");
  double sim = (double)g_stall.total_ticks() / (double)TICKS_PER_S;

  double small_advance = probe.x_small_end - probe.x_small_start;
  printf(
      "  stalled: |delta|=%.0f w=%.0f stall=%d | at %.0f/s: advance=%.0f "
      "(expect %.0f), w=%.0f | sim=%.2f s\n",
      fabs(probe.delta_stalled), probe.w_stalled, probe.stall, VSMALL,
      small_advance, VSMALL * T_COAST_SMALL, probe.w_small_end, sim);

  check(probe.stall == 1, "the half-step slip stalls the rotor mid-coast");
  check(fabs(probe.delta_stalled) > 64.0,
        "the slip runs |delta| past a full step");
  // The small-speed coast: the rotor follows the field again.
  check(fabs(small_advance - VSMALL * T_COAST_SMALL) <
            0.2 * VSMALL * T_COAST_SMALL,
        "the stalled rotor re-joins and follows the field at the small speed");
  check(fabs(probe.w_small_end - VSMALL) < 0.3 * VSMALL,
        "the rotor speed matches the small speed");
  check(fabs(sim - T_TOTAL) < 0.05, "the stall trace runs to the profile end");
  check(rows > 100, "the coast-stall gnuplot trace has rows");
  check(wav > 0, "the coast-stall wav was written");
  check(peak_sample > 1000,
        "the recorded wav carries the stall transient, not silence");
}

// ---- T13: a full-step slip loses exactly one step --------------------------
static void test_full_step_slip_loses_one_step() {
  printf("=== T13: full-step slip loses exactly one step ===\n");
  g_stall.reset();
  move_probe probe;
  run_move(g_stall, -64.0, false, &probe);
  dwell(g_stall, SETTLE_TICKS);
  double off = g_stall.delta() / 64.0;
  printf(
      "  stalled: |delta|=%.1f w=%.0f | final delta=%.1f (%.2f full steps)\n",
      fabs(probe.delta_stalled), probe.w_stalled, g_stall.delta(), off);
  check(probe.w_stalled > 9000.0,
        "a full-step slip keeps the rotor tracking (no runaway)");
  check(fabs(off + 1.0) < 0.15, "it ends exactly one full step behind");
  check(fabs((probe.x_small_end - probe.x_small_start) -
             VSMALL * T_COAST_SMALL) < 0.2 * VSMALL * T_COAST_SMALL,
        "the rotor followed the field through the small-speed coast");
}

static void test_wav_generation(const char* good_wav, const char* bad_wav) {
  printf("=== T10: wav synthesis ===\n");
  g_plant.reset();
  for (int i = 0; i < 400; i++) g_plant.step(1, true, 1600);
  bool g = g_plant.to_wav(good_wav);
  check(g, "to_wav() succeeds for the moving plant");

  g_plant.reset();
  g_plant.step(80, true, 1);  // stall
  bool b = g_plant.to_wav(bad_wav);
  check(b, "to_wav() succeeds for the stalled plant");

  FILE* f = fopen(good_wav, "rb");
  check(f != NULL, "moving wav file was written");
  if (f) {
    char riff[4];
    size_t n = fread(riff, 1, 4, f);
    check(n == 4 && memcmp(riff, "RIFF", 4) == 0,
          "moving wav starts with RIFF");
    fclose(f);
  }
  f = fopen(bad_wav, "rb");
  check(f != NULL, "stalled wav file was written");
  if (f) {
    char riff[4];
    size_t n = fread(riff, 1, 4, f);
    check(n == 4 && memcmp(riff, "RIFF", 4) == 0,
          "stalled wav starts with RIFF");
    fclose(f);
  }
}

// ---- gnuplot figure for the two runs --------------------------------------
// One six-panel figure per run, side by side, so the shared profile and the
// divergent ending can be read off directly. Dat columns:
//   1:t 2:x 3:x_c 4:delta 5:w 6:a 7:tau 8:friction 9:stall 10:phase
static void write_gnuplot_asset() {
  FILE* g = fopen("test_27.gnuplot", "w");
  if (!g) return;
  fprintf(
      g,
      "set term pngcairo size 1600,1400\n"
      "set output \"test_27_coast_stall.png\"\n"
      "set multiplot layout 3,2 title "
      "\"PhysicalStepper: reference vs half-step-slip stall (same ramp)\"\n"
      "set xlabel \"t [s]\"\n"
      "D=64.0\n"
      "set title \"position [steps]\"\n"
      "plot \"test_27_trapezoid.dat\" using 1:2 with lines title "
      "\"ref x\", \"test_27_trapezoid.dat\" using 1:3 with lines dt 2 "
      "title \"ref x_c\", \"test_27_coast_stall.dat\" using 1:2 with "
      "lines title \"stall x\", \"test_27_coast_stall.dat\" using 1:3 with "
      "lines dt 2 title \"stall x_c\"\n"
      "set title \"step error delta [steps]\"\n"
      "plot \"test_27_coast_stall.dat\" using 1:4 with lines title "
      "\"stall delta\", \"test_27_trapezoid.dat\" using 1:4 with lines dt 2 "
      "title \"ref delta\", D lt 0 title \"+D\", -D lt 0 notitle\n"
      "set title \"rotor speed [steps/s]\"\n"
      "plot \"test_27_coast_stall.dat\" using 1:5 with lines title "
      "\"stall w\", \"test_27_trapezoid.dat\" using 1:5 with lines dt 2 "
      "title \"ref w\"\n"
      "set title \"raw rotor acceleration [steps/s^2]\"\n"
      "plot \"test_27_coast_stall.dat\" using 1:6 with lines title "
      "\"stall a\", \"test_27_trapezoid.dat\" using 1:6 with lines dt 2 "
      "title \"ref a\"\n"
      "set title \"magnetic force and friction\"\n"
      "plot \"test_27_coast_stall.dat\" using 1:7 with lines title "
      "\"tau\", \"test_27_coast_stall.dat\" using 1:8 with lines title "
      "\"friction\"\n"
      "set title \"stall observation (1 = rotor slipping)\"\n"
      "set yrange [0:1.1]\n"
      "set ytics 0,0.5,1\n"
      "plot \"test_27_coast_stall.dat\" using 1:9 with lines title "
      "\"stall\"\n"
      "unset yrange\n"
      "unset multiplot\n"
      "unset output\n");
  fclose(g);
}

int main() {
  printf("=====================================================\n");
  printf(" FastAccelStepper PC-Based Test 27\n");
  printf(" PhysicalStepper rotordynamic plant test drive\n");
  printf("=====================================================\n");

  test_dwell();
  test_single_step();
  test_gentle_burst();
  test_too_fast_stalls();
  test_reversal();
  test_canonical_profile();
  test_weak_motor_stalls();
  test_overdrive_outruns_rotor();
  test_reengagement_emerges();
  test_half_step_slip_stalls();
  test_full_step_slip_loses_one_step();
  test_wav_generation("test_27_good.wav", "test_27_bad.wav");
  write_gnuplot_asset();

  printf("=====================================================\n");
  if (failures != 0) {
    printf("TEST_27 FAILED (%d failures)\n", failures);
    return 1;
  }
  printf(
      "TEST_27 PASSED (plant follows a rest-to-rest profile; a half-step\n"
      "slip mid-coast stalls it while a full-step slip loses one step; the\n"
      "stall re-joins the field at the small speed and stops with it)\n");
  return 0;
}
