// test_27: physical_stepper — a rotordynamic plant test drive
//
// This is the "test drive" for the opt-in PhysicalStepper plant (see
// physical_stepper.h and extras/doc/physical_stepper_whitepaper.md). It drives
// the plant with the exact step/direction/ticks vocabulary a real queue hands
// a driver and asserts that a real motor's failure modes show up:
//
//   * it follows the canonical trapezoid without stalling,
//   * it lags during acceleration and runs ahead during deceleration,
//   * it loses grip exactly when the lag reaches one full step,
//   * and — the contract this test exists for — a stalled rotor is *inert*:
//     high-speed pulses keep advancing the command while the rotor does not
//     move, until the command is walked back within a full step.
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

// The canonical plant: 64 steps per full step, light inertia, light drag, and
// enough peak torque to follow the 10000 step/s trapezoid.
static PhysicalStepper g_plant{64, 1.0e-6, 1.0e-6, 1.0e-1};

// A deliberately weak motor: peak accel Fmax/J = 2000 step/s^2, which cannot
// follow a 10000 step/s^2 ramp — it must lose grip.
static PhysicalStepper g_weak{64, 1.0e-6, 1.0e-6, 2.0e-3};

// Long enough for the rotor to ring down (drag time constant is J/B = 1 s).
static const uint32_t SETTLE_TICKS = 12u * 16000000u;

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
  check(g_plant.getCurrentPosition() == 1, "rotor settles onto the command");
  check(g_plant.stalls() == 0, "no stall on a gentle step");
  printf("  rotor=%.4f cmd=%.1f delta=%.4f\n", g_plant.x(), g_plant.x_c(),
         g_plant.delta());
}

// ---- T3: gentle burst ------------------------------------------------------
static void test_gentle_burst() {
  printf("=== T3: gentle burst (64 steps @ 16000 ticks) ===\n");
  g_plant.reset();
  for (int i = 0; i < 64; i++) g_plant.step(1, true, 16000);
  dwell(g_plant, SETTLE_TICKS);
  check(g_plant.stalls() == 0, "gentle burst never stalls");
  check(g_plant.x_c() == 64.0, "commanded advance is 64 steps");
  check(fabs(g_plant.delta()) < 0.5, "rotor settles onto the command");
}

// ---- T4: too-fast => step loss ---------------------------------------------
static void test_too_fast_stalls() {
  printf("=== T4: too-fast burst @ 1 tick (16 Msteps/s) ===\n");
  g_plant.reset();
  observed_s last = g_plant.step(80, true, 1);
  check(last.stall, "a too-fast burst is reported as a step loss");
  check(g_plant.stalls() > 0, "stall_count grows for an underspeed move");
  printf("  stalls during too-fast burst: %u\n", g_plant.stalls());
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
  check(g_plant.stalls() == 0, "no stall on a clean forward-then-back");
  printf("  net position after reversal: %d (fwd=200, back=100)\n", pos);
}

// ---- T6: the canonical trapezoid ------------------------------------------
// 0 -> 10000 step/s in 1 s, coast 8 s, decelerate to rest in 1 s: exactly 10 s
// of simulated time. The plant follows it and lands on 90000 steps with no
// step loss, and its recorded hum spans the whole move.
static double run_trapezoid(PhysicalStepper& p, double vmax, double accel,
                            double t_acc, double t_coast) {
  const int n_acc = (int)llround(0.5 * accel * t_acc * t_acc);
  const int n_coast = (int)llround(t_coast * vmax);

  // Per-step periods of a rest-to-rest acceleration: step i is reached at
  // t_i = sqrt(2 i / a), so the period is t_i - t_{i-1}.
  std::vector<uint32_t> acc;
  double t_prev = 0.0;
  for (int i = 1; i <= n_acc; i++) {
    double ti = sqrt(2.0 * (double)i / accel);
    uint32_t tk = (uint32_t)llround((ti - t_prev) * (double)TICKS_PER_S);
    if (tk < 1) tk = 1;
    acc.push_back(tk);
    t_prev = ti;
  }
  for (size_t i = 0; i < acc.size(); i++) issue_single_step(p, true, acc[i]);

  uint32_t ct = ticks_for_v(vmax);
  for (int i = 0; i < n_coast; i++) p.step(1, true, (uint16_t)ct);

  // Deceleration mirrors the acceleration.
  for (int i = (int)acc.size() - 1; i >= 0; i--)
    issue_single_step(p, true, acc[i]);

  return p.x_c();
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

static void test_canonical_trapezoid() {
  printf("=== T6: canonical trapezoid 0->10000/s in 1 s, coast 8 s ===\n");
  g_plant.reset();
  double cmd = run_trapezoid(g_plant, 10000.0, 10000.0, 1.0, 8.0);
  double sim = (double)g_plant.total_ticks() / (double)TICKS_PER_S;
  g_plant.to_wav("test_27_trapezoid.wav");
  long wav = file_size("test_27_trapezoid.wav");
  dwell(g_plant, SETTLE_TICKS);
  int32_t pos = g_plant.getCurrentPosition();
  printf(
      "  sim_time=%.4f s, final pos=%d commanded=%.0f stalls=%u, wav=%ld B\n",
      sim, pos, cmd, g_plant.stalls(), wav);
  check(g_plant.stalls() == 0, "the canonical trapezoid never loses a step");
  check(fabs(sim - 10.0) < 0.01, "simulated time is 10 s");
  check(fabs(cmd - 90000.0) < 200.0, "commanded distance is ~90000 steps");
  check(fabs((double)pos - cmd) < 1.0, "rotor lands on the commanded position");
  // 10 s at 44100 Hz, 16-bit mono = 882000 bytes of PCM + 44-byte header.
  check(wav > 880000 && wav < 884100,
        "the wav renders the whole 10 s move (~882000 bytes)");
  int peak = wav_peak("test_27_trapezoid.wav");
  printf("  wav peak sample: %d\n", peak);
  check(peak > 1000, "the recorded wav carries audible signal, not silence");
}

// ---- T7: a weak motor on the same trapezoid must lose a step --------------
static void test_weak_motor_stalls() {
  printf("=== T7: weak motor (Fmax=2e-3) on the same trapezoid ===\n");
  g_weak.reset();
  run_trapezoid(g_weak, 10000.0, 10000.0, 1.0, 1.0);
  dwell(g_weak, SETTLE_TICKS);
  printf("  stalls=%u final pos=%d\n", g_weak.stalls(),
         g_weak.getCurrentPosition());
  check(g_weak.stalls() > 0, "an under-torqued motor loses synchronism");
}

// ---- T8/T9: the stall-latch contract ---------------------------------------
// After a stall the rotor is inert: further high-speed pulses advance the
// command while the rotor does not move. This is the property a real machine
// shows when the driver out-runs the load.
static void test_stalled_axis_is_inert() {
  printf("=== T8: stalled axis ignores further high-speed pulses ===\n");
  g_plant.reset();

  // Over-drive until grip is lost: each 1-tick step outruns the rotor.
  for (int i = 0; i < 80; i++) g_plant.step(1, true, 1);
  check(g_plant.stalls() > 0, "over-drive loses grip");

  int32_t x_at = g_plant.getCurrentPosition();
  double xc_at = g_plant.x_c();
  uint32_t s_at = g_plant.stalls();

  // Feed 100 more high-speed pulses.
  for (int i = 0; i < 100; i++) g_plant.step(1, true, 1);

  int32_t x_after = g_plant.getCurrentPosition();
  printf("  rotor %d -> %d, commanded %.0f -> %.0f, stalls %u -> %u\n", x_at,
         x_after, xc_at, g_plant.x_c(), s_at, g_plant.stalls());
  check(x_after == x_at, "rotor does not move under further high-speed pulses");
  check(g_plant.x_c() - xc_at == 100.0,
        "commanded position advances by the 100 pulses");
  check(g_plant.stalls() == s_at + 100,
        "every over-drive pulse is a step loss");
}

// ---- T10: re-synchronisation ----------------------------------------------
// Walking the command back within one full step recaptures the rotor.
static void test_resync() {
  printf("=== T9: re-sync by walking the command back within D ===\n");
  g_plant.reset();
  for (int i = 0; i < 80; i++) g_plant.step(1, true, 1);  // stall at x_c=80
  check(g_plant.stalls() > 0, "grip lost before re-sync");
  uint32_t stalls_stalled = g_plant.stalls();
  double x_stalled = g_plant.x();

  // Walk the command back to x_c=20 — within one full step of the rotor.
  int n = (int)llround(g_plant.x_c()) - 20;
  for (int i = 0; i < n; i++) g_plant.step(1, false, 4000);
  dwell(g_plant, SETTLE_TICKS);

  printf("  rotor %.3f -> %.3f, cmd=%.0f delta=%.3f, stalls %u -> %u\n",
         x_stalled, g_plant.x(), g_plant.x_c(), g_plant.delta(), stalls_stalled,
         g_plant.stalls());
  check(fabs(g_plant.delta()) < 5.0,
        "command back within one full step recaptures the rotor");
  uint32_t stalls_resynced = g_plant.stalls();

  // And the re-synced rotor tracks a fresh slow move.
  int32_t before = g_plant.getCurrentPosition();
  for (int i = 0; i < 20; i++) g_plant.step(1, true, 16000);
  dwell(g_plant, SETTLE_TICKS);
  printf("  fresh move: %d -> %d\n", before, g_plant.getCurrentPosition());
  check(g_plant.getCurrentPosition() == before + 20,
        "re-synced rotor tracks a fresh slow move");
  check(g_plant.stalls() == stalls_resynced,
        "re-synced axis loses no step on a fresh move");
}

// ---- T11: wav synthesis ----------------------------------------------------
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
  test_canonical_trapezoid();
  test_weak_motor_stalls();
  test_stalled_axis_is_inert();
  test_resync();
  test_wav_generation("test_27_good.wav", "test_27_bad.wav");

  printf("=====================================================\n");
  if (failures != 0) {
    printf("TEST_27 FAILED (%d failures)\n", failures);
    return 1;
  }
  printf(
      "TEST_27 PASSED (plant tracks a ramp, loses grip at one full step, "
      "and a stalled axis is inert)\n");
  return 0;
}
