// test_29 — FasTimed, the faithful timed trajectory (problem 2).
//
// A waypoint is one constant rate: per-axis delta steps in [-128, 128] and
// one shared duration in [MIN_CMD_TICKS, 65535] driver ticks. The planner
// does not invent a ramp. PC only; the stepper is SimPort.
//
// Acceleration figures below are the ramp map at TICKS_PER_S = 16 MHz:
//   a = 1e6,  period 4000 -> P = 8,  period 8000 -> P = 2,  period 12000 -> P =
//   0 a = 1e7,  period 2000 -> P = 3,  period 3333 -> P = 1
// A chunk may change ramp-step by at most its own step count, and a
// reversal or a zero-step axis is legal only from P = 0.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FasTimed.h"
#include "fas_arch/test_pc.h"
#include "naxis_sim_port.h"
#include "../../../examples/NaxesTimed/NaxesTimed_ramp.h"

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

char TCCR1A;
char TCCR1B;
char TCCR1C;
char TIMSK1;
char TIFR1;
unsigned short OCR1A;
unsigned short OCR1B;

static TestFastAccelStepperEngine g_engine;

static const char* add_name(TimedAdd a) {
  if (a == TimedAdd::Ok) {
    return "Ok";
  }
  if (a == TimedAdd::Rejected) {
    return "Rejected";
  }
  if (a == TimedAdd::TimingNotAchievable) {
    return "TimingNotAchievable";
  }
  return "?";
}

static void expect_add(TimedAdd got, TimedAdd want, const char* what) {
  if (got != want) {
    printf("addDelta %s: got %s\n", what, add_name(got));
  }
  test(got == want, what);
}

struct CmdSeen {
  uint16_t sum;
  uint8_t steps;
  bool count_up;
};

// Pull up to cap commands. Returns how many were pending.
static int drain_cmds(SimPort* p, CmdSeen* out, int cap) {
  int n = 0;
  while (n < cap && !p->isQueueEmpty()) {
    int64_t st = 0;
    bool up = false;
    uint32_t sum = p->drain_one(&st, &up);
    out[n].sum = (uint16_t)sum;
    out[n].steps = (uint8_t)st;
    out[n].count_up = up;
    n++;
  }
  return n;
}

static void anchor_ramp() {
  test(MIN_CMD_TICKS == 3200, "PC MIN_CMD_TICKS is 3200");
  RampMap slow(500, 1000000);
  test(slow.calculate_ramp_steps(4000) == 8, "P(4000) at 1e6 is 8");
  test(slow.calculate_ramp_steps(8000) == 2, "P(8000) at 1e6 is 2");
  test(slow.calculate_ramp_steps(12000) == 0, "P(12000) at 1e6 is 0");
  RampMap fast(80, 10000000);
  test(fast.calculate_ramp_steps(2000) == 3, "P(2000) at 1e7 is 3");
  test(fast.calculate_ramp_steps(3333) == 1, "P(3333) at 1e7 is 1");
  RampMap huge(80, 100000000);
  test(huge.calculate_ramp_steps(500) <= 128, "128 steps cover period 500");
  test(huge.calculate_ramp_steps(1066) <= 3, "3 steps cover a 3200-tick split");
}

// Limits, the ±128 / tick-window contract, and a dwell at both ends.
static void t_range_and_dwell() {
  SimPort x(80, 64);
  SimPort y(80, 64);
  x.setAcceleration(100000000);
  y.setAcceleration(100000000);
  FasTimed<2, 4, SimPort, TestFastAccelStepperEngine> plan(g_engine);
  test(plan.addAxis(0, &x), "range add X");
  test(plan.addAxis(1, &y), "range add Y");

  int16_t z[2] = {0, 0};
  expect_add(plan.addDelta(z, 3200), TimedAdd::Rejected, "not synced");
  plan.syncFromSteppers();

  x.setRampGeneratorActive(true);
  FasTimed<2, 4, SimPort, TestFastAccelStepperEngine> busy(g_engine);
  test(!busy.addAxis(0, &x), "ramp active rejects addAxis");
  x.setRampGeneratorActive(false);

  int16_t hi[2] = {129, 0};
  int16_t lo[2] = {-129, 0};
  expect_add(plan.addDelta(hi, 64000), TimedAdd::Rejected, "+129 rejected");
  expect_add(plan.addDelta(lo, 64000), TimedAdd::Rejected, "-129 rejected");
  expect_add(plan.addDelta(z, 0), TimedAdd::Rejected, "0 ticks rejected");
  expect_add(plan.addDelta(z, (uint16_t)(MIN_CMD_TICKS - 1)),
             TimedAdd::Rejected, "below MIN_CMD_TICKS rejected");
  test(plan.plannedPosition(0) == 0 && plan.actualTicks() == 0,
       "a rejected call changes nothing");

  expect_add(plan.addDelta(z, (uint16_t)MIN_CMD_TICKS), TimedAdd::Ok,
             "dwell at MIN_CMD_TICKS");
  expect_add(plan.addDelta(z, 65535), TimedAdd::Ok, "dwell at 65535");
  test(plan.actualTicks() == (uint32_t)MIN_CMD_TICKS + 65535,
       "dwell durations add");

  int32_t origin[2] = {10, -4};
  plan.setCurrentPosition(origin);
  test(plan.plannedPosition(0) == 10 && plan.plannedPosition(1) == -4,
       "setCurrentPosition rebases and clears the plan");
  test(plan.actualTicks() == 0, "rebase clears actual ticks");

  int16_t full[2] = {128, -128};
  expect_add(plan.addDelta(full, 64000), TimedAdd::Ok, "+/-128 accepted");
  test(plan.plannedPosition(0) == 138 && plan.plannedPosition(1) == -132,
       "planned position sums the deltas");
  test(plan.actualTicks() == 64000, "128-step chunk keeps its duration");

  test(plan.pump() == TimedStatus::Running, "boundary chunk is running");
  CmdSeen cx[2];
  CmdSeen cy[2];
  test(drain_cmds(&x, cx, 2) == 1, "X is one command");
  test(drain_cmds(&y, cy, 2) == 1, "Y is one command");
  test(cx[0].steps == 128 && cx[0].sum == 64000 && cx[0].count_up,
       "X +128 in 64000");
  test(cy[0].steps == 128 && cy[0].sum == 64000 && !cy[0].count_up,
       "Y -128 in 64000");
  test(x.clock() == y.clock() && x.clock() == 64000, "boundary clocks match");
  test(x.getCurrentPosition() == 128 && y.getCurrentPosition() == -128,
       "steppers moved by the delta, not the rebased origin");
  test(plan.pump() == TimedStatus::Idle, "boundary run settles");
}

// Two axes, one command each, then the same rate again (cruise).
static void t_exact_and_cruise() {
  SimPort x(500, 64);
  SimPort y(500, 64);
  x.setAcceleration(1000000);
  y.setAcceleration(1000000);
  FasTimed<2, 8, SimPort, TestFastAccelStepperEngine> plan(g_engine);
  plan.addAxis(0, &x);
  plan.addAxis(1, &y);
  plan.syncFromSteppers();

  int16_t d[2] = {8, -4};
  expect_add(plan.addDelta(d, 32000), TimedAdd::Ok, "exact (8,-4)");
  expect_add(plan.addDelta(d, 32000), TimedAdd::Ok, "cruise (8,-4)");
  test(plan.plannedPosition(0) == 16 && plan.plannedPosition(1) == -8,
       "cruise planned position");
  test(plan.actualTicks() == 64000, "two exact chunks");
  test(plan.isBusy(), "queued chunks are busy before pump");

  test(plan.pump() == TimedStatus::Running, "exact pump");
  test(x.isRunning() && y.isRunning(), "kick-off started both queues");
  CmdSeen cx[4];
  CmdSeen cy[4];
  test(drain_cmds(&x, cx, 4) == 2, "X two cruise commands");
  test(drain_cmds(&y, cy, 4) == 2, "Y two cruise commands");
  test(cx[0].steps == 8 && cx[0].sum == 32000 && cx[0].count_up, "X cmd 0");
  test(cx[1].steps == 8 && cx[1].sum == 32000 && cx[1].count_up, "X cmd 1");
  test(cy[0].steps == 4 && cy[0].sum == 32000 && !cy[0].count_up, "Y cmd 0");
  test(cy[1].steps == 4 && cy[1].sum == 32000 && !cy[1].count_up, "Y cmd 1");
  test(x.clock() == 64000 && y.clock() == 64000, "cruise clocks");
  test(x.getCurrentPosition() == 16 && y.getCurrentPosition() == -8,
       "cruise stepper positions");
  test(plan.pump() == TimedStatus::Idle, "cruise settles");
  test(!plan.isBusy(), "idle is not busy");
}

// Master is one command; the short axis splits into q+1 and q groups.
// Idle axis pauses for the same sum.
static void t_split() {
  SimPort x(500, 64);
  SimPort y(500, 64);
  x.setAcceleration(1000000);
  y.setAcceleration(1000000);
  FasTimed<2, 4, SimPort, TestFastAccelStepperEngine> plan(g_engine);
  plan.addAxis(0, &x);
  plan.addAxis(1, &y);
  plan.syncFromSteppers();

  int16_t d[2] = {8, 3};
  expect_add(plan.addDelta(d, 32000), TimedAdd::Ok, "split slave (8,3)");
  plan.pump();
  CmdSeen cx[2];
  CmdSeen cy[4];
  test(drain_cmds(&x, cx, 2) == 1, "master is one command");
  test(cx[0].steps == 8 && cx[0].sum == 32000 && cx[0].count_up, "master 8");
  test(drain_cmds(&y, cy, 4) == 2, "slave is two commands");
  // 32000 = 2 * 10667 + 1 * 10666
  test(cy[0].steps == 2 && cy[0].sum == 21334 && cy[0].count_up,
       "slave long group");
  test(cy[1].steps == 1 && cy[1].sum == 10666 && cy[1].count_up,
       "slave short group");
  test(x.clock() == 32000 && y.clock() == 32000, "split clocks match");
  test(x.getCurrentPosition() == 8 && y.getCurrentPosition() == 3,
       "split positions");

  // 3 steps in 10000: groups 1*3334 and 2*3333. Needs the faster map so
  // P(3333) = 1 fits in 3 steps. The partner pauses.
  SimPort a(80, 64);
  SimPort b(80, 64);
  a.setAcceleration(10000000);
  b.setAcceleration(10000000);
  FasTimed<2, 4, SimPort, TestFastAccelStepperEngine> fast(g_engine);
  fast.addAxis(0, &a);
  fast.addAxis(1, &b);
  fast.syncFromSteppers();
  int16_t e[2] = {3, 0};
  expect_add(fast.addDelta(e, 10000), TimedAdd::Ok, "3 steps in 10000");
  fast.pump();
  CmdSeen ca[4];
  CmdSeen cb[2];
  test(drain_cmds(&a, ca, 4) == 2, "3-step split is two commands");
  test(ca[0].steps == 1 && ca[0].sum == 3334 && ca[0].count_up, "remainder 1");
  test(ca[1].steps == 2 && ca[1].sum == 6666 && ca[1].count_up, "pair at q");
  test(drain_cmds(&b, cb, 2) == 1, "idle axis is one pause");
  test(cb[0].steps == 0 && cb[0].sum == 10000 && cb[0].count_up,
       "pause keeps DIR and the full duration");
  test(a.clock() == 10000 && b.clock() == 10000, "pause clock matches");
  test(a.getCurrentPosition() == 3 && b.getCurrentPosition() == 0,
       "pause axis did not step");
}

// 10 * 2000 = 20000 is the nearest legal sum to 20001 (one tick short).
// 3 steps in 3200 cannot be a legal split; the next legal sum is 3 * 1067.
// Acceleration is 1e8 so both periods are inside the 3-step and 10-step ramp.
static void t_quantize() {
  SimPort x(80, 64);
  SimPort y(80, 64);
  x.setAcceleration(100000000);
  y.setAcceleration(100000000);
  FasTimed<2, 4, SimPort, TestFastAccelStepperEngine> plan(g_engine);
  plan.addAxis(0, &x);
  plan.addAxis(1, &y);
  plan.syncFromSteppers();

  int16_t d[2] = {10, 0};
  expect_add(plan.addDelta(d, 20001), TimedAdd::Ok, "20001 quantizes");
  test(plan.actualTicks() == 20000, "nearest sum is 20000");
  plan.pump();
  CmdSeen cx[2];
  CmdSeen cy[2];
  test(drain_cmds(&x, cx, 2) == 1, "quantized X is one command");
  test(cx[0].steps == 10 && cx[0].sum == 20000 && cx[0].count_up, "10 * 2000");
  test(drain_cmds(&y, cy, 2) == 1 && cy[0].steps == 0 && cy[0].sum == 20000,
       "idle axis follows the issued sum, not 20001");
  test(x.clock() == y.clock() && x.clock() == 20000, "quantize clocks");

  int16_t up[2] = {3, 0};
  expect_add(plan.addDelta(up, 3200), TimedAdd::Ok, "3200 rounds up");
  test(plan.actualTicks() == 20000 + 3201, "3 * 1067 = 3201");
  plan.pump();
  test(drain_cmds(&x, cx, 2) == 1, "rounded X is one command");
  test(cx[0].steps == 3 && cx[0].sum == 3201, "period 1067");
  test(drain_cmds(&y, cy, 2) == 1 && cy[0].steps == 0 && cy[0].sum == 3201,
       "rounded pause matches");
}

static void t_too_fast_and_accel() {
  SimPort x(500, 64);
  SimPort y(500, 64);
  x.setAcceleration(1000000);
  y.setAcceleration(1000000);
  FasTimed<2, 4, SimPort, TestFastAccelStepperEngine> plan(g_engine);
  plan.addAxis(0, &x);
  plan.addAxis(1, &y);
  plan.syncFromSteppers();

  int16_t fast[2] = {8, 0};
  // 8 * 500 = 4000 > 3200, so the average period is under the speed floor.
  expect_add(plan.addDelta(fast, 3200), TimedAdd::TimingNotAchievable,
             "faster than ticks_cfg");
  test(plan.plannedPosition(0) == 0 && plan.actualTicks() == 0,
       "too-fast does not commit");
  test(plan.pump() == TimedStatus::Idle, "too-fast queues nothing");

  int16_t four[2] = {4, 0};
  // Period 4000, P = 8, and 4 steps cannot cover that jump from rest.
  expect_add(plan.addDelta(four, 16000), TimedAdd::TimingNotAchievable,
             "from rest P=8 needs more than 4 steps");

  int16_t two[2] = {2, 0};
  expect_add(plan.addDelta(two, 16000), TimedAdd::Ok, "from rest P=2 in 2");
  int16_t jump[2] = {4, 0};
  // Period 4000 is P = 8, delta 6, and this chunk has only 4 steps.
  expect_add(plan.addDelta(jump, 16000), TimedAdd::TimingNotAchievable,
             "accel jump of 6 does not fit in 4 steps");
  test(plan.plannedPosition(0) == 2 && plan.actualTicks() == 16000,
       "failed jump leaves the first chunk");

  int16_t ok[2] = {8, 0};
  expect_add(plan.addDelta(ok, 32000), TimedAdd::Ok,
             "delta P of 6 fits in 8 steps");
  test(plan.plannedPosition(0) == 10, "position after the legal accel");

  // The partner axis is still at rest, so a later chunk may move only X.
  int16_t only_y[2] = {0, 2};
  expect_add(plan.addDelta(only_y, 16000), TimedAdd::TimingNotAchievable,
             "X at P=8 cannot take a zero-step chunk");
}

// Decel to P = 0, then reverse. An immediate reverse, and a dwell while
// still at speed, are TimingNotAchievable.
static void t_reversal_and_dwell() {
  SimPort x(500, 64);
  x.setAcceleration(1000000);
  FasTimed<1, 8, SimPort, TestFastAccelStepperEngine> plan(g_engine);
  plan.addAxis(0, &x);
  plan.syncFromSteppers();

  int16_t a = 2;
  int16_t rev = -2;
  int16_t dwell = 0;
  expect_add(plan.addDelta(&a, 16000), TimedAdd::Ok, "leave rest at P=2");
  expect_add(plan.addDelta(&rev, 16000), TimedAdd::TimingNotAchievable,
             "reverse while P=2");
  expect_add(plan.addDelta(&dwell, 8000), TimedAdd::TimingNotAchievable,
             "dwell while P=2");
  test(plan.plannedPosition(0) == 2 && plan.actualTicks() == 16000,
       "illegal reverse and dwell do not commit");

  expect_add(plan.addDelta(&a, 24000), TimedAdd::Ok, "decel to P=0");
  expect_add(plan.addDelta(&dwell, 8000), TimedAdd::Ok, "dwell from rest");
  expect_add(plan.addDelta(&rev, 16000), TimedAdd::Ok, "reverse from P=0");
  test(plan.plannedPosition(0) == 2, "2 + 2 + 0 - 2");
  test(plan.actualTicks() == 64000, "16000+24000+8000+16000");

  plan.pump();
  CmdSeen c[6];
  test(drain_cmds(&x, c, 6) == 4, "four commands");
  test(c[0].steps == 2 && c[0].sum == 16000 && c[0].count_up, "out");
  test(c[1].steps == 2 && c[1].sum == 24000 && c[1].count_up, "decel");
  test(c[2].steps == 0 && c[2].sum == 8000 && c[2].count_up,
       "dwell keeps the last DIR");
  test(c[3].steps == 2 && c[3].sum == 16000 && !c[3].count_up, "back");
  test(x.getCurrentPosition() == 2 && x.clock() == 64000, "reversal landed");
  test(plan.pump() == TimedStatus::Idle, "reversal settles");
}

// A one-shot QueueFull holds that axis's command for the next pump.
static void t_retry() {
  SimPort x(500, 64);
  SimPort y(500, 64);
  x.setAcceleration(1000000);
  y.setAcceleration(1000000);
  FasTimed<2, 4, SimPort, TestFastAccelStepperEngine> plan(g_engine);
  plan.addAxis(0, &x);
  plan.addAxis(1, &y);
  plan.syncFromSteppers();

  int16_t d[2] = {2, 2};
  expect_add(plan.addDelta(d, 16000), TimedAdd::Ok, "retry chunk");
  y.failNext(AQE_QUEUE_FULL);
  test(plan.pump() == TimedStatus::Running, "faulting pump still running");
  test(x.queueEntries() == 1, "X took its command");
  test(y.queueEntries() == 0, "Y held the refused command");
  test(plan.isBusy(), "held command keeps the plan busy");

  test(plan.pump() == TimedStatus::Running, "retry pump");
  test(y.queueEntries() == 1, "Y accepts the held command");
  CmdSeen cx[2];
  CmdSeen cy[2];
  test(drain_cmds(&x, cx, 2) == 1 && drain_cmds(&y, cy, 2) == 1, "one each");
  test(cx[0].steps == 2 && cx[0].sum == 16000, "X retry payload");
  test(cy[0].steps == 2 && cy[0].sum == 16000, "Y retry payload");
  test(x.clock() == y.clock(), "retry clocks");
  test(plan.pump() == TimedStatus::Idle, "retry settles");
}

// Queue shorter than the plan: draining it dry while chunks remain is
// Underrun, and the following pumps still deliver the rest.
static void t_underrun() {
  SimPort x(500, 4);
  SimPort y(500, 4);
  x.setAcceleration(1000000);
  y.setAcceleration(1000000);
  FasTimed<2, 8, SimPort, TestFastAccelStepperEngine> plan(g_engine);
  plan.addAxis(0, &x);
  plan.addAxis(1, &y);
  plan.syncFromSteppers();

  int16_t d[2] = {2, 2};
  for (int i = 0; i < 4; i++) {
    expect_add(plan.addDelta(d, 16000), TimedAdd::Ok, "underrun chunk");
  }
  TimedStatus st = plan.pump();
  test(st == TimedStatus::Running, "short queue still running");
  test(x.queueEntries() > 0 && x.queueEntries() < 4, "X did not take all 4");

  while (!x.isQueueEmpty() || !y.isQueueEmpty()) {
    x.drain();
    y.drain();
  }
  st = plan.pump();
  test(st == TimedStatus::Underrun, "empty queue with chunks left");
  // Underrun latches, but the feeder keeps filling.
  for (int i = 0; i < 4 && plan.isBusy(); i++) {
    x.drain();
    y.drain();
    plan.pump();
  }
  test(x.getCurrentPosition() == 8 && y.getCurrentPosition() == 8,
       "underrun still delivers every step");
  test(x.clock() == y.clock() && x.clock() == 64000, "underrun clocks");
}

static void t_horizon() {
  SimPort x(80, 64);
  x.setAcceleration(10000000);
  FasTimed<1, 2, SimPort, TestFastAccelStepperEngine> plan(g_engine);
  plan.addAxis(0, &x);
  plan.syncFromSteppers();
  int16_t z = 0;
  expect_add(plan.addDelta(&z, 3200), TimedAdd::Ok, "horizon 0");
  expect_add(plan.addDelta(&z, 3200), TimedAdd::Ok, "horizon 1");
  expect_add(plan.addDelta(&z, 3200), TimedAdd::Rejected, "horizon full");
  test(plan.actualTicks() == 6400, "full ring did not take the third");
  plan.pump();
  x.drain();
  test(plan.pump() == TimedStatus::Idle, "horizon drained");
  expect_add(plan.addDelta(&z, 3200), TimedAdd::Ok, "ring slides");
  test(plan.actualTicks() == 9600, "committed sum keeps the drained dwells");
}

static void t_stop_and_inject() {
  SimPort x(500, 64);
  x.setAcceleration(1000000);
  FasTimed<1, 4, SimPort, TestFastAccelStepperEngine> plan(g_engine);
  plan.addAxis(0, &x);
  plan.syncFromSteppers();
  int16_t d = 2;
  expect_add(plan.addDelta(&d, 16000), TimedAdd::Ok, "stop setup");
  x.setStopCause(StepperStopCause::ForceStop);
  test(plan.pump() == TimedStatus::Stopped, "external stop");
  test(plan.pump() == TimedStatus::Stopped, "stop latches");
  expect_add(plan.addDelta(&d, 16000), TimedAdd::Rejected,
             "no add while stopped");
  plan.syncFromSteppers();
  int16_t z = 0;
  expect_add(plan.addDelta(&z, 3200), TimedAdd::Ok, "sync clears the stop");

  SimPort y(500, 64);
  y.setAcceleration(1000000);
  y.setInjectMode(SimPort::InjectDirPauses);
  FasTimed<1, 4, SimPort, TestFastAccelStepperEngine> rev(g_engine);
  rev.addAxis(0, &y);
  rev.syncFromSteppers();
  int16_t a = 2;
  int16_t back = -2;
  expect_add(rev.addDelta(&a, 16000), TimedAdd::Ok, "inject out");
  expect_add(rev.addDelta(&a, 24000), TimedAdd::Ok, "inject decel");
  expect_add(rev.addDelta(&back, 16000), TimedAdd::Ok, "inject reverse");
  test(rev.pump() == TimedStatus::Error, "unplanned DIR pause is Error");
}

// A square streamed through a ring of 8 chunks. The pitch ladder is
// examples/NaxesTimed/NaxesTimed_ramp.h: about one second rising from 440 Hz to
// 2500 Hz, then about one second falling back to a stop. The trace is
// test_29.dat / test_29.gnuplot; the first two motors are the left and
// right channels of test_29.wav.
static const uint32_t kAudioSr = 44100;
static const double kTicksPerS = 16000000.0;

struct Seg {
  int16_t x;
  int16_t y;
  uint16_t ticks;
};

static void push_seg(Seg* s, int* n, int axis, int sign, int steps,
                     uint16_t ticks) {
  s[*n].x = 0;
  s[*n].y = 0;
  if (axis == 0) {
    s[*n].x = (int16_t)(sign * steps);
  } else {
    s[*n].y = (int16_t)(sign * steps);
  }
  s[*n].ticks = ticks;
  (*n)++;
}

// Hold one step period. The chunk uses as many steps as fit in 65535 ticks,
// which is enough for the ramp-step change into this period at a = 1e5.
static void push_hold(Seg* s, int* n, int axis, int sign, uint16_t period,
                      int repeats) {
  uint32_t steps = 65535u / period;
  if (steps > 128) {
    steps = 128;
  }
  if (steps < 1) {
    steps = 1;
  }
  uint16_t ticks = (uint16_t)(steps * period);
  for (int i = 0; i < repeats; i++) {
    push_seg(s, n, axis, sign, (int)steps, ticks);
  }
}

static void push_side(Seg* s, int* n, int axis, int sign) {
  for (unsigned i = 0;
       i < sizeof(kNaxesTimedUpPeriod) / sizeof(kNaxesTimedUpPeriod[0]); i++) {
    push_hold(s, n, axis, sign, kNaxesTimedUpPeriod[i], kNaxesTimedUpHold);
  }
  for (unsigned i = 0;
       i < sizeof(kNaxesTimedDownPeriod) / sizeof(kNaxesTimedDownPeriod[0]);
       i++) {
    push_hold(s, n, axis, sign, kNaxesTimedDownPeriod[i], kNaxesTimedDownHold);
  }
}

static void wav_u16(FILE* f, uint16_t v) {
  fputc((int)(v & 0xff), f);
  fputc((int)((v >> 8) & 0xff), f);
}
static void wav_u32(FILE* f, uint32_t v) {
  fputc((int)(v & 0xff), f);
  fputc((int)((v >> 8) & 0xff), f);
  fputc((int)((v >> 16) & 0xff), f);
  fputc((int)((v >> 24) & 0xff), f);
}

static void click(int16_t* pcm, uint32_t frames, uint32_t at, int channel) {
  static const int16_t kPulse[4] = {8000, 22000, 14000, 5000};
  for (int s = 0; s < 4; s++) {
    uint32_t i = at + (uint32_t)s;
    if (i >= frames) {
      return;
    }
    int32_t v = pcm[i * 2 + channel] + kPulse[s];
    if (v > 32767) {
      v = 32767;
    }
    pcm[i * 2 + channel] = (int16_t)v;
  }
}

static uint32_t tick_to_sample(uint32_t tick, uint32_t frames) {
  uint32_t s = (uint32_t)(((uint64_t)tick * kAudioSr) / 16000000ull);
  if (s >= frames) {
    s = frames - 1;
  }
  return s;
}

static void write_stereo_wav(const char* path, const int16_t* pcm,
                             uint32_t frames) {
  FILE* fp = fopen(path, "wb");
  test(fp != NULL, "open test_29.wav");
  const uint16_t bps = 16;
  const uint16_t ch = 2;
  const uint32_t data_bytes = frames * ch * (bps / 8);
  fwrite("RIFF", 1, 4, fp);
  wav_u32(fp, 36 + data_bytes);
  fwrite("WAVE", 1, 4, fp);
  fwrite("fmt ", 1, 4, fp);
  wav_u32(fp, 16);
  wav_u16(fp, 1);
  wav_u16(fp, ch);
  wav_u32(fp, kAudioSr);
  wav_u32(fp, kAudioSr * ch * (bps / 8));
  wav_u16(fp, ch * (bps / 8));
  wav_u16(fp, bps);
  fwrite("data", 1, 4, fp);
  wav_u32(fp, data_bytes);
  for (uint32_t i = 0; i < frames * 2; i++) {
    wav_u16(fp, (uint16_t)pcm[i]);
  }
  fclose(fp);
}

static void write_gnuplot() {
  FILE* g = fopen("test_29.gnuplot", "w");
  test(g != NULL, "open test_29.gnuplot");
  fprintf(g,
          "set term pngcairo size 1200,900\n"
          "set output \"test_29.png\"\n"
          "set multiplot layout 2,2 title "
          "\"FasTimed square: pitch rises and falls on each side\"\n"
          "set title \"path [steps]\"\n"
          "set xlabel \"x\"\n"
          "set ylabel \"y\"\n"
          "plot \"test_29.dat\" using 2:3 with lines title \"head\"\n"
          "set title \"position\"\n"
          "set xlabel \"t [s]\"\n"
          "set ylabel \"steps\"\n"
          "plot \"test_29.dat\" using 1:2 with lines title \"x\", "
          "\"test_29.dat\" using 1:3 with lines title \"y\"\n"
          "set title \"speed\"\n"
          "set ylabel \"steps/s\"\n"
          "plot \"test_29.dat\" using 1:4 with lines title \"vx\", "
          "\"test_29.dat\" using 1:5 with lines title \"vy\"\n"
          "set title \"|speed|\"\n"
          "plot \"test_29.dat\" using 1:(abs($4)) with lines title \"|vx|\", "
          "\"test_29.dat\" using 1:(abs($5)) with lines title \"|vy|\"\n"
          "unset multiplot\n"
          "unset output\n");
  fclose(g);
}

static int wav_peak(const char* path, int* left_nz, int* right_nz) {
  FILE* f = fopen(path, "rb");
  test(f != NULL, "reopen wav");
  unsigned char hdr[44];
  test(fread(hdr, 1, 44, f) == 44, "wav header");
  test(memcmp(hdr, "RIFF", 4) == 0 && memcmp(hdr + 8, "WAVE", 4) == 0,
       "wav is RIFF/WAVE");
  test(hdr[22] == 2 && hdr[23] == 0, "wav is stereo");
  int peak = 0;
  *left_nz = 0;
  *right_nz = 0;
  unsigned char b[4];
  while (fread(b, 1, 4, f) == 4) {
    int16_t l = (int16_t)(b[0] | (b[1] << 8));
    int16_t r = (int16_t)(b[2] | (b[3] << 8));
    int al = l < 0 ? -l : l;
    int ar = r < 0 ? -r : r;
    if (al > peak) {
      peak = al;
    }
    if (ar > peak) {
      peak = ar;
    }
    if (al > 1000) {
      (*left_nz)++;
    }
    if (ar > 1000) {
      (*right_nz)++;
    }
  }
  fclose(f);
  return peak;
}

static void record_pair(SimPort* x, SimPort* y, FILE* dat, int16_t* pcm,
                        uint32_t frames) {
  uint32_t t0 = x->clock();
  test(y->clock() == t0, "axes share the clock before the command");
  int64_t sx = 0;
  int64_t sy = 0;
  bool ux = true;
  bool uy = true;
  uint32_t dx = x->drain_one(&sx, &ux);
  uint32_t dy = y->drain_one(&sy, &uy);
  test(dx == dy && dx > 0, "paired commands cover the same ticks");
  double t = (double)t0 / kTicksPerS;
  double vx = 0.0;
  double vy = 0.0;
  if (sx > 0) {
    vx = (ux ? 1.0 : -1.0) * kTicksPerS * (double)sx / (double)dx;
  }
  if (sy > 0) {
    vy = (uy ? 1.0 : -1.0) * kTicksPerS * (double)sy / (double)dy;
  }
  uint32_t span = dx;
  uint32_t nstep = (uint32_t)sx;
  if ((uint32_t)sy > nstep) {
    nstep = (uint32_t)sy;
  }
  if (nstep == 0) {
    nstep = 1;
  }
  int32_t x_at = x->getCurrentPosition() - (ux ? (int32_t)sx : -(int32_t)sx);
  int32_t y_at = y->getCurrentPosition() - (uy ? (int32_t)sy : -(int32_t)sy);
  uint32_t x_left = (uint32_t)sx;
  uint32_t y_left = (uint32_t)sy;
  uint32_t period = span / nstep;
  for (uint32_t k = 0; k < nstep; k++) {
    uint32_t tick = t0 + k * period;
    if (x_left > 0) {
      x_at += ux ? 1 : -1;
      x_left--;
      click(pcm, frames, tick_to_sample(tick, frames), 0);
    }
    if (y_left > 0) {
      y_at += uy ? 1 : -1;
      y_left--;
      click(pcm, frames, tick_to_sample(tick, frames), 1);
    }
    double tk = (double)(tick + period) / kTicksPerS;
    fprintf(dat, "%.6f %d %d %.3f %.3f\n", tk, x_at, y_at, vx, vy);
    (void)t;
  }
}

static void t_square_trace() {
  RampMap map(500, 100000);
  test(map.calculate_ramp_steps(36000) == 0, "trace start is ramp-step 0");
  test(map.calculate_ramp_steps(6400) == 31, "trace top is ramp-step 31");
  test(16000000.0 / 6400 > 4.0 * (16000000.0 / 36000),
       "top pitch is more than two octaves above the start");

  Seg segs[2200];
  int n = 0;
  push_side(segs, &n, 0, +1);
  push_side(segs, &n, 1, +1);
  push_side(segs, &n, 0, -1);
  push_side(segs, &n, 1, -1);
  uint32_t total_ticks = 0;
  for (int i = 0; i < n; i++) {
    total_ticks += segs[i].ticks;
  }
  uint32_t frames =
      (uint32_t)(((uint64_t)total_ticks * kAudioSr) / 16000000ull) + 8;
  int16_t* pcm = (int16_t*)calloc(frames * 2, sizeof(int16_t));
  test(pcm != NULL, "pcm buffer");
  FILE* dat = fopen("test_29.dat", "w");
  test(dat != NULL, "open test_29.dat");
  fprintf(dat, "0.000000 0 0 0.000 0.000\n");

  SimPort x(500, 64);
  SimPort y(500, 64);
  x.setAcceleration(kNaxesTimedAccel);
  y.setAcceleration(kNaxesTimedAccel);
  FasTimed<2, 8, SimPort, TestFastAccelStepperEngine> plan(g_engine);
  test(plan.addAxis(0, &x), "trace add X");
  test(plan.addAxis(1, &y), "trace add Y");
  plan.syncFromSteppers();

  int i = 0;
  while (i < n) {
    int16_t d[2] = {segs[i].x, segs[i].y};
    TimedAdd a = plan.addDelta(d, segs[i].ticks);
    if (a == TimedAdd::Ok) {
      i++;
      continue;
    }
    expect_add(a, TimedAdd::Rejected, "horizon 8 backpressure");
    test(plan.pump() == TimedStatus::Running, "trace pump while streaming");
    while (!x.isQueueEmpty() && !y.isQueueEmpty()) {
      record_pair(&x, &y, dat, pcm, frames);
    }
    test(plan.pump() == TimedStatus::Idle, "drained batch settles the ring");
  }
  test(plan.pump() == TimedStatus::Running, "trace flushes the tail");
  while (!x.isQueueEmpty() && !y.isQueueEmpty()) {
    record_pair(&x, &y, dat, pcm, frames);
  }
  test(plan.pump() == TimedStatus::Idle, "trace settles");
  test(x.getCurrentPosition() == 0 && y.getCurrentPosition() == 0,
       "square closes");
  test(x.clock() == y.clock() && x.clock() == total_ticks,
       "square clocks match the script");
  fclose(dat);
  write_stereo_wav("test_29.wav", pcm, frames);
  write_gnuplot();
  free(pcm);

  int left_nz = 0;
  int right_nz = 0;
  int peak = wav_peak("test_29.wav", &left_nz, &right_nz);
  test(peak > 1000, "stereo wav is audible");
  test(left_nz > 100 && right_nz > 100, "both motors are in the wav");
  FILE* g = fopen("test_29.gnuplot", "rb");
  test(g != NULL, "gnuplot exists");
  char buf[256];
  size_t got = fread(buf, 1, sizeof(buf) - 1, g);
  buf[got] = 0;
  fclose(g);
  test(strstr(buf, "test_29.dat") != NULL, "gnuplot reads the trace");
  FILE* rows = fopen("test_29.dat", "rb");
  test(rows != NULL, "dat exists");
  int lines = 0;
  int c;
  while ((c = fgetc(rows)) != EOF) {
    if (c == '\n') {
      lines++;
    }
  }
  fclose(rows);
  test(lines > 200, "gnuplot trace has the square");
  printf("test_29 trace: test_29.gnuplot test_29.dat test_29.wav (%u frames)\n",
         frames);
}

int main() {
  anchor_ramp();
  t_range_and_dwell();
  t_exact_and_cruise();
  t_split();
  t_quantize();
  t_too_fast_and_accel();
  t_reversal_and_dwell();
  t_retry();
  t_underrun();
  t_horizon();
  t_stop_and_inject();
  t_square_trace();
  puts("test_29 passed");
  return 0;
}
