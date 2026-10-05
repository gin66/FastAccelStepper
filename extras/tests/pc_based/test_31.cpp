// test_31: the queue admission latch
//
// A forceStop() / forceStopAndNewPosition() sets a latch that makes
// addQueueEntry() refuse every command. Before the fix the latch:
//   - was cleared by fill_queue() before it could refuse anything (so it was
//     inert for a ramp user), and never cleared on the addQueueEntry() path
//     (so it was permanent for a low-level user),
//   - sat *inside* the function, after the entry fields were written and after
//     the DIR pin was driven, so a refused command still moved the pin, still
//     updated queue_end.dir, and still counted towards the pause statistics,
//   - returned AQE_OK for the dropped command, so the caller advanced its own
//     position model by steps that were never queued,
//   - and let addQueueEntry(NULL, start) start a suspended queue anyway.
//
// The harness that found this (extras/tests/saleae_based) cannot express it:
// every scenario reconnects, and CONFIG calls stepperConnectToPin() ->
// _initVars(), which memsets the queue and silently rearms the latch.
//
// StepperISR_test.cpp stubs StepperQueue::forceStop() to an empty body, so the
// queue is not emptied here. That is irrelevant: the defect is the refusal, not
// the emptying, and the real FastAccelStepper.o sets the latch for real.

#include <stdio.h>

#include "FastAccelStepper.h"
#include "fas_queue/stepper_queue.h"

// `test` is a macro in fas_arch/test_pc.h, so this file's helper is `check`.
static int failures = 0;
static void check(bool cond, const char* msg) {
  if (!cond) {
    printf("FAIL: %s\n", msg);
    failures++;
  }
}

// StepperISR_test.cpp, which links into LIB_O. Not declared in a library header
// because no production caller needs it; test_23 declares it the same way.
extern void fas_reset_stepper_allocation();

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

static FastAccelStepperEngine engine;
static FastAccelStepper s;

// FastAccelStepper declares this class a friend (FastAccelStepper.h:1007), which
// is how the other tests reach fill_queue() and _queue() without the library
// widening its public API for a test.
class FastAccelStepperTest {
 public:
  StepperQueue* q(FastAccelStepper* sp) const { return sp->_queue(); }
  void fill(FastAccelStepper* sp) { sp->fill_queue(); }
};

static FastAccelStepperTest t;

// A fresh connection per case: stepperConnectToPin() is the one thing that
// rearms the latch, so reusing a stepper across cases would test nothing.
// MIN_CMD_TICKS is 3200 here, so 40 steps at 80 ticks is the smallest single
// command the queue accepts (1 step at 80 ticks is refused as too low).
static struct stepper_command_s cmd = {80, 40, true};

static void connect() {
  fas_reset_stepper_allocation();
  // FastAccelStepper::init() memsets the stepper but never touches the queue.
  // The queue is rearmed by _initVars() -- which is exactly why every Saleae
  // scenario, all of which reconnect, misses this defect.
  fas_queue[0]._initVars();
  fas_queue[1]._initVars();
  s.init(&engine, 0, 0);
  s.setDirectionPin(PIN_UNDEFINED, false);
  s.setSpeedInTicks(80);
}

// A: after forceStop() every command is refused with a terminal code, not OK.
static void a_refused_not_ok() {
  connect();
  s.forceStop();
  check(s.areCommandsSuspended(), "A: latch is set by forceStop()");
  AqeResultCode rc = s.addQueueEntry(&cmd, true);
  check(rc == AQE_ERROR_COMMANDS_SUSPENDED,
        "A: forceStop() refuses with ErrorCommandsSuspended");
  // Terminal, so a caller looping on a retryable code stops rather than spins.
  check(!aqeRetry(rc), "A: the refusal is not retryable");
}

// B: nothing about the queue moved. This is the silent divergence: the caller
// was told OK, so it advanced its own position model by 40 steps the queue
// never accepted.
static void b_queue_untouched() {
  connect();
  s.forceStop();
  int32_t pos_before = s.getCurrentPosition();
  uint8_t entries_before = s.queueEntries();
  int32_t queued_before = t.q(&s)->queue_end.pos;

  s.addQueueEntry(&cmd, true);

  check(s.getCurrentPosition() == pos_before, "B: position does not move");
  check(s.queueEntries() == entries_before, "B: no queue entry is added");
  check(t.q(&s)->queue_end.pos == queued_before,
        "B: queue_end.pos is not advanced");
}

// C: a suspended queue cannot be started either. addQueueEntry(NULL, start)
// returned before the gate, so the queue could be released after an abort.
static void c_cannot_start() {
  connect();
  // Fill a command in *before* the stop: the queue has to be non-empty for the
  // start path to be reachable at all.
  check(s.addQueueEntry(&cmd, false) == AQE_OK, "C: prefill accepted");
  s.forceStop();
  check(s.addQueueEntry(NULL, true) == AQE_ERROR_COMMANDS_SUSPENDED,
        "C: a suspended queue cannot be started");
}

// D: resumeCommands() is the key. Without it a low-level caller that aborted
// could never queue again on the same connection -- the whole point of the
// documented addQueueEntry() interface.
static void d_rearm() {
  connect();
  s.forceStop();
  check(s.addQueueEntry(&cmd, false) == AQE_ERROR_COMMANDS_SUSPENDED,
        "D: refused while suspended");
  s.resumeCommands();
  check(!s.areCommandsSuspended(), "D: resumeCommands() clears the latch");
  check(s.addQueueEntry(&cmd, false) == AQE_OK,
        "D: queueing works again on the same connection");
  check(s.queueEntries() == 1, "D: and the command is really queued");
  // queue_end.pos, not getCurrentPosition(): the pc_based queue has no ISR
  // stepping it, so the performed count stays 0 and only the queued position
  // moves. That divergence is the point of the original defect.
  check(t.q(&s)->queue_end.pos == 40,
        "D: the queued position advanced by the queued steps");
}

// E: forceStopAndNewPosition() sets it too, and the caller's new position is
// the base later commands build on.
static void e_force_stop_and_new_position() {
  connect();
  s.addQueueEntry(&cmd, false);
  s.forceStopAndNewPosition(1234);
  check(s.areCommandsSuspended(),
        "E: latch is set by forceStopAndNewPosition()");
  check(s.addQueueEntry(&cmd, false) == AQE_ERROR_COMMANDS_SUSPENDED,
        "E: refused after forceStopAndNewPosition()");
  s.resumeCommands();
  check(s.addQueueEntry(&cmd, false) == AQE_OK, "E: queueable after rearm");
  // queue_end.pos was set to 1234 by the stop; +40 for the queued command.
  check(t.q(&s)->queue_end.pos == 1274,
        "E: the new position is the base for later commands");
}

// F: the refusal must not be laundered through moveTimed(). tmrFrom() is a
// value-preserving cast, so AQE_ERROR_COMMANDS_SUSPENDED needs its own value in
// MoveTimedResultCode: -4 would alias MOVE_TIMED_TOO_LARGE_ERROR.
static void f_movetimed_value_preserved() {
  check(static_cast<int8_t>(AQE_ERROR_COMMANDS_SUSPENDED) == -5,
        "F: the AQE code is -5");
  check(static_cast<int8_t>(AQE_ERROR_COMMANDS_SUSPENDED) !=
            static_cast<int8_t>(MOVE_TIMED_TOO_LARGE_ERROR),
        "F: distinct from MOVE_TIMED_TOO_LARGE_ERROR");
  check(static_cast<MoveTimedResultCode>(AQE_ERROR_COMMANDS_SUSPENDED) ==
            MOVE_TIMED_COMMANDS_SUSPENDED,
        "F: tmrFrom() preserves it, no aliasing");

  connect();
  s.forceStop();
  uint32_t actual = 0;
  MoveTimedResultCode rc = s.moveTimed(40, 80 * 40, &actual, false);
  check(rc == MOVE_TIMED_COMMANDS_SUSPENDED,
        "F: moveTimed() surfaces the suspension, not a move error");
}

// G: the ramp path is unaffected. fill_queue() clears the latch on every active
// pass, which is why the old placement made the gate inert here.
static void g_ramp_rearms_itself() {
  connect();
  s.setSpeedInUs(1000);
  s.setAcceleration(1000);
  s.move(2000);
  t.fill(&s);
  check(!s.areCommandsSuspended(), "G: an active ramp leaves the latch clear");
  check(!s.isQueueEmpty(), "G: and the ramp queued commands");
}

int main() {
  a_refused_not_ok();
  b_queue_untouched();
  c_cannot_start();
  d_rearm();
  e_force_stop_and_new_position();
  f_movetimed_value_preserved();
  g_ramp_rearms_itself();
  if (failures != 0) {
    printf("%d failure(s)\n", failures);
    return 1;
  }
  printf("TEST_31 PASSED\n");
  return 0;
}
