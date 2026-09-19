// test_25: moveTimed() pause reporting, DIR-preserving dwells,
// and prepare_revert
//
// When a direction change forces the queue driver to inject a pause, the
// moveTimed() call returns AQE_DIR_CHANGE_PAUSE_INJECTED (6) or
// AQE_DIR_PIN_2MS_PAUSE_ADDED (5) and the command was NOT enqueued. In this
// case *actual_duration carries the ticks of the one injected pause and the
// caller accumulates the value (extra) across retries, computing the drift
// only after the move is accepted: drift = duration - (actual + extra).
//
// Scenario A: per-call zeroing - actual_duration is never cumulative.
// Scenario B: regular dir pin with before+after pause (one pause per call).
// Scenario C: external dir pin with the 2ms pause mechanism.
//
// Like test_24, this is self-contained: it defines the queue/stepper members
// used by the inline bodies of fas_member/move_timed.h and
// fas_member/add_queue_entry.h and must not be linked against LIB_O.

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

#include "fas_arch/common.h"

#include "FastAccelStepper.h"
#include "fas_queue/stepper_queue.h"

// Direction-change pause insertion is provided by the pd_test default in
// pd_test/dir_change_pause.h; this standalone test does not link the library
// .o files, so it includes that default itself. It must be included after
// StepperQueue and its BEFORE/AFTER macros are known. The pd_test target
// #defines inline away, so this must be included in exactly one TU per binary.
#include "pd_test/dir_change_pause.h"

// Production bodies of addQueueEntry() and moveTimed(), compiled against this
// test's StepperQueue/FastAccelStepper instead of the hardware queues.
#include "fas_member/add_queue_entry.h"
#include "fas_member/move_timed.h"

static StepperQueue test_q;

static int failures = 0;
static void check(int cond, const char* msg) {
  if (!cond) {
    printf("FAIL: %s\n", msg);
    failures++;
  }
}

// Test-stepper configuration (applied by the overridden init() below).
static uint8_t g_dir_pin = 1;
static uint16_t g_dir_delay_ticks = 0;
static bool g_external_dir_pin_fail = false;

// The test is not linked against FastAccelStepper.o, so provide the
// FastAccelStepper members that addQueueEntry()/moveTimed() reference. Their
// production bodies come from fas_member/add_queue_entry.h and
// fas_member/move_timed.h above.
StepperQueue* FastAccelStepper::_queue() const { return &test_q; }
bool FastAccelStepper::enableOutputs() { return true; }
bool FastAccelStepper::isQueueRunning() const { return _queue()->isRunning(); }
bool FastAccelStepper::handleExternalDirectionPin(StepperQueue* q,
                                                  bool count_up) {
  (void)q;
  (void)count_up;
  if (g_external_dir_pin_fail) {
    g_external_dir_pin_fail = false;
    return false;
  }
  return true;
}
uint8_t FastAccelStepper::queueEntries() const {
  return _queue()->queueEntries();
}
bool FastAccelStepper::isQueueEmpty() const { return _queue()->isQueueEmpty(); }

// Test-only init: TEST makes init() public. Configure the stepper for the test
// (no engine auto-enable, dir pin from g_dir_pin).
void FastAccelStepper::init(FastAccelStepperEngine* engine, uint8_t num,
                            uint8_t step_pin) {
  (void)engine;
  (void)step_pin;
  _engine = NULL;
  _queue_num = num;
  _dirPin = g_dir_pin;
  _dirHighCountsUp = true;
  _autoEnable = false;
  _on_delay_ticks = 0;
  _dir_change_delay_ticks = g_dir_delay_ticks;
  _auto_disable_delay_counter = 0;
  _off_delay_count = 1;
}

// Test has no real engine, so the dir-pin is never busy.
bool FastAccelStepperEngine::isDirPinBusy(uint8_t dirPin,
                                          uint8_t except_stepper) {
  (void)dirPin;
  (void)except_stepper;
  return false;
}

// StepperQueue::addQueueEntry() (queue_add_entry.cpp) for the standalone test:
// maintains queue_end position, toggle flags and the write index, including
// the pause-stats bookkeeping used by SUPPORT_PAUSE_CMD_COUNTING. startQueue()
// is emulated by setting _isRunning.
AqeResultCode StepperQueue::addQueueEntry(const struct stepper_command_s* cmd,
                                          bool start) {
  if (cmd == NULL) {
    if (start && !isRunning()) {
      if (next_write_idx == read_idx) {
        return AQE_ERROR_EMPTY_QUEUE_TO_START;
      }
      _isRunning = true;
    }
    return AQE_OK;
  }
  if (isQueueFull()) {
    return AQE_QUEUE_FULL;
  }
  uint16_t period = cmd->ticks;
  uint8_t steps = cmd->steps;
  uint32_t command_rate_ticks = period;
  if (steps > 1) {
    command_rate_ticks *= steps;
  }
  if (command_rate_ticks < MIN_CMD_TICKS) {
    return AQE_ERROR_TICKS_TOO_LOW;
  }
  uint8_t wp = next_write_idx;
  struct queue_entry* e = &entry[wp & QUEUE_LEN_MASK];
  bool dir = (cmd->count_up == dirHighCountsUp);
  bool toggle_dir = false;
  bool dir_changed = false;
  if (dirPin != PIN_UNDEFINED) {
    dir_changed = (dir != queue_end.dir);
    if ((isQueueEmpty() && !isRunning()) &&
        ((dirPin & PIN_EXTERNAL_FLAG) == 0)) {
      queue_end.dir = dir;
    } else {
      toggle_dir = dir_changed;
    }
  }
  e->steps = steps;
  e->dirPinState = dir;
  e->toggle_dir = toggle_dir;
  e->countUp = cmd->count_up ? 1 : 0;
  e->moreThanOneStep = steps > 1 ? 1 : 0;
  e->hasSteps = steps > 0 ? 1 : 0;
  e->ticks = period;
  struct queue_end_s next_queue_end = queue_end;
#if defined(SUPPORT_QUEUE_ENTRY_END_POS_U16)
  e->end_pos_last16 = (uint32_t)next_queue_end.pos & 0xffff;
#endif
  next_queue_end.pos = next_queue_end.pos + (cmd->count_up ? steps : -steps);
  next_queue_end.dir = dir;
  next_queue_end.count_up = cmd->count_up;
  next_write_idx = wp + 1;
  queue_end = next_queue_end;

  if (!isRunning() && start) {
    _isRunning = true;
  }
  if ((steps > 0) || dir_changed) {
    clear_pause_stats();
  } else {
    if (_nr_of_pauses < 255) {
      _nr_of_pauses++;
    }
    if (65535 - _last_pause_ticks >= period) {
      _last_pause_ticks += period;
    } else {
      _last_pause_ticks = 65535;
    }
  }
  return AQE_OK;
}

// Test stepper: points addQueueEntry()/moveTimed() at test_q (num 0).
static FastAccelStepper test_stepper;

static void reset(uint8_t dir_pin, uint16_t before_delay_ticks,
                  uint16_t after_delay_ticks, uint16_t dir_delay_ticks) {
  __builtin_memset(&test_q, 0, sizeof(test_q));
  test_q._base_initVars();
  test_q._pd_initVars();
  test_q._before_dir_change_delay_ticks = before_delay_ticks;
  test_q._after_dir_change_delay_ticks = after_delay_ticks;
  test_q.setDirPin(dir_pin, true);
  g_dir_pin = dir_pin;
  g_dir_delay_ticks = dir_delay_ticks;
  g_external_dir_pin_fail = false;
  test_stepper.init(NULL, 0, 0);
}

// ---- Scenario A: per-call zeroing ---------------------------------------

static void test_per_call_zeroing() {
  printf("Running: actual_duration is per-call (not cumulative)\n");
  reset(1, 0, 0, 0);

  uint32_t actual = 0;
  uint32_t duration = 12000;
  MoveTimedResultCode rc = test_stepper.moveTimed(3, duration, &actual, false);
  check(rc == MOVE_TIMED_OK || rc == MOVE_TIMED_EMPTY,
        "A: first move accepted");
  check(actual == duration, "A: first actual == duration");

  // Direction change WITHOUT any configured delay must not inject a pause and
  // the same 'actual' variable must hold only this move's ticks, i.e. the
  // result is not the sum of previous calls.
  duration = 24000;
  rc = test_stepper.moveTimed(-3, duration, &actual, false);
  check(rc == MOVE_TIMED_OK || rc == MOVE_TIMED_EMPTY,
        "A: second move accepted");
  check(actual == duration, "A: actual is per-call (not cumulative)");

  printf("  first=%u second=%u ticks\n", 12000u, duration);
}

// ---- Scenario B: before+after dir-change pause, one per call -------------

static void test_dir_change_pause_reporting() {
  printf("Running: dir-change pause(s) reported in the failing call\n");
  reset(1, US_TO_TICKS(1000), 0, US_TO_TICKS(500));

  uint32_t actual = 0;
  MoveTimedResultCode rc = test_stepper.moveTimed(2, 8000, &actual, false);
  check(rc == MOVE_TIMED_OK || rc == MOVE_TIMED_EMPTY,
        "B: first (UP) move accepted");
  check(actual == 8000, "B: first actual == duration");
  check(test_q.queueEntries() == 1, "B: first move enqueues one entry");
  printf("  UP move: rc=%d actual=%u\n", (int)rc, actual);

  // Reversal UP -> DOWN. Queue driver injects at most one pause per call:
  // call 1 injects the BEFORE pause, call 2 the AFTER (dir delay) pause,
  // call 3 enqueues the move itself. Every failing call reports its pause.
  uint32_t extra = 0;
  uint32_t before_ticks = US_TO_TICKS(1000);
  uint32_t after_ticks = US_TO_TICKS(500);
  uint32_t duration = 8000;
  printf("  expected before-pause=%u after-pause=%u ticks\n", before_ticks,
         after_ticks);

  rc = test_stepper.moveTimed(-2, duration, &actual, false);
  check(rc == MoveTimedResultCode::DirChangePauseInjected,
        "B: call 1 injects a pause");
  check(actual == before_ticks, "B: call 1 actual == before-pause ticks");
  check(test_q.queueEntries() == 2, "B: call 1 enqueues exactly one entry");
  extra += actual;
  printf("  DOWN call1: rc=%d actual=%u extra=%u\n", (int)rc, actual, extra);

  rc = test_stepper.moveTimed(-2, duration, &actual, false);
  check(rc == MoveTimedResultCode::DirChangePauseInjected,
        "B: call 2 injects a pause");
  check(actual == after_ticks, "B: call 2 actual == after-pause ticks");
  check(test_q.queueEntries() == 3, "B: call 2 enqueues exactly one entry");
  extra += actual;
  printf("  DOWN call2: rc=%d actual=%u extra=%u\n", (int)rc, actual, extra);

  rc = test_stepper.moveTimed(-2, duration, &actual, false);
  check(rc == MOVE_TIMED_OK || rc == MOVE_TIMED_EMPTY,
        "B: call 3 move accepted");
  check(actual == duration, "B: call 3 actual == move ticks");
  check(test_q.queueEntries() == 4, "B: call 3 enqueues the move entry");
  int32_t drift = (int32_t)(duration - (actual + extra));
  printf("  DOWN call3: rc=%d actual=%u drift=%d\n", (int)rc, actual, drift);
  check(drift == (int32_t)(duration - (duration + before_ticks + after_ticks)),
        "B: drift accounts for both injected pauses");
}

// ---- Scenario C: external dir pin 2ms pause ------------------------------

static void test_external_dir_pin_2ms() {
  printf("Running: external dir pin 2ms pause reported in failing call\n");
  reset(PIN_EXTERNAL_FLAG | 1, 0, 0, 0);
  g_external_dir_pin_fail = true;

  uint32_t actual = 0;
  MoveTimedResultCode rc = test_stepper.moveTimed(2, 8000, &actual, false);
  check(rc == MOVE_TIMED_OK || rc == MOVE_TIMED_EMPTY,
        "C: first (UP) move accepted");
  check(actual == 8000, "C: first actual == duration");
  printf("  UP move: rc=%d actual=%u\n", (int)rc, actual);

  // Reversal to DOWN: the offline external dir-pin callback fails once, so a
  // 2ms pause is injected and the call returns DirPin2msPauseAdded. The
  // reported actual_duration must be the 2ms pause ticks, not the move's.
  uint32_t extra = 0;
  uint32_t pause_ticks = US_TO_TICKS(2000);
  uint32_t duration = 8000;
  rc = test_stepper.moveTimed(-2, duration, &actual, false);
  check(rc == MoveTimedResultCode::DirPin2msPauseAdded,
        "C: 2ms pause injected");
  check(actual == pause_ticks, "C: actual == 2ms pause ticks");
  check(test_q.queueEntries() == 2, "C: pause call enqueues one entry");
  extra += actual;
  printf("  DOWN call1: rc=%d actual=%u extra=%u\n", (int)rc, actual, extra);

  rc = test_stepper.moveTimed(-2, duration, &actual, false);
  check(rc == MOVE_TIMED_OK || rc == MOVE_TIMED_EMPTY,
        "C: move accepted on retry");
  check(actual == duration, "C: retry actual == move ticks");
  check(test_q.queueEntries() == 3, "C: retry enqueues the move entry");
  int32_t drift = (int32_t)(duration - (actual + extra));
  printf("  DOWN call2: rc=%d actual=%u drift=%d\n", (int)rc, actual, drift);
  check(drift == (int32_t)-(pause_ticks),
        "C: drift accounts for the 2ms pause");
  check(g_external_dir_pin_fail == false,
        "C: external callback only failed once");
}

// ---- Scenario D: timekeeping pause keeps last direction --------------------

static void test_pause_inherits_direction() {
  printf("Running: moveTimed(0, dt) after DOWN does not toggle DIR\n");
  reset(1, 0, 0, 0);

  uint32_t actual = 0;
  MoveTimedResultCode rc = test_stepper.moveTimed(-3, 12000, &actual, false);
  check(rc == MOVE_TIMED_OK || rc == MOVE_TIMED_EMPTY, "D: DOWN move accepted");
  check(test_q.queue_end.count_up == false, "D: queue_end is DOWN");
  uint8_t entries_before = test_q.queueEntries();

  rc = test_stepper.moveTimed(0, 8000, &actual, false);
  check(rc == MOVE_TIMED_OK || rc == MOVE_TIMED_EMPTY, "D: pause accepted");
  check(actual == 8000, "D: pause actual == duration");
  check(test_q.queue_end.count_up == false, "D: pause kept DOWN");
  check(test_q.queueEntries() == (uint8_t)(entries_before + 1),
        "D: pause enqueued one entry");
  uint8_t wi = (uint8_t)(test_q.next_write_idx - 1);
  check(test_q.entry[wi & QUEUE_LEN_MASK].countUp == 0,
        "D: pause entry countUp is DOWN");
  check(test_q.entry[wi & QUEUE_LEN_MASK].toggle_dir == 0,
        "D: pause entry did not toggle DIR");
  check(test_q.entry[wi & QUEUE_LEN_MASK].hasSteps == 0,
        "D: pause has no steps");
}

// ---- Scenario E: prepare_revert pause flips DIR --------------------------

static void test_prepare_revert() {
  printf("Running: moveTimed(0, dt, prepare_revert) flips DIR\n");
  reset(1, 0, 0, 0);

  uint32_t actual = 0;
  MoveTimedResultCode rc = test_stepper.moveTimed(2, 8000, &actual, false);
  check(rc == MOVE_TIMED_OK || rc == MOVE_TIMED_EMPTY, "E: UP move accepted");
  check(test_q.queue_end.count_up == true, "E: queue_end is UP");

  rc = test_stepper.moveTimed(0, 8000, &actual, false, true);
  check(rc == MOVE_TIMED_OK || rc == MOVE_TIMED_EMPTY,
        "E: revert pause accepted");
  check(actual == 8000, "E: revert pause actual == duration");
  check(test_q.queue_end.count_up == false, "E: queue_end is DOWN");
  uint8_t wi = (uint8_t)(test_q.next_write_idx - 1);
  check(test_q.entry[wi & QUEUE_LEN_MASK].countUp == 0,
        "E: pause entry countUp is DOWN");
  check(test_q.entry[wi & QUEUE_LEN_MASK].toggle_dir == 1,
        "E: pause entry toggled DIR");
  check(test_q.entry[wi & QUEUE_LEN_MASK].hasSteps == 0,
        "E: pause has no steps");

  rc = test_stepper.moveTimed(-2, 8000, &actual, false);
  check(rc == MOVE_TIMED_OK || rc == MOVE_TIMED_EMPTY,
        "E: reversing moveTimed does not inject");
  check(actual == 8000, "E: reversing move actual == duration");
}

int main() {
  test_per_call_zeroing();
  test_dir_change_pause_reporting();
  test_external_dir_pin_2ms();
  test_pause_inherits_direction();
  test_prepare_revert();

  if (failures != 0) {
    printf("TEST_25 FAILED (%d failures)\n", failures);
    return 1;
  }
  printf(
      "TEST_25 PASSED (moveTimed pause ticks reported via "
      "*actual_duration; pauses inherit DIR; prepare_revert)\n");
  return 0;

}