#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

// Issue 370: a moveTimed() driven direction-change profile produces a
// library-position drift of -2 while the commanded step sum is 0.
//
// This test feeds the exact 87-command profile (from extras/issues/Issue370)
// through moveTimedFill() and checks that the generated low level commands
// still sum to the commanded value (net 0 per cycle). If moveTimedFill()
// corrupts the step accounting, the bug is in moveTimed(); otherwise the
// drift lives in the pulse-driver position tracking (RMT prefetch), not in
// command generation.
//
// The queue is modelled as 32 free entries (MT_QUEUE_LEN). Each command
// added via MT_ADD_ENTRY reduces fake_free by one. When moveTimedFill()
// returns a busy/retry code (queue full), fake_free is reset back to 32 to
// simulate the queue having drained, and the same command is retried.

#define MT_QUEUE_LEN 32

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

static struct captured {
  uint16_t ticks;
  uint8_t steps;
  bool count_up;
} log_cmd[2048];
static int log_idx;
static int fake_free;

#include "fas_arch/common.h"

static AqeResultCode capture(const struct stepper_command_s* cmd);

#define MT_QUEUE_EMPTY(s) (fake_free == MT_QUEUE_LEN)
#define MT_FREE_ENTRIES(s) (MT_QUEUE_LEN - fake_free)
#define MT_ADD_ENTRY(s, cmd, start) \
  ((cmd) == NULL ? AQE_OK : capture((cmd)))

#include "FastAccelStepper.h"
#include "fas_moveTimed/move_timed.h"

static AqeResultCode capture(const struct stepper_command_s* cmd) {
  log_cmd[log_idx].ticks = cmd->ticks;
  log_cmd[log_idx].steps = cmd->steps;
  log_cmd[log_idx].count_up = cmd->count_up;
  log_idx++;
  fake_free--;
  return AQE_OK;
}

static void reset() {
  log_idx = 0;
  fake_free = MT_QUEUE_LEN;
}

static int failures = 0;
static void check(int cond, const char* msg) {
  if (!cond) {
    printf("FAIL: %s\n", msg);
    failures++;
  }
}

struct Cmd {
  int16_t steps;
  uint32_t ticks;
};

// The 87-command profile from Issue370 (net = 0)
static const Cmd pattern[] = {
    {+26, 132450}, {+26, 132450}, {+26, 132450}, {+26, 132450},
    {+26, 132450}, {+26, 132450}, {+26, 132450}, {+26, 132450},
    {+26, 132450}, {+26, 132450}, {+26, 132450}, {+26, 132450},
    {+26, 132450}, {+26, 132450}, {+26, 132450}, {+26, 132450},
    {+26, 132450}, {+26, 132450}, {+26, 132450}, {+26, 132450},
    {-2, 132450},  {-7, 132450},
    {-26, 132450}, {-26, 132450}, {-26, 132450}, {-26, 132450},
    {-26, 132450}, {-26, 132450}, {-26, 132450}, {-26, 132450},
    {-26, 132450}, {-26, 132450}, {-26, 132450}, {-26, 132450},
    {-26, 132450}, {-26, 132450}, {-26, 132450}, {-26, 132450},
    {-26, 132450}, {-26, 132450},
    {-17, 132450}, {-10, 132450}, {-7, 132450}, {-5, 132450},
    {-2, 132450},  {-1, 132450},  {-1, 132450},
    {+1, 132450},  {+2, 132450},  {+5, 132450},  {+10, 132450},
    {+17, 132450},
    {-1, 132450}, {-1, 132450}, {-1, 132450}, {-1, 132450},
    {-1, 132450}, {-1, 132450}, {-1, 132450}, {-1, 132450},
    {-1, 132450}, {-1, 132450}, {-1, 132450}, {-1, 132450},
    {-1, 132450}, {-1, 132450}, {-1, 132450}, {-1, 132450},
    {-1, 132450}, {-1, 132450}, {-1, 132450}, {-1, 132450},
    {-1, 132450}, {-1, 132450}, {-1, 132450}, {-1, 132450},
    {-1, 132450}, {-1, 132450}, {-1, 132450}, {-1, 132450},
    {-1, 132450}, {-1, 132450}, {-1, 132450}, {-1, 132450},
    {-1, 132450}, {-1, 132450}, {-1, 132450},
};

static bool isRetry(MoveTimedResultCode rc) {
  // Only these codes mean the move was NOT appended. MOVE_TIMED_EMPTY (7)
  // looks positive but is a success ("queue was empty, move appended").
  switch (rc) {
    case MOVE_TIMED_BUSY:
    case MoveTimedResultCode::QueueFull:
    case MoveTimedResultCode::DirPinIsBusy:
    case MoveTimedResultCode::WaitForEnablePinActive:
    case MoveTimedResultCode::DeviceNotReady:
      return true;
    default:
      return false;
  }
}

int main() {
  int64_t commanded_sum = 0;
  int64_t generated_sum = 0;
  int pattern_len = (int)(sizeof(pattern) / sizeof(pattern[0]));
  int cycles = 2;

  reset();
  check(fake_free == MT_QUEUE_LEN, "queue must start with MT_QUEUE_LEN free");

  for (int c = 0; c < cycles; c++) {
    for (int i = 0; i < pattern_len; i++) {
      commanded_sum += pattern[i].steps;
      int retries = 0;
      for (;;) {
        MoveTimedResultCode rc = moveTimedFill(NULL, pattern[i].steps,
                                               pattern[i].ticks, NULL, false);
        if (isRetry(rc)) {
          // move was NOT appended: simulate queue drain and resend SAME
          // command, like the Issue370 feedCommand() loop
          fake_free = MT_QUEUE_LEN;
          retries++;
          if (retries > 1000) {
            check(false, "too many retries");
            rc = MOVE_TIMED_OK;
            break;
          }
          continue;
        }
        check(rc == MOVE_TIMED_OK || rc == MOVE_TIMED_EMPTY,
              "moveTimedFill rejected a valid command");
        break;
      }
    }
  }

  check(fake_free <= MT_QUEUE_LEN, "fake_free may not exceed MT_QUEUE_LEN");
  int queue_cmds = log_idx;
  for (int i = 0; i < log_idx; i++) {
    generated_sum += log_cmd[i].count_up ? log_cmd[i].steps
                                         : -(int)log_cmd[i].steps;
  }

  printf("pattern_len  = %d (x%d cycles)\n", pattern_len, cycles);
  printf("queue cmds   = %d\n", queue_cmds);
  printf("free after   = %d\n", fake_free);
  printf("commanded    = %" PRId64 "\n", (int64_t)commanded_sum);
  printf("generated    = %" PRId64 "\n", (int64_t)generated_sum);

  check(commanded_sum == 0, "pattern itself must net to zero");
  check(generated_sum == commanded_sum,
        "moveTimedFill must preserve the commanded step sum");

  if (failures != 0) {
    printf("TEST_24 FAILED (%d failures)\n", failures);
    return 1;
  }
  printf("TEST_24 PASSED (moveTimed generation preserves commanded sum)\n");
  return 0;
}