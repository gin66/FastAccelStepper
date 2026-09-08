#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Issue 370: a moveTimed() driven direction-change profile produces a
// library-position drift of -2 while the commanded step sum is 0. The -2
// appears only from the second cycle on (-2 * (cycles - 1)).
//
// Phase 1 replays the Issue370 runCycles() feed on a real 16-entry queue:
// prefill until queue-full, start, then feed with drift compensation. When a
// command cannot be enqueued (BUSY), the RMT ISR is simulated by draining a
// chunk with the real rmt_fill_buffer(). Checks:
//   - every cycle nets to the commanded sum (= 0)
//   - all cycles produce identical command streams (steps + count_up)
//   - the queued position (queue_end.pos) returns to zero
//
// Phase 2 feeds the SAME captured command stream through the real ESP32 RMT
// translation and validates that the encoded symbols reproduce the exact
// step count and total ticks, that the direction-change pause insertion is
// consistent, and that the queue_end position still nets to zero.
//
// If both phases pass, command generation and RMT translation are faithful:
// the -2 per cycle must live in the wire-level DIR-toggle alignment (RMT
// one-symbol prefetch racing the external driver), not in the library.

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

static struct captured {
  uint16_t ticks;
  uint8_t steps;
  bool count_up;
} log_cmd[2048];
static int log_idx;

#include "fas_arch/common.h"

// Phase 2: feed the captured command stream through the real ESP32 RMT fill
// buffer translation (the code that produces the RMT symbols on hardware).
// This must come before fas_moveTimed/move_timed.h so that StepperQueue is a
// complete type while the MT_* macros are expanded.
uint16_t debug_part_size = 24;
#define SUPPORT_ESP32_RMT
#define IRAM_ATTR
#define RMT_CHANNEL_T int
#define LL_TOGGLE_PIN(dirPin)
#include "pd_esp32/StepperISR_esp32xx_rmt.cpp"

#include "FastAccelStepper.h"

static StepperQueue feed_q;
static StepperQueue rmt_q;

#define MAX_RMT_ENTRIES 32768
static uint32_t rmt_entries[MAX_RMT_ENTRIES];
static uint32_t drain_syms[64];

static int failures = 0;
static void check(int cond, const char* msg) {
  if (!cond) {
    printf("FAIL: %s\n", msg);
    failures++;
  }
}

// Mirror StepperQueue::addQueueEntry() (queue_add_entry.cpp) for the
// standalone test: maintains queue_end position, toggle flags and the write
// index. startQueue() is emulated by setting _isRunning.
static AqeResultCode enqueue(StepperQueue* q,
                             const struct stepper_command_s* cmd, bool start) {
  if (cmd == NULL) {
    if (start && !q->isRunning()) {
      if (q->next_write_idx == q->read_idx) {
        return AQE_ERROR_EMPTY_QUEUE_TO_START;
      }
      q->_isRunning = true;
    }
    return AQE_OK;
  }
  if (q->isQueueFull()) {
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
  uint8_t wp = q->next_write_idx;
  struct queue_entry* e = &q->entry[wp & QUEUE_LEN_MASK];
  bool dir = (cmd->count_up == q->dirHighCountsUp);
  bool toggle_dir = false;
  if (q->dirPin != PIN_UNDEFINED) {
    if ((q->isQueueEmpty() && !q->isRunning()) &&
        ((q->dirPin & PIN_EXTERNAL_FLAG) == 0)) {
      q->queue_end.dir = dir;
    } else {
      toggle_dir = (dir != q->queue_end.dir);
    }
  }
  e->steps = steps;
  e->dirPinState = dir;
  e->toggle_dir = toggle_dir;
  e->countUp = cmd->count_up ? 1 : 0;
  e->moreThanOneStep = steps > 1 ? 1 : 0;
  e->hasSteps = steps > 0 ? 1 : 0;
  e->ticks = period;
  struct queue_end_s next_queue_end = q->queue_end;
#if defined(SUPPORT_QUEUE_ENTRY_END_POS_U16)
  e->end_pos_last16 = (uint32_t)next_queue_end.pos & 0xffff;
#endif
  next_queue_end.pos = next_queue_end.pos + (cmd->count_up ? steps : -steps);
  next_queue_end.dir = dir;
  next_queue_end.count_up = cmd->count_up;
  q->next_write_idx = wp + 1;
  q->queue_end = next_queue_end;

  if (q == &feed_q &&
      (uint32_t)log_idx < sizeof(log_cmd) / sizeof(log_cmd[0])) {
    log_cmd[log_idx].ticks = period;
    log_cmd[log_idx].steps = steps;
    log_cmd[log_idx].count_up = cmd->count_up;
    log_idx++;
  }

  if (!q->isRunning() && start) {
    q->_isRunning = true;
  }
  q->_last_command_ticks = period;
  return AQE_OK;
}

#define MT_QUEUE_EMPTY(s) (feed_q.isQueueEmpty())
#define MT_FREE_ENTRIES(s) (feed_q.queueEntries())
#define MT_ADD_ENTRY(s, cmd, start) enqueue(&feed_q, (cmd), (start))

#include "fas_moveTimed/move_timed.h"

static void reset() {
  log_idx = 0;
  __builtin_memset(&feed_q, 0, sizeof(feed_q));
  feed_q._base_initVars();
  feed_q._pd_initVars();
  feed_q.setDirPin(1, true);
}

// ---- Phase 2: RMT translation of the captured command stream --------------

struct RmtResult {
  uint32_t step_count;
  uint64_t total_ticks;
};

static uint32_t rmt_pause_symbol() {
  return 0x00010001 * ((MIN_CMD_TICKS + 2 * PART_SIZE - 1) / (2 * PART_SIZE));
}

static RmtResult analyze_rmt(uint32_t count) {
  RmtResult result = {0, 0};
  bool step_high = false;
  for (uint32_t i = 0; i < count; i++) {
    uint16_t low = rmt_entries[i] & 0xffff;
    uint16_t high = rmt_entries[i] >> 16;
    uint16_t low_ticks = low & 0x7fff;
    uint16_t high_ticks = high & 0x7fff;
    if ((low & 0x8000) && !step_high) {
      result.step_count++;
      step_high = true;
    } else if (!(low & 0x8000) && step_high) {
      step_high = false;
    }
    result.total_ticks += low_ticks;
    if ((high & 0x8000) && !step_high) {
      result.step_count++;
      step_high = true;
    } else if (!(high & 0x8000) && step_high) {
      step_high = false;
    }
    result.total_ticks += high_ticks;
  }
  return result;
}

static void feed_rmt() {
  __builtin_memset(&rmt_q, 0, sizeof(rmt_q));
  rmt_q._base_initVars();
  rmt_q._pd_initVars();
  rmt_q.setDirPin(1, true);

  uint32_t rmt_offset = 0;
  uint32_t fed = 0;
  uint32_t toggle_entries = 0;
  uint64_t base_ticks = 0;

  while (fed < (uint32_t)log_idx || rmt_q.read_idx != rmt_q.next_write_idx) {
    while (fed < (uint32_t)log_idx) {
      uint8_t queued = rmt_q.next_write_idx - rmt_q.read_idx;
      if (queued >= QUEUE_LEN) {
        break;
      }
      struct stepper_command_s cmd = {.ticks = log_cmd[fed].ticks,
                                      .steps = log_cmd[fed].steps,
                                      .count_up = log_cmd[fed].count_up};
      AqeResultCode rc = enqueue(&rmt_q, &cmd, false);
      check(rc == AQE_OK, "phase2: enqueue rejected a generated command");
      if (rc != AQE_OK) {
        return;
      }
      const struct queue_entry* e =
          &rmt_q.entry[(uint8_t)(rmt_q.next_write_idx - 1) & QUEUE_LEN_MASK];
      if (e->toggle_dir) {
        toggle_entries++;
      }
      if (e->steps > 0) {
        base_ticks += (uint64_t)e->steps * e->ticks;
      } else {
        base_ticks += e->ticks;
      }
      fed++;
    }
    if (rmt_q.read_idx == rmt_q.next_write_idx) {
      break;
    }
    if (rmt_offset + PART_SIZE > MAX_RMT_ENTRIES) {
      check(false, "phase2: RMT entry buffer overflow");
      break;
    }
    rmt_fill_buffer(&rmt_q, true, &rmt_entries[rmt_offset]);
    rmt_offset += PART_SIZE;
  }

  // Direction-change pauses are now inserted by addQueueEntry()
  // (BEFORE_DIR_CHANGE_DELAY_TICKS), NOT by the RMT fill buffer. So the RMT
  // translation must not add any pause symbols of its own: every toggle_dir
  // entry is emitted as-is and the total ticks must equal the base command
  // ticks (no injected pause ticks).
  uint32_t pause_sym = rmt_pause_symbol();
  uint32_t pauses = 0;
  for (uint32_t i = 0; i + PART_SIZE <= rmt_offset; i++) {
    if (rmt_entries[i] == pause_sym) {
      pauses++;
      i += PART_SIZE - 1;
    }
  }

  RmtResult rmt = analyze_rmt(rmt_offset);
  uint32_t steps_expected = 0;
  for (int i = 0; i < log_idx; i++) {
    steps_expected += log_cmd[i].steps;
  }

  printf("phase2 feed  = %u cmds -> %u rmt entries\n", fed, rmt_offset);
  printf("toggle cmds  = %u (pause symbols injected: %u)\n", toggle_entries,
         pauses);
  printf("base ticks   = %" PRIu64 "\n", (int64_t)base_ticks);
  printf("RMT steps    = %u (expected %u)\n", rmt.step_count, steps_expected);
  printf("RMT ticks    = %" PRIu64 " (expected %" PRIu64 ")\n",
         (int64_t)rmt.total_ticks, (int64_t)base_ticks);
  printf("queue_end.pos= %" PRId32 "\n", (int32_t)rmt_q.queue_end.pos);

  check(fed == (uint32_t)log_idx, "phase2: not all commands were fed");
  check(pauses == 0, "phase2: RMT fill buffer must not inject pause symbols");
  check(rmt.step_count == steps_expected,
        "phase2: step pulses were lost or added by rmt_fill_buffer");
  check(rmt.total_ticks == base_ticks,
        "phase2: total ticks changed by rmt_fill_buffer");
  check(rmt_q.queue_end.pos == 0,
        "phase2: queue_end position drifted from commanded sum");
}

struct Cmd {
  int16_t steps;
  uint32_t ticks;
};

// The 87-command profile from Issue370 (net = 0 per cycle)
static const Cmd pattern[] = {
    {+26, 105960}, {+26, 105960}, {+26, 105960}, {+26, 105960}, {+26, 105960},
    {+26, 105960}, {+26, 105960}, {+26, 105960}, {+26, 105960}, {+26, 105960},
    {+26, 105960}, {+26, 105960}, {+26, 105960}, {+26, 105960}, {+26, 105960},
    {+26, 105960}, {+26, 105960}, {+26, 105960}, {+26, 105960}, {+26, 105960},
    {-2, 105960},  {-7, 105960},  {-26, 105960}, {-26, 105960}, {-26, 105960},
    {-26, 105960}, {-26, 105960}, {-26, 105960}, {-26, 105960}, {-26, 105960},
    {-26, 105960}, {-26, 105960}, {-26, 105960}, {-26, 105960}, {-26, 105960},
    {-26, 105960}, {-26, 105960}, {-26, 105960}, {-26, 105960}, {-26, 105960},
    {-17, 105960}, {-10, 105960}, {-7, 105960},  {-5, 105960},  {-2, 105960},
    {-1, 105960},  {-1, 105960},  {+1, 105960},  {+2, 105960},  {+5, 105960},
    {+10, 105960}, {+17, 105960}, {-1, 105960},  {-1, 105960},  {-1, 105960},
    {-1, 105960},  {-1, 105960},  {-1, 105960},  {-1, 105960},  {-1, 105960},
    {-1, 105960},  {-1, 105960},  {-1, 105960},  {-1, 105960},  {-1, 105960},
    {-1, 105960},  {-1, 105960},  {-1, 105960},  {-1, 105960},  {-1, 105960},
    {-1, 105960},  {-1, 105960},  {-1, 105960},  {-1, 105960},  {-1, 105960},
    {-1, 105960},  {-1, 105960},  {-1, 105960},  {-1, 105960},  {-1, 105960},
    {-1, 105960},  {-1, 105960},  {-1, 105960},  {-1, 105960},  {-1, 105960},
    {-1, 105960},  {-1, 105960},
};

// Feed one pattern command like Issue370::feedCommand(): retry on queue-full,
// with the real RMT ISR drain (one chunk per retry). track drift.
static bool feedCommand(int16_t steps, uint32_t ticks, int32_t& drift,
                        int64_t& commanded_sum) {
  uint32_t duration = (uint32_t)((int64_t)ticks + drift);
  uint32_t actual = 0;
  for (;;) {
    MoveTimedResultCode rc =
        moveTimedFill(NULL, steps, duration, &actual, false);
    switch (rc) {
      case MOVE_TIMED_OK:
      case MOVE_TIMED_EMPTY:
        drift = (int32_t)(duration - actual);
        commanded_sum += steps;
        return true;
      case MOVE_TIMED_BUSY:
      case MoveTimedResultCode::QueueFull:
      case MoveTimedResultCode::DirPinIsBusy:
      case MoveTimedResultCode::WaitForEnablePinActive:
        // Simulate the RMT ISR consuming one chunk from the queue
        rmt_fill_buffer(&feed_q, true, drain_syms);
        break;
      case MoveTimedResultCode::DeviceNotReady:
        break;
      default:
        printf("feedCommand error: rc=%d steps=%d dur=%lu\n", (int)rc,
               (int)steps, (unsigned long)duration);
        return false;
    }
  }
}

// Phase 0: capacity guard for the direction-change reservation (Issue 370).
// moveTimedFill() must reserve 2 queue slots for the direction-change pauses
// that FastAccelStepper::addQueueEntry() inserts on top of the step command.
// So a move generating k commands must be rejected while fewer than k+2 slots
// are free, instead of being admitted and then failing mid-append (which would
// silently drop steps).
static void cap_check() {
  reset();
  if (feed_q.queueEntries() != 0) {
    check(false, "cap: queue must start empty");
    return;
  }
  uint32_t actual = 0;
  // Empty queue: all 16 slots free. A 255-step fast move (1 command, rate 100
  // ticks/step) needs 1 + 2 reserved = 3 slots for a direction change, so it
  // must be accepted.
  MoveTimedResultCode rc = moveTimedFill(NULL, 255, 255 * 100, &actual, false);
  if (rc != MOVE_TIMED_OK && rc != MOVE_TIMED_EMPTY) {
    printf("cap: expected accept with full free queue, got rc=%d\n", (int)rc);
    check(false, "cap: fully-free queue rejected a 1-command move");
    return;
  }
  // The single command occupies one slot; 15 remain free. Two more 255-step
  // commands (one slot each) must still fit within the k+2 guard, but a
  // direction change needs 3 slots so the queue must reject only when the
  // free budget (minus 2 reserved) is exhausted.
  reset();
  // Pre-fill the queue so that exactly 2 slots remain free BEFORE the
  // reservation: emulate the dir-change pause need by asserting that a 1-step
  // move is rejected when free < 1 + 2.
  for (int i = 0; i < QUEUE_LEN - 3; i++) {
    struct stepper_command_s c = {.ticks = 10000, .steps = 1, .count_up = true};
    AqeResultCode erc = enqueue(&feed_q, &c, false);
    if (erc != AQE_OK) {
      check(false, "cap: setup enqueue failed");
      return;
    }
  }
  // 3 slots free now. A 1-command move needs 1 + 2 = 3 -> fits.
  rc = moveTimedFill(NULL, 1, 10000, NULL, false);
  check(rc == MOVE_TIMED_OK || rc == MOVE_TIMED_EMPTY,
        "cap: 1-command move with exactly 3 free slots must be accepted");
  // 2 slots free now. The same move needs 3 -> must be rejected (BUSY).
  rc = moveTimedFill(NULL, 1, 10000, NULL, false);
  check(rc == MOVE_TIMED_BUSY,
        "cap: 1-command move with 2 free slots must be rejected (BUSY)");
}

int main() {
  cap_check();
  const int cycles = 3;
  int64_t commanded_sum = 0;
  int pattern_len = (int)(sizeof(pattern) / sizeof(pattern[0]));
  int cycle_start[4];

  reset();
  check(feed_q.queueEntries() == 0, "queue must start empty");

  // Replay Issue370::runCycles(): prefill without starting until the 16-entry
  // queue is full, then start and keep feeding with drift compensation.
  int32_t drift = 0;
  bool started = false;
  int prefilled = 0;
  for (int c = 0; c < cycles; c++) {
    cycle_start[c] = log_idx;
    for (int i = 0; i < pattern_len; i++) {
      if (!started) {
        uint32_t actual = 0;
        MoveTimedResultCode rc = moveTimedFill(
            NULL, pattern[i].steps, pattern[i].ticks, &actual, false);
        if (rc == MOVE_TIMED_OK || rc == MOVE_TIMED_EMPTY) {
          drift = (int32_t)(pattern[i].ticks - actual);
          commanded_sum += pattern[i].steps;
          prefilled++;
          continue;
        }
        if (rc != MOVE_TIMED_BUSY) {
          printf("prefill error: rc=%d at cmd %d\n", (int)rc, i);
          return 1;
        }
        moveTimedFill(NULL, 0, 0, NULL, true);  // start the queue
        started = true;
      }
      if (!feedCommand(pattern[i].steps, pattern[i].ticks, drift,
                       commanded_sum)) {
        return 1;
      }
    }
  }
  cycle_start[cycles] = log_idx;

  // Wait for physical completion: drain the queue via the real ISR fill code
  while (feed_q.read_idx != feed_q.next_write_idx) {
    rmt_fill_buffer(&feed_q, true, drain_syms);
  }

  int64_t generated_sum = 0;
  for (int i = 0; i < log_idx; i++) {
    generated_sum +=
        log_cmd[i].count_up ? log_cmd[i].steps : -(int)log_cmd[i].steps;
  }

  printf("pattern_len  = %d (x%d cycles)\n", pattern_len, cycles);
  printf("prefilled    = %d cmds, then queue started\n", prefilled);
  printf("queue cmds   = %d\n", log_idx);
  printf("commanded    = %" PRId64 "\n", (int64_t)commanded_sum);
  printf("generated    = %" PRId64 "\n", (int64_t)generated_sum);
  printf("queue_end.pos= %" PRId32 "\n", (int32_t)feed_q.queue_end.pos);

  check(commanded_sum == 0, "pattern itself must net to zero");
  check(generated_sum == commanded_sum,
        "moveTimedFill must preserve the commanded step sum");
  check(feed_q.queue_end.pos == 0,
        "queue_end position must return to zero after a full run");

  for (int c = 0; c < cycles; c++) {
    int64_t sum = 0;
    int n = cycle_start[c + 1] - cycle_start[c];
    for (int j = cycle_start[c]; j < cycle_start[c + 1]; j++) {
      sum += log_cmd[j].count_up ? log_cmd[j].steps : -(int)log_cmd[j].steps;
    }
    printf("cycle %d       = %d cmds, sum %" PRId64 "\n", c, n, (int64_t)sum);
    check(sum == 0, "each cycle must net to zero");
    check(n == cycle_start[1] - cycle_start[0],
          "all cycles must generate the same command count");
    if (c > 0) {
      for (int j = 0; j < n; j++) {
        const struct captured* a = &log_cmd[cycle_start[0] + j];
        const struct captured* b = &log_cmd[cycle_start[c] + j];
        if (a->steps != b->steps || a->count_up != b->count_up) {
          check(false, "all cycles must generate identical commands");
          break;
        }
      }
    }
  }

  feed_rmt();

  if (failures != 0) {
    printf("TEST_24 FAILED (%d failures)\n", failures);
    return 1;
  }
  printf(
      "TEST_24 PASSED "
      "(moveTimed + RMT translation preserve commanded position)\n");
  return 0;
}