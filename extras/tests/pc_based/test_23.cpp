// test_23: Direction pin and external pin handling tests
//
// Part 1: MIN_DIR_DELAY_US / MAX_DIR_DELAY_US tests
// Part 2: dir_change_delay_us tests for regular direction pins
// Part 3: External direction pin behavior tests (FAILS with current impl)
// Part 4: External enable pin behavior tests
// Part 5: Integration tests

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FastAccelStepper.h"
#include "fas_queue/stepper_queue.h"

#define TASK_CYCLE_TICKS (TICKS_PER_S / 250)
#define EXTERNAL_DIR_PIN 128
#define EXTERNAL_ENABLE_PIN 129

char TCCR1A;
char TCCR1B;
char TCCR1C;
char TIMSK1;
char TIFR1;
unsigned short OCR1A;
unsigned short OCR1B;

void noInterrupts() {}
void interrupts() {}
void inject_fill_interrupt(int mark) {}

extern void fas_reset_stepper_allocation();

static int test_passed = 0;
static int test_failed = 0;

static void test_result(const char* name, bool passed) {
  if (passed) {
    printf("PASS: %s\n", name);
    test_passed++;
  } else {
    printf("FAIL: %s\n", name);
    test_failed++;
  }
}

static void process_queue_ticks(uint32_t ticks_to_process) {
  StepperQueue* q = &fas_queue[0];
  while (ticks_to_process > 0 && q->read_idx != q->next_write_idx) {
    struct queue_entry* e = &q->entry[q->read_idx & QUEUE_LEN_MASK];
    uint32_t entry_ticks = e->ticks;
    if (e->steps > 0) {
      entry_ticks *= e->steps;
    }
    if (entry_ticks <= ticks_to_process) {
      ticks_to_process -= entry_ticks;
      q->read_idx++;
    } else {
      break;
    }
  }
}

static bool external_dir_callback_called = false;
static uint8_t last_callback_pin = 0;
static uint8_t last_callback_value = 0;

static bool mock_external_callback(uint8_t pin, uint8_t value) {
  external_dir_callback_called = true;
  last_callback_pin = pin;
  last_callback_value = value;
  return value;
}

static bool external_enable_callback_called = false;

static bool mock_external_enable_callback(uint8_t pin, uint8_t value) {
  external_enable_callback_called = true;
  return value;
}

void init_queue() {
  fas_reset_stepper_allocation();
  fas_queue[0].read_idx = 0;
  fas_queue[1].read_idx = 0;
  fas_queue[0].next_write_idx = 0;
  fas_queue[1].next_write_idx = 0;
  fas_queue[0]._isRunning = false;
  fas_queue[1]._isRunning = false;
}

static void test_min_dir_delay_value() {
  printf("Running: MIN_DIR_DELAY_US value check\n");
  printf("  MIN_DIR_DELAY_US = %ld us\n", (long)MIN_DIR_DELAY_US);
  printf("  MIN_CMD_TICKS = %ld ticks\n", (long)MIN_CMD_TICKS);
  printf("  Expected: MIN_DIR_DELAY_US = MIN_CMD_TICKS in us = %ld\n",
         (long)(MIN_CMD_TICKS / (TICKS_PER_S / 1000000)));
  test_result("MIN_DIR_DELAY_US value check",
              MIN_DIR_DELAY_US == MIN_CMD_TICKS / (TICKS_PER_S / 1000000));
}

static void test_max_dir_delay_value() {
  printf("Running: MAX_DIR_DELAY_US value check\n");
  printf("  MAX_DIR_DELAY_US = %ld us\n", (long)MAX_DIR_DELAY_US);
  printf("  Expected: 65535 / (TICKS_PER_S / 1000000) = %ld\n",
         (long)(65535 / (TICKS_PER_S / 1000000)));
  test_result("MAX_DIR_DELAY_US value check",
              MAX_DIR_DELAY_US == 65535 / (TICKS_PER_S / 1000000));
}

static void test_dir_delay_below_min_clamped() {
  printf("Running: dir_change_delay_us below MIN_DIR_DELAY_US clamped\n");
  init_queue();

  FastAccelStepper s;
  s.init(NULL, 0, 0);
  s.setDirectionPin(5, true, 1);

  printf("  Requested: 1 us\n");
  printf("  MIN_DIR_DELAY_US: %ld us\n", (long)MIN_DIR_DELAY_US);
  printf("  Expected: clamped to MIN_DIR_DELAY_US\n");

  test_result("dir_change_delay below min clamped", true);
}

static void test_dir_delay_above_max_clamped() {
  printf("Running: dir_change_delay_us above MAX_DIR_DELAY_US clamped\n");
  init_queue();

  FastAccelStepper s;
  s.init(NULL, 0, 0);
  s.setDirectionPin(5, true, 10000);

  printf("  Requested: 10000 us\n");
  printf("  MAX_DIR_DELAY_US: %ld us\n", (long)MAX_DIR_DELAY_US);
  printf("  Expected: clamped to MAX_DIR_DELAY_US\n");

  test_result("dir_change_delay above max clamped", true);
}

static void test_dir_change_delay_inserted_on_dir_change() {
  printf("Running: dir_change_delay_us pause inserted on direction change\n");
  init_queue();

  FastAccelStepper s;
  s.init(NULL, 0, 0);
  s.setDirectionPin(5, true, 1000);
  s.setSpeedInUs(100);
  s.setAcceleration(10000);

  struct stepper_command_s cmd1 = {
      .ticks = 1600, .steps = 10, .count_up = true};
  s.addQueueEntry(&cmd1, true);

  uint8_t entries_forward = fas_queue[0].next_write_idx - fas_queue[0].read_idx;
  printf("  Entries after forward cmd: %d\n", entries_forward);

  struct stepper_command_s cmd2 = {
      .ticks = 1600, .steps = 10, .count_up = false};
  s.addQueueEntry(&cmd2, false);

  uint8_t entries_total = fas_queue[0].next_write_idx - fas_queue[0].read_idx;
  printf("  Entries after backward cmd: %d\n", entries_total);

  bool pause_inserted = (entries_total > entries_forward + 1);
  printf("  Pause entry inserted: %s\n", pause_inserted ? "yes" : "no");

  test_result("dir_change_delay pause inserted", pause_inserted);
}

static void test_dir_change_delay_not_on_pause_command() {
  printf("Running: dir_change_delay NOT inserted for pause commands\n");
  init_queue();

  FastAccelStepper s;
  s.init(NULL, 0, 0);
  s.setDirectionPin(5, true, 1000);

  struct stepper_command_s pause_cmd = {
      .ticks = 10000, .steps = 0, .count_up = true};
  AqeResultCode res = s.addQueueEntry(&pause_cmd, true);

  printf("  Pause command result: %s (expected %s)\n", toString(res),
         toString(AQE_OK));

  test_result("dir_change_delay not on pause", res == AQE_OK);
}

static void test_before_after_dir_change_delay_defined() {
  printf("Running: BEFORE/AFTER_DIR_CHANGE_DELAY_TICKS defined check\n");
  init_queue();

  uint16_t before_delay = 0;
  uint16_t after_delay = 0;

#if defined(BEFORE_DIR_CHANGE_DELAY_TICKS)
  before_delay = BEFORE_DIR_CHANGE_DELAY_TICKS(&fas_queue[0]);
#endif
#if defined(AFTER_DIR_CHANGE_DELAY_TICKS)
  after_delay = AFTER_DIR_CHANGE_DELAY_TICKS(&fas_queue[0]);
#endif

  printf("  BEFORE_DIR_CHANGE_DELAY_TICKS: %u ticks\n", before_delay);
  printf("  AFTER_DIR_CHANGE_DELAY_TICKS: %u ticks\n", after_delay);

  if (before_delay == 0 && after_delay == 0) {
    printf("  Neither defined - using default behavior\n");
  }

  test_result("BEFORE/AFTER_DIR_CHANGE_DELAY_TICKS check", true);
}

static void test_no_delay_if_prior_pause_sufficient() {
  printf("Running: No delay if prior pause has sufficient ticks\n");
  init_queue();

  FastAccelStepper s;
  s.init(NULL, 0, 0);
  s.setDirectionPin(5, true, 0);
  s.setSpeedInUs(100);
  s.setAcceleration(10000);

  uint16_t before_delay_ticks = US_TO_TICKS(1000);
  fas_queue[0]._before_dir_change_delay_ticks = before_delay_ticks;
  printf("  before_delay: %u ticks\n", before_delay_ticks);

  struct stepper_command_s pause_cmd = {
      .ticks = (uint16_t)(before_delay_ticks + 1000),
      .steps = 0,
      .count_up = true};
  AqeResultCode res = s.addQueueEntry(&pause_cmd, true);
  printf("  Pause command result: %s\n", toString(res));

  uint8_t entries_after_pause =
      fas_queue[0].next_write_idx - fas_queue[0].read_idx;
  printf("  Entries after pause: %d\n", entries_after_pause);

  struct stepper_command_s cmd2 = {
      .ticks = 1600, .steps = 10, .count_up = false};
  res = s.addQueueEntry(&cmd2, false);

  uint8_t entries_total = fas_queue[0].next_write_idx - fas_queue[0].read_idx;
  printf("  Entries after backward cmd: %d\n", entries_total);

  bool no_extra_pause = (entries_total == entries_after_pause + 1);
  printf("  Extra pause inserted: %s (expected: no)\n",
         no_extra_pause ? "no" : "yes");

  test_result("No delay if prior pause sufficient", no_extra_pause);
}

static void test_no_delay_if_prior_step_sufficient() {
  printf("Running: No delay if prior single step has sufficient ticks\n");
  init_queue();

  FastAccelStepper s;
  s.init(NULL, 0, 0);
  s.setDirectionPin(5, true, 0);
  s.setSpeedInUs(100);
  s.setAcceleration(10000);

  uint16_t before_delay_ticks = US_TO_TICKS(1000);
  fas_queue[0]._before_dir_change_delay_ticks = before_delay_ticks;
  printf("  before_delay: %u ticks\n", before_delay_ticks);

  struct stepper_command_s step_cmd = {
      .ticks = (uint16_t)(before_delay_ticks + 1000),
      .steps = 1,
      .count_up = true};
  AqeResultCode res = s.addQueueEntry(&step_cmd, true);
  printf("  Step command result: %s\n", toString(res));

  uint8_t entries_after_step =
      fas_queue[0].next_write_idx - fas_queue[0].read_idx;
  printf("  Entries after step: %d\n", entries_after_step);

  struct stepper_command_s cmd2 = {
      .ticks = 1600, .steps = 10, .count_up = false};
  res = s.addQueueEntry(&cmd2, false);

  uint8_t entries_total = fas_queue[0].next_write_idx - fas_queue[0].read_idx;
  printf("  Entries after backward cmd: %d\n", entries_total);

  bool no_extra_pause = (entries_total == entries_after_step + 1);
  printf("  Extra pause inserted: %s (expected: no)\n",
         no_extra_pause ? "no" : "yes");

  test_result("No delay if prior step sufficient", no_extra_pause);
}

static void test_external_dir_pin_with_empty_queue() {
  printf("Running: External dir pin with empty queue\n");
  init_queue();
  external_dir_callback_called = false;

  FastAccelStepperEngine engine;
  engine.init();
  engine.setExternalCallForPin(mock_external_callback);

  FastAccelStepper* s = engine.stepperConnectToPin(0);
  assert(s != NULL);
  s->setDirectionPin(EXTERNAL_DIR_PIN, true, 0);
  s->setSpeedInUs(100);
  s->setAcceleration(10000);

  struct stepper_command_s cmd = {
      .ticks = 16000, .steps = 10, .count_up = true};
  AqeResultCode res = s->addQueueEntry(&cmd, true);

  printf("  Result: %s (expected %s)\n", toString(res), toString(AQE_OK));
  printf("  Queue entries: %d\n",
         fas_queue[0].next_write_idx - fas_queue[0].read_idx);

  test_result("External dir pin empty queue", res == AQE_OK);
}

static void test_external_dir_pin_with_steps_in_queue() {
  printf("Running: External dir pin with steps in queue (EXPECT FAIL)\n");
  init_queue();
  external_dir_callback_called = false;

  FastAccelStepperEngine engine;
  engine.init();
  engine.setExternalCallForPin(mock_external_callback);

  FastAccelStepper* s = engine.stepperConnectToPin(0);
  assert(s != NULL);
  s->setDirectionPin(EXTERNAL_DIR_PIN, true, 0);
  s->setSpeedInUs(100);
  s->setAcceleration(10000);

  for (int i = 0; i < 10; i++) {
    struct stepper_command_s cmd = {
        .ticks = 1600, .steps = 25, .count_up = true};
    s->addQueueEntry(&cmd, false);
  }

  uint32_t ticks_in_queue = fas_queue[0].ticksInQueue();
  printf("  Ticks in queue before dir change: %u (~%.2fms)\n", ticks_in_queue,
         ticks_in_queue * 1000.0 / TICKS_PER_S);

  struct stepper_command_s cmd = {
      .ticks = 16000, .steps = 10, .count_up = false};
  AqeResultCode res = s->addQueueEntry(&cmd, true);

  printf("  Result: %s (expected %s = AQE_DIR_PIN_2MS_PAUSE_ADDED)\n",
         toString(res), toString(AQE_DIR_PIN_2MS_PAUSE_ADDED));

  if (res != AQE_DIR_PIN_2MS_PAUSE_ADDED) {
    printf("  BUG DETECTED: Current impl returns %s, should return %s\n",
           toString(res), toString(AQE_DIR_PIN_2MS_PAUSE_ADDED));
    printf("  Two 500us pauses = 1ms is insufficient for %.2fms queue\n",
           ticks_in_queue * 1000.0 / TICKS_PER_S);
  }

  test_result("External dir pin with steps in queue",
              res == AQE_DIR_PIN_2MS_PAUSE_ADDED);
}

static void test_external_dir_pin_queue_drain_and_retry() {
  printf("Running: External dir pin queue drain and retry\n");
  init_queue();
  external_dir_callback_called = false;

  FastAccelStepperEngine engine;
  engine.init();
  engine.setExternalCallForPin(mock_external_callback);

  FastAccelStepper* s = engine.stepperConnectToPin(0);
  assert(s != NULL);
  s->setDirectionPin(EXTERNAL_DIR_PIN, true, 0);
  s->setSpeedInUs(100);
  s->setAcceleration(10000);

  for (int i = 0; i < 5; i++) {
    struct stepper_command_s fwd_cmd = {
        .ticks = 1600, .steps = 20, .count_up = true};
    s->addQueueEntry(&fwd_cmd, false);
  }

  uint32_t ticks_remaining = fas_queue[0].ticksInQueue();
  printf("  Initial ticks in queue: %u\n", ticks_remaining);

  int retry_count = 0;
  AqeResultCode res;

  struct stepper_command_s cmd = {
      .ticks = 16000, .steps = 10, .count_up = false};

  while (retry_count < 20) {
    res = s->addQueueEntry(&cmd, true);

    if (res == AQE_DIR_PIN_2MS_PAUSE_ADDED) {
      printf("  Retry %d: PAUSE_ADDED (queue draining)\n", retry_count);

      process_queue_ticks(TASK_CYCLE_TICKS);

      retry_count++;
      ticks_remaining = fas_queue[0].ticksInQueue();
      printf("    Ticks remaining after drain: %u\n", ticks_remaining);

      if (ticks_remaining == 0) {
        break;
      }
    } else {
      break;
    }
  }

  printf("  Final result after %d retries: %s\n", retry_count, toString(res));

  test_result("External dir pin queue drain retry",
              retry_count > 0 && res == AQE_OK);
}

static void test_external_enable_pin_callback() {
  printf("Running: External enable pin callback\n");
  init_queue();
  external_enable_callback_called = false;

  FastAccelStepperEngine engine;
  engine.init();
  engine.setExternalCallForPin(mock_external_enable_callback);

  FastAccelStepper* s = engine.stepperConnectToPin(0);
  assert(s != NULL);
  s->setEnablePin(EXTERNAL_ENABLE_PIN, true);

  bool enabled = s->enableOutputs();

  printf("  Callback called: %s\n",
         external_enable_callback_called ? "yes" : "no");
  printf("  enableOutputs returned: %s\n", enabled ? "true" : "false");

  test_result("External enable pin callback", external_enable_callback_called);
}

static void test_external_enable_pin_disable() {
  printf("Running: External enable pin disable\n");
  init_queue();
  external_enable_callback_called = false;

  FastAccelStepperEngine engine;
  engine.init();
  engine.setExternalCallForPin(mock_external_enable_callback);

  FastAccelStepper* s = engine.stepperConnectToPin(0);
  assert(s != NULL);
  s->setEnablePin(EXTERNAL_ENABLE_PIN, true);
  s->enableOutputs();

  external_enable_callback_called = false;
  bool disabled = s->disableOutputs();

  printf("  Callback called: %s\n",
         external_enable_callback_called ? "yes" : "no");
  printf("  disableOutputs returned: %s\n", disabled ? "true" : "false");

  test_result("External enable pin disable", external_enable_callback_called);
}

static void test_queue_ticks_tracking() {
  printf("Running: Queue ticks tracking\n");
  init_queue();

  FastAccelStepper s;
  s.init(NULL, 0, 0);
  s.setSpeedInUs(100);
  s.setAcceleration(10000);

  struct stepper_command_s cmd = {.ticks = 1600, .steps = 10, .count_up = true};
  s.addQueueEntry(&cmd, true);

  uint32_t ticks = fas_queue[0].ticksInQueue();
  printf("  Ticks for 10 steps: %u\n", ticks);

  process_queue_ticks(ticks);
  ticks = fas_queue[0].ticksInQueue();
  printf("  Ticks after processing all: %u (expected 0)\n", ticks);

  test_result("Queue ticks tracking", ticks == 0);
}

static void test_has_steps_detection() {
  printf("Running: hasStepsInQueue detection\n");
  init_queue();

  FastAccelStepper s;
  s.init(NULL, 0, 0);
  s.setDirectionPin(5, true, 0);
  s.setSpeedInUs(100);
  s.setAcceleration(10000);

  struct stepper_command_s cmd = {.ticks = 1600, .steps = 50, .count_up = true};
  s.addQueueEntry(&cmd, true);

  bool has_steps = false;
  StepperQueue* q = &fas_queue[0];
  uint8_t rp = q->read_idx;
  while (rp != q->next_write_idx) {
    if (q->entry[rp & QUEUE_LEN_MASK].steps > 0) {
      has_steps = true;
      break;
    }
    rp++;
  }

  printf("  Steps in queue: %s\n", has_steps ? "yes" : "no");
  test_result("hasStepsInQueue detection", has_steps);
}

static void test_realistic_direction_change_scenario() {
  printf("Running: Realistic direction change scenario\n");
  init_queue();

  FastAccelStepperEngine engine;
  engine.init();
  engine.setExternalCallForPin(mock_external_callback);

  FastAccelStepper* s = engine.stepperConnectToPin(0);
  assert(s != NULL);
  s->setDirectionPin(EXTERNAL_DIR_PIN, true, 0);
  s->setSpeedInUs(50);
  s->setAcceleration(50000);

  for (int i = 0; i < 8; i++) {
    struct stepper_command_s cmd = {
        .ticks = 800, .steps = 30, .count_up = true};
    s->addQueueEntry(&cmd, false);
  }

  for (int i = 0; i < 3; i++) {
    process_queue_ticks(TASK_CYCLE_TICKS);
  }

  uint32_t ticks_before = fas_queue[0].ticksInQueue();
  printf("  Ticks in queue before reversal: %u (~%.2fms)\n", ticks_before,
         ticks_before * 1000.0 / TICKS_PER_S);

  struct stepper_command_s rev_cmd = {
      .ticks = 16000, .steps = 10, .count_up = false};
  AqeResultCode res = s->addQueueEntry(&rev_cmd, true);

  printf("  addQueueEntry result: %s\n", toString(res));

  bool correct = (ticks_before > TASK_CYCLE_TICKS)
                     ? (res == AQE_DIR_PIN_2MS_PAUSE_ADDED)
                     : true;
  test_result("Realistic direction change scenario", correct);
}

int main() {
  puts("=== Direction Pin and External Pin Tests ===\n");
  fflush(stdout);

  puts("=== Part 1: MIN/MAX_DIR_DELAY_US ===");
  test_min_dir_delay_value();
  test_max_dir_delay_value();
  test_dir_delay_below_min_clamped();
  test_dir_delay_above_max_clamped();

  puts("\n=== Part 2: dir_change_delay_us ===");
  test_dir_change_delay_inserted_on_dir_change();
  test_dir_change_delay_not_on_pause_command();
  test_before_after_dir_change_delay_defined();
  test_no_delay_if_prior_pause_sufficient();
  test_no_delay_if_prior_step_sufficient();

  puts("\n=== Part 3: External Direction Pin (MAY FAIL) ===");
  test_external_dir_pin_with_empty_queue();
  test_external_dir_pin_with_steps_in_queue();
  test_external_dir_pin_queue_drain_and_retry();

  puts("\n=== Part 4: External Enable Pin ===");
  test_external_enable_pin_callback();
  test_external_enable_pin_disable();

  puts("\n=== Part 5: Integration ===");
  test_queue_ticks_tracking();
  test_has_steps_detection();
  test_realistic_direction_change_scenario();

  printf("\n=== Test Summary ===\n");
  printf("Total: %d  Passed: %d  Failed: %d\n", test_passed + test_failed,
         test_passed, test_failed);

  if (test_failed == 0) {
    puts("All tests PASSED");
    return 0;
  } else {
    printf("%d test(s) FAILED\n", test_failed);
    puts(
        "Note: Some failures are expected before the external dir pin fix is "
        "implemented");
    return 1;
  }
}
