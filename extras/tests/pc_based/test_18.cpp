#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool dump_rmt_symbols = false;

uint16_t debug_part_size = 24;

#include "FastAccelStepper.h"

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

#define SUPPORT_ESP32_RMT
#define SUPPORT_EXTERNAL_DIRECTION_PIN
#define IRAM_ATTR
#define RMT_CHANNEL_T int
#define LL_TOGGLE_PIN(dirPin)
#include "StepperISR_esp32xx_rmt.cpp"

StepperQueue fas_queue[NUM_QUEUES];

#define QUEUE_SIZE 32
#define MAX_RMT_ENTRIES 8192

struct QueueCommand {
  uint8_t steps;
  uint16_t ticks;
};

struct RmtAnalysis {
  uint32_t step_count;
  uint64_t total_ticks;
  uint16_t min_symbol_period;
  uint32_t total_high_ticks;
  uint32_t total_low_ticks;
};

static uint32_t global_rmt_entries[MAX_RMT_ENTRIES];

class RmtBufferTest {
 public:
  uint32_t* rmt_entries;
  uint32_t rmt_offset;

  RmtBufferTest() : rmt_entries(global_rmt_entries), rmt_offset(0) {}

  void init_queue() {
    fas_queue[0].read_idx = 0;
    fas_queue[0].next_write_idx = 0;
    fas_queue[0].lastChunkContainsSteps = false;
    rmt_offset = 0;
  }

  void add_command(uint8_t steps, uint16_t ticks) {
    uint8_t idx = fas_queue[0].next_write_idx & QUEUE_LEN_MASK;
    fas_queue[0].entry[idx].steps = steps;
    fas_queue[0].entry[idx].ticks = ticks;
    fas_queue[0].entry[idx].toggle_dir = 0;
    fas_queue[0].entry[idx].repeat_entry = 0;
    fas_queue[0].next_write_idx++;
  }

  void fill_commands(const QueueCommand* commands, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
      add_command(commands[i].steps, commands[i].ticks);
    }
  }

  void process_all() {
    int iterations = 0;
    while (fas_queue[0].read_idx != fas_queue[0].next_write_idx) {
      if (rmt_offset + PART_SIZE > MAX_RMT_ENTRIES) {
        break;
      }
      rmt_fill_buffer(&fas_queue[0], true, &rmt_entries[rmt_offset]);
      rmt_offset += PART_SIZE;
      iterations++;
      if (iterations > 100) {
        break;
      }
    }
  }

  RmtAnalysis analyze() {
    RmtAnalysis result = {0, 0, 0xffff, 0, 0};
    bool step_high = false;

    for (uint32_t i = 0; i < rmt_offset; i++) {
      uint32_t entry = rmt_entries[i];
      uint16_t low = entry & 0xffff;
      uint16_t high = entry >> 16;

      uint16_t low_ticks = low & 0x7fff;
      uint16_t high_ticks = high & 0x7fff;

      if (low_ticks > 0 && low_ticks < result.min_symbol_period) {
        result.min_symbol_period = low_ticks;
      }
      if (high_ticks > 0 && high_ticks < result.min_symbol_period) {
        result.min_symbol_period = high_ticks;
      }

      if ((low & 0x8000) && !step_high) {
        result.step_count++;
        step_high = true;
      } else if (!(low & 0x8000) && step_high) {
        step_high = false;
      }
      result.total_ticks += low_ticks;
      if (step_high) {
        result.total_high_ticks += low_ticks;
      } else {
        result.total_low_ticks += low_ticks;
      }

      if ((high & 0x8000) && !step_high) {
        result.step_count++;
        step_high = true;
      } else if (!(high & 0x8000) && step_high) {
        step_high = false;
      }
      result.total_ticks += high_ticks;
      if (step_high) {
        result.total_high_ticks += high_ticks;
      } else {
        result.total_low_ticks += high_ticks;
      }
    }
    return result;
  }

  void dump_rmt(uint32_t max_entries = 0) {
    uint32_t count = (max_entries > 0 && max_entries < rmt_offset) ? max_entries
                                                                   : rmt_offset;
    printf("RMT entries (showing %u of %u):\n", count, rmt_offset);
    uint32_t i = 0;
    while (i < count) {
      uint32_t entry = rmt_entries[i];
      uint32_t repeat = 1;
      while (i + repeat < count && rmt_entries[i + repeat] == entry) {
        repeat++;
      }
      uint16_t low = entry & 0xffff;
      uint16_t high = entry >> 16;
      if (repeat > 1) {
        printf("  [%3u-%3u] 0x%08x  L=%u%s H=%u%s x%u\n", i, i + repeat - 1,
               entry, low & 0x7fff, (low & 0x8000) ? "(S)" : "", high & 0x7fff,
               (high & 0x8000) ? "(S)" : "", repeat);
      } else {
        printf("  [%3u] 0x%08x  L=%u%s H=%u%s\n", i, entry, low & 0x7fff,
               (low & 0x8000) ? "(S)" : "", high & 0x7fff,
               (high & 0x8000) ? "(S)" : "");
      }
      i += repeat;
    }
  }
};

struct TestCase {
  const char* name;
  const QueueCommand* commands;
  uint32_t command_count;
  uint32_t expected_steps;
  uint64_t expected_ticks;
};

QueueCommand cmd_single_step_16000[] = {{1, 16000}};
QueueCommand cmd_single_step_32000[] = {{1, 32000}};
QueueCommand cmd_single_step_65535[] = {{1, 65535}};

QueueCommand cmd_multi_step_1000[] = {{10, 1000}};
QueueCommand cmd_multi_step_500[] = {{20, 500}};

QueueCommand cmd_pause_step[] = {{0, 10000}, {1, 16000}, {0, 5000}};

QueueCommand cmd_step_pause_step[] = {{1, 16000}, {0, 32000}, {1, 16000}};

QueueCommand cmd_many_steps[] = {{5, 8000}, {10, 4000}, {3, 12000}};

QueueCommand cmd_ramp_1000us[] = {
    {1, 35808}, {1, 25312}, {1, 25312}, {1, 35808}, {1, 35808}};
QueueCommand cmd_ramp_2000us[] = {
    {1, 35808}, {1, 32000}, {1, 32000}, {1, 35808}, {1, 35808}};
QueueCommand cmd_ramp_500us_step_pause[] = {
    {1, 56608}, {0, 56608}, {1, 40032}, {0, 40032}, {1, 65344},
    {1, 56608}, {1, 50624}, {1, 56608}, {1, 65344}, {1, 40032},
    {0, 40032}, {1, 56608}, {0, 56608}, {1, 56608}, {0, 56608}};

QueueCommand cmd_single_step_8000[] = {{1, 8000}};
QueueCommand cmd_multi_step_8000[] = {{10, 8000}};

QueueCommand cmd_1000us_period[] = {{1, 16000}};
QueueCommand cmd_999us_period[] = {{1, 15984}};

bool run_tests() {
  printf("\n=== Testing with PART_SIZE=%u ===\n", debug_part_size);
  printf("MIN_CMD_TICKS=%ld, TICKS_PER_S=%ld\n\n", MIN_CMD_TICKS, TICKS_PER_S);

  RmtBufferTest test;
  bool all_passed = true;
  int fail_count = 0;

  TestCase cases[] = {
      {"Single step 16000 ticks (1000us)", cmd_single_step_16000, 1, 1, 16000},
      {"Single step 32000 ticks (2000us)", cmd_single_step_32000, 1, 1, 32000},
      {"Single step 65535 ticks (4095us)", cmd_single_step_65535, 1, 1, 65535},
      {"Single step 8000 ticks (500us)", cmd_single_step_8000, 1, 1, 8000},
      {"10 steps @ 1000 ticks each", cmd_multi_step_1000, 1, 10, 10000},
      {"20 steps @ 500 ticks each", cmd_multi_step_500, 1, 20, 10000},
      {"10 steps @ 8000 ticks each (500us)", cmd_multi_step_8000, 1, 10, 80000},
      {"Pause + Step + Pause", cmd_pause_step, 3, 1, 31000},
      {"Step + Pause + Step", cmd_step_pause_step, 3, 2, 64000},
      {"Mixed: 5@8000 + 10@4000 + 3@12000", cmd_many_steps, 3, 18, 116000},
      {"Ramp 1000us (from ramp_helper)", cmd_ramp_1000us, 5, 5, 158048},
      {"Ramp 2000us (from ramp_helper)", cmd_ramp_2000us, 5, 5, 171424},
      {"Ramp 500us step+pause (from ramp_helper)", cmd_ramp_500us_step_pause,
       15, 10, 794304},
      {"1000us period (16000 ticks)", cmd_1000us_period, 1, 1, 16000},
      {"999us period (15984 ticks)", cmd_999us_period, 1, 1, 15984},
  };

  int num_cases = sizeof(cases) / sizeof(cases[0]);

  for (int i = 0; i < num_cases; i++) {
    TestCase* tc = &cases[i];
    printf("Test %d: %s\n", i + 1, tc->name);

    test.init_queue();
    test.fill_commands(tc->commands, tc->command_count);
    test.process_all();

    RmtAnalysis result = test.analyze();

    bool steps_ok = (result.step_count == tc->expected_steps);
    bool ticks_ok = (result.total_ticks == tc->expected_ticks);

    uint32_t high_low_ratio = 0;
    if (result.total_low_ticks > 0) {
      high_low_ratio = (result.total_high_ticks * 100) / result.total_low_ticks;
    }

    printf("  Steps: %" PRIu32 " (expected %" PRIu32 ") %s\n",
           result.step_count, tc->expected_steps, steps_ok ? "OK" : "FAIL");
    printf("  Ticks: %" PRIu64 " (expected %" PRIu64 ") %s\n",
           result.total_ticks, tc->expected_ticks, ticks_ok ? "OK" : "FAIL");
    printf("  High/Low: %" PRIu32 "/%" PRIu32 " ticks (ratio %u%%)\n",
           result.total_high_ticks, result.total_low_ticks, high_low_ratio);
    printf("  Min symbol: %u ticks\n", result.min_symbol_period);

    bool ratio_ok = (high_low_ratio >= 30 && high_low_ratio <= 200);
    if (!ratio_ok && result.step_count > 0) {
      printf("  WARNING: High/Low ratio outside 30-200%% range\n");
    }

    if (!steps_ok || !ticks_ok) {
      all_passed = false;
      test.dump_rmt(50);
    } else if (dump_rmt_symbols) {
      printf("  RMT symbols for this test:\n");
      test.dump_rmt(20);
    }
    printf("\n");
  }

  printf("\n=== Detailed Analysis: 1000us vs 999us Period Issue ===\n\n");

  // Test 1000us (16000 ticks) vs 999us (15984 ticks)
  uint16_t period_ticks[] = {16000, 15984};
  const char* period_names[] = {"1000us (16000 ticks)", "999us (15984 ticks)"};

  for (int p = 0; p < 2; p++) {
    uint16_t ticks = period_ticks[p];
    printf("Testing %s:\n", period_names[p]);
    printf("  Command: steps=1, ticks=%u\n", ticks);

    test.init_queue();
    test.add_command(1, ticks);

    // Print queue state before processing
    printf("  Queue before processing:\n");
    printf("    read_idx=%u, next_write_idx=%u\n", fas_queue[0].read_idx,
           fas_queue[0].next_write_idx);
    printf("    entry[0]: steps=%u, ticks=%u\n", fas_queue[0].entry[0].steps,
           fas_queue[0].entry[0].ticks);

    test.process_all();

    // Print RMT symbols
    printf("  RMT symbols generated:\n");
    test.dump_rmt(50);

    RmtAnalysis result = test.analyze();

    printf("  Analysis:\n");
    printf("    Steps: %" PRIu32 " (expected 1) %s\n", result.step_count,
           result.step_count == 1 ? "OK" : "FAIL");
    printf("    Ticks: %" PRIu64 " (expected %u) %s\n", result.total_ticks,
           ticks, result.total_ticks == ticks ? "OK" : "FAIL");
    printf("    High/Low: %" PRIu32 "/%" PRIu32 " ticks\n",
           result.total_high_ticks, result.total_low_ticks);
    printf("    Min symbol: %u ticks\n", result.min_symbol_period);

    if (result.step_count != 1 || result.total_ticks != ticks) {
      all_passed = false;
      fail_count++;
    }
    printf("\n");
  }

  printf("=== Part 2: Step count limits (0-255 steps) ===\n\n");

  uint16_t test_ticks[] = {MIN_CMD_TICKS, 16000, 32000, 65535};
  int num_ticks = sizeof(test_ticks) / sizeof(test_ticks[0]);

  for (int t = 0; t < num_ticks; t++) {
    uint16_t ticks = test_ticks[t];
    int round_fails = 0;

    printf("Testing %u ticks with 0-255 steps: ", ticks);
    fflush(stdout);

    for (int steps = 0; steps <= 255; steps++) {
      test.init_queue();
      test.add_command(steps, ticks);
      test.process_all();

      RmtAnalysis result = test.analyze();

      uint32_t expected_steps = steps;
      uint64_t expected_ticks = (uint64_t)steps * ticks;
      if (steps == 0) {
        expected_ticks = ticks;
      }

      bool steps_ok = (result.step_count == expected_steps);
      bool ticks_ok = (result.total_ticks == expected_ticks);

      if (!steps_ok || !ticks_ok) {
        if (round_fails == 0) {
          printf("\n");
        }
        printf("  FAIL steps=%d: got %" PRIu32 " steps, %" PRIu64
               " ticks (expected %" PRIu32 ", %" PRIu64 ")\n",
               steps, result.step_count, result.total_ticks, expected_steps,
               expected_ticks);
        round_fails++;
        all_passed = false;
        if (round_fails <= 2) {
          test.dump_rmt(30);
        }
      } else if (dump_rmt_symbols &&
                 (steps == 0 || steps == 1 || steps == 255)) {
        // Dump for edge cases when dump_rmt_symbols is enabled
        printf("    steps=%d: ", steps);
        test.dump_rmt(5);
      }
    }

    if (round_fails == 0) {
      printf("OK\n");
    } else {
      printf("  %d failures for ticks=%u\n", round_fails, ticks);
    }
    fail_count += round_fails;
  }

  printf("\n=== Part 3: High/Low ratio check ===\n\n");

  uint16_t ratio_test_ticks[] = {1000, 5000, 8000, 10000, 16000, 32000, 65535};
  int num_ratio_tests = sizeof(ratio_test_ticks) / sizeof(ratio_test_ticks[0]);

  for (int t = 0; t < num_ratio_tests; t++) {
    uint16_t ticks = ratio_test_ticks[t];
    test.init_queue();
    test.add_command(1, ticks);
    test.process_all();

    RmtAnalysis result = test.analyze();

    uint32_t high_low_ratio = 0;
    if (result.total_low_ticks > 0) {
      high_low_ratio = (result.total_high_ticks * 100) / result.total_low_ticks;
    }

    bool ratio_ok = (high_low_ratio >= 30 && high_low_ratio <= 200);
    printf("Period %5u ticks: high=%" PRIu32 " low=%" PRIu32
           " ratio=%u%% min_sym=%u %s\n",
           ticks, result.total_high_ticks, result.total_low_ticks,
           high_low_ratio, result.min_symbol_period, ratio_ok ? "OK" : "FAIL");

    if (!ratio_ok) {
      all_passed = false;
      fail_count++;
      test.dump_rmt(30);
    } else if (dump_rmt_symbols) {
      printf("    RMT symbols: ");
      test.dump_rmt(5);
    }
  }

  printf("\n=== Part 4: Reported Hardware Failure Analysis ===\n\n");
  printf("Testing reported hardware issues:\n");
  printf("1. v=1000us (16000 ticks) => no pulses on real hardware\n");
  printf("2. v=2000us (32000 ticks) => no pulses on real hardware\n");
  printf(
      "3. v=500us (8000 ticks) => pulses but incorrect pattern (2:1 high:low "
      "ratio)\n\n");

  uint16_t hw_failure_ticks[] = {16000, 32000, 8000};
  const char* hw_failure_desc[] = {
      "1000us (16000 ticks)", "2000us (32000 ticks)", "500us (8000 ticks)"};
  int num_hw_tests = sizeof(hw_failure_ticks) / sizeof(hw_failure_ticks[0]);

  for (int t = 0; t < num_hw_tests; t++) {
    uint16_t ticks = hw_failure_ticks[t];
    printf("HW Failure Test: %s\n", hw_failure_desc[t]);

    test.init_queue();
    test.add_command(1, ticks);
    test.process_all();

    RmtAnalysis result = test.analyze();

    uint32_t high_low_ratio = 0;
    if (result.total_low_ticks > 0) {
      high_low_ratio = (result.total_high_ticks * 100) / result.total_low_ticks;
    }

    printf("  Steps: %" PRIu32 " (expected 1) %s\n", result.step_count,
           result.step_count == 1 ? "OK" : "FAIL");
    printf("  Ticks: %" PRIu64 " (expected %u) %s\n", result.total_ticks, ticks,
           result.total_ticks == ticks ? "OK" : "FAIL");
    printf("  High/Low: %" PRIu32 "/%" PRIu32 " ticks (ratio %u%%)\n",
           result.total_high_ticks, result.total_low_ticks, high_low_ratio);
    printf("  Min symbol: %u ticks (must be >= 2) %s\n",
           result.min_symbol_period,
           result.min_symbol_period >= 2 ? "OK" : "FAIL");

    bool ratio_ok = (high_low_ratio >= 30 && high_low_ratio <= 200);
    if (!ratio_ok) {
      printf("  WARNING: High/Low ratio outside 30-200%% range\n");
    }

    if (ticks == 8000) {
      printf("  Expected issue: 2:1 high:low ratio (200%%) on real hardware\n");
      printf("  Simulated ratio: %u%%\n", high_low_ratio);
    }

    if (result.step_count != 1 || result.total_ticks != ticks ||
        result.min_symbol_period < 2) {
      all_passed = false;
      fail_count++;
      test.dump_rmt(30);
    } else if (dump_rmt_symbols) {
      printf("  RMT symbols: ");
      test.dump_rmt(10);
    }
    printf("\n");
  }

  if (all_passed) {
    printf("ALL TESTS PASSED for PART_SIZE=%u\n", debug_part_size);
    return true;
  } else {
    printf("TESTS FAILED for PART_SIZE=%u (%d failures)\n", debug_part_size,
           fail_count);
    return false;
  }
}

int main(int argc, char* argv[]) {
  // Parse command line arguments
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-d") == 0) {
      dump_rmt_symbols = true;
    } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      fprintf(stderr, "Usage: %s [options]\n", argv[0]);
      fprintf(stderr, "Options:\n");
      fprintf(stderr,
              "  -d          Enable dumping of RMT symbols for all tests\n");
      fprintf(stderr, "  -h, --help  Show this help message\n");
      return 0;
    } else {
      fprintf(stderr, "Unknown option: %s\n", argv[i]);
      fprintf(stderr, "Use -h for help\n");
      return 1;
    }
  }

  printf("=== RMT Fill Buffer Test with Multiple PART_SIZE Values ===\n");
  if (dump_rmt_symbols) {
    printf("RMT symbol dumping enabled (-d flag)\n");
  }

  uint16_t part_sizes[] = {22, 24, 30, 32};
  int num_part_sizes = sizeof(part_sizes) / sizeof(part_sizes[0]);

  bool all_passed = true;
  int total_failures = 0;

  for (int i = 0; i < num_part_sizes; i++) {
    debug_part_size = part_sizes[i];
    bool passed = run_tests();

    if (!passed) {
      all_passed = false;
      total_failures++;
    }

    if (i < num_part_sizes - 1) {
      printf(
          "\n============================================================\n");
    }
  }

  printf("\n=== Summary ===\n");
  if (all_passed) {
    printf("ALL TESTS PASSED for all PART_SIZE values\n");
    return 0;
  } else {
    printf("TESTS FAILED for %d out of %d PART_SIZE values\n", total_failures,
           num_part_sizes);
    return 1;
  }
}
