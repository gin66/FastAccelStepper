#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

// Define debug_part_size before including any headers
uint16_t debug_part_size = 31;  // Default for PC tests

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

struct Esp32C6TestCase {
  uint32_t speed_us;
  uint32_t steps;
  const char* description;
  bool expect_pulses;
  uint32_t expected_min_period_ticks;
  uint32_t expected_max_period_ticks;
};

class FastAccelStepperEsp32C6Test {
 public:
  uint32_t offset;
  uint32_t rmt_entries[100000];

  void init_queue() {
    fas_queue[0].read_idx = 0;
    fas_queue[0].next_write_idx = 0;
  }

  bool test_speed_range(Esp32C6TestCase tc) {
    printf("Testing ESP32-C6 timing: %s (%" PRIu32 "us speed, %" PRIu32
           " steps)\n",
           tc.description, tc.speed_us, tc.steps);
    init_queue();
    offset = 0;
    FastAccelStepper s = FastAccelStepper();
    s.init(NULL, 0, 0);
    assert(0 == s.getCurrentPosition());

    s.setSpeedInUs(tc.speed_us);
    s.setAcceleration(10000000);  // High acceleration for immediate speed

    // Generate steps
    for (uint32_t i = 0; i < tc.steps; i++) {
      if (!s.isQueueEmpty()) {
        if (offset + PART_SIZE > sizeof(rmt_entries) / sizeof(rmt_entries[0])) {
          printf("RMT buffer overflow\n");
          return false;
        }
        rmt_fill_buffer(&fas_queue[0], true, &rmt_entries[offset]);
        offset += PART_SIZE;
      }

      // Add a single step command
      struct queue_entry* e =
          &fas_queue[0].entry[fas_queue[0].next_write_idx & QUEUE_LEN_MASK];
      e->steps = 1;
      e->ticks = (TICKS_PER_S * tc.speed_us) / 1000000;
      e->toggle_dir = 0;
      fas_queue[0].next_write_idx++;
    }

    // Flush remaining buffer
    while (!s.isQueueEmpty()) {
      if (offset + PART_SIZE > sizeof(rmt_entries) / sizeof(rmt_entries[0])) {
        printf("RMT buffer overflow\n");
        return false;
      }
      rmt_fill_buffer(&fas_queue[0], true, &rmt_entries[offset]);
      offset += PART_SIZE;
    }

    return analyze_rmt_output(tc);
  }

  bool analyze_rmt_output(Esp32C6TestCase tc) {
    printf("RMT entries generated: %d\n", offset);
    uint64_t total_ticks = 0;
    uint32_t steps_found = 0;
    uint32_t last_step_tick = 0;
    uint32_t min_period = UINT32_MAX;
    uint32_t max_period = 0;
    bool step_high = false;

    for (uint32_t i = 0; i < offset; i++) {
      uint32_t entry = rmt_entries[i];
      for (int j = 0; j < 2; j++) {
        uint16_t subentry = (j == 0) ? entry & 0xffff : entry >> 16;
        if (((subentry & 0x8000) != 0) && !step_high) {
          // Step transition detected
          step_high = true;
          steps_found++;
          if (last_step_tick > 0) {
            uint32_t period = total_ticks - last_step_tick;
            if (period < min_period) min_period = period;
            if (period > max_period) max_period = period;
          }
          last_step_tick = total_ticks;
        } else if ((subentry & 0x8000) == 0) {
          step_high = false;
        }
        total_ticks += subentry & 0x7fff;
      }
    }

    printf("Steps expected: %" PRIu32 ", found: %" PRIu32 "\n", tc.steps,
           steps_found);
    printf("Period range: min=%" PRIu32 " ticks, max=%" PRIu32 " ticks\n",
           min_period, max_period);

    if (tc.expected_min_period_ticks > 0) {
      if (min_period < tc.expected_min_period_ticks) {
        printf("ERROR: Minimum period too small: %" PRIu32 " < %" PRIu32 "\n",
               min_period, tc.expected_min_period_ticks);
        return false;
      }
      if (max_period > tc.expected_max_period_ticks) {
        printf("ERROR: Maximum period too large: %" PRIu32 " > %" PRIu32 "\n",
               max_period, tc.expected_max_period_ticks);
        return false;
      }
    }

    // Check if pulses were generated when expected
    bool pulses_ok = (tc.expect_pulses && steps_found > 0) ||
                     (!tc.expect_pulses && steps_found == 0);
    if (!pulses_ok) {
      printf("ERROR: Expected %s, got %" PRIu32 " steps\n",
             tc.expect_pulses ? "pulses" : "no pulses", steps_found);
      return false;
    }

    if (tc.expect_pulses && steps_found != tc.steps) {
      printf("ERROR: Expected %" PRIu32 " steps, got %" PRIu32 "\n", tc.steps,
             steps_found);
      return false;
    }

    printf("PASS: %s\n", tc.description);
    return true;
  }
};

int main() {
  FastAccelStepperEsp32C6Test test;

  // Test cases specifically targeting ESP32-C6 timing gaps
  Esp32C6TestCase test_cases[] = {
      // Working range: <=990us (should work)
      {.speed_us = 500,
       .steps = 100,
       .description = "Fast speed (500us)",
       .expect_pulses = true,
       .expected_min_period_ticks = 8000,
       .expected_max_period_ticks = 8000},
      {.speed_us = 990,
       .steps = 100,
       .description = "Edge fast speed (990us)",
       .expect_pulses = true,
       .expected_min_period_ticks = 15840,
       .expected_max_period_ticks = 15840},

      // Problematic range: 1000-4000us (dead zone in original)
      {.speed_us = 1000,
       .steps = 100,
       .description = "Dead zone start (1000us)",
       .expect_pulses = true,
       .expected_min_period_ticks = 16000,
       .expected_max_period_ticks = 16000},
      {.speed_us = 2000,
       .steps = 50,
       .description = "Mid dead zone (2000us)",
       .expect_pulses = true,
       .expected_min_period_ticks = 32000,
       .expected_max_period_ticks = 32000},
      {.speed_us = 4000,
       .steps = 25,
       .description = "Dead zone end (4000us)",
       .expect_pulses = true,
       .expected_min_period_ticks = 64000,
       .expected_max_period_ticks = 64000},

      // Working range: 4100-8500us (should work)
      {.speed_us = 5000,
       .steps = 20,
       .description = "Mid working range (5000us)",
       .expect_pulses = true,
       .expected_min_period_ticks = 80000,
       .expected_max_period_ticks = 80000},
      {.speed_us = 8500,
       .steps = 12,
       .description = "Edge slow working (8500us)",
       .expect_pulses = true,
       .expected_min_period_ticks = 136000,
       .expected_max_period_ticks = 136000},

      // Problematic range: >8500us (another dead zone)
      {.speed_us = 10000,
       .steps = 10,
       .description = "Slow speed start (10000us)",
       .expect_pulses = true,
       .expected_min_period_ticks = 160000,
       .expected_max_period_ticks = 160000},
      {.speed_us = 20000,
       .steps = 5,
       .description = "Very slow speed (20000us)",
       .expect_pulses = true,
       .expected_min_period_ticks = 320000,
       .expected_max_period_ticks = 320000},

      // Edge cases for timing calculation
      {.speed_us = 80,
       .steps = 200,
       .description = "Minimum possible speed (80us)",
       .expect_pulses = true,
       .expected_min_period_ticks = 1280,
       .expected_max_period_ticks = 1280},
  };

  bool all_passed = true;
  for (int i = 0; i < sizeof(test_cases) / sizeof(test_cases[0]); i++) {
    printf("\n=== Test Case %d ===\n", i + 1);
    bool passed = test.test_speed_range(test_cases[i]);
    if (!passed) {
      all_passed = false;
      printf("FAILED: %s\n", test_cases[i].description);
    }
  }

  if (all_passed) {
    printf("\nTEST_18 PASSED - All ESP32-C6 timing ranges work correctly\n");
    return 0;
  } else {
    printf("\nTEST_18 FAILED - ESP32-C6 timing issues detected\n");
    return 1;
  }
}