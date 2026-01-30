#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "FastAccelStepper.h"
#include "StepperISR.h"

char TCCR1A;
char TCCR1B;
char TCCR1C;
char TIMSK1;
char TIFR1;
unsigned short OCR1A;
unsigned short OCR1B;

StepperQueue fas_queue[NUM_QUEUES];

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

class FastAccelStepperTest {
 public:
  void init_queue() {
    fas_queue[0].read_idx = 0;
    fas_queue[1].read_idx = 0;
    fas_queue[0].next_write_idx = 0;
    fas_queue[1].next_write_idx = 0;
  }

  void dump_ramp_commands(uint32_t speed_us, uint32_t acceleration,
                          int32_t steps) {
    printf("=== RAMP HELPER: speed=%uus, accel=%u, steps=%d ===\n", speed_us,
           acceleration, steps);
    printf("Target speed: %u ticks (%.1fus period)\n", speed_us * 16,
           speed_us / 1.0);

    init_queue();
    FastAccelStepper s = FastAccelStepper();
    s.init(NULL, 0, 0);

    assert(s.isQueueEmpty());
    s.setSpeedInUs(speed_us);
    s.setAcceleration(acceleration);
    s.fill_queue();
    assert(s.isQueueEmpty());

    s.move(steps);
    s.fill_queue();
    assert(!s.isQueueEmpty());

    printf("\nCommands generated (until coasting reached):\n");
    printf("Each step period includes step command + following pauses\n");
    printf("--------------------------------------------------------\n");

    uint64_t total_ticks = 0;
    bool coasting_reached = false;
    uint32_t coasting_period_ticks = 0;
    int command_count = 0;

    // Simple approach: track step commands and accumulate following pauses
    for (int i = 0; i < 10000; i++) {
      if (!s.isRampGeneratorActive()) {
        printf("Ramp generator finished\n");
        break;
      }

      // Check ramp state before filling queue
      uint8_t ramp_state = s._rg.rampState();
      if ((ramp_state & RAMP_STATE_COAST) && !coasting_reached) {
        coasting_reached = true;
        printf("\n*** COASTING STATE REACHED (ramp_state=0x%02x) ***\n",
               ramp_state);
      }

      s.fill_queue();

      // Process commands in batches: step command + following pauses
      while (!s.isQueueEmpty()) {
        // Peek at first command
        struct queue_entry* first_cmd =
            &fas_queue_A.entry[fas_queue[0].read_idx & QUEUE_LEN_MASK];

        if (first_cmd->steps == 0) {
          // Should not start with pause (unless leftover from previous)
          printf("  [UNEXPECTED PAUSE] ticks=%u\n", first_cmd->ticks);
          total_ticks += first_cmd->ticks;
          fas_queue[0].read_idx++;
          command_count++;
          continue;
        }

        // We have a step command - process it and following pauses
        uint32_t step_period_ticks = 0;
        int steps_in_period = 0;
        uint32_t period_start = total_ticks;

        // Process step command(s) until we hit another step command or queue
        // empty
        while (!s.isQueueEmpty()) {
          struct queue_entry* e =
              &fas_queue_A.entry[fas_queue[0].read_idx & QUEUE_LEN_MASK];
          command_count++;

          if (e->steps == 0) {
            // Pause - add to current period
            step_period_ticks += e->ticks;
            total_ticks += e->ticks;
            printf("    [PAUSE] ticks=%u\n", e->ticks);
          } else {
            if (steps_in_period > 0) {
              // New step command - break to start new period
              break;
            }
            // First step command in period
            steps_in_period = e->steps;
            for (int j = 0; j < e->steps; j++) {
              step_period_ticks += e->ticks;
              total_ticks += e->ticks;
            }
            printf("    [STEP] steps=%d, ticks=%u per step\n", e->steps,
                   e->ticks);
          }

          fas_queue[0].read_idx++;
        }

        // Print completed step period
        if (steps_in_period > 0) {
          uint32_t avg_ticks_per_step = step_period_ticks / steps_in_period;
          float period_us = step_period_ticks / 16.0;
          float avg_us_per_step = avg_ticks_per_step / 16.0;

          printf("  Step period: %u ticks total (%.1fus)\n", step_period_ticks,
                 period_us);
          printf("    %d step(s), avg %u ticks/step (%.1fus)\n",
                 steps_in_period, avg_ticks_per_step, avg_us_per_step);
          printf("    Ends at: %.1fus\n",
                 (period_start + step_period_ticks) / 16.0);

          // Store coasting period
          if (coasting_reached && coasting_period_ticks == 0) {
            coasting_period_ticks = avg_ticks_per_step;
            printf("    <-- COASTING: %u ticks/step (%.1fus)\n",
                   coasting_period_ticks, coasting_period_ticks / 16.0);
          }
        }
      }

      if (coasting_reached && coasting_period_ticks != 0) {
        printf("\nCoasting commands (for test input):\n");
        printf("Coasting period: %u ticks/step (%.1fus)\n",
               coasting_period_ticks, coasting_period_ticks / 16.0);
        printf("Use in tests: steps=1, ticks=%u\n", coasting_period_ticks);
        printf("\nTotal commands: %d\n", command_count);
        break;
      }
    }

    // Empty any remaining queue
    while (!s.isQueueEmpty()) {
      struct queue_entry* e =
          &fas_queue_A.entry[fas_queue[0].read_idx & QUEUE_LEN_MASK];
      if (e->steps == 0) {
        total_ticks += e->ticks;
      } else {
        for (int j = 0; j < e->steps; j++) {
          total_ticks += e->ticks;
        }
      }
      fas_queue[0].read_idx++;
    }

    printf("\nTotal ramp time: %.3fms\n", total_ticks / 16000.0);
    printf("============================================\n\n");
  }
};

int main(int argc, char* argv[]) {
  if (argc != 4) {
    printf("Usage: %s <speed_us> <acceleration> <steps>\n", argv[0]);
    printf("Example: %s 1000 10000 1000\n", argv[0]);
    printf("  speed_us: period in microseconds (e.g., 1000 for 1ms)\n");
    printf("  acceleration: steps/s² (e.g., 10000)\n");
    printf("  steps: total steps to move (e.g., 1000)\n");
    printf("\nCommon test cases for ESP32C6 RMT issue:\n");
    printf("  %s 1000 10000 1000  # 1000us (16000 ticks)\n", argv[0]);
    printf("  %s 2000 10000 1000  # 2000us (32000 ticks)\n", argv[0]);
    printf("  %s 4000 10000 1000  # 4000us (64000 ticks)\n", argv[0]);
    printf("  %s 16000 10000 1000 # 16000us (256000 ticks)\n", argv[0]);
    return 1;
  }

  uint32_t speed_us = atoi(argv[1]);
  uint32_t acceleration = atoi(argv[2]);
  int32_t steps = atoi(argv[3]);

  FastAccelStepperTest test;
  test.dump_ramp_commands(speed_us, acceleration, steps);

  return 0;
}