/*
 * sam_adapter.cpp — SAM/SAMD51-specific initialization.
 *
 * Uses TCC timers for step pulse generation.
 * Queue capacity varies by chip (typically 32).
 */

#if defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)

#include <Arduino.h>

void init_stepper_platform(void) {
  // SAM/SAMD51: Initialize TCC timers for stepper pulse generation
}

void stop_current_move(void) {
  // Stop TCC timer
}

void stop_all_steppers(void) {
  stop_current_move();
}

void clear_all_queues(void) {
  // Clear queue buffer
}

int32_t getCurrentPosition(void) {
  return 0;
}

uint8_t getQueueLevel(void) {
  return 0;
}

uint8_t getQueueCapacity(void) {
  return 32;  // SAM/SAMD51 queue capacity
}

#endif  // ARDUINO_ARCH_SAM || ARDUINO_ARCH_SAMD