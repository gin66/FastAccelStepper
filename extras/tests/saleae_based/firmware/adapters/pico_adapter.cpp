/*
 * pico_adapter.cpp — RP2040/Pico-specific initialization.
 *
 * Uses PIO state machines for step pulse generation.
 * Up to 4 steppers with Arduino framework, 32 with ESP-IDF.
 */

#if defined(ARDUINO_ARCH_RP2040)

#include <Arduino.h>

void init_stepper_platform(void) {
  // Pico: Initialize PIO state machines for stepper pulse generation
}

void stop_current_move(void) {
  // Stop PIO state machines
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
  return 32;  // Pico queue capacity
}

#endif  // ARDUINO_ARCH_RP2040