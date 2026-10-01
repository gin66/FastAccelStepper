/*
 * avr_adapter.cpp — AVR-specific initialization (ATmega328P/2560).
 *
 * Limited to 2 steppers (ATmega328P) or 4 steppers (ATmega2560).
 * Uses Timer 1 for step pulses (pins 9/10).
 */

#if defined(ARDUINO_ARCH_AVR)

#include <Arduino.h>

void init_stepper_platform(void) {
  // AVR: Initialize Timer 1 for stepper pulse generation
  // (pins 9/10 on ATmega328P, pins 5/2/3/10 on ATmega2560)
}

void stop_current_move(void) {
  // Stop Timer 1
  TCCR1B = 0;
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
  return 16;  // AVR queue capacity
}

#endif  // ARDUINO_ARCH_AVR