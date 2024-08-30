#ifndef FAS_COMMON_H
#define FAS_COMMON_H

#define TICKS_FOR_STOPPED_MOTOR 0xffffffff

#define MOVE_OK 0
#define MOVE_ERR_NO_DIRECTION_PIN -1
#define MOVE_ERR_SPEED_IS_UNDEFINED -2
#define MOVE_ERR_ACCELERATION_IS_UNDEFINED -3

// Low level stepper motor command.
//
// You can add these using the addQueueEntry method.
// They will be executed sequentially until the queue is empty.
//
// There are some constraints on the values:
// - `ticks` must be greater or equal to FastAccelStepper::getMaxSpeedInTicks.
// - `ticks*steps` must be greater or equal to MIN_CMD_TICKS
//
// For example:
// A command with ticks=TICKS_PER_S/1000, steps = 3, count_up = true means that:
// 1. The direction pin is set to HIGH.
// 2. One step is generated.
// 3. Exactly 1 ms after the first step, the second step is issued.
// 4. Exactly 1 ms after the second step, the third step is issued.
// 5. The stepper waits for 1 ms.
// 6. The next command is processed.
struct stepper_command_s {
  // Number of ticks between each step.
  // There are `TICKS_PER_S` ticks per second. This may vary between different
  // platforms.
  uint16_t ticks;

  // Number of steps to send to the stepper motor during this command.
  // If zero, then this command will be treated as a pause, lasting for a number
  // of ticks given by `ticks`.
  uint8_t steps;

  // True if the direction pin should be high during this command, false if it
  // should be low.
  bool count_up;
};

struct actual_ticks_s {
  uint32_t ticks;  // ticks == 0 means standstill
  bool count_up;
};

// I doubt, volatile is needed.
struct queue_end_s {
  volatile int32_t pos;  // in steps
  volatile bool count_up;
  volatile bool dir;
};

// use own min/max/abs function, because the lib versions are messed up
#define fas_min(a, b) ((a) > (b) ? (b) : (a))
#define fas_max(a, b) ((a) > (b) ? (a) : (b))
#define fas_abs(x) ((x) >= 0 ? (x) : (-x))

//==============================================================================
// All architecture specific definitions should be located here
//==============================================================================

//==========================================================================
#if defined(TEST)
// TEST "architecture" is in use with pc_based testing.
#include "fas_arch/test_pc.h"

#elif defined(ARDUINO_ARCH_ESP32)
// ESP32 derivates using arduino core
#include "fas_arch/arduino_esp32.h"

#elif defined(ESP_PLATFORM)
// ESP32 derivates using espidf
#include "fas_arch/espidf_esp32.h"

#elif defined(ARDUINO_ARCH_SAM)
// SAM-architecture
#include "fas_arch/arduino_sam.h"

#elif defined(ARDUINO_ARCH_AVR)
// AVR family
#include "fas_arch/arduino_avr.h"

#else
#error "Unsupported devices"
#endif

// in order to avoid spikes, first set the value and then make an output
// esp32 idf5 does not like this approach
#ifndef PIN_OUTPUT
#define PIN_OUTPUT(pin, value)  \
  {                             \
    digitalWrite(pin, (value)); \
    pinMode(pin, OUTPUT);       \
  }
#endif

// disable inject_fill_interrupt() for all real devices. Only defined in TEST
#ifndef TEST
#define inject_fill_interrupt(x)
#endif

#endif /* FAS_COMMON_H */
