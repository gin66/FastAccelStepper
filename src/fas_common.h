#ifndef COMMON_H
#define COMMON_H

#define TICKS_FOR_STOPPED_MOTOR 0xffffffff

#define MOVE_OK 0
#define MOVE_ERR_NO_DIRECTION_PIN -1
#define MOVE_ERR_SPEED_IS_UNDEFINED -2
#define MOVE_ERR_ACCELERATION_IS_UNDEFINED -3

// Low level stepper motor command.
//	If steps is 0, then a pause is generated
//	If steps is 0, then a pause is generated
//
// You can add these using the addQueueEntry method.
// They will be executed sequentially until the queue runs out.
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
  //
  // There are `TICKS_PER_S` ticks per second. This may vary between different platforms.
  uint16_t ticks;
  // Number of steps to send to the stepper motor during this command.
  //
  // If zero, then this command will be treated as a pause, lasting for a number of ticks given by `ticks`.
  uint8_t steps;
  // True if the direction pin should be high during this command, false if it should be low.
  bool count_up;
};

struct actual_ticks_s {
  uint32_t ticks;  // ticks == 0 means standstill
  bool count_up;
};

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

// disable inject_fill_interrupt() for all real devices
#ifndef TEST
#define inject_fill_interrupt(x)
#endif

//==========================================================================
//
// The TEST "architecture" is in use with pc_based testing.
//
//
//==========================================================================
#if defined(TEST)
#include "fas_arch/test_pc.h"

//==========================================================================
//
// This for ESP32 derivates using arduino core
//
//==========================================================================
#elif defined(ARDUINO_ARCH_ESP32)
#include "fas_arch/arduino_esp32.h"

//==========================================================================
//
// This for ESP32 derivates using espidf
//
// This is most likely broken and not tested on github actions
//
//==========================================================================
#elif defined(ESP_PLATFORM)
#include "fas_arch/espidf_esp32.h"

//==========================================================================
//
// This for SAM-architecture
//
//==========================================================================
#elif defined(ARDUINO_ARCH_SAM)
#include "fas_arch/arduino_sam.h"

//==========================================================================
//
// This for the AVR family
//
//==========================================================================
#elif defined(ARDUINO_ARCH_AVR)
#include "fas_arch/arduino_avr.h"

//==========================================================================
//
// For all unsupported devices
//
//==========================================================================
#else
#error "Unsupported devices"

#endif

#ifdef __ESP32_IDF_V44__
#include <driver/periph_ctrl.h>
#include <soc/periph_defs.h>
#endif /* __ESP32_IDF_V44__ */

//==========================================================================
// determine, if driver type selection should be supported
#if defined(QUEUES_MCPWM_PCNT) && defined(QUEUES_RMT)
#if (QUEUES_MCPWM_PCNT > 0) && (QUEUES_RMT > 0)
#define SUPPORT_SELECT_DRIVER_TYPE
#endif
#endif

#endif /* COMMON_H */
