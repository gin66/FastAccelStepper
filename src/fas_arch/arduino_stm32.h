#ifndef FAS_ARCH_ARDUINO_STM32_H
#define FAS_ARCH_ARDUINO_STM32_H

#define FAS_STM32

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>

// PRIMASK reentrant-safe interrupt control
// Saves and restores PRIMASK to support nested disable/enable calls
#define fasDisableInterrupts() \
  uint32_t __fas_prim = __get_PRIMASK(); __disable_irq()
#define fasEnableInterrupts() \
  __set_PRIMASK(__fas_prim)

#define FAS_PSTR(s)  (s)
// PIN_UNDEFINED (255) and PIN_EXTERNAL_FLAG (128) are defined in
// FastAccelStepper.h. Do NOT redefine here to avoid -Wmacro-redefined.

#endif /* FAS_ARCH_ARDUINO_STM32_H */