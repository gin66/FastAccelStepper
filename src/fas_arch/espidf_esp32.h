#ifndef FAS_ARCH_ESPIDF_ESP32_H
#define FAS_ARCH_ESPIDF_ESP32_H

// esp32 specific includes
#include <math.h>

// on espidf need to use portDISABLE/ENABLE_INTERRUPTS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#define fasDisableInterrupts portDISABLE_INTERRUPTS
#define fasEnableInterrupts portENABLE_INTERRUPTS

// The espidf-platform needs a couple of arduino like definitions
#define LOW 0
#define HIGH 1
#define OUTPUT GPIO_MODE_OUTPUT
#define pinMode(pin, mode)            \
  gpio_set_direction((gpio_num_t)pin, \
                     (mode) == OUTPUT ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT)
#define digitalWrite(pin, level) gpio_set_level((gpio_num_t)pin, level)
#define digitalRead(pin) gpio_get_level((gpio_num_t)pin)

#include "fas_arch/common_esp32.h"

#endif /* FAS_ARCH_ESPIDF_ESP32_H */
