#ifndef FAS_ARCH_PICO_SDK_RP_PICO_H
#define FAS_ARCH_PICO_SDK_RP_PICO_H

#include "hardware/sync.h"

// For pico using arduino, just use arduino definition
#define fasEnableInterrupts portEXIT_CRITICAL
#define fasDisableInterrupts portENTER_CRITICAL

#define digitalWrite gpio_put
#define HIGH true
#define LOW false

#define PIN_OUTPUT(pin, value)   \
  {                              \
    gpio_init(pin);              \
    gpio_put(pin, (value));      \
    gpio_set_dir(pin, GPIO_OUT); \
  }

#include "fas_arch/common_rp_pico.h"

#endif /* FAS_ARCH_PICO_SDK_RP_PICO_H */
