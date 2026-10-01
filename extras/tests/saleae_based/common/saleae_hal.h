/*
 * saleae_hal.h — Minimal hardware abstraction for the Saleae test apps.
 *
 * The common test logic only uses these four primitives, so the exact same
 * code compiles on Arduino (ESP32/Pico) and on plain ESP-IDF, where the
 * platform-specific HAL implementation lives in the respective app.
 */

#ifndef SALEAE_HAL_H
#define SALEAE_HAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void saleae_hal_pin_output(int pin);
void saleae_hal_write(int pin, int level);
uint32_t saleae_hal_millis(void);
void saleae_hal_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* SALEAE_HAL_H */
