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

// Serial console (host command channel)
void saleae_hal_serial_begin(uint32_t baud);
int saleae_hal_serial_read(void);  // returns a byte 0..255, or -1 if none
void saleae_hal_serial_write(const char* text);    // `text` is in RAM
void saleae_hal_serial_write_p(const char* text);  // `text` is in flash

#ifdef __cplusplus
}
#endif

#endif /* SALEAE_HAL_H */
