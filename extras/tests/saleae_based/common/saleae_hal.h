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

// Is this an ESP32, whatever the SDK? The pin map is a property of the board and
// the wiring, not of the framework, and the two files that carry it
// (saleae_test.cpp's SR_00 pins, saleae_app.cpp's SAL_CHAN_PINS) must agree --
// one is the identification pattern, the other is what the scenarios measure.
// Testing for Arduino instead put the plain ESP-IDF builds on the generic table
// {2, 3, 4, 5, ...}, whose GPIO3 is UART0's RX: the harness reconfigured its own
// console pin as a stepper direction output (half a console, since TX is GPIO1),
// and the analyzer, watching GPIO0, never saw the direction at all.
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM) || \
    defined(CONFIG_IDF_TARGET_ESP32)
#define SALEAE_TARGET_ESP32 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

void saleae_hal_pin_output(int pin);
void saleae_hal_write(int pin, int level);
uint32_t saleae_hal_millis(void);
void saleae_hal_delay_ms(uint32_t ms);

// One idle pass's worth of wait, as a *blocking* delay on an RTOS.
//
// Deliberately not `saleae_hal_delay_ms(1)`: that resolves to a busy spin below
// one tick, which is right where an edge has to land (SR_00) and wrong where it
// merely means "nothing to do". Spinning in the idle path starves IDLE, and the
// task watchdog then resets a board that is doing nothing at all -- measured on
// ESP-IDF 4.4.3, ten seconds after boot, with no command outstanding. Blocking
// for one tick is what the project's own ESP-IDF entry point does:
// `DELAY_MS(10)` in examples/StepperDemo/StepperDemo.ino, which is
// `vTaskDelay(pdMS_TO_TICKS(10))`.
void saleae_hal_idle(void);

// Serial console (host command channel)
void saleae_hal_serial_begin(uint32_t baud);
int saleae_hal_serial_read(void);  // returns a byte 0..255, or -1 if none
void saleae_hal_serial_write(const char* text);    // `text` is in RAM
void saleae_hal_serial_write_p(const char* text);  // `text` is in flash

#ifdef __cplusplus
}
#endif

#endif /* SALEAE_HAL_H */
