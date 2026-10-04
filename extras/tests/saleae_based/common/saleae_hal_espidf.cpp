/*
 * saleae_hal_espidf.cpp — Plain ESP-IDF HAL for the Saleae test apps.
 *
 * Uses the GPIO driver and the esp_timer / FreeRTOS tick instead of the
 * Arduino API, so the ESP-IDF build does not depend on the Arduino component.
 */

// Both HALs are linked into every build (see scripts/link_app.sh), so each
// compiles to nothing on the platform it does not serve.
#if defined(ESP_PLATFORM) && !defined(ARDUINO)

#include "saleae_hal.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_idf_version.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SALEAE_UART UART_NUM_0

// No gpio_reset_pin(). The IDF driver logs every pin reset at INFO, and the
// harness reconfigures all eight pins on every SR00 and every CONFIG. That log
// traffic is not information and it is not free: the UART is installed with
// tx_buffer_size 0, so uart_write_bytes() blocks its caller until a 115200-baud
// line has drained, and the stall lands in the very loop that generates the
// SR_00 pattern -- measured as a 25 ms stretch on one channel, which SR_00
// reads as a spurious edge. Direction plus level is all these pins need;
// nothing else drives them.
extern "C" void saleae_hal_pin_output(int pin) {
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
}

extern "C" void saleae_hal_write(int pin, int level) {
  gpio_set_level((gpio_num_t)pin, level ? 1 : 0);
}

extern "C" uint32_t saleae_hal_millis(void) {
  return (uint32_t)(esp_timer_get_time() / 1000);
}

// pdMS_TO_TICKS(1) is 0 at FreeRTOS's 100 Hz, which is the IDF default on
// ESP32, and vTaskDelay(0) only yields. The "1 ms" idle delay was therefore a
// 100 %-CPU spin: harmless-looking, but SR_00's 1 Hz pattern then lands
// wherever the scheduler happens to switch (measured 428 us / 455.9 ms widths
// against a commanded 50/950 ms), and SR_00 is the wiring pre-check every other
// test is gated on. Spin on esp_timer below one tick; sleep above it. The spin
// only ever runs when the harness is idle or in SR_00 -- a scenario in flight
// goes through qe_pump() instead -- so it does not perturb a measurement.
extern "C" void saleae_hal_delay_ms(uint32_t ms) {
  const uint32_t tick_ms = 1000U / configTICK_RATE_HZ;
  if ((ms == 0) || ((tick_ms > 1) && (ms < tick_ms))) {
    const int64_t until = esp_timer_get_time() + (int64_t)ms * 1000;
    while (esp_timer_get_time() < until) {
    }
    return;
  }
  vTaskDelay(pdMS_TO_TICKS(ms));
}

extern "C" void saleae_hal_serial_begin(uint32_t baud) {
  uart_config_t cfg = {};
  cfg.baud_rate = baud;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  cfg.source_clk = UART_SCLK_DEFAULT;
#endif
  uart_driver_install(SALEAE_UART, 1024, 0, 0, NULL, 0);
  uart_param_config(SALEAE_UART, &cfg);
  uart_set_pin(SALEAE_UART, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

extern "C" int saleae_hal_serial_read(void) {
  uint8_t c;
  return uart_read_bytes(SALEAE_UART, &c, 1, 0) == 1 ? c : -1;
}

extern "C" void saleae_hal_idle(void) {
  // One tick, blocked. See saleae_hal.h for why this is not delay_ms(1): a
  // sub-tick delay spins, and a spinning idle loop keeps IDLE from ever running,
  // so the task watchdog fires on a board that is only waiting for a command.
  // Note the watchdog subscription that fires is IDLE's, so resetting the WDT
  // from this task would not help -- only blocking lets IDLE reset its own.
  vTaskDelay(1);
}

extern "C" void saleae_hal_serial_write(const char* text) {
  uart_write_bytes(SALEAE_UART, text, strlen(text));
}

// See the note in saleae_hal_arduino.cpp: the flash/RAM distinction only exists
// on AVR, and uart_write_bytes() takes a length, so it never has to read the
// string as C would.
extern "C" void saleae_hal_serial_write_p(const char* text) {
  uart_write_bytes(SALEAE_UART, text, strlen(text));
}

#endif  // ESP_PLATFORM
