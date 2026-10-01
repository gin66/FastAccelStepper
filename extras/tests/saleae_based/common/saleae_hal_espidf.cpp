/*
 * saleae_hal_espidf.cpp — Plain ESP-IDF HAL for the Saleae test apps.
 *
 * Uses the GPIO driver and the esp_timer / FreeRTOS tick instead of the
 * Arduino API, so the ESP-IDF build does not depend on the Arduino component.
 */

#include "saleae_hal.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_idf_version.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SALEAE_UART UART_NUM_0

extern "C" void saleae_hal_pin_output(int pin) {
  gpio_reset_pin((gpio_num_t)pin);
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
}

extern "C" void saleae_hal_write(int pin, int level) {
  gpio_set_level((gpio_num_t)pin, level ? 1 : 0);
}

extern "C" uint32_t saleae_hal_millis(void) {
  return (uint32_t)(esp_timer_get_time() / 1000);
}

extern "C" void saleae_hal_delay_ms(uint32_t ms) {
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

extern "C" void saleae_hal_serial_write(const char *text) {
  uart_write_bytes(SALEAE_UART, text, strlen(text));
}
