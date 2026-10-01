/*
 * saleae_hal_espidf.cpp — Plain ESP-IDF HAL for the Saleae test apps.
 *
 * Uses the GPIO driver and the esp_timer / FreeRTOS tick instead of the
 * Arduino API, so the ESP-IDF build does not depend on the Arduino component.
 */

#include "saleae_hal.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
