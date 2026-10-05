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
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SALEAE_UART UART_NUM_0

// Whether app_main is subscribed to the task watchdog. See
// saleae_hal_wdt_subscribe() below: `esp_task_wdt_reset()` on a task that is
// not subscribed logs an error every call, and that is once per main-loop
// pass on the UART the host protocol runs on.
static bool wdt_subscribed = false;

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

// Subscribe app_main to the task watchdog, and keep it fed.
//
// Measured on this harness, ESP-IDF 5.5.3: the board trips its own task
// watchdog while doing nothing at all -- `open_board()`, no command, no
// capture, six `task_wdt` lines within ~5.3 s and an IDLE0 backtrace naming
// `main` as the running task. IDLE0 is subscribed to the WDT by default, and
// it starves because app_main never blocks: `saleae_hal_serial_read()` is a
// zero-timeout `uart_read_bytes()`, and the sub-tick spin in
// `saleae_hal_delay_ms(1)` means a pass through the loop ends in a busy wait.
//
// Why this matters more than a crashed idle board: a panic inside a capture
// window truncates the run, and SR_00 then reports the truncated 1 Hz pattern
// as eight dead pins -- the one fault the wiring pre-check exists to catch. It
// did exactly that, and the result was recorded as `SR_00: failed`.
//
// Subscribing rather than removing the spin. The spin has to stay in
// `saleae_test_loop()`, where an edge has to land on time and where
// `saleae_hal_delay_ms(1)` is deliberate (see the note above); the feeder uses
// it too, and blocking there grew trailing pulses on mcpwm_pcnt. So the spin is
// fixed and the watchdog is fed, which is the arrangement in which the timing
// the matrix was measured with survives.
//
// `esp_task_wdt_add(NULL)` subscribes the *calling* task and is what the IDF
// docs prescribe for exactly this. It is called once, from setup, because
// adding the same task twice returns ESP_ERR_INVALID_STATE and logs an error.
extern "C" void saleae_hal_wdt_subscribe(void) {
  if (esp_task_wdt_add(NULL) == ESP_OK) {
    wdt_subscribed = true;
  }
}

extern "C" void saleae_hal_wdt_reset(void) {
  // Only once subscribed: `esp_task_wdt_reset()` on an unsubscribed task logs
  // `E task_wdt: esp_task_wdt_reset(707): task not found`, and this runs once
  // per main-loop pass, on the UART the host protocol uses. Measured: that call
  // alone flooded the console and the capture window with it.
  if (wdt_subscribed) {
    esp_task_wdt_reset();
  }
}

// One whole tick, blocked -- which is what saleae_hal.h asks for and what this
// function was not doing.
//
// `saleae_hal_idle()` was `saleae_hal_delay_ms(1)`, and that resolves to the
// sub-tick *spin* branch above: at FreeRTOS's 100 Hz `pdMS_TO_TICKS(1)` is 0,
// so app_main spun at 100 % CPU instead of ever blocking. The header says
// "deliberately not saleae_hal_delay_ms(1) ... Spinning in the idle path
// starves IDLE, and the task watchdog then resets a board that is doing nothing
// at all -- measured on ESP-IDF 4.4.3, ten seconds after boot, with no command
// outstanding."
//
// That reproduction came back on 5.5.3 while closing this item, and it is worse
// than a crashed idle board: a panic inside a capture window truncates the run,
// and SR_00 then reports the truncated 1 Hz pattern as eight dead pins -- the
// one fault the wiring pre-check exists to catch. Measured here: `open_board()`,
// no command, no capture, six `task_wdt` lines within ~5.3 s with an IDLE0
// backtrace naming `main`. And the spin was *also* what the note below called
// out as deliberate, so the fix has to reconcile two measurements rather than
// pick one.
//
// The reconciliation: the spin that had to stay is in `saleae_test_loop()`, the
// SR_00 pattern, where an edge has to land on time and where this function is
// never called. This one is the idle path, which the header already required to
// block. So the pattern keeps its sub-tick spin and the idle path stops
// spinning, which is the only arrangement in which both statements hold.
//
// One tick and not `DELAY_MS(10)`: 10 ms would put a 10 ms floor on how
// promptly a command is picked up, and the feeder is this same loop. Measured
// after this change: the watchdog stays quiet, and SR_00's widths are unchanged
// (the pattern path uses `saleae_hal_delay_ms(1)`, which is still the spin, and
// has to be -- that is where an edge has to land).
//
// Neither change alone is enough, and both were needed -- measured, in this
// order. Blocking the idle path alone left the watchdog firing during SR_00,
// whose pattern loop is the spin this comment is about. Feeding the watchdog
// alone was worse than useless until the subscription existed: every
// `esp_task_wdt_reset()` logged `task not found` and flooded the console.
extern "C" void saleae_hal_idle(void) {
  vTaskDelay(pdMS_TO_TICKS(1) ? pdMS_TO_TICKS(1) : 1);
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
