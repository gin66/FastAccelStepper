#ifndef GENERIC_H
#define GENERIC_H

#if defined(ARDUINO_ARCH_AVR)
#include <Arduino.h>
#endif

#if defined(ARDUINO_ARCH_AVR)
#define get_char(x) pgm_read_byte(x)
#define MSG_TYPE PGM_P
#elif defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_SAM)
#define get_char(x) *x
#define MSG_TYPE const char*
#elif defined(ESP_PLATFORM)
#define get_char(x) *x
#define MSG_TYPE const char*
#elif defined(PICO_RP2040) || defined(PICO_RP2350)
#define get_char(x) *x
#define MSG_TYPE const char*
#else
#error "Unsupported derivate"
#endif

#if !defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)

#define PRINT_INIT() Serial.begin(115200);
#define PRINTLN Serial.println
#define PRINTCH Serial.print
#define PRINTU8 Serial.print
#define PRINTU16 Serial.print
#define PRINTI16 Serial.print
#define PRINTU32 Serial.print
#define PRINTI32 Serial.print
#define PRINT Serial.print
#define POLL_CHAR_IF_ANY(ch) \
  if (Serial.available()) {  \
    ch = Serial.read();      \
  }
#define MILLIS() millis()
#define DELAY_US(v) delayMicroseconds(v)

#else

#include <thread>
#include <driver/uart.h>
#include <driver/uart_vfs.h>
#include <esp_timer.h>
#include <cstring>
#include <cinttypes>

const uart_port_t uart_num = UART_NUM_0;

#define PRINT_INIT()                                     \
  {                                                      \
    if (!uart_is_driver_installed(uart_num)) {           \
      uart_config_t config = {                           \
          .baud_rate = 115200,                           \
          .data_bits = UART_DATA_8_BITS,                 \
          .parity = UART_PARITY_DISABLE,                 \
          .stop_bits = UART_STOP_BITS_1,                 \
          .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,         \
          .rx_flow_ctrl_thresh = 0,                      \
          .source_clk = UART_SCLK_DEFAULT,               \
          .flags = 0,                                    \
      };                                                 \
      uart_param_config(uart_num, &config);              \
      uart_driver_install(uart_num, 256, 0, 0, NULL, 0); \
    }                                                    \
    uart_vfs_dev_use_driver(uart_num);                   \
  }

#define PRINTLN puts
#define PRINTU8(v) printf("%u", v)
#define PRINTU16(v) printf("%u", v)
#define PRINTI16(v) printf("%d", v)
#define PRINTU32(v) printf("%" PRIu32, v)
#define PRINTI32(v) printf("%" PRIi32, v)
#define PRINTCH(ch) printf("%c", ch)
#define PRINT(s) printf("%s", s)
#define POLL_CHAR_IF_ANY(ch)                       \
  {                                                \
    uint8_t _ch;                                   \
    int n = uart_read_bytes(uart_num, &_ch, 1, 0); \
    if (n == 1) {                                  \
      ch = _ch;                                    \
    }                                              \
  }
#define MILLIS() (esp_timer_get_time() / 1000)
#define DELAY_US(v) std::this_thread::sleep_for(std::chrono::microseconds(v))
#define DELAY_MS(v) vTaskDelay(pdMS_TO_TICKS(v))
#define NEED_APP_MAIN

#endif

#if defined(ESP_PLATFORM)
#define PROGMEM
#endif

#endif
