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
#else
#error "Unsupported derivate"
#endif

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32C3) && (ARDUINO_USB_MODE == 1)

#define PRINT_INIT() \
   	USBSerial.begin(115200); \
    while (!Serial) { \
      /* wait for USB serial port to connect */ \
    }
#define PRINTLN USBSerial.println
#define PRINTCH USBSerial.print
#define PRINTU8 USBSerial.print
#define PRINTU16 USBSerial.print
#define PRINTI16 USBSerial.print
#define PRINTU32 USBSerial.print
#define PRINTI32 USBSerial.print
#define PRINT USBSerial.print
#define POLL_CHAR_IF_ANY(ch) \
    if (USBSerial.available()) { \
      ch = USBSerial.read(); \
    }
#define MILLIS() millis()
#define DELAY_US(v) delayMicroseconds(v)

#elif !defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)

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
    if (Serial.available()) { \
      ch = Serial.read(); \
    }
#define MILLIS() millis()
#define DELAY_US(v) delayMicroseconds(v)

#else

#include <thread>
#include <driver/uart.h>
#include <esp_timer.h>
#include <cstring>
#include <cinttypes>

const uart_port_t uart_num = UART_NUM_0;

#define PRINT_INIT()
#ifdef OLD
uart_config_t config;\
    config.baud_rate = 115200;\
    config.data_bits = UART_DATA_8_BITS;\
    config.parity = UART_PARITY_DISABLE;\
    config.stop_bits = UART_STOP_BITS_1;\
    config.flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS;\
    config.rx_flow_ctrl_thresh = 122; \
uart_param_config(uart_num, &config);
#endif

#define PRINTLN puts
#define PRINTU8(v) printf("%u",v)
#define PRINTU16(v) printf("%u",v)
#define PRINTI16(v) printf("%d",v)
#define PRINTU32(v) printf("%" PRIu32,v)
#define PRINTI32(v) printf("%" PRIi32,v)
#define PRINTCH(ch) printf("%c",ch)
#define PRINT(s)    printf("%s",s)
#define POLL_CHAR_IF_ANY(ch) {uint8_t _ch = getchar();if (_ch != 255) { ch = _ch;}}
#ifdef OLD
#define POLL_CHAR_IF_ANY(ch) \
	     uint8_t _ch; \
     int n = uart_read_bytes(uart_num, &_ch, 1, 0); \
		 if (n == 1) { \
			 ch = _ch; \
		 }
#endif
#define MILLIS() (esp_timer_get_time()/1000) 
#define DELAY_US(v) std::this_thread::sleep_for(std::chrono::microseconds(v))
#define DELAY_MS(v) vTaskDelay(pdMS_TO_TICKS(v))
#define NEED_APP_MAIN

#endif

#if defined(ESP_PLATFORM)
#define PROGMEM
#endif

#endif
