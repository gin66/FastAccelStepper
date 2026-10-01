/*
 * main.cpp — Saleae test firmware entry point (Arduino framework).
 *
 * Serial command parser for stepper-motor signal validation.
 * Supports ESP32 via Arduino framework.
 *
 * Serial protocol (921600 baud):
 *   Header: 0xAA 0x55
 *   Command: [CMD_ID] (1 byte)
 *   Length:  [uint16_t] (2 bytes)
 *   Data:    [payload]
 *   Checksum: [uint16_t] XOR of all preceding bytes
 *   End: 0x55 0xAA
 */

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#define SERIAL_BAUD 921600

// Command IDs
#define CMD_LIST        0x01
#define CMD_CONFIG      0x02
#define CMD_PORT_TOGGLE 0x03
#define CMD_PORT_STOP   0x04
#define CMD_DOWNLOAD    0x05
#define CMD_RUN         0x06
#define CMD_STATUS      0x07
#define CMD_METRICS     0x08
#define CMD_TAG         0x09
#define CMD_STOP        0x0A
#define CMD_RESET       0x0B
#define CMD_SIMPLE      0x0C

// SR_00 connection self-test (simple_test.cpp)
extern void simple_test_start(void);
extern void simple_test_stop(void);
extern void simple_test_loop(void);

// Response codes
#define RESP_OK     0x80
#define RESP_ERR    0x81

// Protocol constants
#define PROTO_HEADER_LEN 2
#define PROTO_CMD_LEN 1
#define PROTO_LEN_LEN 2
#define PROTO_CHECKSUM_LEN 2
#define PROTO_FOOTER_LEN 2
#define PROTO_MIN_FRAME_LEN (PROTO_HEADER_LEN + PROTO_CMD_LEN + PROTO_LEN_LEN + PROTO_CHECKSUM_LEN + PROTO_FOOTER_LEN)

// State machine for frame parsing
typedef enum {
  STATE_WAIT_HEADER_0,
  STATE_WAIT_HEADER_1,
  STATE_WAIT_CMD,
  STATE_WAIT_LENGTH_LO,
  STATE_WAIT_LENGTH_HI,
  STATE_WAIT_DATA,
  STATE_WAIT_CHECKSUM_LO,
  STATE_WAIT_CHECKSUM_HI,
  STATE_WAIT_FOOTER_0,
  STATE_WAIT_FOOTER_1,
  STATE_ERROR
} proto_state_t;

typedef struct {
  proto_state_t state;
  uint8_t frame[256];
  uint16_t frame_pos;
  uint16_t payload_len;
  uint16_t checksum;
  uint8_t cmd_id;
} proto_state_machine_t;

static proto_state_machine_t g_proto = { STATE_WAIT_HEADER_0, {0}, 0, 0, 0, 0 };

// Toggle pin state
typedef struct {
  uint8_t pin;
  uint32_t hz;
  uint32_t half_period_us;
  uint32_t last_toggle_ms;
  bool active;
} toggle_pin_t;

static toggle_pin_t g_toggle_pins[16];
static uint8_t g_toggle_count = 0;
static bool g_running = false;

// Test marker pins
#define TEST_MARKER_PIN 25
#define QUEUE_EMPTY_PIN 26

static uint16_t compute_checksum(uint8_t *data, uint16_t len) {
  uint16_t checksum = 0;
  for (uint16_t i = 0; i < len; i++) {
    checksum ^= ((uint16_t)data[i] << 8);
  }
  return checksum;
}

static void send_response(uint8_t cmd_id, uint8_t *data, uint16_t len) {
  uint8_t frame[256];
  uint16_t pos = 0;

  frame[pos++] = 0xAA;
  frame[pos++] = 0x55;
  frame[pos++] = cmd_id | 0x80;
  frame[pos++] = len & 0xFF;
  frame[pos++] = (len >> 8) & 0xFF;

  if (data && len > 0) {
    memcpy(&frame[pos], data, len);
    pos += len;
  }

  uint16_t checksum = compute_checksum(frame, pos);
  frame[pos++] = checksum & 0xFF;
  frame[pos++] = (checksum >> 8) & 0xFF;
  frame[pos++] = 0x55;
  frame[pos++] = 0xAA;

  Serial.write(frame, pos);
}

static void send_ok_response(const char *msg) {
  uint16_t len = strlen(msg);
  uint8_t *data = (uint8_t *)malloc(len);
  if (data) {
    memcpy(data, msg, len);
    send_response(CMD_LIST, data, len);
    free(data);
  }
}

static void send_error_response(uint8_t error_code) {
  uint8_t data[2] = { RESP_ERR, error_code };
  send_response(CMD_LIST, data, 2);
}

static void process_frame(void);

static void process_byte(uint8_t byte) {
  switch (g_proto.state) {
    case STATE_WAIT_HEADER_0:
      if (byte == 0xAA) {
        g_proto.frame[0] = byte;
        g_proto.state = STATE_WAIT_HEADER_1;
      }
      break;

    case STATE_WAIT_HEADER_1:
      if (byte == 0x55) {
        g_proto.frame[1] = byte;
        g_proto.state = STATE_WAIT_CMD;
      } else {
        g_proto.state = STATE_WAIT_HEADER_0;
      }
      break;

    case STATE_WAIT_CMD:
      g_proto.frame[2] = byte;
      g_proto.cmd_id = byte;
      g_proto.state = STATE_WAIT_LENGTH_LO;
      break;

    case STATE_WAIT_LENGTH_LO:
      g_proto.frame[3] = byte;
      g_proto.state = STATE_WAIT_LENGTH_HI;
      break;

    case STATE_WAIT_LENGTH_HI:
      g_proto.frame[4] = byte;
      g_proto.payload_len = g_proto.frame[3] | (g_proto.frame[4] << 8);
      if (g_proto.payload_len > 254) {
        g_proto.state = STATE_ERROR;
      } else {
        g_proto.frame_pos = 5;
        g_proto.state = STATE_WAIT_DATA;
      }
      break;

    case STATE_WAIT_DATA:
      if (g_proto.frame_pos < sizeof(g_proto.frame)) {
        g_proto.frame[g_proto.frame_pos++] = byte;
        if (g_proto.frame_pos >= 5 + g_proto.payload_len) {
          g_proto.state = STATE_WAIT_CHECKSUM_LO;
        }
      } else {
        g_proto.state = STATE_ERROR;
      }
      break;

    case STATE_WAIT_CHECKSUM_LO:
      g_proto.frame[g_proto.frame_pos++] = byte;
      g_proto.state = STATE_WAIT_CHECKSUM_HI;
      break;

    case STATE_WAIT_CHECKSUM_HI:
      g_proto.frame[g_proto.frame_pos++] = byte;
      g_proto.state = STATE_WAIT_FOOTER_0;
      break;

    case STATE_WAIT_FOOTER_0:
      if (byte == 0x55) {
        g_proto.frame[g_proto.frame_pos++] = byte;
        g_proto.state = STATE_WAIT_FOOTER_1;
      } else {
        g_proto.state = STATE_ERROR;
      }
      break;

    case STATE_WAIT_FOOTER_1:
      if (byte == 0xAA) {
        g_proto.frame[g_proto.frame_pos++] = byte;
        process_frame();
        g_proto.state = STATE_WAIT_HEADER_0;
        g_proto.frame_pos = 0;
      } else {
        g_proto.state = STATE_ERROR;
      }
      break;

    case STATE_ERROR:
      if (byte == 0xAA) {
        g_proto.state = STATE_WAIT_HEADER_1;
      } else {
        g_proto.state = STATE_WAIT_HEADER_0;
      }
      break;
  }
}

static void process_frame(void) {
  uint8_t cmd = g_proto.cmd_id & 0x7F;  // Clear response bit
  uint8_t *data = g_proto.frame + 5;
  uint16_t len = g_proto.payload_len;
  char response[128];

  // Any real test command takes over the pins from the SR_00 self-test.
  if (cmd != CMD_SIMPLE && cmd != CMD_LIST && cmd != CMD_STATUS &&
      cmd != CMD_METRICS && cmd != CMD_TAG) {
    simple_test_stop();
  }

  switch (cmd) {
    case CMD_LIST:
      strcpy(response, "OK: SR_00,SR_01,SR_02,SR_03,SR_04,SR_05,SR_06,SR_07,SR_08,SR_09,SR_10,SR_11,SR_12,SR_13,SR_14,SR_15,SR_16,SR_17,SR_18,SR_19,SR_20,SR_21,SR_22,SR_23,SR_24,SR_25,SR_26,SR_27,SR_28,SR_29,SR_30,SR_31,SR_32,SR_33,SR_34,SR_35,SR_36,SR_37,SR_38,SR_39,SR_40");
      send_ok_response(response);
      break;

    case CMD_SIMPLE:
      simple_test_start();
      send_ok_response("OK: SR_00 simple test started");
      break;

    case CMD_CONFIG:
      if (len > 0) {
        strncpy(response, (char *)data, len);
        response[len] = '\0';
        snprintf(response, sizeof(response), "OK: config=%s", (char *)data);
      } else {
        strcpy(response, "OK: config=4ch_rmt");
      }
      send_ok_response(response);
      break;

    case CMD_PORT_TOGGLE:
      if (len >= 2) {
        uint8_t pin = data[0];
        uint32_t hz = data[1];
        
        if (g_toggle_count < 16) {
          int slot = -1;
          for (int i = 0; i < 16; i++) {
            if (!g_toggle_pins[i].active || g_toggle_pins[i].pin == pin) {
              slot = i;
              break;
            }
          }
          
          if (slot >= 0) {
            pinMode(pin, OUTPUT);
            uint32_t period_us = 1000000 / (hz > 0 ? hz : 1);
            g_toggle_pins[slot].pin = pin;
            g_toggle_pins[slot].hz = hz > 0 ? hz : 1;
            g_toggle_pins[slot].half_period_us = period_us / 2;
            g_toggle_pins[slot].last_toggle_ms = millis();
            g_toggle_pins[slot].active = true;
            if (slot >= g_toggle_count) g_toggle_count = slot + 1;
            
            snprintf(response, sizeof(response), "OK: pin=%u freq=%luHz", pin, (unsigned long)g_toggle_pins[slot].hz);
            send_ok_response(response);
          } else {
            send_error_response(0x01);
          }
        }
      } else {
        send_error_response(0x02);
      }
      break;

    case CMD_PORT_STOP:
      for (int i = 0; i < g_toggle_count; i++) {
        if (g_toggle_pins[i].active) {
          digitalWrite(g_toggle_pins[i].pin, LOW);
          g_toggle_pins[i].active = false;
        }
      }
      g_toggle_count = 0;
      send_ok_response("OK: stopped");
      break;

    case CMD_DOWNLOAD:
      if (len >= 2) {
        uint16_t count = data[0] | (data[1] << 8);
        snprintf(response, sizeof(response), "OK: %u entries queued (stub)", count);
        send_ok_response(response);
      } else {
        send_error_response(0x03);
      }
      break;

    case CMD_RUN:
      if (len > 0) {
        strncpy(response, (char *)data, len);
        response[len] = '\0';
        snprintf(response, sizeof(response), "OK: test %s started (stub)", (char *)data);
      } else {
        strcpy(response, "OK: test started (stub)");
      }
      g_running = true;
      pinMode(TEST_MARKER_PIN, OUTPUT);
      digitalWrite(TEST_MARKER_PIN, HIGH);
      send_ok_response(response);
      break;

    case CMD_STATUS: {
      snprintf(response, sizeof(response), "OK: running=%d, toggle_pins=%u", 
               g_running ? 1 : 0, g_toggle_count);
      send_ok_response(response);
      break;
    }

    case CMD_METRICS:
      strcpy(response, "OK: metrics=unavailable (stub)");
      send_ok_response(response);
      break;

    case CMD_TAG:
      strcpy(response, "OK: tagged");
      send_ok_response(response);
      break;

    case CMD_STOP:
      g_running = false;
      pinMode(TEST_MARKER_PIN, OUTPUT);
      digitalWrite(TEST_MARKER_PIN, LOW);
      pinMode(QUEUE_EMPTY_PIN, OUTPUT);
      digitalWrite(QUEUE_EMPTY_PIN, LOW);
      send_ok_response("OK: stopped");
      break;

    case CMD_RESET:
      g_running = false;
      for (int i = 0; i < g_toggle_count; i++) {
        if (g_toggle_pins[i].active) {
          digitalWrite(g_toggle_pins[i].pin, LOW);
          g_toggle_pins[i].active = false;
        }
      }
      g_toggle_count = 0;
      pinMode(TEST_MARKER_PIN, OUTPUT);
      digitalWrite(TEST_MARKER_PIN, LOW);
      pinMode(QUEUE_EMPTY_PIN, OUTPUT);
      digitalWrite(QUEUE_EMPTY_PIN, LOW);
      send_ok_response("OK: reset");
      break;

    default:
      send_error_response(0xFF);
      break;
  }
}

static void toggle_pins_loop(void) {
  uint32_t now = millis();
  static bool pin_states[16] = {false};

  for (int i = 0; i < g_toggle_count; i++) {
    if (g_toggle_pins[i].active) {
      uint32_t elapsed = now - g_toggle_pins[i].last_toggle_ms;
      uint32_t threshold_ms = g_toggle_pins[i].half_period_us / 1000;
      
      if (threshold_ms == 0) threshold_ms = 1;
      
      if (elapsed >= threshold_ms) {
        pin_states[i] = !pin_states[i];
        digitalWrite(g_toggle_pins[i].pin, pin_states[i] ? HIGH : LOW);
        g_toggle_pins[i].last_toggle_ms = now;
      }
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(100);
  }

  pinMode(TEST_MARKER_PIN, OUTPUT);
  digitalWrite(TEST_MARKER_PIN, LOW);
  pinMode(QUEUE_EMPTY_PIN, OUTPUT);
  digitalWrite(QUEUE_EMPTY_PIN, LOW);

  Serial.println("OK: Saleae test firmware ready");
  Serial.println("Commands: LIST, SIMPLE, CONFIG <name>, PORT_TOGGLE <pin> <hz>,");
  Serial.println("          DOWNLOAD <count>, RUN <test_id>, STATUS, METRICS, TAG, STOP, RESET");

  // SR_00 runs first: prove every I/O is alive / correctly wired.
  simple_test_start();
}

void loop() {
  while (Serial.available()) {
    process_byte(Serial.read());
  }
  simple_test_loop();
  toggle_pins_loop();
}