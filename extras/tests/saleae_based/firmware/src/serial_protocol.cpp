/*
 * serial_protocol.cpp — Binary serial command/response protocol.
 *
 * Protocol format:
 *   [0xAA 0x55]        Header (2 bytes)
 *   [CMD_ID]           Command identifier (1 byte)
 *   [uint16_t]         Payload length (2 bytes, little-endian)
 *   [data]             Payload bytes
 *   [uint16_t]         XOR checksum of all preceding bytes (2 bytes)
 *   [0x55 0xAA]        Footer (2 bytes)
 *
 * Response format is identical, with CMD_ID replaced by RESPONSE_CMD_ID.
 */

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

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
  uint8_t frame[256];  // Maximum frame size
  uint16_t frame_pos;
  uint16_t payload_len;
  uint16_t checksum;
} proto_state_machine_t;

static proto_state_machine_t g_proto = { STATE_WAIT_HEADER_0, {0}, 0, 0, 0 };

static uint16_t compute_checksum(uint8_t *data, uint16_t len) {
  uint16_t checksum = 0;
  for (uint16_t i = 0; i < len; i++) {
    checksum ^= ((uint16_t)data[i] << 8);
  }
  return checksum;
}

static void send_response(uint8_t cmd_id, uint8_t *data, uint16_t len) {
  uint16_t frame_len = PROTO_HEADER_LEN + PROTO_CMD_LEN + PROTO_LEN_LEN + len + PROTO_CHECKSUM_LEN + PROTO_FOOTER_LEN;
  uint8_t frame[256];
  uint16_t pos = 0;

  // Header
  frame[pos++] = 0xAA;
  frame[pos++] = 0x55;

  // Command ID (echo back with response bit set)
  frame[pos++] = cmd_id | 0x80;

  // Length
  frame[pos++] = len & 0xFF;
  frame[pos++] = (len >> 8) & 0xFF;

  // Payload
  if (data && len > 0) {
    memcpy(&frame[pos], data, len);
    pos += len;
  }

  // Checksum
  uint16_t checksum = compute_checksum(frame, pos);
  frame[pos++] = checksum & 0xFF;
  frame[pos++] = (checksum >> 8) & 0xFF;

  // Footer
  frame[pos++] = 0x55;
  frame[pos++] = 0xAA;

  Serial.write(frame, pos);
}

static void send_ok_response(const char *msg) {
  uint16_t len = strlen(msg);
  uint8_t *data = (uint8_t *)malloc(len);
  if (data) {
    memcpy(data, msg, len);
    send_response(CMD_LIST, data, len);  // Use appropriate cmd_id
    free(data);
  }
}

static void send_error_response(uint8_t error_code) {
  uint8_t data[2] = { RESP_ERR, error_code };
  send_response(CMD_LIST, data, 2);
}

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
        // Frame complete — process it
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

// Forward declaration
extern void process_frame(void);

void process_serial_commands(void) {
  while (Serial.available()) {
    process_byte(Serial.read());
  }
}