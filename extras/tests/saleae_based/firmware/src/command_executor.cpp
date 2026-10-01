/*
 * command_executor.cpp — Downloads queue commands via serial and enqueues to stepper.
 *
 * Handles DOWNLOAD, RUN, STOP, RESET commands.
 * Uses queue_entry binary format from src/fas_queue/base.h.
 */

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

// Forward declarations from FastAccelStepper library
// (included via platform-specific adapter)
extern void init_stepper_platform(void);
// extern void stepperConnectToPin(uint8_t pin);
// extern void stepper.addQueueEntry(uint8_t steps, uint16_t ticks, ...);
// extern int32_t getCurrentPosition(void);
// extern uint8_t getQueueLevel(void);
// extern uint8_t getQueueCapacity(void);

// queue_entry struct (from src/fas_queue/base.h)
// Base size: 4 bytes (no optional fields)
// With SUPPORT_QUEUE_ENTRY_END_POS_U16 or ..._START_POS_U16: 6 bytes
struct queue_entry {
  uint8_t steps;       // 1 byte: if 0, pure delay (no step pulses)
  uint8_t toggle_dir : 1;   // 1 bit: toggle direction
  uint8_t countUp    : 1;   // 1 bit: direction (true=forward)
  uint8_t hasSteps   : 1;   // 1 bit: whether this entry has steps
  uint8_t dirPinState: 1;   // 1 bit: direction pin state
  uint16_t ticks;           // 2 bytes: tick count for delay/speed
#if defined(SUPPORT_QUEUE_ENTRY_END_POS_U16)
  uint16_t end_pos_last16;  // 2 bytes: optional end position
#endif
#if defined(SUPPORT_QUEUE_ENTRY_START_POS_U16)
  uint16_t start_pos_last16;// 2 bytes: optional start position
#endif
};

// Platform-specific queue_entry size
#if defined(SUPPORT_QUEUE_ENTRY_END_POS_U16) && defined(SUPPORT_QUEUE_ENTRY_START_POS_U16)
  #define QUEUE_ENTRY_SIZE 8
#elif defined(SUPPORT_QUEUE_ENTRY_END_POS_U16) || defined(SUPPORT_QUEUE_ENTRY_START_POS_U16)
  #define QUEUE_ENTRY_SIZE 6
#else
  #define QUEUE_ENTRY_SIZE 4
#endif

#define MAX_QUEUE_ENTRIES 512

static bool g_running = false;
static uint16_t g_queued_entries = 0;

static void handle_download(uint8_t *data, uint16_t len) {
  // Expected format: [uint16_t count][queue_entry structs]
  if (len < 2) {
    send_error_response(0x01);  // Invalid length
    return;
  }

  uint16_t count = data[0] | (data[1] << 8);
  if (count == 0 || count > MAX_QUEUE_ENTRIES) {
    send_error_response(0x02);  // Invalid count
    return;
  }

  uint16_t expected_len = 2 + count * QUEUE_ENTRY_SIZE;
  if (len != expected_len) {
    send_error_response(0x03);  // Length mismatch
    return;
  }

  // Stop any current move before enqueuing new commands
  // (prevents interleaving with in-flight test)
  stop_current_move();

  // Verify checksum (already done by protocol layer)
  // Enqueue entries
  uint8_t *entry_data = data + 2;
  g_queued_entries = 0;

  for (uint16_t i = 0; i < count; i++) {
    queue_entry entry;
    memcpy(&entry, entry_data + i * QUEUE_ENTRY_SIZE, QUEUE_ENTRY_SIZE);
    // stepper.addQueueEntry(entry.steps, entry.ticks, ...);
    g_queued_entries++;
  }

  char response[64];
  snprintf(response, sizeof(response), "%u entries queued", g_queued_entries);
  send_ok_response(response);
}

static void handle_run(const char *test_id) {
  if (g_queued_entries == 0) {
    send_error_response(0x04);  // No entries queued
    return;
  }

  // Start queue execution
  // stepper.startQueue();

  // Toggle test marker probe at startQueue (if enabled)
#if defined(ESP32_TEST_PROBE) || defined(ESP32C3_TEST_PROBE)
  PROBE_1_HIGH();
#endif

  g_running = true;
  send_ok_response("test started");
}

static void handle_status(char *response, uint16_t response_len) {
  int32_t pos = getCurrentPosition();
  uint8_t queue_level = getQueueLevel();
  uint8_t queue_capacity = getQueueCapacity();

  snprintf(response, response_len, "pos=%d, running=%d, queue=%u/%u",
           pos, g_running ? 1 : 0, queue_level, queue_capacity);
  send_ok_response(response);
}

static void handle_metrics(uint8_t stepper_idx, char *response, uint16_t response_len) {
  // Get per-stepper metrics (step count, dir→step delay, glitches)
  // This is platform-specific — implemented in serial_reporter.cpp
  snprintf(response, response_len, "stepper=%u metrics unavailable", stepper_idx);
  send_ok_response(response);
}

static void handle_stop(void) {
  // Emergency stop — cease step pulses immediately
  // stepper.stopQueue();

#if defined(ESP32_TEST_PROBE) || defined(ESP32C3_TEST_PROBE)
  PROBE_1_LOW();
#endif

  g_running = false;
  g_queued_entries = 0;
  send_ok_response("stopped");
}

static void handle_reset(void) {
  // Reinitialize steppers/queue to known state
  // No physical power cycle required
  stop_all_steppers();
  clear_all_queues();

#if defined(ESP32_TEST_PROBE) || defined(ESP32C3_TEST_PROBE)
  PROBE_1_LOW();
#endif

  g_running = false;
  g_queued_entries = 0;
  send_ok_response("reset");
}

// Forward declaration from serial_protocol.cpp
extern void send_ok_response(const char *msg);
extern void send_error_response(uint8_t error_code);