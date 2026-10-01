/*
 * esp32_adapter.cpp — ESP32-specific initialization for Saleae test firmware.
 *
 * Supports ESP32, ESP32-S3, ESP32-C3, ESP32-C6, ESP32-H2, ESP32-P4.
 * Uses FastAccelStepper library with platform-specific driver selection.
 */

#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include <esp_idf_version.h>

// Platform-specific configuration
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
  #define USE_RMT_V2 1
  #define DRIVER_TYPE "rmt_v2"
#else
  #define USE_RMT_V2 0
  #define DRIVER_TYPE "mcpwm_pcnt"
#endif

// Default channel configuration (overridable via CONFIG command)
typedef enum {
  CH_CONFIG_4CH_RMT = 0,
  CH_CONFIG_8CH_STEP_ONLY,
  CH_CONFIG_7CH_SHARED_DIR,
  CH_CONFIG_4CH_MCPWM,
  CH_CONFIG_2RMT_2I2S,
  CH_CONFIG_MIXED
} channel_config_t;

static channel_config_t g_channel_config = CH_CONFIG_4CH_RMT;

// Default pin mapping (4ch_rmt config)
static const uint8_t default_pins[] = {
  2,   // Stepper A Step  (GPIO2)
  0,   // Stepper A Dir   (GPIO0) — must be HIGH at boot
  4,   // Stepper B Step  (GPIO4)
  16,  // Stepper B Dir   (GPIO16)
  17,  // Stepper C Step  (GPIO17)
  5,   // Stepper C Dir   (GPIO5)
  18,  // Stepper D Step  (GPIO18)
  19   // Stepper D Dir   (GPIO19)
};

// Test marker pins (for Saleae trigger)
#define TEST_MARKER_PIN 25  // GPIO25 — toggled at startQueue
#define QUEUE_EMPTY_PIN 26  // GPIO26 — toggled when queue empties

void init_stepper_platform(void) {
  // Configure test marker pins as outputs
  pinMode(TEST_MARKER_PIN, OUTPUT);
  pinMode(QUEUE_EMPTY_PIN, OUTPUT);
  digitalWrite(TEST_MARKER_PIN, LOW);
  digitalWrite(QUEUE_EMPTY_PIN, LOW);

  // Initialize steppers based on channel configuration
  // (platform-specific stepper initialization)
  // Example for 4ch_rmt:
  //   stepperConnectToPin(2);  // Stepper A Step
  //   stepperConnectToPin(0);  // Stepper A Dir
  //   ...

  (void)g_channel_config;  // Suppress unused warning
}

void stop_current_move(void) {
  // Platform-specific: stop current queue execution
  // (stepper.stopQueue() or equivalent)
}

void stop_all_steppers(void) {
  // Platform-specific: stop all steppers
}

void clear_all_queues(void) {
  // Platform-specific: clear all command queues
}

int32_t getCurrentPosition(void) {
  // Platform-specific: return current stepper position
  return 0;
}

uint8_t getQueueLevel(void) {
  // Platform-specific: return current queue fill level
  return 0;
}

uint8_t getQueueCapacity(void) {
  // Platform-specific: return queue capacity
  return 32;  // Typical ESP32 queue capacity
}

#endif  // ARDUINO_ARCH_ESP32