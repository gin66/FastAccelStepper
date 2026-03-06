#pragma once

#include <stdint.h>
#include <string.h>

struct SystemConfig {
  char wifi_ssid[32];
  char wifi_password[64];
  char hostname[32];
  bool ap_mode;
  uint16_t web_server_port;
  uint16_t websocket_port;
  uint32_t status_update_interval_ms;

  void setDefaults() {
    wifi_ssid[0] = '\0';
    wifi_password[0] = '\0';
    strncpy(hostname, "esp32-stepper", sizeof(hostname) - 1);
    ap_mode = false;
    web_server_port = 80;
    websocket_port = 81;
    status_update_interval_ms = 100;
  }
};
