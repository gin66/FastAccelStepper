/*
 * saleae_main.cpp — ESP-IDF entry point for the Saleae test app.
 *
 * The logic lives in common/ (saleae_app, saleae_test, saleae_hal_*); this file
 * only maps it onto the ESP-IDF app_main() contract.
 */

#include "saleae_app.h"

extern "C" void app_main(void) {
  saleae_app_setup();
  for (;;) {
    saleae_app_loop();
  }
}
