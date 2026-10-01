/*
 * saleae_app.cpp — ESP-IDF entry point for the Saleae test app.
 *
 * The test logic is shared in common/saleae_test.cpp; this file only maps it
 * onto the ESP-IDF app_main() contract.
 */

#include "saleae_test.h"

extern "C" void app_main(void) {
  saleae_test_setup();
  for (;;) {
    saleae_test_loop();
  }
}
