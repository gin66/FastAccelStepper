/*
 * saleae_app.ino — Arduino entry point for the Saleae test app.
 *
 * The test logic is shared in common/saleae_test.cpp; this file only maps it
 * onto the Arduino setup()/loop() contract. The same entry point works for
 * any Arduino core (ESP32, RP2040/Pico, ...).
 */

#include "saleae_test.h"

void setup() { saleae_test_setup(); }

void loop() { saleae_test_loop(); }
