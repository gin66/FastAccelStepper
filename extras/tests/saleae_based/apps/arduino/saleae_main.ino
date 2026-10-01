/*
 * saleae_main.ino — Arduino entry point for the Saleae test app.
 *
 * The logic lives in common/ (saleae_app, saleae_test, saleae_hal_*); this file
 * only maps it onto the Arduino setup()/loop() contract. The same entry point
 * works for any Arduino core (ESP32, RP2040, ...).
 */

#include "saleae_app.h"

void setup() { saleae_app_setup(); }

void loop() { saleae_app_loop(); }
