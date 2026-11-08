// Probe the RP2350 stepper allocation count.
//
// An RP2350 has three PIO blocks with four state machines each. The Arduino
// core's cycle counter occupies one state machine (two instruction words) on
// one block. The stepper program is 27 words and relocatable, so it fits
// beside the counter and that block still serves three steppers:
// 4 + 4 + 3 = 11.
//
// The step pins must be in 16..31 so all three PIO windows cover them:
// PIO0/PIO1 cover 0..31, PIO2's window starts at 16. Below 16 the first two
// blocks take every pin and PIO2 is never reached, which reports 8.

#include <Arduino.h>

#include "FastAccelStepper.h"

#define FIRST_STEP_PIN 16
#define LAST_STEP_PIN 31

FastAccelStepperEngine engine = FastAccelStepperEngine();

static uint8_t connected = 0;
static int16_t first_failed = -1;

static void probe() {
  engine.init();
  connected = 0;
  first_failed = -1;
  for (uint8_t pin = FIRST_STEP_PIN; pin <= LAST_STEP_PIN; pin++) {
    FastAccelStepper* stepper = engine.stepperConnectToPin(pin);
    if (stepper == NULL) {
      if (first_failed < 0) {
        first_failed = (int16_t)pin;
      }
    } else {
      connected++;
    }
  }
}

void setup() {
  Serial.begin(115200);
  probe();
}

void loop() {
  Serial.print("PROBE pins=");
  Serial.print(FIRST_STEP_PIN);
  Serial.print('-');
  Serial.print(LAST_STEP_PIN);
  Serial.print(" connected=");
  Serial.print(connected);
  Serial.print(" first_failed=");
  Serial.println(first_failed);
  delay(1000);
}
