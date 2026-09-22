#include "FastAccelStepper.h"

// Minimal, decisive test of FastAccelStepper's enable-pin control,
// completely decoupled from auto-enable timing and from any driver board -
// wire an LED directly to enablePinStepper (through a resistor to GND) to
// watch it.
//
// This calls the library's own public enableOutputs()/disableOutputs()
// directly, alternating every 2 seconds, printing what it did and what
// digitalRead() sees right after. If the LED doesn't change in sync with
// these prints, the problem is confirmed to be in the library's pin
// control - if it does, the library is fine and something else (timing,
// wiring, or how another sketch used it) was the real cause.

#define stepPinStepper 2
#define dirPinStepper 3
#define enablePinStepper 4

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  Serial.println("START - enable pin blink test");

  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (!stepper) {
    while (true) {
      Serial.println("NO STEPPER - check stepPinStepper is a valid pin");
      delay(1000);
    }
  }
  stepper->setDirectionPin(dirPinStepper);
  stepper->setEnablePin(enablePinStepper);
  // NOT calling setAutoEnable() - we're driving enable manually here so
  // there's no ambiguity about when it fires.

  Serial.print("Right after setEnablePin(), digitalRead(pin)=");
  Serial.println(digitalRead(enablePinStepper));
  Serial.println(
      "Expect HIGH=1 here (disabled is the safe default) - the LED should "
      "be ON now if wired pin->resistor->LED->GND.");
}

bool enabled = false;
uint32_t lastToggleMs = 0;

void loop() {
  uint32_t now = millis();
  if (now - lastToggleMs >= 2000) {
    lastToggleMs = now;
    enabled = !enabled;
    bool ok;
    if (enabled) {
      ok = stepper->enableOutputs();
      Serial.print("enableOutputs() -> ");
    } else {
      ok = stepper->disableOutputs();
      Serial.print("disableOutputs() -> ");
    }
    Serial.print(ok ? "true" : "false");
    Serial.print("   digitalRead(pin) now = ");
    Serial.print(digitalRead(enablePinStepper));
    Serial.println(enabled ? "   (expect LOW=0, LED should turn OFF)"
                           : "   (expect HIGH=1, LED should turn ON)");
  }
}
