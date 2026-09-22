#include "FastAccelStepper.h"

// Move, sit idle for a few seconds, move again - repeating. Unlike
// TeensyBringUp.ino (which starts the next move the instant the previous
// one finishes, so the enable pin never gets a chance to go back to
// "disabled"), this sketch has a real idle gap between moves, so you can
// actually watch the enable pin (and, if wired, an LED on it) switch:
//   HIGH (disabled) while idle  <->  LOW (enabled) while moving
// (default polarity - LOW enables the motor. See setEnablePin() docs if
// your driver needs the opposite.)

#define stepPinStepper 2
#define dirPinStepper 3
#define enablePinStepper 4

#define IDLE_MS 3000  // how long to sit still between moves

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  Serial.println("START - move, pause, move");

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
  stepper->setAutoEnable(true);
  // Disable (almost) as soon as the motor stops, instead of waiting for
  // the default delay, so the HIGH/disabled state during the idle gap
  // below is easy to see.
  stepper->setDelayToDisable(50);

  stepper->setSpeedInHz(2000);
  stepper->setAcceleration(4000);
}

void printEnableState(const char *label) {
  Serial.print(label);
  Serial.print(" enablePin=");
  Serial.println(digitalRead(enablePinStepper));
}

void loop() {
  int32_t pos = stepper->getCurrentPosition();
  int32_t target = (pos <= 0) ? 3200 : -3200;

  Serial.print("Moving to ");
  Serial.print(target);
  Serial.println(" ...");
  MoveResultCode res = stepper->moveTo(target, true);  // built-in blocking mode
  Serial.print("moveTo() returned: ");
  Serial.println(toString(res));
  Serial.print("Reached ");
  Serial.println(stepper->getCurrentPosition());

  Serial.print("Idling for ");
  Serial.print(IDLE_MS);
  Serial.println(" ms - watch the enable pin/LED now.");
  uint32_t idleStart = millis();
  while (millis() - idleStart < IDLE_MS) {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      printEnableState("[idle]");
    }
  }
}
