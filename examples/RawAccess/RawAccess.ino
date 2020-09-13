#include "FastAccelStepper.h"

#define dirPinStepper1 5
#define enablePinStepper1 6
#define stepPinStepper1 9 /* OC1A */

#define dirPinStepper2 7
#define enablePinStepper2 8
#define stepPinStepper2 10 /* OC1B */

#define LED 13

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

void setup() {
  Serial.begin(115200);
  Serial.println("Demo Raw Access");

  // Check stepper motor+driver is operational
  // This is not done via FastAccelStepper-Library for test purpose only
  pinMode(stepPinStepper1, OUTPUT);
  pinMode(dirPinStepper1, OUTPUT);
  pinMode(enablePinStepper1, OUTPUT);
  digitalWrite(enablePinStepper1, LOW);
  digitalWrite(dirPinStepper1, LOW);
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(stepPinStepper1, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPinStepper1, LOW);
    delayMicroseconds(190);
  }
  digitalWrite(dirPinStepper1, HIGH);
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(stepPinStepper1, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPinStepper1, LOW);
    delayMicroseconds(190);
  }
  digitalWrite(enablePinStepper1, HIGH);

  pinMode(stepPinStepper2, OUTPUT);
  pinMode(dirPinStepper2, OUTPUT);
  pinMode(enablePinStepper2, OUTPUT);
  digitalWrite(enablePinStepper2, LOW);
  digitalWrite(dirPinStepper2, LOW);
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(stepPinStepper2, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPinStepper2, LOW);
    delayMicroseconds(190);
  }
  digitalWrite(dirPinStepper2, HIGH);
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(stepPinStepper2, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPinStepper2, LOW);
    delayMicroseconds(190);
  }
  digitalWrite(enablePinStepper2, HIGH);
  // Done

  engine.init();
  engine.setDebugLed(LED);
  stepper1 = engine.stepperConnectToPin(stepPinStepper1);
  stepper2 = engine.stepperConnectToPin(stepPinStepper2);

  stepper1->setDirectionPin(dirPinStepper1);
  stepper2->setDirectionPin(dirPinStepper2);

  stepper1->setEnablePin(enablePinStepper1);
  stepper2->setEnablePin(enablePinStepper2);

  stepper1->setAutoEnable(true);
  stepper2->setAutoEnable(true);
}

uint32_t dt = ABSOLUTE_MAX_TICKS;
bool up = true;

void loop() {
  uint8_t steps = min(max(100000L / dt, 1), 127);
  if (stepper1->addQueueEntry(dt, steps, true) == AQE_OK) {
    if (up) {
      dt -= dt / 100;
      if (dt < TICKS_PER_S / 40000) {  // 40000 steps/s
        up = false;
      }
    } else {
      dt += 1;
      if (dt == ABSOLUTE_MAX_TICKS) {
        up = true;
      }
    }
  }
}
