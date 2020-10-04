#include "FastAccelStepper.h"

#define enablePinStepper 21
#define dirPinStepper 22

// for avr: either use pin 9 or 10 aka OC1A or OC1B
#define stepPinStepper 23

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper;

void setup() {
  Serial.begin(115200);
  Serial.println("Demo RawAccess");

  engine.init();

  stepper = engine.stepperConnectToPin(stepPinStepper);

  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);
  }
}

// This loop drives the stepper up to 80000 microsteps/s.
// For a NEMA-17 with 200 steps/revolution and 16 microsteps, this means 25
// revolutions/s

bool direction = false;

#define MAX_SPEED 80000 /* steps/s */

void loop() {
  if (!stepper) {
    return;
  }
#define COMMAND_CNT (MAX_SPEED / 100)
  Serial.println("Start");
  for (uint16_t i = 1; i < 2 * COMMAND_CNT; i++) {
    uint8_t steps = 100;
    uint32_t steps_per_s = min(i, 2 * COMMAND_CNT - i) * 100;
    uint32_t ticks = TICKS_PER_S / steps_per_s;
    while (true) {
      int rc = stepper->addQueueEntry(ticks, steps, direction);
      // Serial.println(rc);
      if (rc == AQE_OK) {
        break;
      }
      // adding a delay(1) causes problems
      delayMicroseconds(1000);
    }
  }

  Serial.println("no more commands to be created");
  while (!stepper->isQueueEmpty()) {
  }
  Serial.println("queue is empty");

  while (stepper->isRunning()) {
  }

  Serial.println("stepper has stopped");

  delay(10000);
  direction = !direction;
}
