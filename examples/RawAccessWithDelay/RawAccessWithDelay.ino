#include "FastAccelStepper.h"

#define enablePinStepper 12
#define dirPinStepper 27

// for avr: either use pin 9 or 10 aka OC1A or OC1B
#define stepPinStepper 26

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper;

void setup() {
  Serial.begin(115200);
  Serial.println("Demo RawAccessWithDelay");

  engine.init();

  stepper = engine.stepperConnectToPin(stepPinStepper);

  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(false);
    stepper->enableOutputs();
  }
}

// This loop should use the new delay only command to generate steps every 2 s

void loop() {
  if (!stepper) {
    return;
  }
  Serial.println("Start");
  struct stepper_command_s cmd_step = {
      .ticks = 64000, .steps = 1, .count_up = true};  // delay 4ms
  struct stepper_command_s cmd_delay = {
      .ticks = 64000, .steps = 0, .count_up = true};  // delay 4ms
  for (uint16_t i = 0; i < 499; i++) {
    while (true) {
      int rc = stepper->addQueueEntry(&cmd_delay);
      // Serial.println(rc);
      if (rc == AQE_OK) {
        Serial.print(".");
        break;
      }
      // adding a delay(1) causes problems
      delayMicroseconds(1000);
    }
  }
  while (true) {
    int rc = stepper->addQueueEntry(&cmd_step);
    // Serial.println(rc);
    if (rc == AQE_OK) {
      Serial.println("STEP");
      break;
    }
    // adding a delay(1) causes problems
    delayMicroseconds(1000);
  }
}
