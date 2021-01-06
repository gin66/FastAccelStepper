#include "FastAccelStepper.h"

// Exclusively for test purposes !

// for avr: either use pin 9 or 10 aka OC1A or OC1B
#define stepPinStepper 17
#define enablePinStepper 26
#define dirPinStepper 18

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

  Serial.println("Start");
  uint32_t start = micros();
  uint32_t cnt = 0;
  for (uint32_t delay = 0; delay < 20000; delay+=100) {
	// just issue a step with 1ms pause
    struct stepper_command_s cmd = {
        .ticks = 16000, .steps = 1, .count_up = true};
    int rc = stepper->addQueueEntry(&cmd);
    if (rc != AQE_OK) {
      Serial.print("Queue error:");
      Serial.println(rc);
    }
    else {
		cnt++;
		while (stepper->isRunning()) {
		}
    }
    delayMicroseconds(delay);
    start += delay;
  }
  Serial.print("Done. Stepper commands executed within us=");
  Serial.print((micros()-start)/cnt);
  Serial.println(". Expected ~1000");
}
}

void loop() {
}
