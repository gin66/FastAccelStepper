#include "AVRStepperPins.h"
#include "FastAccelStepper.h"

// As in StepperDemo for Motor 1 on AVR
#define dirPinStepperX    5
#define enablePinStepperX 6
#define stepPinStepperX   stepPinStepper1A

// As in StepperDemo for Motor 2 on AVR
#define dirPinStepperY    7
#define enablePinStepperY 8
#define stepPinStepperY   stepPinStepper1B

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperX = NULL;
FastAccelStepper *stepperY = NULL;

void setup() {
  engine.init();
  stepperX = engine.stepperConnectToPin(stepPinStepperX);
  stepperY = engine.stepperConnectToPin(stepPinStepperY);
  if (!stepperX || !stepperY) {
    while(0==0) {
       Serial.println("Cannot initialize steppers");
    }
  }
  stepperX->setDirectionPin(dirPinStepperX);
  stepperX->setEnablePin(enablePinStepperY);
  stepperX->setAutoEnable(false);
  stepperY->setDirectionPin(dirPinStepperX);
  stepperY->setEnablePin(enablePinStepperY);
  stepperY->setAutoEnable(false);
}

void loop() {}
