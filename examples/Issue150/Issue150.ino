#include "FastAccelStepper.h"

// As in StepperDemo for Motor 1 on ESP32
#define dirPinStepper 18
#define stepPinStepper 17
#define enablePinStepper 26

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper;

void setup() {
    Serial.begin(115200);
    engine.init();

    // pins are set to outputs here automatically
    stepper = engine.stepperConnectToPin(stepPinStepper);
    stepper->setDirectionPin(dirPinStepper);
	stepper->setEnablePin(enablePinStepper, true);
	stepper->setAutoEnable(true);
    stepper->setAcceleration(1000000);
}

void loop() {
    int targetSpeed = rand() % 40000 + 100;
    stepper->setSpeedInUs(targetSpeed);  // the parameter is us/step !!!
    stepper->runForward();
    delay(500);
    Serial.printf("target: %d\n", targetSpeed);
    Serial.printf("actual: %d\n", stepper->getCurrentSpeedInUs());
}
