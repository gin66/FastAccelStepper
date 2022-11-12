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
    stepper->setAcceleration(1000);
}

uint16_t loopCnt = 0;

void loop() {
    loopCnt++;
    int targetSpeed = rand() % 40000 + 100;
    stepper->setSpeedInUs(targetSpeed);  // the parameter is us/step !!!
    stepper->runForward();
    delay(500);
	Serial.printf("Loop: %d\n", loopCnt);
    Serial.printf("target: %d\n", targetSpeed);
    Serial.printf("actual: %d\n", stepper->getCurrentSpeedInUs());
    if (loopCnt == 10) {
		bool pass = true;
		stepper->setSpeedInUs(1000);  // the parameter is us/step !!!
		stepper->applySpeedAcceleration();
        delay(3000);
        if (1000 != stepper->getCurrentSpeedInUs()) {
			pass = false;
			Serial.printf("target: %d\n", targetSpeed);
			Serial.printf("actual: %d\n", stepper->getCurrentSpeedInUs());
		}
		stepper->setSpeedInUs(500);  // the parameter is us/step !!!
		stepper->applySpeedAcceleration();
        delay(3000);
        if (500 != stepper->getCurrentSpeedInUs()) {
			pass = false;
			Serial.printf("target: %d\n", targetSpeed);
			Serial.printf("actual: %d\n", stepper->getCurrentSpeedInUs());
		}
		stepper->stopMove();
		while(true) {
			if (pass) {
				Serial.printf("PASS");
			}
			else {
				Serial.printf("FAIL");
			}
			delay(1000);
		}
    }
}
