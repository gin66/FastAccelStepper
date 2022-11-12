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

uint16_t fail = 0;
uint16_t loopCnt = 0;
int lastTarget;

void loop() {
    loopCnt++;
    int targetSpeed = rand() % 40000 + 100;
    stepper->setSpeedInUs(targetSpeed);  // the parameter is us/step !!!
    stepper->runForward();
    delay(500);
    int actual = stepper->getCurrentSpeedInUs();
	Serial.printf("Loop: %d\n", loopCnt);
    Serial.printf("target: %d\n", targetSpeed);
    Serial.printf("actual: %d\n", actual);
	if ((actual == lastTarget) && (targetSpeed != lastTarget)) {
		fail++;
	}
	if (fail == 0) {
		if (loopCnt > 100) {
			Serial.printf("PASS\n");
		}
	}
	else {
		Serial.printf("FAILS = %d\n", fail);
	}
	lastTarget = targetSpeed;
}
