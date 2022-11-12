#include "FastAccelStepper.h"


#if defined(__AVR_ATmega328P__)
#include "AVRStepperPins.h"
#define dirPinStepperAVR 5
#define stepPinStepperAVR stepPinStepper1A
#define enablePinStepperAVR 6
#elif defined(ARDUINO_ARCH_ESP32)
#define dirPinStepperESP 18
#define stepPinStepperESP 17
#define enablePinStepperESP 26
#endif


FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper;

void setup() {
    Serial.begin(115200);
    engine.init();

    // pins are set to outputs here automatically
#if defined(__AVR_ATmega328P__)
    stepper = engine.stepperConnectToPin(stepPinStepperAVR);
    stepper->setDirectionPin(dirPinStepperAVR);
	stepper->setEnablePin(enablePinStepperAVR, true);
#elif defined(ARDUINO_ARCH_ESP32)
    stepper = engine.stepperConnectToPin(stepPinStepperESP);
    stepper->setDirectionPin(dirPinStepperESP);
	stepper->setEnablePin(enablePinStepperESP, true);
#endif

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
	Serial.print("Loop");
	Serial.println(loopCnt);
    Serial.print("target: ");
    Serial.println(targetSpeed);
    Serial.print("actual: ");
    Serial.println(actual);
	if ((actual == lastTarget) && (targetSpeed != lastTarget)) {
		fail++;
	}
	if (fail == 0) {
		if (loopCnt > 100) {
			Serial.println("PASS");
		}
	}
	else {
		Serial.print("FAILS = ");
		Serial.println(fail);
	}
	lastTarget = targetSpeed;
}
