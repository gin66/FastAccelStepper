#include "FastAccelStepper.h"

#define dirPinStepper1 5
#define enablePinStepper1 6
#define stepPinStepper1 9 /* OC1A */

#define dirPinStepper2 7
#define enablePinStepper2 8
#define stepPinStepper2 10 /* OC1B */

#define LED 13

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = engine.stepperA();
FastAccelStepper *stepper2 = engine.stepperB();

void setup() {
  Serial.begin(115200);

  engine.init();
  engine.setDebugLed(LED);

  stepper1->setDirectionPin(dirPinStepper1);
  stepper2->setDirectionPin(dirPinStepper2);

  stepper1->setEnablePin(enablePinStepper1);
  stepper2->setEnablePin(enablePinStepper2);

  stepper1->set_auto_enable(true);
  stepper2->set_auto_enable(true);
}

uint32_t dt = ABSOLUTE_MAX_TICKS;
bool up = true;

void loop() {
  if (stepper2->add_queue_entry(dt, 2, true, 0) == AQE_OK) {
     if (up) {
        dt -= dt / 100;
        if (dt < 16000000/40000) {
	        up = false;
        }
     }
     else {
        dt += 1;
        if (dt == ABSOLUTE_MAX_TICKS) {
	        up = true;
        }
     }
  }
}
