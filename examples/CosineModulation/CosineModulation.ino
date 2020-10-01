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

void setup() {
  Serial.begin(115200);
  Serial.println("Cosine modulation at high speed");
  Serial.println("1. ramp up to ~30000 Steps/s");
  Serial.println("2. For time period T vary the speed slightly");
  Serial.println("3. Run at constant speed for 3*T, then back to 2");
  Serial.println("");
  Serial.println(
      "Depending on your motor and current setting, the stepper may stop "
      "spinning at high speed");

  engine.init();
  engine.setDebugLed(LED);

  stepper1 = engine.stepperConnectToPin(stepPinStepper1);
  stepper1->setDirectionPin(dirPinStepper1);
  stepper1->setEnablePin(enablePinStepper1);
  stepper1->setAutoEnable(true);
}

uint32_t dt = 100000;  // start with 10steps/s
bool run_saw = false;
uint16_t count = 0;
uint32_t dt_const;
const uint8_t cos_tab[64] = {0,   2,   9,   21,  37,  56,  79,  103,
                             127, 152, 176, 199, 218, 234, 246, 253,
                             255, 253, 246, 234, 218, 199, 176, 152,
                             128, 103, 79,  56,  37,  21,  9,   2};

void loop() {
  while (stepper1->addQueueEntry(dt, 3, true) == AQE_OK) {
    if (!run_saw) {
      dt -= dt / 100;
      if (dt < 16000000 / 30000) {  // 30000 steps/s
        run_saw = true;
        dt_const = dt;
      }
    } else {
      dt = dt_const;
      if ((count & 0xc000) == 0) {
        dt += cos_tab[count & 63] >> 3;
      }
      count++;
    }
  }
}
