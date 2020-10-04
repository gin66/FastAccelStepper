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
bool run_cos = false;
uint16_t count = 0;
uint32_t dt_const;
uint8_t steps = 3;
const uint8_t cos_tab[32] = {0,   2,   9,   21,  37,  56,  79,  103,
                             127, 152, 176, 199, 218, 234, 246, 253,
                             255, 253, 246, 234, 218, 199, 176, 152,
                             128, 103, 79,  56,  37,  21,  9,   2};

void loop() {
  while (stepper1->addQueueEntry(dt, steps, true) == AQE_OK) {
    if (!run_cos) {
      dt -= dt / 100;
      if (dt < TICKS_PER_S / 30000) {  // steps/s
        run_cos = true;
        dt_const = dt;
      }
    } else {
      dt = dt_const;
      dt += cos_tab[count & 31] >> 3;
      steps = 3 + (0x03 & (count >> 12));
      count++;
    }
  }
}
