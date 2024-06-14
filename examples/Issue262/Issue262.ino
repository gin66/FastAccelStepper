#include <FastAccelStepper.h>

#define stepPin 9
#define dirPin 5
#define USB_BAUD 250000

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

const int32_t quickAcc = 100000 * 4;
const int32_t acc = 2000;
const int32_t minSpeed = 1;
const int32_t maxSpeed = 2000;

void memoryMix() {
  delay(30);
}

void setup() {
  engine.init();
  stepper = engine.stepperConnectToPin(stepPin);
  stepper->setDirectionPin(dirPin);
  Serial.begin(USB_BAUD);
  randomSeed(42);

}

void loop() {
  uint32_t sp, oldSp;
  int i, count;
  long target;
 // Serial.println("Starting");
  target = random(minSpeed, maxSpeed);
  stepper->setAcceleration(acc);
  stepper->setSpeedInHz(target);
  stepper->runForward();
  while (stepper-> getCurrentSpeedInMilliHz()/1000 < target - 100)
    {};
 // Serial.println("Stopping");
  stepper ->moveByAcceleration(-quickAcc, false);
  stepper ->applySpeedAcceleration();
  oldSp = stepper -> getCurrentSpeedInMilliHz();
  count = 0;
  while ((sp = stepper -> getCurrentSpeedInMilliHz()) != 0) {
    count ++;
    if (count > 20) {
      Serial.print("No stop iter ");
      Serial.print(count);
      Serial.print(".");
      if (!stepper -> isRunning())
        Serial.print("isRunning() reports stopped");
      Serial.println();
  }
    if (sp > oldSp) {
      Serial.print("Speed moves wrong way ");
      Serial.print(sp);
      Serial.print(">");
      Serial.print(oldSp);
      Serial.println();
    }
    oldSp = sp;
    memoryMix();
  }
  for (i =0; i< 100; i++) {
    memoryMix();
    sp = stepper-> getCurrentSpeedInMilliHz();
    if (sp !=0) {
      Serial.print("Speed error. Received value: ");
      Serial.print(sp);
      Serial.print(". Target speed was ");
      Serial.print(target);
      Serial.println();
    }
  }
}
