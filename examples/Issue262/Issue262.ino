#include <FastAccelStepper.h>

#define stepPin 9
#define dirPin 10
#define USB_BAUD 250000

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

const int32_t quickAcc = 100000 * 4;
const int32_t acc = 2000;
const int32_t minSpeed = 1;
const int32_t maxSpeed = 2000;

void setup() {
  engine.init();
  stepper = engine.stepperConnectToPin(stepPin);
  stepper->setDirectionPin(dirPin);
  Serial.begin(USB_BAUD);
  randomSeed(42);
  delay(100);
  Serial.println("hello");
}

void loop() {
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
  count = 0;
  while (stepper -> getCurrentSpeedInMilliHz() != 0) {
    count ++;
    if (count > 20) {
      Serial.print(millis());
      Serial.print("ms: No stop iter# ");
      Serial.print(count);
      Serial.print(".");
      Serial.print(" Target: ");
      Serial.print(target);
      Serial.print(".");
      if (stepper -> isRunning())
        Serial.print("isRunning() reports true.");
      else
        Serial.print("isRunning() reports false.");
      Serial.println();
  }
    delay(5);
    if (count > 200)
      break;
  }
}
