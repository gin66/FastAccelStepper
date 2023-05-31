#include "FastAccelStepper.h"
#include <Arduino.h>
#ifdef SIMULATOR
#include <avr/sleep.h>
#endif

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
  stepper->setAcceleration(100000);
  stepper->setSpeedInUs(100);  // the parameter is us/step !!!

  // time to reach speed of 10.000 Hz with 100.000 steps/s^2 is 0.1s.
  // 0.1s is reached in 500 steps

  stepper->move(10000);
  while ((stepper->rampState() & RAMP_STATE_MASK) != RAMP_STATE_COAST) {
    // wait for coasting
  }
  int32_t pos = stepper->getCurrentPosition();
  Serial.print("coast at ");
  Serial.println(pos);

  // stepper is coasting, so stop the stepper
  stepper->stopMove();

  // but we only wait until the deceleration starts
  while ((stepper->rampState() & RAMP_STATE_MASK) == RAMP_STATE_COAST) {
    // wait for deceleration to start
  }

  // get the position at the end of the queue
  pos = stepper->getPositionAfterCommandsCompleted();
  uint32_t target_pos = stepper->targetPos();

  // As per issue 172, the stepper will run towards 10000, but expectation is ~0
  stepper->move(-1);
  stepper->move(1 - target_pos);

  while ((stepper->rampState() & RAMP_STATE_MASK) != RAMP_STATE_ACCELERATE) {
    // wait for acceleration to start
  }
  uint32_t max_pos = stepper->getCurrentPosition();
  while (stepper->isRunning()) {
    // wait for stepper to stop
  }

  // lets print the position after the move(), so that the ramp generator can
  // not add more commands to the queue after
  // getPositionAfterCommandsCompleted()
  Serial.print("target position after stopMove() processed ");
  Serial.println(target_pos);
  Serial.print("stop at and move back ");
  Serial.println(pos);
  Serial.print("max position ");
  Serial.println(max_pos);

  // let's check the position. if 0, then all ok
  pos = stepper->getCurrentPosition();
  Serial.print("stop at ");
  Serial.println(pos);
  if (pos == 0) {
    Serial.println("PASS");
  } else {
    Serial.print("FAILS = ");
    Serial.println(pos);
  }
#ifdef SIMULATOR
  // if result is Ok. Toggle port twice, otherwise once
#define PIN 10
#define DIRPIN 7
  pinMode(DIRPIN, OUTPUT);
  digitalWrite(DIRPIN, HIGH);
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN, HIGH);
  digitalWrite(PIN, LOW);
  if (pos == 0) {
    digitalWrite(PIN, HIGH);
    digitalWrite(PIN, LOW);
  }
  delay(1000);
  noInterrupts();
  sleep_cpu();
#endif
}

void loop() {}
