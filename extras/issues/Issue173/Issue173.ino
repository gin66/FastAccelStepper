#include "FastAccelStepper.h"
#include <Arduino.h>
#ifdef SIMULATOR
#include <avr/sleep.h>
#endif

#if (defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || \
     defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__))
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
  bool ok = true;
  Serial.begin(115200);
  engine.init();

  // pins are set to outputs here automatically
#if (defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || \
     defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__))
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

  stepper->move(10000);
  while ((stepper->rampState() & RAMP_STATE_MASK) != RAMP_STATE_COAST) {
    // wait for coasting
  }
  int32_t pos = stepper->getCurrentPosition();
  Serial.print("coast at ");
  Serial.println(pos);

  // stepper is coasting, so stop the stepper
  stepper->forceStop();
  while (stepper->isRunning()) {
    // wait for stepper to stop
  }
  pos = stepper->getCurrentPosition();
  Serial.print("position after force stop: ");
  Serial.print(pos);
  Serial.print(" with target pos: ");
  Serial.println(stepper->targetPos());

  stepper->setCurrentPosition(0);
  uint32_t target_pos = stepper->targetPos();
  Serial.print("position after setCurrent position: ");
  Serial.print(stepper->getCurrentPosition());
  Serial.print(" with target pos: ");
  Serial.println(target_pos);

  if (target_pos != 0) {
    ok = false;
  }

  stepper->move(-1);
  stepper->move(1 - pos);

  while (stepper->isRunning()) {
    // wait for stepper to stop
  }

  // the simulator should show position 0

  if (ok) {
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
  if (ok) {
    digitalWrite(PIN, HIGH);
    digitalWrite(PIN, LOW);
  }
  delay(1000);
  noInterrupts();
  sleep_cpu();
#endif
}

void loop() {}
