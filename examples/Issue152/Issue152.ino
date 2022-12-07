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
  stepper->setAcceleration(1000000);
}

void loop() {
  // test to check single stepper high speed operation
  int32_t target = 20000;
  stepper->setSpeedInHz(70000);
  stepper->moveTo(target);
  while(stepper->isRunning()) {
  };
#ifdef SIMULATOR
    // if result is Ok. Toggle port twice, otherwise once
#define PIN 10
#define DIRPIN 7
    pinMode(DIRPIN, OUTPUT);
    digitalWrite(DIRPIN, HIGH);
    pinMode(PIN, OUTPUT);
    digitalWrite(PIN, HIGH);
    digitalWrite(PIN, LOW);
    Serial.println(stepper->getCurrentPosition());
    if (stepper->getCurrentPosition() == target) {
      digitalWrite(PIN, HIGH);
      digitalWrite(PIN, LOW);
    }

    delay(1000);
    noInterrupts();
    sleep_cpu();
#endif
}
