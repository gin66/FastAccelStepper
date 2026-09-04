#include "FastAccelStepper.h"
#include <Arduino.h>
#ifdef SIMULATOR
#include <avr/sleep.h>
#endif

#if defined(ARDUINO_ARCH_AVR)
#include "AVRStepperPins.h"
#define dirPinStepperAVR 5
#define stepPinStepperAVR stepPinStepperA
#define enablePinStepperAVR 6
#elif defined(ARDUINO_ARCH_ESP32)
#define dirPinStepperESP 18
#define stepPinStepperESP 17
#define enablePinStepperESP 26
#endif

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper;

bool test() {
  bool verbose = false;
  int32_t errCnt = 0;
  Serial.println("Start test: speed up");
  int32_t last_v_mHz = 0;
  stepper->setAcceleration(10000);
  stepper->setSpeedInHz(36800);
  stepper->runForward();

  while (stepper->rampState() != (RAMP_STATE_COAST | RAMP_DIRECTION_COUNT_UP)) {
    int32_t v_mHz = stepper->getCurrentSpeedInMilliHz(false);
    if (v_mHz == last_v_mHz) {
      continue;
    }
    if (verbose) Serial.println(v_mHz);
    if (v_mHz < last_v_mHz) {
      Serial.print("FAIL: last=");
      Serial.print(last_v_mHz);
      Serial.print("  new=");
      Serial.println(v_mHz);
      errCnt++;
      //       return false;
    }
    last_v_mHz = v_mHz;
  }
  // There can be still speed increases in the queue, even so the ramp generator
  // is already coasting
  delay(20);
  last_v_mHz = stepper->getCurrentSpeedInMilliHz(false);

  Serial.println("Reverse");

  stepper->runBackward();

  while (stepper->rampState() !=
         (RAMP_STATE_COAST | RAMP_DIRECTION_COUNT_DOWN)) {
    int32_t v_mHz = stepper->getCurrentSpeedInMilliHz(false);
    if (v_mHz == last_v_mHz) {
      continue;
    }
    if (verbose) Serial.println(v_mHz);
    if (v_mHz > last_v_mHz) {
      Serial.print("FAIL: last=");
      Serial.print(last_v_mHz);
      Serial.print("  new=");
      Serial.println(v_mHz);
      errCnt++;
      //       return false;
    }
    last_v_mHz = v_mHz;
  }
  if (errCnt > 0) {
    Serial.print("Errors=");
    Serial.println(errCnt);
  }
  return errCnt == 0;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Init");
  engine.init();

  // pins are set to outputs here automatically
#if defined(ARDUINO_ARCH_AVR)
  Serial.println("AVR");
  Serial.println(stepPinStepperAVR);
  stepper = engine.stepperConnectToPin(stepPinStepperAVR);
  stepper->setDirectionPin(dirPinStepperAVR);
//  stepper->setEnablePin(enablePinStepperAVR, true);
#elif defined(ARDUINO_ARCH_ESP32)
  stepper = engine.stepperConnectToPin(stepPinStepperESP);
  stepper->setDirectionPin(dirPinStepperESP);
  stepper->setEnablePin(enablePinStepperESP, true);
#endif

  stepper->setAutoEnable(true);
  Serial.println(stepper != NULL ? "OK" : "ERROR");
  Serial.println("Done");

  bool pass = test();
  stepper->stopMove();

#ifndef SIMULATOR
  if (pass) {
    Serial.print("PASS");
  } else {
    Serial.print("FAIL");
  }
#endif

#ifdef SIMULATOR
  // if result is Ok. Toggle port twice, otherwise once
#define PIN stepPinStepperB
#define DIRPIN 7
  pinMode(DIRPIN, OUTPUT);
  digitalWrite(DIRPIN, HIGH);
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN, HIGH);
  digitalWrite(PIN, LOW);
  if (pass) {
    Serial.println("PASS");
    digitalWrite(PIN, HIGH);
    digitalWrite(PIN, LOW);
  } else {
    Serial.println("FAIL");
  }
  delay(1000);
  noInterrupts();
  sleep_cpu();
#endif
}

void loop() {}
