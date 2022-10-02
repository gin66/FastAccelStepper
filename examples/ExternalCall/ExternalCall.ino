#include "FastAccelStepper.h"

#ifdef SIMULATOR
#include <avr/sleep.h>
#endif
#if defined(ARDUINO_ARCH_ESP32)
#include <esp_task_wdt.h>
#endif

// As in StepperDemo for Motor 1 on AVR
#define dirPinStepper1 5
#define enablePinStepper1 6
#define stepPinStepper1 9  // OC1A in case of AVR

// As in StepperDemo for Motor 2 on AVR
#define dirPinStepper2 7
#define enablePinStepper2 8
#define stepPinStepper2 10  // OC1B in case of AVR

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

void setup() {
  Serial.begin(115200);
  Serial.println("External call test");
  Serial.println("Stepper 1 uses external dir pin");
  Serial.println("Stepper 2 uses direct dir pin");

  engine.init();
  engine.setExternalCallForPin(setExternalPin);
  stepper1 = engine.stepperConnectToPin(stepPinStepper1);
  if (stepper1) {
    stepper1->setDirectionPin(dirPinStepper1 | PIN_EXTERNAL_FLAG);
    stepper1->setEnablePin(enablePinStepper1);
    stepper1->setAutoEnable(true);

    stepper1->setSpeedInUs(10000);  // the parameter is us/step !!!
    stepper1->setAcceleration(1000);
  }

  stepper2 = engine.stepperConnectToPin(stepPinStepper2);
  if (stepper2) {
    stepper2->setDirectionPin(dirPinStepper2);
    stepper2->setEnablePin(enablePinStepper2);
    stepper2->setAutoEnable(true);

    stepper2->setSpeedInUs(10000);  // the parameter is us/step !!!
    stepper2->setAcceleration(1000);
  }
}

bool setExternalPin(uint8_t pin, uint8_t value) {
  // This example returns the previous value of the output.
  // Consequently, FastAccelStepper needs to call setExternalPin twice
  // in order to successfully change the output value.
  pin = pin & ~PIN_EXTERNAL_FLAG;
  pinMode(pin, OUTPUT);
  bool oldValue = digitalRead(pin);
  digitalWrite(pin, value);
  return oldValue;
}

void loop() {
  // Blocking moves are used.
  if (stepper1) {
    stepper1->move(1000, true);
    stepper1->move(-900, true);
    stepper1->move(500, true);
    stepper1->move(-450, true);
    stepper1->move(40, true);
  }
  if (stepper2) {
    stepper2->move(1000, true);
    stepper2->move(-900, true);
    stepper2->move(500, true);
    stepper2->move(-450, true);
    stepper2->move(40, true);
  }

#ifdef SIMULATOR
  delay(1000);
  noInterrupts();
  sleep_cpu();
#endif
}
