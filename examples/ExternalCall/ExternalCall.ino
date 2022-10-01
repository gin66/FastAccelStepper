#include "FastAccelStepper.h"

#ifdef SIMULATOR
#include <avr/sleep.h>
#endif
#if defined(ARDUINO_ARCH_ESP32)
#include <esp_task_wdt.h>
#endif

// As in StepperDemo for Motor 1 on AVR
#define dirPinStepper    5
#define enablePinStepper 6
#define stepPinStepper   9  // OC1A in case of AVR

// As in StepperDemo for Motor 1 on ESP32
//#define dirPinStepper 18
//#define enablePinStepper 26
//#define stepPinStepper 17

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(115200);
  Serial.println("External call test");

  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    // stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);

    stepper->setSpeedInUs(1000);  // the parameter is us/step !!!
    stepper->setAcceleration(100);
  }
}

void loop() {
  stepper->move(1000, true);
  stepper->move(-1000, true);

#ifdef SIMULATOR
  delay(1000);
  noInterrupts();
  sleep_cpu();
#endif
}
