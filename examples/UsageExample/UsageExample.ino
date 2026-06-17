#include "FastAccelStepper.h"

// As in StepperDemo for Motor 1 on AVR
// #define dirPinStepper    5
// #define enablePinStepper 6
// #define stepPinStepper   9  // OC1A in case of AVR

// As in StepperDemo for Motor 1 on ESP32
#if defined(ARDUINO_ARCH_AVR)
#define dirPinStepper 5
#define enablePinStepper 6
#define stepPinStepper 9
#elif defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
#define dirPinStepper 18
#define enablePinStepper 26
#define stepPinStepper 17
#elif defined(PICO_RP2040) || defined(PICO_RP2350)
#define dirPinStepper 15
#define enablePinStepper 13
#define stepPinStepper 14
#elif defined(ARDUINO_ARCH_STM32)
#define dirPinStepper PB0
#define enablePinStepper PA4
#define stepPinStepper PA0
#else
#define dirPinStepper 18
#define enablePinStepper 26
#define stepPinStepper 17
#endif

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
#if defined(ARDUINO_ARCH_STM32)
    stepper->setDirectionPin(dirPinStepper, true, 1000);
#else
    stepper->setDirectionPin(dirPinStepper);
#endif
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    // stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);

    stepper->setSpeedInUs(1000);  // the parameter is us/step !!!
    stepper->setAcceleration(100);
    stepper->move(1000);
  }
}

void loop() {}
