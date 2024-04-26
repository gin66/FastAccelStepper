#include "FastAccelStepper.h"

#ifdef SIMULATOR
#include <util/delay.h>
#include <avr/sleep.h>
#endif

// As in StepperDemo for Motor 1 on AVR
#define dirPinStepper    5
#define enablePinStepper 6
#define stepPinStepper   9  // OC1A in case of AVR

// As in StepperDemo for Motor 1 on ESP32
//#define dirPinStepper 18
//#define enablePinStepper 26
//#define stepPinStepper 19

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(115200);
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    stepper->setDelayToEnable(50);
    stepper->setDelayToDisable(50);

    stepper->setSpeedInHz(40000);
    stepper->setAcceleration(100000);

  }
}

uint16_t loopcnt = 0;

void loop() {
  loopcnt++;
  Serial.print("Loop=");
  Serial.println(loopcnt);
  uint32_t start_ms;
  uint32_t delayForward = (rand() % 50) + 50;
  uint32_t delayBackward = (rand() % 50) + 50;
  stepper->runForward();
  start_ms = millis();
  while (millis() < delayForward + start_ms) {
     noInterrupts();
     _delay_us(25);
     interrupts();
  }
  stepper->runBackward();
  start_ms = millis();
  while (millis() < delayBackward + start_ms) {
     noInterrupts();
     _delay_us(25);
     interrupts();
  }
  if (loopcnt == 200) {
#ifdef SIMULATOR
     stepper->moveTo(0, true);
     noInterrupts();
     sleep_cpu();
#endif
  }
}
