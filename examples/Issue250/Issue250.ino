#include "FastAccelStepper.h"

#ifdef SIMULATOR
#include <util/delay.h>
#include <avr/sleep.h>
#endif

// As in StepperDemo for Motor 1 on AVR
#define dirPinStepper 5
#define enablePinStepper 6
#define stepPinStepper 9  // OC1A in case of AVR

// As in StepperDemo for Motor 1 on ESP32
// #define dirPinStepper 18
// #define enablePinStepper 26
// #define stepPinStepper 19

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

// fails with blocking 30us
#ifndef BLOCK_INTERRUPT_US
#define BLOCK_INTERRUPT_US 20
#endif
#ifndef TOGGLE_DIRECTION
#define TOGGLE_DIRECTION true
#endif
#ifndef USE_MOVETO
#define USE_MOVETO true
#endif
#ifndef LOOPS
#define LOOPS 200
#endif

uint32_t previous_runtime = 0;

void loop() {
  loopcnt++;
  Serial.print("Loop=");
  Serial.println(loopcnt);
  uint32_t runtime = (rand() % 50) + 50;
  if (((loopcnt & 1) == 1) || !TOGGLE_DIRECTION) {
    if (USE_MOVETO) {
      stepper->moveTo(10000);
    } else {
      stepper->runForward();
    }
  } else {
    if (USE_MOVETO) {
      stepper->moveTo(-10000);
    } else {
      stepper->runBackward();
    }
  }
  uint32_t start_ms = millis();
  while (millis() < 2 * previous_runtime + runtime + start_ms) {
    noInterrupts();
#ifdef SIMULATOR
    _delay_us(BLOCK_INTERRUPT_US);
#endif
    interrupts();
  }
  previous_runtime = runtime;

  if (loopcnt == LOOPS) {
    stepper->stopMove();
    while (stepper->isRunning()) {
    }
#ifdef SIMULATOR
    Serial.print("Reached Position=");
    Serial.println(stepper->getCurrentPosition());
    stepper->moveTo(0, true);
    Serial.print("Position=");
    Serial.println(stepper->getCurrentPosition());
    delay(100);
    noInterrupts();
    sleep_cpu();
#endif
  }
}
