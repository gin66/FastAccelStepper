// Minimal synchronizedStart() check for the ATmega2560 (Timer1).
//
// Both steppers are queued without starting, then released by one engine
// call.  StepA (OC1A, pin 11) and StepB (OC1B, pin 12) must emit their first
// pulse together; judge_sync.awk compares the two VCD traces.

#include <avr/sleep.h>

#include "AVRStepperPins.h"
#include "FastAccelStepper.h"

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* stepperA = nullptr;
FastAccelStepper* stepperB = nullptr;

#define STEPS 200
#define TICKS_PER_STEP 500  // 31.25 us/step at 16 MHz

void setup() {
  engine.init();
  stepperA = engine.stepperConnectToPin(stepPinStepperA);
  stepperB = engine.stepperConnectToPin(stepPinStepperB);
  if (stepperA == nullptr || stepperB == nullptr) {
    noInterrupts();
    sleep_cpu();
  }

  stepperA->moveTimed(STEPS, (uint32_t)STEPS * TICKS_PER_STEP, nullptr, false);
  stepperB->moveTimed(STEPS, (uint32_t)STEPS * TICKS_PER_STEP, nullptr, false);

  FastAccelStepper* steppers[] = {stepperA, stepperB};
  engine.synchronizedStart(steppers, 2);

  delay(20);
  noInterrupts();
  sleep_cpu();
}

void loop() {}
