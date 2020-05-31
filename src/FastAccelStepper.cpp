#include "FastAccelStepper.h"

#define stepPinStepper1   9  /* OC1A */
#define stepPinStepper2   10 /* OC1B */

FastAccelStepperEngine::FastAccelStepperEngine() {
}

FastAccelStepper *FastAccelStepperEngine::stepperA(uint8_t dirPin) {
   _stepperA.setDirectionPin(dirPin);
   pinMode(stepPinStepper1, OUTPUT);
   return &_stepperA;
}
FastAccelStepper *FastAccelStepperEngine::stepperB(uint8_t dirPin) {
   _stepperB.setDirectionPin(dirPin);
   pinMode(stepPinStepper2, OUTPUT);
   return &_stepperB;
}

FastAccelStepper::FastAccelStepper(bool channelA) {
   _channelA = channelA;
}
void FastAccelStepper::setDirectionPin(uint8_t dirPin) {
   _dirPin = dirPin;
   pinMode(dirPin, OUTPUT);
}


