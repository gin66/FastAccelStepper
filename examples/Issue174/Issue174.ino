#include "FastAccelStepper.h"

// This is only needed to exit the simavr simulator
#include <Arduino.h>
#ifdef SIMULATOR
#include <avr/sleep.h>
#endif

// fitting to my hardware
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
FastAccelStepper *stepper = NULL;

// the API of FastAccelStepper does not use float
// not sure, if the compiler properly change float to uint32_t.
// so better to due to calculation in integer domain
uint32_t steps_per_rev = 800;
uint32_t rpm = 2000;
uint32_t maxStepperSpeed = (rpm*steps_per_rev/60);   //needs to be in us per step || 1 sec = 1000000 us
uint32_t maxStepperAccel = 100;

void setup()
{
  Serial.begin(250000);

// delay is not needed
//  delay(1000);

  //FastAccelStepper setup
  engine.init();
#if defined(__AVR_ATmega328P__)
  stepper = engine.stepperConnectToPin(stepPinStepperAVR);
#elif defined(ARDUINO_ARCH_ESP32)
  stepper = engine.stepperConnectToPin(stepPinStepperESP);
#endif
  if (stepper) {
    Serial.println("Setup stepper!");
    
#if defined(__AVR_ATmega328P__)
  stepper->setDirectionPin(dirPinStepperAVR);
#elif defined(ARDUINO_ARCH_ESP32)
  stepper->setDirectionPin(dirPinStepperESP);
#endif
    stepper->setSpeedInHz(maxStepperSpeed);   // steps/s
    stepper->setAcceleration(maxStepperAccel);  // 100 steps/s²
  }
  // obsolete, but does not hurt
  stepper->forceStopAndNewPosition(0);
  stepper->moveTo(0);
}

long currentTime = 0;
long elapsedTime = 0;
long previousTime = 0;

long Position_Next = 0;

long targetCycleTime = 130;

void exitSimulator(bool ok) {
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

void loop()
{ 
  // obtain time
  currentTime = micros();
  elapsedTime = currentTime - previousTime;
  if (elapsedTime<1){elapsedTime=1;}

  // the average cycle time on my device is approx. 130 us --> mimic that
  long waitTime = targetCycleTime - elapsedTime;
  if (waitTime > 0){delayMicroseconds(waitTime); }

  previousTime = currentTime;

  // compute target position
  Position_Next += 500; // increment position
  Position_Next %= 10000; // reset position

  // the targetPos shows, what actual targetPos the stepper intends to run to
//  Serial.print(Position_Next);
//  Serial.print(':');
//  Serial.print(stepper->targetPos());
//  Serial.print(':');
//  Serial.println(stepper->getCurrentPosition());

  // add target position
  stepper->moveTo(Position_Next, false);

  // give the stepper some time to move. Enabling this, the code works
  // delay(1500);

  if ((currentTime > 50000000) && (Position_Next == 0)) {
    // let the stepper run to position 0
    while (stepper->isRunning()) {
    }
    // test expectation is, that the sum of all step pulses taken into 
    // consideration the direction pin yields a position of 0.
    // This works with every delay value above - as it should.
    Serial.println(stepper->getCurrentPosition());
    exitSimulator(true);
  }
}
