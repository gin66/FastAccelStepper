#include "FastAccelStepper.h"

#ifdef SIMULATOR
#include <avr/sleep.h>
#endif

// As in StepperDemo for Motor 1 on AVR
#if defined(ARDUINO_ARCH_AVR)
#define dirPinStepper 5
#define enablePinStepper 6
#define stepPinStepper 9  // OC1A in case of AVR
#elif defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
#define dirPinStepper 18
#define enablePinStepper 26
#define stepPinStepper 17
#elif defined(ARDUINO_ARCH_STM32)
#define dirPinStepper PB0
#define enablePinStepper PA4
#define stepPinStepper PA0
#else
#define dirPinStepper 5
#define enablePinStepper 6
#define stepPinStepper 9
#endif

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(115200);
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

    stepper->setSpeedInHz(40000);
    stepper->setAcceleration(100000);
  }
}

void loop() {
  // Run the motor
  stepper->runForward();

  // Wait till the motor is coasting
  while ((stepper->rampState() & RAMP_STATE_MASK) != RAMP_STATE_COAST) {
  }

  // Now we stop the motor
  stepper->stopMove();

  // Let the stepper task process the stopMove()
  delay(20);

  bool err = false;
  if (!stepper->isStopping()) {
    Serial.println("Stepper is not stopping");
    err = true;
  }

  // Then update speed
  stepper->setAcceleration(90000);
  stepper->applySpeedAcceleration();

  // Now let's wait
  Serial.println(stepper->getCurrentSpeedInUs());
  uint32_t start = millis();
  while (millis() - start < 2000) {
    if (!stepper->isRunning()) {
      break;
    }
    delay(10);
  }
  Serial.println(stepper->getCurrentSpeedInUs());

  // If still running, then this is an error
  if (stepper->isRunning()) {
    Serial.println("Stepper did not stop");
    err = true;
  }

  if (err) {
    Serial.println("Test failed");
  }
#ifdef SIMULATOR
  Serial.print("Reached Position=");
  Serial.println(stepper->getCurrentPosition());
  if (!err) {
    // Test has passed, so run to position 0, so that the test environment
    // detects pass
    stepper->moveTo(0, true);
    Serial.print("Position=");
    Serial.println(stepper->getCurrentPosition());
  }
  delay(100);
  noInterrupts();
  sleep_cpu();
#endif
}