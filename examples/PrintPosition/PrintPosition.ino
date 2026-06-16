#include "FastAccelStepper.h"

// As in StepperDemo for Motor 1 on AVR
#if defined(ARDUINO_ARCH_AVR)
#define stepPinStepper 9
#define enablePinStepper 6
#define dirPinStepper 5
#elif defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
#define stepPinStepper 16
#define enablePinStepper 25
#define dirPinStepper 17
#elif defined(ARDUINO_ARCH_STM32)
#define stepPinStepper PA0
#define enablePinStepper PA4
#define dirPinStepper PB0
#else
#define stepPinStepper 16
#define enablePinStepper 25
#define dirPinStepper 17
#endif

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(115200);

  Serial.println("Starting");
  Serial.flush();
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  Serial.println("Starting");
  Serial.print("Stepper Pin:");
  Serial.println(stepPinStepper);
  Serial.flush();
  Serial.println((unsigned int)stepper);
  Serial.println((unsigned int)&engine);
  if (stepper) {
#if defined(ARDUINO_ARCH_STM32)
    stepper->setDirectionPin(dirPinStepper, true, 1000);
#else
    stepper->setDirectionPin(dirPinStepper);
#endif
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);

    stepper->setSpeedInUs(5);  // the parameter is us/step !!!
    stepper->setAcceleration(1000);
  delay(5000);
    // stepper->move(7000);
    stepper->runBackward();
  } else {
    Serial.println("Stepper Not initialized!");
    delay(1000);
  }
  Serial.print("    F_CPU=");
  Serial.println(F_CPU);
  Serial.print("    TICKS_PER_S=");
  Serial.println(TICKS_PER_S);
  Serial.flush();
}

void loop() {
  delay(100);
  if (stepper) {
    if (stepper->isRunning()) {
      Serial.print("@");
      Serial.println(stepper->getCurrentPosition());
    }
  } else {
    Serial.println("Stepper died?");
    Serial.flush();
    delay(10000);
  }
}