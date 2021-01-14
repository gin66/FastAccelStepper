#include "FastAccelStepper.h"

// As in StepperDemo for Motor 1 on AVR
//#define dirPinStepper    5
//#define enablePinStepper 6
//#define stepPinStepper   9  // OC1A in case of AVR

// As in StepperDemo for Motor 1 on ESP32
#define dirPinStepper 19
#define enablePinStepper 12
#define stepPinStepper 2

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(115200);
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
}

void loop() {
  const struct stepper_command_s cmd_step1 = {
      .ticks = 6000, .steps = 1, .count_up = true};
  const struct stepper_command_s cmd_step2 = {
      .ticks = 3000, .steps = 2, .count_up = true};
  const struct stepper_command_s cmd_step3 = {
      .ticks = 2000, .steps = 3, .count_up = true};
  const struct stepper_command_s cmd_pause = {
      .ticks = 6000, .steps = 0, .count_up = true};

  stepper->addQueueEntry(&cmd_step3);
  stepper->addQueueEntry(&cmd_step1);
  stepper->addQueueEntry(&cmd_step2);
  stepper->addQueueEntry(&cmd_pause);
  stepper->addQueueEntry(&cmd_step3);
  while (stepper->isRunning()) {
  }
  delayMicroseconds(150);

  stepper->addQueueEntry(&cmd_step3);
  stepper->addQueueEntry(&cmd_step1);
  stepper->addQueueEntry(&cmd_step2);
  stepper->addQueueEntry(&cmd_pause);
  stepper->addQueueEntry(&cmd_step3);
  stepper->addQueueEntry(&cmd_pause);
  delay(300);
}
