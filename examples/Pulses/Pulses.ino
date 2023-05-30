#include "FastAccelStepper.h"

//
// This is for TEST PURPOSES
//

// As in StepperDemo for Motor 1 on AVR
//#define dirPinStepper    5
//#define enablePinStepper 6
//#define stepPinStepper   9  // OC1A in case of AVR

// As in StepperDemo for Motor 7 on ESP32
#define dirPinStepper 19
#define enablePinStepper 26
#define stepPinStepper 14

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(115200);

  if (false) {
	  pinMode(stepPinStepper, OUTPUT);
	  digitalWrite(stepPinStepper, HIGH);
	  delay(1);
	  digitalWrite(stepPinStepper, LOW);
	  delay(1);
	  digitalWrite(stepPinStepper, HIGH);
	  delay(1);
	  digitalWrite(stepPinStepper, LOW);
	  delay(1);

	  pinMode(18, OUTPUT);
	  digitalWrite(18, HIGH);
	  delay(1);
	  digitalWrite(18, LOW);
	  delay(1);
	  digitalWrite(18, HIGH);
	  delay(1);
	  digitalWrite(18, LOW);
	  delay(1);
	}

  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper, DRIVER_RMT);
}

void loop1() {
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
void loop() {
  uint16_t min_ticks = 0;
  if (stepper) {
    min_ticks = stepper->getMaxSpeedInTicks();
  }
min_ticks = 60000;
  const struct stepper_command_s cmd_step1 = {
      .ticks = MIN_CMD_TICKS, .steps = 2, .count_up = true};
  const struct stepper_command_s cmd_step = {
      .ticks = min_ticks, .steps = 10, .count_up = true};
  const struct stepper_command_s cmd_pause = {
      .ticks = 5000, .steps = 0, .count_up = true};
  uint8_t res;
  if (stepper) {
//  stepper->addQueueEntry(&cmd_step1);
    res = stepper->addQueueEntry(&cmd_step);
//  stepper->addQueueEntry(&cmd_step1);
//  stepper->addQueueEntry(&cmd_step1);
//  stepper->addQueueEntry(&cmd_step);
//  stepper->addQueueEntry(&cmd_pause);
    Serial.println(res);
  }
  delay(100);
}
