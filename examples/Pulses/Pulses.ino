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
  }
  if (false) {
    pinMode(18, OUTPUT);
    digitalWrite(18, HIGH);
    delay(1000);
    digitalWrite(18, LOW);
    delay(1000);
    digitalWrite(18, HIGH);
    delay(1000);
    digitalWrite(18, LOW);
    delay(1000);
  }

  engine.init();
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  stepper = engine.stepperConnectToPin(stepPinStepper, DRIVER_RMT);
#else
  stepper = engine.stepperConnectToPin(stepPinStepper);
#endif
#if defined(SUPPORT_ESP32_PULSE_COUNTER)
  stepper->attachToPulseCounter(QUEUES_MCPWM_PCNT, 0, 0);
  stepper->clearPulseCounter();
#endif
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
  min_ticks = 30000;
  const struct stepper_command_s cmd_step2 = {
      .ticks = MIN_CMD_TICKS, .steps = 2, .count_up = true};
  const struct stepper_command_s cmd_step10 = {
      .ticks = min_ticks, .steps = 10, .count_up = true};
  //  const struct stepper_command_s cmd_step5 = {
  //      .ticks = 45000, .steps = 5, .count_up = true};
  //  const struct stepper_command_s cmd_pause = {
  //      .ticks = 5000, .steps = 0, .count_up = true};
  uint8_t res[20];
  uint8_t *r = res;
  if (stepper) {
#if defined(SUPPORT_ESP32_PULSE_COUNTER)
    stepper->clearPulseCounter();
#endif
    *r++ = stepper->addQueueEntry(&cmd_step2, false);
    *r++ = stepper->addQueueEntry(&cmd_step10, false);
    //  stepper->addQueueEntry(&cmd_step2);
    //  stepper->addQueueEntry(&cmd_step2);
    //*r++ = stepper->addQueueEntry(&cmd_step5, false);
    //*r++ = stepper->addQueueEntry(&cmd_step10, false);
    //*r++ = stepper->addQueueEntry(&cmd_step5, false);
    //*r++ = stepper->addQueueEntry(&cmd_step10, false);
    //*r++ = stepper->addQueueEntry(&cmd_step5, false);
    //*r++ = stepper->addQueueEntry(&cmd_step10, false);
    //*r++ = stepper->addQueueEntry(&cmd_pause);
    *r++ = stepper->addQueueEntry(NULL);
    Serial.print(res[0]);
    Serial.print('-');
    Serial.print(res[1]);
    Serial.print('-');
    Serial.println(res[2]);
  }
  while (stepper->isRunning()) {
  }
#if defined(SUPPORT_ESP32_PULSE_COUNTER)
  int16_t pc = stepper->readPulseCounter();
  digitalWrite(dirPinStepper, pc == 12 ? HIGH : LOW);
  Serial.print(pc);
  Serial.println(pc == 12 ? " OK" : " FAIL");
  digitalWrite(dirPinStepper, LOW);
  delay(100);
  digitalWrite(dirPinStepper, pc == 12 ? HIGH : LOW);
  Serial.print(pc);
  Serial.println(pc == 12 ? " OK" : " FAIL");
  delay(100);
  digitalWrite(dirPinStepper, LOW);
#endif
  delay(100);
}
