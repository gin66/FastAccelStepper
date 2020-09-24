#include "FastAccelStepper.h"

#define dirPinStepper1 5
#define enablePinStepper1 6
#define stepPinStepper1 9 /* OC1A */

#define dirPinStepper2 7
#define enablePinStepper2 8
#define stepPinStepper2 10 /* OC1B */

#define LED 13

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

void setup() {
  Serial.begin(115200);
  Serial.println("Demo DualStepper");

  // Check stepper motor+driver is operational
  // This is not done via FastAccelStepper-Library for test purpose only
  pinMode(stepPinStepper1, OUTPUT);
  pinMode(dirPinStepper1, OUTPUT);
  pinMode(enablePinStepper1, OUTPUT);
  digitalWrite(enablePinStepper1, LOW);
  digitalWrite(dirPinStepper1, LOW);
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(stepPinStepper1, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPinStepper1, LOW);
    delayMicroseconds(190);
  }
  digitalWrite(dirPinStepper1, HIGH);
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(stepPinStepper1, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPinStepper1, LOW);
    delayMicroseconds(190);
  }
  digitalWrite(enablePinStepper1, HIGH);

  pinMode(stepPinStepper2, OUTPUT);
  pinMode(dirPinStepper2, OUTPUT);
  pinMode(enablePinStepper2, OUTPUT);
  digitalWrite(enablePinStepper2, LOW);
  digitalWrite(dirPinStepper2, LOW);
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(stepPinStepper2, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPinStepper2, LOW);
    delayMicroseconds(190);
  }
  digitalWrite(dirPinStepper2, HIGH);
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(stepPinStepper2, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPinStepper2, LOW);
    delayMicroseconds(190);
  }
  digitalWrite(enablePinStepper2, HIGH);
  // Done

  engine.init();
  engine.setDebugLed(LED);
  stepper1 = engine.stepperConnectToPin(stepPinStepper1);
  stepper2 = engine.stepperConnectToPin(stepPinStepper2);

  stepper1->setDirectionPin(dirPinStepper1);
  stepper2->setDirectionPin(dirPinStepper2);

  stepper1->setEnablePin(enablePinStepper1);
  stepper2->setEnablePin(enablePinStepper2);

  stepper1->setAutoEnable(true);
  stepper2->setAutoEnable(true);
}

uint8_t in_ptr = 0;
char in_buffer[256];
bool stopped = false;
FastAccelStepper *selected = stepper1;

void info(FastAccelStepper *s) {
  Serial.print(s->isrSpeedControlEnabled() ? " AUTO " : " MANU ");
  Serial.print(" Curr=");
  Serial.print(s->getCurrentPosition());
  Serial.print(" QueueEnd=");
  Serial.print(s->getPositionAfterCommandsCompleted());
  Serial.print(" Target=");
  Serial.print(s->targetPos());
  if (s->isRunning()) {
    Serial.print("  RUNNING");
  } else {
    Serial.print("  PAUSED ");
  }
  Serial.print("  state=");
  Serial.print(s->rampState());
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
  Serial.print("  max/us=");
  Serial.print(s->max_micros);
#endif
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
  Serial.print("  checksum=");
  Serial.print(s->checksum);
#endif
}

void loop() {
  bool cmd_ok = false;

  if (Serial.available()) {
    char ch = Serial.read();
    if (in_ptr == 255) {
      in_ptr = 0;
    } else if ((ch == ' ') || (ch == '\n')) {
      int32_t val;
      if (strcmp(in_buffer, "M1") == 0) {
        Serial.println("Select stepper 1");
        selected = stepper1;
      } else if (strcmp(in_buffer, "M2") == 0) {
        Serial.println("Select stepper 2");
        selected = stepper2;
      } else if (sscanf(in_buffer, "A%ld", &val) == 1) {
        Serial.print("Set acceleration to ");
        Serial.println(val);
        selected->setAcceleration(val);
      } else if (sscanf(in_buffer, "V%ld", &val) == 1) {
        Serial.print("Set speed (us) to ");
        Serial.println(val);
        selected->setSpeed(val);
      } else if (sscanf(in_buffer, "R%ld", &val) == 1) {
        Serial.print("Move steps ");
        Serial.println(val);
        selected->move(val);
      } else if (sscanf(in_buffer, "P%ld", &val) == 1) {
        Serial.print("Move to position ");
        Serial.println(val);
        selected->moveTo(val);
      } else if (strcmp(in_buffer, "S") == 0) {
        Serial.print("Stop");
        selected->stopMove();
      }
      in_ptr = 0;
    } else {
      in_buffer[in_ptr++] = toupper(ch);
      in_buffer[in_ptr] = 0;
    }
  }

  if (!stopped) {
    Serial.print("Stepper 1: ");
    info(stepper1);

    Serial.print("  Stepper 2: ");
    info(stepper2);

    Serial.println();
    stopped = !(stepper1->isRunning() || stepper2->isRunning());
    if (stopped) {
      Serial.println("Enter command separated by space or newline:");
      Serial.println("     M1/M2         ... to select stepper");
      Serial.println(
          "     A<accel>     ... Set selected stepper's acceleration");
      Serial.println("     V <speed>     ... Set selected stepper's speed");
      Serial.println(
          "     P<position>  ... Move selected stepper to position (absolute "
          "+/-)");
      Serial.println(
          "     R<steps>     ... Move selected stepper to steps (+/-)");
      Serial.println(
          "     S             ... Stop selected stepper with deceleration");
    }
  } else {
    stopped = !(stepper1->isRunning() || stepper2->isRunning());
  }
}
