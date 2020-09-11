#include "FastAccelStepper.h"

#define dirPinStepper1 5
#define enablePinStepper1 6
#define stepPinStepper1 9 /* OC1A */

#define dirPinStepper2 7
#define enablePinStepper2 8
#define stepPinStepper2 10 /* OC1B */

#define LED 13

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = engine.stepperA();
FastAccelStepper *stepper2 = engine.stepperB();

void setup() {
  Serial.begin(115200);

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
  // Done

  engine.init();
  engine.setDebugLed(LED);

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

void loop() {
  bool cmd_ok = false;
  bool queue_ok = false;

  if (Serial.available()) {
    char ch = Serial.read();
    in_buffer[in_ptr++] = toupper(ch);
    in_buffer[in_ptr] = 0;
    if (in_ptr == 255) {
      in_ptr = 0;
    } else if (ch == '\n') {
      int32_t val;
      if (strcmp(in_buffer, "M1\n") == 0) {
        Serial.println("Select stepper 1");
        selected = stepper1;
      } else if (strcmp(in_buffer, "M2\n") == 0) {
        Serial.println("Select stepper 2");
        selected = stepper2;
      } else if (sscanf(in_buffer, "A %ld\n", &val) == 1) {
        Serial.print("Set acceleration to ");
        Serial.println(val);
        selected->setAcceleration(val);
      } else if (sscanf(in_buffer, "V %ld\n", &val) == 1) {
        Serial.print("Set speed (us) to ");
        Serial.println(val);
        selected->setSpeed(val);
      } else if (sscanf(in_buffer, "R %ld\n", &val) == 1) {
        Serial.print("Move steps ");
        Serial.println(val);
        selected->move(val);
      } else if (sscanf(in_buffer, "T %ld\n", &val) == 1) {
        Serial.print("Move to position ");
        Serial.println(val);
        selected->moveTo(val);
      } else if (strcmp(in_buffer, "S\n") == 0) {
        Serial.print("Stop");
        selected->stopMove();
      }
      in_ptr = 0;
    }
  }

  if (queue_ok) {
    long motor = 0;
    FastAccelStepper *stepper = motor == 1 ? stepper1 : stepper2;
    // NOT NEEDED IN RAW ACCESS: stepper->setSpeed(16384);
    // NOT NEEDED IN RAW ACCESS: stepper->setAcceleration(100.0);
    Serial.println(stepper->addQueueEntry(5L * 16384, 120, true));
    Serial.println(stepper->addQueueEntry(4L * 16384, 120, true));
    Serial.println(stepper->addQueueEntry(3L * 16384, 120, true));
    Serial.println(stepper->addQueueEntry(2L * 16384, 120, true));
    Serial.println(stepper->addQueueEntry(6L * 4096, 120, true));
    Serial.println(stepper->addQueueEntry(5L * 4096, 120, true));
    Serial.println(stepper->addQueueEntry(4L * 4096, 120, true));
    Serial.println(stepper->addQueueEntry(3L * 4096, 120, true));
    Serial.println(stepper->addQueueEntry(2L * 4096, 120, true));
    Serial.println(stepper->addQueueEntry(2L * 4096, 120, true));
    Serial.println(stepper->addQueueEntry(2L * 4096, 120, true));
    Serial.println(stepper->addQueueEntry(2L * 4096, 120, true));
    Serial.println(stepper->addQueueEntry(2L * 4096, 120, true));
    Serial.println(stepper->addQueueEntry(2L * 4096, 120, true));
    Serial.println(stepper->addQueueEntry(2L * 4096, 120, true));
  }

  if (!stopped) {
    Serial.print("Stepper 1: ");
    Serial.print(stepper1->isrSpeedControlEnabled() ? " AUTO " : " MANU ");
    Serial.print(stepper1->getCurrentPosition());
    if (stepper1->isRunning()) {
      Serial.print("  RUNNING");
    } else {
      Serial.print("  PAUSED ");
    }
    Serial.print("  state=");
    Serial.print(stepper1->rampState());
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
    Serial.print("  max/us=");
    Serial.print(stepper1->max_micros);
#endif
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
    Serial.print("  checksum=");
    Serial.print(stepper1->checksum);
#endif

    Serial.print("  Stepper 2: ");
    Serial.print(stepper2->isrSpeedControlEnabled() ? " AUTO " : " MANU ");
    Serial.print(stepper2->getCurrentPosition());
    if (stepper2->isRunning()) {
      Serial.print("  RUNNING");
    } else {
      Serial.print("  PAUSED ");
    }
    Serial.print("  state=");
    Serial.print(stepper2->rampState());
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
    Serial.print("  max/us=");
    Serial.print(stepper2->max_micros);
#endif
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
    Serial.print("  checksum=");
    Serial.print(stepper2->checksum);
#endif

    Serial.print("  TCCR1A=");
    Serial.print(TCCR1A);
    Serial.print("  TCCR1B=");
    Serial.print(TCCR1B);
    Serial.print("  TIMSK1=");
    Serial.print(TIMSK1);
    Serial.println("");
    stopped = !(stepper1->isRunning() || stepper2->isRunning());
    if (stopped) {
      Serial.println("Enter command:");
      Serial.println("     M1/M2         ... to select stepper");
      Serial.println(
          "     A <accel>     ... Set selected stepper's acceleration");
      Serial.println("     V <speed>     ... Set selected stepper's speed");
      Serial.println(
          "     T <position>  ... Move selected stepper to position (absolute "
          "+/-)");
      Serial.println(
          "     R <steps>     ... Move selected stepper to steps (+/-)");
      Serial.println(
          "     S             ... Stop selected stepper with deceleration");
    }
  } else {
    stopped = !(stepper1->isRunning() || stepper2->isRunning());
  }
}
