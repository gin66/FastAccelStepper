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
uint8_t in_val_ptr = 0;
long in_vals[8];
bool stopped = false;

void loop() {
  bool cmd_ok = false;
  bool queue_ok = false;

  if (Serial.available()) {
    char ch = Serial.read();
    if ((ch == '\n') || (ch == ' ')) {
      if (in_ptr > 0) {
        in_buffer[in_ptr] = 0;
        in_ptr = 0;
        if (in_val_ptr < 8) {
          in_vals[in_val_ptr++] = atol(in_buffer);
        }
      }
    } else {
      in_buffer[in_ptr++] = ch;
    }
    if (ch == '\n') {
      if (in_val_ptr == 4) {
        cmd_ok = true;
      }
      if (in_val_ptr == 1) {
        queue_ok = true;
      }
      if (in_val_ptr == 0) {
        stopped = false;
      }
      in_val_ptr = 0;
      in_ptr = 0;
    }
  }

  if (queue_ok) {
    long motor = in_vals[0];
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

  if (cmd_ok) {
    long motor = in_vals[0];
    long move = in_vals[1];
    long ticks = in_vals[2];
    long accel = in_vals[3];
    if (((motor == 1) || (motor == 2)) && move) {
      Serial.print("ticks=");
      Serial.print(ticks);
      Serial.print("  accel=");
      Serial.print(accel);
      Serial.print("  move=");
      Serial.print(move);
      if (motor == 1) {
        stopped = false;
        stepper1->setSpeed(ticks);
        stepper1->setAcceleration(accel);
        stepper1->move(move);
        Serial.print("  Start stepper 1: ");
        Serial.println(stepper1->getCurrentPosition());
      }
      if (motor == 2) {
        stopped = false;
        stepper2->setSpeed(ticks);
        stepper2->setAcceleration(accel);
        stepper2->move(move);
        Serial.print("  Start stepper 2: ");
        Serial.println(stepper2->getCurrentPosition());
      }
    }
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
    Serial.print(stepper1->ramp_state);
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
    Serial.print(stepper2->ramp_state);
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
      Serial.println(
          "Please enter one line with <motor> <steps> <speed> <acceleration> "
          "e.g.");
      Serial.println("1 10000 1000 100");
    }
  } else {
    stopped = !(stepper1->isRunning() || stepper2->isRunning());
  }
}
