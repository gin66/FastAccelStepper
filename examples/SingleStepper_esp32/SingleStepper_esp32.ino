#include "FastAccelStepper.h"

#define dirPinStepper 22
#define enablePinStepper 21
#define stepPinStepper 23

#define LED_PIN 2

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1;
FastAccelStepper *stepper;

void setup() {
  Serial.begin(115200);
  Serial.println("Check Stepper");

  // Check stepper motor+driver is operational
  pinMode(stepPinStepper, OUTPUT);
  pinMode(dirPinStepper, OUTPUT);
  pinMode(enablePinStepper, OUTPUT);
  digitalWrite(enablePinStepper, LOW);
  digitalWrite(dirPinStepper, LOW);
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(stepPinStepper, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPinStepper, LOW);
    delayMicroseconds(190);
  }
  digitalWrite(dirPinStepper, HIGH);
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(stepPinStepper, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPinStepper, LOW);
    delayMicroseconds(190);
  }
  digitalWrite(enablePinStepper, HIGH);
  Serial.println("Init FastAccelStepper");
  engine.init();
  engine.setDebugLed(LED_PIN);

  stepper1 = engine.stepperConnectToPin(5);
  stepper = engine.stepperConnectToPin(stepPinStepper);

  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);
  }
}

uint8_t in_ptr = 0;
char in_buffer[256];
uint8_t in_val_ptr = 0;
long in_vals[8];
bool stopped = false;

void loop() {
  delay(1);

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
      if (in_val_ptr == 3) {
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
  if (stepper) {
    if (queue_ok) {
      // 3200 steps is one round with 16 microsteps and 200 steps for revolution

      // This loop drives the stepper up to 80000 microsteps/s.
      // with 16 microsteps, this means 25 revolutions/s
#define COMMAND_CNT 800
      for (uint16_t i = 1; i < 2 * COMMAND_CNT; i++) {
        uint8_t steps = 100;
        uint32_t steps_per_s = min(i, 2 * COMMAND_CNT - i) * 100;
        uint32_t ticks = 16000000 / steps_per_s;
        while (true) {
          int rc = stepper->addQueueEntry(ticks, steps, true);
          Serial.println(rc);
          if (rc == AQE_OK) {
            break;
          }
          // adding a delay(1) causes problems
          delayMicroseconds(1000);
        }
      }
    }

    if (cmd_ok) {
      long move = in_vals[0];
      long ticks = in_vals[1];
      long accel = in_vals[2];
      if (move) {
        Serial.print("ticks=");
        Serial.print(ticks);
        Serial.print("  accel=");
        Serial.print(accel);
        Serial.print("  move=");
        Serial.print(move);
        stopped = false;
        stepper->setSpeed(ticks);
        stepper->setAcceleration(accel);
        stepper->move(move);
        Serial.print("  Start stepper: ");
        Serial.println(stepper->getCurrentPosition());
      }
    }

    if (!stopped && true) {
      Serial.print("Stepper: ");
      Serial.print(stepper->isrSpeedControlEnabled() ? " AUTO " : " MANU ");
      Serial.print(stepper->getCurrentPosition());
      if (stepper->isRunning()) {
        Serial.print("  RUNNING");
      } else {
        Serial.print("  PAUSED ");
      }
      Serial.print("  state=");
      Serial.print(stepper->rampState());
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
      Serial.print("  max/us=");
      Serial.print(stepper->max_micros);
#endif
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
      Serial.print("  checksum=");
      Serial.print(stepper->checksum);
#endif
      Serial.println();

      stopped = !stepper->isRunning();
      if (stopped) {
        Serial.println(
            "Please enter one line with <steps> <speed> <acceleration> "
            "e.g.");
        Serial.println("10000 1000 100");
      }
    } else {
      stopped = !stepper->isRunning();
    }
  }
}
