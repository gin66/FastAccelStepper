#include "FastAccelStepper.h"

#define dirPinStepper 11 
#define enablePinStepper 12
#define stepPinStepper 2

#define LED_PIN 2

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = engine.stepperConnectToPin(stepPinStepper);

void setup() {
  Serial.begin(115200);

  engine.init();
  engine.setDebugLed(LED_PIN);

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
  delay(100);

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
      Serial.println(
          stepper->addQueueEntry(5L * 16384, 120, true, -16384 / 119));
      Serial.println(
          stepper->addQueueEntry(4L * 16384, 120, true, -16384 / 119));
      Serial.println(
          stepper->addQueueEntry(3L * 16384, 120, true, -16384 / 119));
      Serial.println(
          stepper->addQueueEntry(2L * 16384, 120, true, -8192 / 119));
      Serial.println(stepper->addQueueEntry(6L * 4096, 120, true, -4096 / 119));
      Serial.println(stepper->addQueueEntry(5L * 4096, 120, true, -4096 / 119));
      Serial.println(stepper->addQueueEntry(4L * 4096, 120, true, -4096 / 119));
      Serial.println(stepper->addQueueEntry(3L * 4096, 120, true, -4096 / 119));
      Serial.println(stepper->addQueueEntry(2L * 4096, 120, true, 0));
      Serial.println(stepper->addQueueEntry(2L * 4096, 120, true, 0));
      Serial.println(stepper->addQueueEntry(2L * 4096, 120, true, 0));
      Serial.println(stepper->addQueueEntry(2L * 4096, 120, true, 0));
      Serial.println(stepper->addQueueEntry(2L * 4096, 120, true, 0));
      Serial.println(stepper->addQueueEntry(2L * 4096, 120, true, 0));
      Serial.println(stepper->addQueueEntry(2L * 4096, 120, true, 0));
    }

    if (cmd_ok) {
      long move = in_vals[1];
      long ticks = in_vals[2];
      long accel = in_vals[3];
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

    if (!stopped) {
      Serial.print("Stepper: ");
      Serial.print(stepper->isr_speed_control_enabled ? " AUTO " : " MANU ");
      Serial.print(stepper->getCurrentPosition());
      if (stepper->isRunning()) {
        Serial.print("  RUNNING");
      } else {
        Serial.print("  PAUSED ");
      }
      Serial.print("  state=");
      Serial.print(stepper->ramp_state);
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
