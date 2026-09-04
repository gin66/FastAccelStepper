
#include <Arduino.h>
#include <FastAccelStepper.h>

/*
 * PINS
 */

#define PIN_STEP 3
#define PIN_DIR 20
#define PIN_ENABLE 6
#define PIN_TX 4
#define PIN_RX 5

/*
 * TMC SETTINGS
 */

#define R_SENSE 0.11f
#define DRIVER_ADDR 0b00

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* stepper = NULL;

/*
 * SPEED SETTINGS
 */

#define STEPS_PER_REV 200
#define STEPS_PITCH 2  // GT2 mm
#define STEPS_PULLEY 60
#define STEPS_PER_MM \
  (float)(STEPS_PER_REV * MICROSTEPS) / (float)(STEPS_PULLEY * STEPS_PITCH)

#define MICROSTEPS 16
#define POWER 750  // mA

#define SPEED_MAX 100.0f    // mm/s
#define SPEED_ACCEL 500.0f  // mm/s²

void setup() {
  Serial.begin(115200);

  Serial.println("[DEBUG]");
  for (uint8_t t = 5; t > 0; t--) {
    Serial.print("(");
    Serial.print(t);
    Serial.println(")");
    delay(1000);
  }

  Serial.println("--- START: Direction Change Test ---");

  // Enable pin initial state (HIGH = disabled)
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, HIGH);

  // FastAccelStepper Setup
  engine.init();
  stepper = engine.stepperConnectToPin(PIN_STEP);

  if (stepper) {
    stepper->setDirectionPin(PIN_DIR, false, 1000);
    stepper->setEnablePin(PIN_ENABLE, true);  // true = LOW is active
    stepper->setAutoEnable(true);

    stepper->setSpeedInHz((uint32_t)(SPEED_MAX * STEPS_PER_MM));
    stepper->setAcceleration((uint32_t)(SPEED_ACCEL * STEPS_PER_MM));
    stepper->applySpeedAcceleration();

    Serial.println("FastAccelStepper Initialized.");
  } else {
    Serial.println("ERROR: Failed to init stepper!");
    while (1);
  }

  delay(1000);
}

// Input
String input;
char endMarker = '\n';

// Test states
int32_t targetAccel = -(SPEED_ACCEL * STEPS_PER_MM);
unsigned long lastUpdate = 0;
unsigned long timer = 0;

// Test
float freq = 100.0;  // Hz
bool run = false;

// Force stop workaround
bool fixEnable = false;
bool fixWaitingForStop = false;

void loop() {
  unsigned long now = millis();

  /*
   * Test
   */

  // 100Hz
  if (run && (now - lastUpdate) >= (1000 / freq)) {
    timer += (now - lastUpdate);
    lastUpdate = now;

    // Change direction every 2 seconds
    if (timer > 2000) {
      targetAccel = -targetAccel;  // Flip sign
      timer = 0;

      if (fixEnable) {
        stepper->stopMove();
        Serial.println(">>> DIRECTION CHANGE COMMANDED w/ WORKAROUND <<<");
      } else {
        Serial.println(">>> DIRECTION CHANGE COMMANDED <<<");
      }
    }

    // Send command

    int res;

    if (fixEnable) {
      // Workaround for FastAccelStepper forceStop() not allowing new moves
      if (stepper->isStopping()) {
        res = -7;  // Waiting for stop
        fixWaitingForStop = true;
      } else if (fixWaitingForStop && stepper->isRunning() &&
                 stepper->getCurrentSpeedInMilliHz() == 0) {
        res = -8;  // Speed zero while running
        stepper->forceStop();
        Serial.println(">>> WORKAROUND: Speed=0, forceStop() <<<");
      } else if (fixWaitingForStop && !stepper->isRunning()) {
        res = -9;  // Stopped
        fixWaitingForStop = false;
        Serial.println(">>> WORKAROUND: Stepper stopped <<<");
      } else {
        res = (int)stepper->moveByAcceleration(targetAccel);
      }
    } else {
      res = (int)stepper->moveTo(targetAccel);
      // res = (int) stepper->moveByAcceleration(targetAccel);
    }

    // Log every 100ms
    if (timer % 100 == 0) {
      Serial.print("@");
      Serial.print(millis());
      Serial.print("\tT:");
      Serial.print(timer);
      Serial.print("\tCmd:");
      Serial.print(targetAccel);
      Serial.print("\tPos:");
      Serial.print(stepper->getCurrentPosition());

      Serial.print("\tSpd:");
      Serial.print(stepper->getCurrentSpeedInMilliHz() / 1000.0);
      Serial.print("\tRes:");
      Serial.print(res);

      Serial.print("\tRun:");
      Serial.print(stepper->isRunning());
      Serial.print("\tStop:");
      Serial.print(stepper->isStopping());
      Serial.print("\tDir:");
#if defined(PICO_RP2040) || defined(PICO_RP2350)
      Serial.print(gpio_get(20));
#endif
      Serial.println();
    }
  }

  /*
   * Control
   */

  if (Serial.available() > 0) {
    char rc = Serial.read();

    if (rc != endMarker) {
      input += rc;
      Serial.print(rc);
    } else {
      Serial.println("<<< " + input);

      // Exec
      input.trim();
      input.replace("\\", "");

      if (input.equalsIgnoreCase("start")) {
        lastUpdate = millis();
        timer = 0;
        run = true;
      } else if (input.equalsIgnoreCase("stop")) {
        stepper->forceStop();
        run = false;
      } else if (input.equalsIgnoreCase("fix")) {
        fixEnable = !fixEnable;
        Serial.print(">>> WORKAROUND ");
        Serial.print(fixEnable ? "ENABLED" : "DISABLED");
        Serial.println(" <<<");
      }

      // reset
      input = "";
    }
  }
}
