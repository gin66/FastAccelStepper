#include "FastAccelStepper.h"

// Multi-stepper test for the EXPERIMENTAL Teensy 4.0/4.1 backend
// (src/pd_teensy). Defaults to the FULL claimed capacity - 16 steppers,
// spanning all 4 QuadTimer modules (TMR1..TMR4) - to find out how many
// this board actually drives, not just how many the code claims to
// support.
//
// WHY THIS MATTERS: this backend allocates one QuadTimer channel per
// stepper, 4 channels per module, in connection order. Steppers 0-3 land
// on module 0 (TMR1), 4-7 on module 1 (TMR2), 8-11 on module 2 (TMR3),
// 12-15 on module 3 (TMR4) - each module has its own register block and
// NVIC interrupt vector. A bug specific to a given module, or a resource
// conflict once all 4 are active simultaneously, would only show up once
// every module is actually exercised.
//
// Real motors are NOT required - this tests software correctness (do all
// 16 channels across all 4 modules run independently, with no missed
// deadlines even under the combined interrupt load of all of them at
// once), not torque/missed-steps under load (measured separately per
// axis with examples/TeensyMaxSpeedFinder). Driving raw pins into LEDs,
// a logic analyzer, or nothing at all is enough.
//
// PINS: 16 unique step pins and 16 unique dir pins are needed (step pins
// specifically MUST be unique - each one claims its own QuadTimer
// channel); the enable pin is shared across all 16 here (this library
// supports sharing enable/dir pins between motors) to keep the pin count
// within Teensy 4.0's 40 available pins. Change ENABLE_PIN or the arrays
// below to match your wiring; set NUM_STEPPERS lower if you don't have
// (or don't want to wire up) all 16.
//
// HOW TO USE:
// 1. Set NUM_STEPPERS below (up to 16) and check the pin arrays match
//    your wiring (or that the pins are simply safe to toggle/unconnected).
// 2. Upload, open the Serial Monitor at 115200.
// 3. Every stepper immediately starts an independent, continuous
//    back-and-forth move at its own speed/acceleration/distance
//    (deliberately different per axis, computed from its index so no two
//    are ever in lockstep) - this maximizes the chance of exposing any
//    cross-talk between channels/modules.
// 4. A status table prints every 500 ms: one row per stepper, showing
//    which TMR module/channel it's on, position, live speed, and
//    queue/ramp state. Every row should keep updating independently;
//    none should ever show a stuck rampState or frozen position while
//    queueRunning=1 (that pattern is exactly the "stuck ramp" bug this
//    library had - see CHANGELOG). Let it run for several minutes and
//    confirm every row - especially the ones on modules 2-4 (index
//    >= 4) - keeps moving smoothly the whole time.

#define NUM_STEPPERS 16  // up to 16 (4 QuadTimer modules x 4 channels)

// clang-format off
const uint8_t STEP_PIN[16] = {
    2,  3,  4,  5,  6,  7,  8,  9,
    10, 11, 12, 14, 15, 16, 17, 18,  // 13 skipped (onboard LED)
};
const uint8_t DIR_PIN[16] = {
    19, 20, 21, 22, 23, 24, 25, 26,
    27, 28, 29, 30, 31, 32, 33, 34,
};
// clang-format on
#define ENABLE_PIN 35  // shared by all steppers - this library supports that

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *steppers[NUM_STEPPERS];
int32_t travelSteps[NUM_STEPPERS];

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  Serial.println("START - Teensy multi-stepper test");
  Serial.print("NUM_STEPPERS=");
  Serial.println(NUM_STEPPERS);

  engine.init();

  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    FastAccelStepper *s = engine.stepperConnectToPin(STEP_PIN[i]);
    steppers[i] = s;
    Serial.print("stepper ");
    Serial.print(i);
    Serial.print(" (step pin ");
    Serial.print(STEP_PIN[i]);
    Serial.print(") -> TMR module ");
    Serial.print(i >> 2);
    Serial.print(" channel ");
    Serial.print(i & 0x03);
    Serial.print(": ");
    if (!s) {
      Serial.println("FAILED to connect - pin busy or out of queues?");
      continue;
    }
    Serial.println("OK");
    s->setDirectionPin(DIR_PIN[i]);
    s->setEnablePin(ENABLE_PIN);
    s->setAutoEnable(true);

    // Deliberately different per axis, and not simple multiples of each
    // other, so the steppers are never in lockstep - makes any timing
    // cross-talk between channels/modules much easier to notice.
    uint32_t speedHz = 2000 + (uint32_t)i * 733;
    uint32_t accel = 4000 + (uint32_t)i * 1500;
    travelSteps[i] = 1800 + (int32_t)(i % 5) * 300;

    s->setSpeedInHz(speedHz);
    s->setAcceleration(accel);
    s->moveTo(travelSteps[i]);
  }
  Serial.println();
}

uint32_t lastReportMs = 0;

void loop() {
  uint32_t now = millis();
  if (now - lastReportMs < 500) {
    return;
  }
  lastReportMs = now;

  Serial.println(
      "ax mod ch      pos    target  speed(mHz) rampState qRun qEmpty "
      "qEntries");
  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    FastAccelStepper *s = steppers[i];
    if (!s) {
      continue;
    }
    if (!s->isRunning()) {
      // reverse direction and go again
      int32_t target =
          (s->getCurrentPosition() <= 0) ? travelSteps[i] : -travelSteps[i];
      s->moveTo(target);
    }
    char line[110];
    snprintf(line, sizeof(line),
            "%2d  %1d  %1d %8ld  %8ld  %10ld  %9d  %4d %6d %8d", (int)i,
            (int)(i >> 2), (int)(i & 0x03), (long)s->getCurrentPosition(),
            (long)s->targetPos(), (long)s->getCurrentSpeedInMilliHz(),
            (int)s->rampState(), (int)s->isQueueRunning(),
            (int)s->isQueueEmpty(), (int)s->queueEntries());
    Serial.println(line);
  }
  Serial.println();
}
