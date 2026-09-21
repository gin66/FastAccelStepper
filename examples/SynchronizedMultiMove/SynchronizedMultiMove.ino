#include "FastAccelStepper.h"

// Test/demo sketch for two features:
//
// 1) S-curve motion (jerk-limited ramp): setLinearAcceleration() makes the
//    acceleration itself ramp up linearly from 0 to the configured value
//    over the given number of steps, instead of jumping to it immediately.
//    That rounds off the corners of the speed profile into an S-shape,
//    instead of a plain trapezoid.
//
// 2) Two (or more) steppers moving to different target positions, started
//    together and arriving back at standstill together, via
//    engine.moveAllToSync(). Each axis still runs its own independent
//    ramp - this is not interpolated straight-line motion - but the
//    speed (and, for very short moves, the acceleration) of every axis
//    but the slowest is scaled down so all of them take the same time.
//
// HOW TO VERIFY IT WORKS:
// Open the Serial Monitor at 115200 baud. Every cycle prints when the move
// starts and, per stepper, how many ms after the start it reached standstill
// again. Both "reached standstill" lines should be close to each other
// (a few ms apart - one stepper task tick, ~4ms - is expected/fine),
// regardless of the very different distance/speed/acceleration configured
// for the two steppers below.
//
// Wiring: connect two step/dir/enable driver boards, or just probe the pins
// with a logic analyzer/scope - real motors are not required to see the
// timing on the pins.

// As in StepperDemo for Motor 1+2 on ESP32
#define dirPinStepper1 18
#define enablePinStepper1 26
#define stepPinStepper1 17

#define dirPinStepper2 19
#define enablePinStepper2 26
#define stepPinStepper2 16

// For AVR (e.g. Arduino Nano/Uno), use instead:
// #define dirPinStepper1    5
// #define enablePinStepper1 6
// #define stepPinStepper1   9   // OC1A
// #define dirPinStepper2    7
// #define enablePinStepper2 6
// #define stepPinStepper2   10  // OC1B

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

void setup() {
  Serial.begin(115200);
  Serial.println("START");
  engine.init();

  stepper1 = engine.stepperConnectToPin(stepPinStepper1);
  stepper2 = engine.stepperConnectToPin(stepPinStepper2);

  if (stepper1 && stepper2) {
    stepper1->setDirectionPin(dirPinStepper1);
    stepper1->setEnablePin(enablePinStepper1);
    stepper1->setAutoEnable(true);

    stepper2->setDirectionPin(dirPinStepper2);
    stepper2->setEnablePin(enablePinStepper2);
    stepper2->setAutoEnable(true);

    // Each axis' own maximum speed/acceleration. stepper2 is nominally
    // much faster than stepper1 - moveAllToSync() will slow it down for
    // any move where stepper1 would otherwise be the bottleneck.
    stepper1->setSpeedInHz(4000);
    stepper1->setAcceleration(8000);
    stepper1->setLinearAcceleration(100);  // S-curve corners

    stepper2->setSpeedInHz(12000);
    stepper2->setAcceleration(30000);
    stepper2->setLinearAcceleration(300);  // S-curve corners
  } else {
    while (true) {
      Serial.println("NO STEPPER - check pin definitions for your board");
      delay(1000);
    }
  }
}

bool going_forward = true;
bool move_active = false;
uint32_t move_start_ms = 0;
bool stepper1_done = false;
bool stepper2_done = false;

void startNextMove() {
  int32_t target1 = going_forward ? 4000 : 0;   // long move
  int32_t target2 = going_forward ? 1000 : 0;   // short move, faster axis
  going_forward = !going_forward;

  FastAccelStepper *steppers[2] = {stepper1, stepper2};
  int32_t targets[2] = {target1, target2};

  move_start_ms = millis();
  stepper1_done = false;
  stepper2_done = false;
  move_active = true;

  Serial.print("t=0ms  starting move -> stepper1:");
  Serial.print(target1);
  Serial.print(" stepper2:");
  Serial.println(target2);

  MoveResultCode res = engine.moveAllToSync(steppers, targets, 2);
  if (!moveIsOk(res)) {
    Serial.print("  moveAllToSync() returned error: ");
    Serial.println(toString(res));
  }
}

void loop() {
  if (!move_active) {
    startNextMove();
    return;
  }

  if (!stepper1_done && !stepper1->isRunning()) {
    stepper1_done = true;
    Serial.print("t=");
    Serial.print(millis() - move_start_ms);
    Serial.println("ms  stepper1 reached standstill");
  }
  if (!stepper2_done && !stepper2->isRunning()) {
    stepper2_done = true;
    Serial.print("t=");
    Serial.print(millis() - move_start_ms);
    Serial.println("ms  stepper2 reached standstill");
  }

  if (stepper1_done && stepper2_done) {
    Serial.println("-- move complete, both arrived --");
    Serial.println();
    delay(1000);  // pause so the log is easy to read
    move_active = false;
  }
}
