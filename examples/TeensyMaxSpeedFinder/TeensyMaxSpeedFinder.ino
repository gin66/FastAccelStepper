#include "FastAccelStepper.h"

// Finds the real maximum reliable step rate on YOUR board/motor/driver
// combination, for the EXPERIMENTAL Teensy 4.0/4.1 backend (src/pd_teensy).
//
// WHY THIS EXISTS: software alone cannot detect missed steps on an open-loop
// stepper (no encoder feedback) - the queue/ramp generator will happily
// report "move complete" even if the motor physically stalled or skipped
// pulses under load. So this test relies on YOU watching the shaft.
//
// HOW TO USE:
// 1. Mark the motor shaft (piece of tape, paint dot, anything visible) so
//    you can see its exact rotational position.
// 2. Upload this sketch and open the Serial Monitor at 115200 baud.
// 3. Type 'y' <enter> to start. For each speed level, the motor moves
//    forward far enough to actually reach and briefly cruise at that
//    speed (see stepsForLeg()), pauses, then moves back the same
//    distance - ending exactly where it started IF (and only if) no
//    steps were lost in either direction. Watch the mark.
//    While it runs, status lines print every 200ms - if those stop
//    appearing, the last line printed pinpoints exactly where it got
//    stuck; share it.
// 4. After each leg finishes, the sketch asks:
//      y <enter>  - shaft returned to the exact same position -> try the
//                   next (faster) speed level
//      n <enter>  - shaft did NOT return to the same position (steps were
//                   lost) -> stop here; the LAST speed that answered 'y'
//                   is your real, hardware-verified ceiling
// 5. Whatever speed you stopped at, set that as the argument to
//    setAbsoluteSpeedLimit() in your real project (see pd_teensy/pd_config.h
//    for SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING), with a safety margin below
//    it (e.g. 80% of the last good value) - don't run right at the edge.
//
// This tests SPEED only, at a fixed, modest acceleration - it does not by
// itself find the max acceleration your motor/load can handle. If a leg
// fails, it could be losing steps because of the acceleration ramp too fast
// for your motor/load's torque, not the step rate itself. If you suspect
// that, lower ACCELERATION below and re-run from the same speed level.

#define stepPinStepper 2
#define dirPinStepper 3
#define enablePinStepper 4

// Distance per leg is computed per speed level (see stepsForLeg()) - a
// FIXED step count can't reach a high target speed at all: too short a
// distance silently turns a high-speed level into a low triangular blip
// that never actually reaches the speed being "tested", making the result
// meaningless. Higher ACCELERATION here means less distance/time needed
// to reach a given speed level (e.g. at 200000 steps/s^2, the 200000 Hz
// level needs ~300000 steps and ~1.5s per leg; lower this and the
// distance/time grows a lot - d,t ~ 1/ACCELERATION for a given speed).
// This is a test default, not a claim about your motor's real max
// acceleration - lower it if you suspect losses are from acceleration
// being too aggressive for your torque, not from the step rate itself.
// Lowered from 200000 to 20000 for round 2: round 1 (200000 steps/s^2)
// found steps lost at 125000 Hz with a hard/fast ramp - this checks
// whether that was a motor torque/acceleration limit rather than a real
// step-rate ceiling, by giving the motor a much gentler ramp to the same
// speeds. Legs will take longer now (t ~ 1/ACCELERATION for a given
// speed) - expect several seconds per leg at the higher levels.
#define ACCELERATION 20000  // steps/s^2

// Round 2: 5000-100000 Hz already confirmed good in round 1 (with an even
// harder ramp), so start straight at 100000 to save time.
const uint32_t speedLevels[] = {100000, 125000, 150000, 175000, 200000};
const uint8_t numLevels = sizeof(speedLevels) / sizeof(speedLevels[0]);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

enum class TestState {
  waitingToStart,
  movingForward,
  pausing,
  movingBack,
  waitingConfirm,
  stopped,
  done
};
TestState state = TestState::waitingToStart;
uint8_t level = 0;
uint32_t stateChangeMs = 0;
uint32_t lastReportMs = 0;
int32_t currentLegSteps = 0;

// Distance needed to accelerate up to speedHz and back down to 0 is
// speedHz^2 / ACCELERATION (both phases together); add 50% more so there
// is an actual cruise segment at speedHz to observe, not just a triangular
// peak that brushes the target speed for an instant.
int32_t stepsForLeg(uint32_t speedHz) {
  uint32_t accelDecelSteps =
      (uint32_t)(((uint64_t)speedHz * speedHz) / ACCELERATION);
  uint32_t steps = accelDecelSteps + accelDecelSteps / 2;
  if (steps < 200) {
    steps = 200;  // sane minimum for the lowest test levels
  }
  return (int32_t)steps;
}

void printStatus(const char *label) {
  Serial.print(label);
  Serial.print(" pos=");
  Serial.print(stepper->getCurrentPosition());
  Serial.print(" target=");
  Serial.print(stepper->targetPos());
  Serial.print(" stepsToStop=");
  Serial.print(stepper->stepsToStop());
  Serial.print(" speed(mHz)=");
  Serial.print(stepper->getCurrentSpeedInMilliHz());
  Serial.print(" rampState=");
  Serial.print(stepper->rampState());
  Serial.print(" queueRunning=");
  Serial.print(stepper->isQueueRunning());
  Serial.print(" queueEmpty=");
  Serial.print(stepper->isQueueEmpty());
  Serial.print(" queueEntries=");
  Serial.println(stepper->queueEntries());
}

void startLeg() {
  currentLegSteps = stepsForLeg(speedLevels[level]);
  Serial.println();
  Serial.print("=== Testing ");
  Serial.print(speedLevels[level]);
  Serial.print(" Hz, ");
  Serial.print(currentLegSteps);
  Serial.println(" steps per leg ===");
  Serial.println("Watch the shaft mark now.");
  stepper->setSpeedInHz(speedLevels[level]);
  stepper->setAcceleration(ACCELERATION);
  MoveResultCode res = stepper->move(currentLegSteps);
  Serial.print("move() forward returned: ");
  Serial.println(toString(res));
  state = TestState::movingForward;
  stateChangeMs = millis();
}

void checkSerialForYesNo() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'y' || c == 'Y') {
      if (state == TestState::waitingConfirm) {
        Serial.println("-> continuing to next speed level");
        level++;
        if (level >= numLevels) {
          state = TestState::done;
          return;
        }
      } else {
        Serial.println("-> starting");
      }
      startLeg();
      return;
    }
    if (c == 'n' || c == 'N') {
      Serial.println(
          "-> stopping here. The last speed that printed 'y' is your "
          "hardware-verified ceiling.");
      state = TestState::stopped;
      return;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  Serial.println("START - Teensy max-speed finder (non-blocking)");

  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (!stepper) {
    while (true) {
      Serial.println("NO STEPPER - check stepPinStepper is a valid pin");
      delay(1000);
    }
  }
  stepper->setDirectionPin(dirPinStepper);
  stepper->setEnablePin(enablePinStepper);
  stepper->setAutoEnable(true);

  // Raise the ceiling so setSpeedInHz() up to the top test level is even
  // accepted - see pd_teensy/pd_config.h, SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING.
  // Guarded here because this method only exists on platforms that define
  // that macro (e.g. not atmelsam) - the CI build matrix compiles every
  // example against every platform.
#if defined(SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING)
  stepper->setAbsoluteSpeedLimit(
      (uint16_t)fas_max((uint32_t)1, TICKS_PER_S / 400000));
#endif

  Serial.println(
      "Mark the shaft now if you haven't. Type 'y' <enter> when ready to "
      "start.");
}

void loop() {
  uint32_t now = millis();
  bool wantReport = (now - lastReportMs >= 200);

  switch (state) {
    case TestState::waitingToStart:
    case TestState::waitingConfirm:
      checkSerialForYesNo();
      break;

    case TestState::movingForward:
      if (wantReport) {
        lastReportMs = now;
        printStatus("[fwd]");
      }
      if (!stepper->isRunning()) {
        Serial.println("Forward leg finished.");
        state = TestState::pausing;
        stateChangeMs = now;
      }
      break;

    case TestState::pausing:
      if (now - stateChangeMs >= 300) {
        MoveResultCode res = stepper->move(-currentLegSteps);
        Serial.print("move() back returned: ");
        Serial.println(toString(res));
        state = TestState::movingBack;
      }
      break;

    case TestState::movingBack:
      if (wantReport) {
        lastReportMs = now;
        printStatus("[back]");
      }
      if (!stepper->isRunning()) {
        Serial.println("Back leg finished.");
        Serial.println(
            "Did the shaft mark return to EXACTLY the same position as "
            "before this leg started? (y/n)");
        state = TestState::waitingConfirm;
      }
      break;

    case TestState::stopped:
      break;

    case TestState::done:
      if (wantReport) {
        lastReportMs = now;
        Serial.println(
            "All speed levels passed! Add higher values to speedLevels[] "
            "and re-flash to keep narrowing in.");
      }
      break;
  }
}
