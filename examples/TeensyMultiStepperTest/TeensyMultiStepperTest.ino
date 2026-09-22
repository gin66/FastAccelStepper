#include "FastAccelStepper.h"

// Multi-stepper test for the EXPERIMENTAL Teensy 4.0/4.1 backend
// (src/pd_teensy). Verifies several steppers run independently and
// correctly at the same time - in particular, crossing from the first
// QuadTimer module (TMR1) into the second (TMR2), which nothing else in
// this library's test sketches has exercised yet.
//
// WHY THE MODULE BOUNDARY MATTERS: this backend allocates one QuadTimer
// channel per stepper, 4 channels per module, in connection order. The
// first 4 steppers you connect all land on module 0 (TMR1); the 5th is
// the first one on module 1 (TMR2), with its own separate NVIC interrupt
// vector and register block. If there were a bug specific to a non-first
// module (wrong register base address, wrong IRQ number, ISR dispatch
// mixing up modules...), it would only show up once a 5th stepper is
// connected - so NUM_STEPPERS defaults to 5 here, not fewer.
//
// Real motors are NOT required to validate this - the concern here is
// software correctness (do all channels/modules run independently
// without interfering with each other), not torque/missed-steps under
// load (already measured separately per axis with
// examples/TeensyMaxSpeedFinder). Driving raw pins into LEDs, a logic
// analyzer, or nothing at all is enough. If you do have fewer than 5
// motors/drivers, wire the remaining step pins to nothing (or an LED) -
// stepperConnectToPin() doesn't need a load on the pin.
//
// HOW TO USE:
// 1. Adjust NUM_STEPPERS and the pins[] table below to what you have
//    wired (keep NUM_STEPPERS >= 5 at least once, to cross the module
//    boundary - you can drop back down afterwards).
// 2. Upload, open the Serial Monitor at 115200.
// 3. Each stepper immediately starts an independent, continuous
//    back-and-forth move at its own speed (deliberately different per
//    axis, see SPEED_HZ[] / ACCEL[] below) - this maximizes the chance
//    of exposing any cross-talk between channels/modules, since they're
//    never in lockstep.
// 4. A status table prints every 500 ms: one row per stepper, showing
//    which TMR module/channel it's on, position, live speed, and
//    queue/ramp state. All rows should update independently and none
//    should ever show a stuck rampState or frozen position while
//    queueRunning=1 (that pattern is exactly the "stuck ramp" bug this
//    library had - see CHANGELOG). Let it run for at least a few
//    minutes and confirm every row keeps moving smoothly the whole time.

#define NUM_STEPPERS 5

struct PinSet {
  uint8_t step, dir, enable;
};

// clang-format off
const PinSet pins[NUM_STEPPERS] = {
    {2,  3,  4},   // stepper 0 -> TMR1 channel 0
    {5,  6,  7},   // stepper 1 -> TMR1 channel 1
    {8,  9,  10},  // stepper 2 -> TMR1 channel 2
    {11, 12, 24},  // stepper 3 -> TMR1 channel 3
    {25, 26, 27},  // stepper 4 -> TMR2 channel 0 (first channel on the 2nd module)
    // add more (up to 16 total) to reach TMR3/TMR4 too, e.g.:
    // {28, 29, 30},  // stepper 5 -> TMR2 channel 1
};
// clang-format on

// Deliberately different per axis, and not simple multiples of each
// other, so the steppers are never in lockstep - makes any timing
// cross-talk between channels/modules much easier to notice.
const uint32_t SPEED_HZ[NUM_STEPPERS] = {2000, 3300, 4700, 6100, 7900};
const uint32_t ACCEL[NUM_STEPPERS] = {4000, 6000, 9000, 12000, 16000};
const int32_t TRAVEL_STEPS[NUM_STEPPERS] = {3200, 2800, 2400, 2000, 1600};

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *steppers[NUM_STEPPERS];

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  Serial.println("START - Teensy multi-stepper test");
  Serial.print("NUM_STEPPERS=");
  Serial.println(NUM_STEPPERS);

  engine.init();

  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    FastAccelStepper *s = engine.stepperConnectToPin(pins[i].step);
    steppers[i] = s;
    Serial.print("stepper ");
    Serial.print(i);
    Serial.print(" (step pin ");
    Serial.print(pins[i].step);
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
    s->setDirectionPin(pins[i].dir);
    s->setEnablePin(pins[i].enable);
    s->setAutoEnable(true);
    s->setSpeedInHz(SPEED_HZ[i]);
    s->setAcceleration(ACCEL[i]);
    s->moveTo(TRAVEL_STEPS[i]);
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
      int32_t target = (s->getCurrentPosition() <= 0) ? TRAVEL_STEPS[i]
                                                       : -TRAVEL_STEPS[i];
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
