#include "FastAccelStepper.h"

// Minimal bring-up/test sketch for the EXPERIMENTAL Teensy 4.0/4.1 backend
// (src/pd_teensy). This backend has NOT been verified on real hardware -
// see the warning at the top of src/pd_teensy/pd_config.h. Use this sketch
// to check it on your board before relying on it in a real project.
//
// What to check:
// 1. Does it compile and upload at all?
// 2. Does the stepper actually move back and forth, the right number of
//    steps (watch the position printed to Serial, or count physical steps)?
// 3. Put a scope/logic analyzer on the STEP pin: pulse HIGH time should be
//    close to FAS_TEENSY_PULSE_WIDTH_US (default 3us, see pd_config.h), and
//    the step-to-step period should ramp smoothly (S-curve corners from
//    setLinearAcceleration(), see below) without jitter or missed edges,
//    especially near the top speed configured below.
// 4. Try raising setSpeedInHz()/setAcceleration() to find where it actually
//    breaks (missed steps, hangs) - the 50k steps/s ceiling in
//    _pd_initVars() is an untested guess, not a measurement.
//
// Any digital pin works for step/dir/enable on this backend (unlike
// SAMD/SAM, it's not restricted to specific muxed pins) - change the pin
// numbers below to whatever you've actually wired up.

#define stepPinStepper 2
#define dirPinStepper 3
#define enablePinStepper 4

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    // wait for the USB serial monitor, but don't hang forever if none is
    // attached
  }
  Serial.println("START - FastAccelStepper Teensy 4.x bring-up test");

  engine.init();

  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    Serial.println("HAVE STEPPER");
    stepper->setDirectionPin(dirPinStepper);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);

    stepper->setSpeedInHz(2000);       // start conservative, raise later
    stepper->setAcceleration(4000);
    stepper->setLinearAcceleration(50);  // small S-curve corners to check

    Serial.print("TICKS_PER_S=");
    Serial.println((uint32_t)TICKS_PER_S);
    Serial.print("max_speed getMaxSpeedInHz()=");
    Serial.println(stepper->getMaxSpeedInHz());
    Serial.print("enablePin state right after setEnablePin() (expect HIGH=1, disabled): ");
    Serial.println(digitalRead(enablePinStepper));
  } else {
    while (true) {
      Serial.println("NO STEPPER - check stepPinStepper is a valid pin");
      delay(1000);
    }
  }
}

uint32_t last_report_ms = 0;

void loop() {
  if (!stepper->isRunning()) {
    int32_t pos = stepper->getCurrentPosition();
    Serial.print("Reached standstill at position ");
    Serial.println(pos);

    int32_t target = (pos <= 0) ? 3200 : -3200;
    Serial.print("Moving to ");
    Serial.println(target);
    MoveResultCode res = stepper->moveTo(target);
    if (!moveIsOk(res)) {
      Serial.print("moveTo() error: ");
      Serial.println(toString(res));
    }
  }

  // Periodic status while running, so you can watch position/speed advance
  // in the Serial Monitor without needing a scope for a first sanity check.
  uint32_t now = millis();
  if (stepper->isRunning() && (now - last_report_ms >= 200)) {
    last_report_ms = now;
    Serial.print("pos=");
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
    Serial.print(" enablePin=");
    Serial.print(digitalRead(enablePinStepper));
    Serial.print(" queueEntries=");
    Serial.println(stepper->queueEntries());
  }
}
