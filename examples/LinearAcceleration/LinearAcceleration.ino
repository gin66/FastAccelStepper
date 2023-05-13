#include "FastAccelStepper.h"

// As in StepperDemo for Motor 1 on AVR
//#define dirPinStepper    5
//#define enablePinStepper 6
//#define stepPinStepper   9  // OC1A in case of AVR

// As in StepperDemo for Motor 1 on ESP32
#define dirPinStepper 18
#define enablePinStepper 26
#define stepPinStepper 17

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(115200);
  Serial.println("START");
  engine.init();

#ifdef SUPPORT_SELECT_DRIVER_TYPE
// The code below is only relevant for original esp32
//#define TEST_DONT_CARE_RMT
#ifdef TEST_DONT_CARE_RMT
  // here occupy first all mcpwm/pcnt modules, then uses rmt
  stepper = engine.stepperConnectToPin(19, DRIVER_DONT_CARE);
  stepper = engine.stepperConnectToPin(16, DRIVER_DONT_CARE);
  stepper = engine.stepperConnectToPin(15, DRIVER_DONT_CARE);
  stepper = engine.stepperConnectToPin(14, DRIVER_DONT_CARE);
  stepper = engine.stepperConnectToPin(13, DRIVER_DONT_CARE);
  stepper = engine.stepperConnectToPin(12, DRIVER_DONT_CARE);
  // now I get the rmt, I want
  stepper = engine.stepperConnectToPin(stepPinStepper, DRIVER_DONT_CARE);
#else
  // This is uses the new interface to enforce usage of RMT
  stepper = engine.stepperConnectToPin(stepPinStepper, DRIVER_RMT);
#endif
#else
  stepper = engine.stepperConnectToPin(stepPinStepper);
#endif
  if (stepper) {
    Serial.println("HAVE STEPPER");
    stepper->setDirectionPin(dirPinStepper);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    // stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);

    // speed up in ~0.025s, which needs 625 steps without linear mode
    stepper->setSpeedInHz(50000);
    stepper->setAcceleration(2000000);
  } else {
    while (true) {
      Serial.println("NO STEPPER");
      delay(1000);
    }
  }
}

enum struct modes_e {
  start_with_linear,
  start_stop_with_linear,
  start_stop_without_linear,
  only_linear,
  done
} modes[] = {
    modes_e::start_with_linear,
    // repeat the next for better comparison
    modes_e::start_stop_with_linear, modes_e::start_stop_without_linear,
    modes_e::start_stop_with_linear, modes_e::start_stop_without_linear,
    modes_e::only_linear, modes_e::done};
uint8_t mode_i = 0;
bool need_clear_linear = false;

void loop() {
  // we are using non-blocking mode to demonstrate, how to use that

  if (!stepper->isRunning()) {
    // Only is stepper is not running
    int32_t pos = stepper->getCurrentPosition();

    if (pos > 0) {
      // next mode
      mode_i = mode_i + 1;
      if (modes[mode_i] == modes_e::done) {
        mode_i = 0;
      }

      // now we have plenty of time to write to the Serial
      switch (modes[mode_i]) {
        case modes_e::start_with_linear:
          Serial.println("start with linear and stop without");
          break;
        case modes_e::start_stop_with_linear:
          Serial.println("start and stop with linear");
          break;
        case modes_e::start_stop_without_linear:
          Serial.println("start and stop without linear");
          break;
        case modes_e::only_linear:
          Serial.println("no constant acceleration");
          break;
        case modes_e::done:
          break;
      }
      // let the user read the message
      delay(2000);
    }

    // we are just going back and forth
    if (pos < 0) {
      pos = 3200;
    } else {
      pos = -3200;
    }
    need_clear_linear = false;
    switch (modes[mode_i]) {
      case modes_e::start_with_linear:
        need_clear_linear = true;
      case modes_e::start_stop_with_linear:
        // 100 steps linear, then constant acceleration
        stepper->setLinearAcceleration(100);
        break;
      case modes_e::start_stop_without_linear:
        stepper->setLinearAcceleration(0);
        break;
      case modes_e::only_linear:
        stepper->setLinearAcceleration(100000);  // just big enough value
        break;
      case modes_e::done:
        break;
    }
    stepper->moveTo(pos);
  }
  if (need_clear_linear && (stepper->rampState() == RAMP_STATE_COAST)) {
    // if needed, as soon as coasting is reached, turn of linear acceleration
    need_clear_linear = false;
    stepper->setLinearAcceleration(0);
  }
}
