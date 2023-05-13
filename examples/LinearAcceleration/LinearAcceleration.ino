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
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    // stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);

	// speed up in ~0.025s, which needs 625 steps without linear mode
    stepper->setSpeedInHz(50000);
    stepper->setAcceleration(2000000);
  }
  Serial.begin(115200);
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
	modes_e::start_stop_with_linear,
	modes_e::start_stop_without_linear,
	modes_e::start_stop_with_linear,
	modes_e::start_stop_without_linear,
	modes_e::only_linear,
	modes_e::done
};
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
		}
		else {
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
				stepper->setLinearAcceleration(100000);// just big enough value
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
