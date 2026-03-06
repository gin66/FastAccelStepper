#include "FastAccelStepper.h"

#ifdef SUPPORT_ESP32_I2S

// I2S_DIRECT example: one stepper per I2S peripheral, bit-level resolution
#define i2sDirectStepPin 15

// I2S_MUX example: multiple steppers via 74HC595 shift register chain
// Directly connect to ESP32 GPIOs:
#define i2sMuxDataPin 32
#define i2sMuxBclkPin 33
#define i2sMuxWsPin 14

// 74HC595 output slot layout (directly usable as step pins with PIN_I2S_FLAG):
// Slot 0: Stepper 0 STEP
// Slot 1: Stepper 0 DIR   (active-high)
// Slot 2: Stepper 0 ENABLE (active-low)
// Slot 3: Stepper 1 STEP
// Slot 4: Stepper 1 DIR   (active-high)
// Slot 5: Stepper 1 ENABLE (active-low)

FastAccelStepperEngine engine = FastAccelStepperEngine();

// I2S_DIRECT stepper
FastAccelStepper* stepperDirect = NULL;

// I2S_MUX steppers
FastAccelStepper* stepperMux0 = NULL;
FastAccelStepper* stepperMux1 = NULL;

void setup() {
  Serial.begin(115200);
  engine.init();

  // --- I2S_DIRECT: one stepper, bit-level resolution (up to ~200kHz) ---
  stepperDirect =
      engine.stepperConnectToPin(i2sDirectStepPin, DRIVER_I2S_DIRECT);
  if (stepperDirect) {
    stepperDirect->setDirectionPin(18);
    stepperDirect->setAutoEnable(true);
    stepperDirect->setSpeedInUs(10);
    stepperDirect->setAcceleration(50000);
  }

  // --- I2S_MUX: multiple steppers via 74HC595, frame-level resolution ---
  bool mux_ok = engine.initI2sMux(i2sMuxDataPin, i2sMuxBclkPin, i2sMuxWsPin);

  if (mux_ok) {
    // Connect stepper 0 to MUX slot 0
    stepperMux0 = engine.stepperConnectToPin(0 | PIN_I2S_FLAG, DRIVER_I2S_MUX);
    if (stepperMux0) {
      stepperMux0->setAutoEnable(true);
      stepperMux0->setSpeedInUs(100);
      stepperMux0->setAcceleration(10000);
    }

    // Connect stepper 1 to MUX slot 3
    stepperMux1 = engine.stepperConnectToPin(3 | PIN_I2S_FLAG, DRIVER_I2S_MUX);
    if (stepperMux1) {
      stepperMux1->setAutoEnable(true);
      stepperMux1->setSpeedInUs(200);
      stepperMux1->setAcceleration(5000);
    }
  }

  // Start moving
  if (stepperDirect) {
    stepperDirect->move(10000);
  }
  if (stepperMux0) {
    stepperMux0->move(1000);
  }
  if (stepperMux1) {
    stepperMux1->move(2000);
  }
}

void loop() {}

#else

void setup() {}
void loop() {}

#endif
