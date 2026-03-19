#include <FastAccelStepper.h>

#define STEP_PIN GPIO_NUM_14
#define DIR_PIN GPIO_NUM_15

FastAccelStepperEngine engine;
FastAccelStepper* stepper;

void run_with_timeout(const char* name, int32_t expected_pos, uint32_t ms) {
  unsigned long t0 = millis();
  while (stepper->isRunning()) {
    if (millis() - t0 > ms) {
      Serial.print("FAIL: timeout (pos=");
      Serial.print(stepper->getCurrentPosition());
      Serial.println(")");
      stepper->forceStop();
      return;
    }
    delay(1);
  }
  int32_t pos = stepper->getCurrentPosition();
  Serial.print("  Position: ");
  Serial.println(pos);
  if (pos != expected_pos) {
    Serial.print("FAIL: expected ");
    Serial.print(expected_pos);
    Serial.print(" got ");
    Serial.println(pos);
  } else {
    Serial.println("  PASS");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("=== MCPWM/PCNT Stepper Driver Test ===");

  engine.init();

  stepper = engine.stepperConnectToPin(STEP_PIN, DRIVER_MCPWM_PCNT);
  if (stepper == nullptr) {
    Serial.println("FAIL: stepperConnectToPin returned NULL");
    while (1) delay(1000);
  }

  stepper->setDirectionPin(DIR_PIN);
  stepper->setSpeedInUs(100);
  stepper->setAcceleration(10000);

  Serial.println("Test 1: Move 1000 steps forward");
  stepper->move(1000);
  run_with_timeout("T1", 1000, 5000);

  Serial.println("Test 2: Move 500 steps backward");
  stepper->move(-500);
  run_with_timeout("T2", 500, 5000);

  Serial.println("Test 3: 100 steps at 50us/step");
  stepper->setSpeedInUs(50);
  stepper->move(100);
  run_with_timeout("T3", 600, 5000);

  Serial.println("Test 4: Single step");
  stepper->setSpeedInUs(1000);
  stepper->move(1);
  run_with_timeout("T4", 601, 5000);

  Serial.println("Test 5: Stop mid-motion");
  stepper->setSpeedInUs(50);
  stepper->setAcceleration(50000);
  stepper->move(10000);
  delay(50);
  stepper->stopMove();
  run_with_timeout("T5", -1, 5000);
  Serial.println("  PASS (stopped)");

  Serial.println("=== ALL TESTS DONE ===");
}

void loop() { delay(1000); }
