#include "FastAccelStepper.h"

#include <cinttypes>

// As in StepperDemo for Motor 1 on ESP32
#define dirPinStepper 18
#define enablePinStepper 26
#define stepPinStepper 17

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  printf("START\n");

  engine.init(0);

  printf("Engine initialized\n");

  stepper = engine.stepperConnectToPin(stepPinStepper);

  printf("Stepper connected\n");
  for (uint8_t i = 0; i < 10; i++) {
    printf("LOOP %d\n", i);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_task_wdt_reset();
  }

  if (stepper) {
    //    stepper->setDirectionPin(dirPinStepper);
    //    stepper->setEnablePin(enablePinStepper);
    //    stepper->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    // stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);

    stepper->setSpeedInUs(1000);  // the parameter is us/step !!!
    stepper->setAcceleration(100);

#ifdef SUPPORT_ESP32_PULSE_COUNTER
	stepper->attachToPulseCounter();
#endif

    printf("Stepper initialized\n");
  } else {
    printf("No stepper\n");
  }
}

extern "C" void app_main() {
  setup();
  while (true) {
    while (stepper->isRunning()) {
      esp_task_wdt_reset();
      printf("pos=%" PRId32, stepper->getCurrentPosition());
#ifdef SUPPORT_ESP32_PULSE_COUNTER
   	  int16_t pcnt = stepper->readPulseCounter();
      printf("  pcnt=%d", pcnt);
#endif
	  printf("\n");
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    stepper->move(1000);
  }
  // WARNING: if program reaches end of function app_main() the MCU will
  // restart.
}
