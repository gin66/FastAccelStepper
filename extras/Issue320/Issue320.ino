#include "FastAccelStepper.h"
#if defined(SUPPORT_ESP32_RMT) && (ESP_IDF_VERSION_MAJOR == 4)
#include "esp32/clk.h"

// Tests done with esp32

#define dirPinStepper 17 // 3,8 will cause watchdog reset loop
#define enablePinStepper 16 
#define stepPinStepper 18 // 6 does not work

//#define stepperSpeed 200000
//#define stepperAcceleration 1000000
//#define stepperSteps 90000
#define stepperSpeed 200000
#define stepperAcceleration 1000000
#define stepperSteps 90000

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  delay(1000);
  Serial.println("Starting");
  Serial.print("ESP-IDF Version: ");
  Serial.println(esp_get_idf_version());
  Serial.printf("APB Base Clock: %d Hz\n", esp_clk_apb_freq());


  engine.init();
  // with v0.31.7:
  // stepperSpeed 200000:
  //   RMT needs 1660ms
  //   MCPWM needs 660ms
  // stepperSpeed 180000:
  //   RMT needs 1680ms
  // stepperSpeed 170000:
  //   RMT needs 1700ms
  // stepperSpeed 168000: => Number of steps 89861 instead of 90000. Steps are interrupted with pauses
  //   RMT needs 1720ms
  // stepperSpeed 165000:
  //   RMT needs 680ms
  // stepperSpeed 160000:
  //   RMT needs 740ms
  // stepperSpeed 150000:
  //   RMT needs 760ms
  // stepperSpeed 100000:
  //   RMT and MCPWM need both ~1020ms
  // stepperSpeed 20000:
  //   RMT and MCPWM both need 4540ms
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  stepper = engine.stepperConnectToPin(stepPinStepper, DRIVER_RMT);
#else
  stepper = engine.stepperConnectToPin(stepPinStepper);
#endif

  if (!stepper) {
     while(true) {
       Serial.println("error connecting to stepper pin");
       delay(1000);
     }
  }

  stepper->setDirectionPin(dirPinStepper);
  stepper->setEnablePin(enablePinStepper);
  stepper->setAutoEnable(true);

  stepper->setSpeedInHz(stepperSpeed);
  stepper->setAcceleration(stepperAcceleration);
}

uint32_t start_move_ms = 0;

void loop() {
  int32_t stepper_position;
  static int32_t previous_position = 0;
  static unsigned long millis_stalled = 0;
  unsigned long stall_time = 0;
  static int direction = 1;

  if (stepper) {
    stepper_position = stepper->getCurrentPosition();
    if (stepper->isRunning()) {
      Serial.print("@");
      Serial.print(stepper_position);
      if (stepper_position == previous_position) {
        if (millis_stalled == 0) {
          millis_stalled = millis();
        }
        stall_time = millis() - millis_stalled;
        Serial.println(" (Stalled for " + String(stall_time) + "ms)");
        if (stall_time > 2000) {
          int32_t new_position = direction > 0 ? stepperSteps : 0;
          Serial.println("Stopping stepper and moving to " + String(new_position));
          stepper->forceStopAndNewPosition(new_position);
          millis_stalled = 0;
        }
        delay(1000);
      } else {
        Serial.println();
      }
      previous_position = stepper_position;
    } else {
      uint32_t end_move_ms = millis();
      Serial.print("@");
      Serial.print(stepper_position);
      if (start_move_ms != 0) {
         Serial.print(" in ");
         Serial.print(end_move_ms - start_move_ms);
         Serial.print("ms");
      }
      Serial.println();
      delay(1000);

      start_move_ms = millis();
      if (stepper_position == 0) {
        stepper->moveTo(stepperSteps);
        char buffer[50];
        snprintf(buffer, sizeof(buffer), "moveTo(%d)", stepperSteps);
        Serial.println(buffer);
        direction = 1;
      } else {
        stepper->moveTo(0);
        Serial.println("moveTo(0)");
        direction = -1;
      }
    }
  }

  delay(20);
}
#else
void setup() {}
void loop() {}
#endif
