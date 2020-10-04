#include "FastAccelStepper.h"

struct stepper_config_s {
  uint8_t step;
  uint8_t enable_low_active;
  uint8_t enable_high_active;
  uint8_t direction;
  bool direction_high_count_up;
  bool auto_enable;
  uint32_t on_delay_us;
  uint16_t off_delay_ms;
};

#if defined(ARDUINO_ARCH_AVR)
// Example hardware configuration for Arduino Nano
// Please adapt to your configuration
const uint8_t led_pin = 13;  // turn off with PIN_UNDEFINED
const struct stepper_config_s stepper_config[MAX_STEPPER] = {
    {
      // stepper 1 shall be connected to OC1A
      step : 9,
      enable_low_active : 6,
      enable_high_active : PIN_UNDEFINED,
      direction : 5,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 500000,
      off_delay_ms : 5000
    },
    {
      // stepper 2 shall be connected to OC1B
      step : 10,
      enable_low_active : 8,
      enable_high_active : PIN_UNDEFINED,
      direction : 7,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    }};
#elif defined(ARDUINO_ARCH_ESP32)
// Example hardware configuration for esp32 board.
// Please adapt to your configuration
const uint8_t led_pin = 2;
const struct stepper_config_s stepper_config[MAX_STEPPER] = {
    {
      step : 23,
      enable_low_active : 21,
      enable_high_active : PIN_UNDEFINED,
      direction : 22,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    },
    {step : 255},  // unused stepper slot
    {step : 255},  // unused stepper slot
    {step : 255},  // unused stepper slot
    {step : 255},  // unused stepper slot
    {step : 255},  // unused stepper slot
};
#endif

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper[MAX_STEPPER];

void test_direct_drive(const struct stepper_config_s *stepper) {
  // Check stepper motor+driver is operational
  // This is not done via FastAccelStepper-Library for test purpose only
  uint8_t step = stepper->step;
  uint8_t enableLow = stepper->enable_low_active;
  uint8_t enableHigh = stepper->enable_high_active;
  uint8_t direction = stepper->direction;
  bool direction_high_count_up = stepper->direction_high_count_up;

  pinMode(step, OUTPUT);
  if (enableLow != PIN_UNDEFINED) {
    digitalWrite(enableLow, LOW);
    pinMode(enableLow, OUTPUT);
  }
  if (enableHigh != PIN_UNDEFINED) {
    digitalWrite(enableHigh, HIGH);
    pinMode(enableHigh, OUTPUT);
  }
  if (direction != PIN_UNDEFINED) {
    digitalWrite(direction, direction_high_count_up);
    pinMode(direction, OUTPUT);
  }
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(step, HIGH);
    delayMicroseconds(10);
    digitalWrite(step, LOW);
    delayMicroseconds(190);
  }
  if (direction != PIN_UNDEFINED) {
    digitalWrite(direction, !direction_high_count_up);
  }
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(step, HIGH);
    delayMicroseconds(10);
    digitalWrite(step, LOW);
    delayMicroseconds(190);
  }
  if (enableLow != PIN_UNDEFINED) {
    digitalWrite(enableLow, HIGH);
  }
  if (enableHigh != PIN_UNDEFINED) {
    digitalWrite(enableHigh, LOW);
  }
  // Done
}

void setup() {
  Serial.begin(115200);
  Serial.println("Demo Stepper");

  // If you are not sure, that the stepper hardware is working,
  // then try first direct port manipulation and uncomment the next line.
  test_direct_drive(&stepper_config[0]);

  engine.init();
  if (led_pin != PIN_UNDEFINED) {
    engine.setDebugLed(led_pin);
  }

  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    FastAccelStepper *s = NULL;
    const struct stepper_config_s *config = &stepper_config[i];
    if (config->step != PIN_UNDEFINED) {
      s = engine.stepperConnectToPin(config->step);
      if (s) {
        s->setDirectionPin(config->direction, config->direction_high_count_up);
        s->setEnablePin(config->enable_low_active, true);
        s->setEnablePin(config->enable_high_active, false);
        s->setAutoEnable(config->auto_enable);
        s->setDelayToEnable(config->on_delay_us);
        s->setDelayToDisable(config->off_delay_ms);
      }
    }
    stepper[i] = s;
  }

  usage();
}

uint8_t in_ptr = 0;
char in_buffer[256];
bool stopped = true;
uint32_t last_time = 0;
FastAccelStepper *selected = NULL;

void info(FastAccelStepper *s) {
  Serial.print(s->isrSpeedControlEnabled() ? "AUTO" : "MANU");
  Serial.print(" Curr=");
  Serial.print(s->getCurrentPosition());
  Serial.print(" QueueEnd=");
  Serial.print(s->getPositionAfterCommandsCompleted());
  Serial.print(" Target=");
  Serial.print(s->targetPos());
  if (s->isRunning()) {
    Serial.print(" RUN ");
  } else {
    Serial.print(" STOP ");
  }
  Serial.print(" state=");
  Serial.print(s->rampState());
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
  Serial.print(" max/us=");
  Serial.print(s->max_micros);
#endif
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
  Serial.print(" checksum=");
  Serial.print(s->checksum());
#endif
  Serial.print(" ");
}

const static char usage_str[] PROGMEM =
    "Enter commands separated by space or newline:\n"
    "     M1/M2/..  ... to select stepper\n"
    "     A<accel>  ... Set selected stepper's acceleration\n"
    "     V<speed>  ... Set selected stepper's speed\n"

    "     P<pos>    ... Move selected stepper to position (can be "
    "negative)\n"

    "     R<n>      ... Move selected stepper by n steps (can be "
    "negative)\n"
    "     E<us>     ... Set selected stepper's delay from enable to steps\n"
    "     D<ms>     ... Set selected stepper's delay from steps to disable\n"
    "     N         ... Turn selected stepper output on (disable auto enable)\n"
    "     F         ... Turn selected stepper output off (disable auto "
    "enable)\n"
    "     O         ... Put selected stepper into auto enable mode\n"
    "     S         ... Stop selected stepper with deceleration\n"
    "\n";

void usage() {
#if defined(ARDUINO_ARCH_AVR)
  char ch;
  PGM_P s = usage_str;
  for(;;) {
    ch = pgm_read_byte(s++);
    if (ch == 0) {
      break;
    }
    Serial.print(ch);
  }
#elif defined(ARDUINO_ARCH_ESP32)
  Serial.print(usage_str);
#endif
}

void output_info() {
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    if (stepper[i]) {
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(": ");
      info(stepper[i]);
    }
  }
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (in_ptr == 255) {
      in_ptr = 0;
    } else if ((ch == ' ') || (ch == '\n')) {
      long val;
      if ((strcmp(in_buffer, "M1") == 0) && stepper[0]) {
        Serial.println("Select stepper 1");
        selected = stepper[0];
      } else if ((strcmp(in_buffer, "M2") == 0) && stepper[1]) {
        Serial.println("Select stepper 2");
        selected = stepper[1];
      } else if ((strcmp(in_buffer, "M3") == 0) && stepper[2]) {
        Serial.println("Select stepper 3");
        selected = stepper[2];
      } else if ((strcmp(in_buffer, "M4") == 0) && stepper[3]) {
        Serial.println("Select stepper 4");
        selected = stepper[3];
      } else if ((strcmp(in_buffer, "M5") == 0) && stepper[4]) {
        Serial.println("Select stepper 5");
        selected = stepper[4];
      } else if ((strcmp(in_buffer, "M6") == 0) && stepper[5]) {
        Serial.println("Select stepper 6");
        selected = stepper[5];
      } else if (selected) {
        if (sscanf(in_buffer, "A%ld", &val) == 1) {
          Serial.print("Set acceleration to ");
          Serial.println(val);
          selected->setAcceleration(val);
        } else if (sscanf(in_buffer, "V%ld", &val) == 1) {
          Serial.print("Set speed (us) to ");
          Serial.println(val);
          selected->setSpeed(val);
        } else if (sscanf(in_buffer, "R%ld", &val) == 1) {
          Serial.print("Move steps ");
          Serial.println(val);
          selected->move(val);
        } else if (sscanf(in_buffer, "P%ld", &val) == 1) {
          Serial.print("Move to position ");
          Serial.println(val);
          selected->moveTo(val);
        } else if (sscanf(in_buffer, "E%ld", &val) == 1) {
          Serial.print("Set enable time to ");
          Serial.println(val);
          selected->setDelayToEnable(val);
        } else if (sscanf(in_buffer, "D%ld", &val) == 1) {
          Serial.print("Set disable time to ");
          Serial.println(val);
          selected->setDelayToDisable(val);
        } else if (strcmp(in_buffer, "N") == 0) {
          Serial.print("Output driver on");
          selected->setAutoEnable(false);
          selected->enableOutputs();
        } else if (strcmp(in_buffer, "F") == 0) {
          Serial.print("Output driver off");
          selected->setAutoEnable(false);
          selected->disableOutputs();
        } else if (strcmp(in_buffer, "O") == 0) {
          Serial.print("Output driver off");
          selected->setAutoEnable(true);
        } else if (strcmp(in_buffer, "S") == 0) {
          Serial.print("Stop");
          selected->stopMove();
        }
      }
      in_ptr = 0;
    } else {
      in_buffer[in_ptr++] = toupper(ch);
      in_buffer[in_ptr] = 0;
    }
  }

  bool running = false;
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    if (stepper[i]) {
      running |= stepper[i]->isRunning();
    }
  }
  if (running) {
    uint32_t now = millis();
    if (now - last_time >= 100) {
      output_info();
      last_time = now;
    }
  }
  if (!stopped && !running) {
    output_info();
    usage();
  }
  stopped = !running;
}
