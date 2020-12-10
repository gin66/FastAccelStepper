#include "FastAccelStepper.h"
#include "test_seq.h"

#define VERSION "post-f7f9f42"

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
const uint8_t led_pin = 21;
const struct stepper_config_s stepper_config[MAX_STEPPER] = {
    {
      step : 33,
      enable_low_active : 25,
      enable_high_active : PIN_UNDEFINED,
      direction : 32,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    },
    {
      step : 26,
      enable_low_active : 12,
      enable_high_active : PIN_UNDEFINED,
      direction : 27,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    },
    {step : PIN_UNDEFINED},  // unused stepper slot
    {step : PIN_UNDEFINED},  // unused stepper slot
    {step : PIN_UNDEFINED},  // unused stepper slot
    {step : PIN_UNDEFINED},  // unused stepper slot
};
#endif

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper[MAX_STEPPER];

bool test_mode = false;
bool test_ongoing = false;
struct test_seq_s test_seq[MAX_STEPPER] = {
#if defined(ARDUINO_ARCH_ESP32)
    {.test = NULL}, {.test = NULL}, {.test = NULL}, {.test = NULL},
#endif
    {.test = NULL}, {.test = NULL}};

void test_direct_drive(FastAccelStepper *_stepper,
                       const struct stepper_config_s *stepper) {
  // Check stepper motor+driver is operational
  // This is not done via FastAccelStepper-Library for test purpose only
  uint8_t step = stepper->step;
  uint8_t enableLow = stepper->enable_low_active;
  uint8_t enableHigh = stepper->enable_high_active;
  uint8_t direction = stepper->direction;
  bool direction_high_count_up = stepper->direction_high_count_up;

  _stepper->detachFromPin();

  pinMode(step, OUTPUT);

  if (enableLow != PIN_UNDEFINED) {
    digitalWrite(enableLow, LOW);
    pinMode(enableLow, OUTPUT);
    delayMicroseconds(10);
    if (digitalRead(enableLow) != LOW) {
      Serial.println("Cannot set enable low pin to LOW");
    }
  }
  if (enableHigh != PIN_UNDEFINED) {
    digitalWrite(enableHigh, HIGH);
    pinMode(enableHigh, OUTPUT);
    delayMicroseconds(10);
    if (digitalRead(enableHigh) != HIGH) {
      Serial.println("Cannot set enable high pin to HIGH");
    }
  }
  if (direction != PIN_UNDEFINED) {
    digitalWrite(direction, direction_high_count_up);
    pinMode(direction, OUTPUT);
    delayMicroseconds(10);
    if (digitalRead(direction) != direction_high_count_up) {
      Serial.print("Cannot set direction pin to ");
      Serial.println(direction_high_count_up ? "HIGH" : "LOW");
    }
  }
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(step, HIGH);
    delayMicroseconds(10);
    if (digitalRead(step) != HIGH) {
      Serial.println("Cannot set step output to HIGH");
    }
    digitalWrite(step, LOW);
    delayMicroseconds(190);
    if (digitalRead(step) != LOW) {
      Serial.println("Cannot set step output to LOW");
    }
  }
  if (direction != PIN_UNDEFINED) {
    digitalWrite(direction, !direction_high_count_up);
    delayMicroseconds(10);
    delayMicroseconds(10);
    if (digitalRead(direction) != !direction_high_count_up) {
      Serial.print("Cannot set direction pin to ");
      Serial.println(!direction_high_count_up ? "HIGH" : "LOW");
    }
  }
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(step, HIGH);
    delayMicroseconds(10);
    if (digitalRead(step) != HIGH) {
      Serial.println("Cannot set step output to HIGH");
    }
    digitalWrite(step, LOW);
    delayMicroseconds(190);
    if (digitalRead(step) != LOW) {
      Serial.println("Cannot set step output to LOW");
    }
  }
  if (enableLow != PIN_UNDEFINED) {
    digitalWrite(enableLow, HIGH);
    delayMicroseconds(10);
    if (digitalRead(enableLow) != HIGH) {
      Serial.println("Cannot set enable low pin to HIGH");
    }
  }
  if (enableHigh != PIN_UNDEFINED) {
    digitalWrite(enableHigh, LOW);
    delayMicroseconds(10);
    if (digitalRead(enableHigh) != LOW) {
      Serial.println("Cannot set enable high pin to LOW");
    }
  }
  _stepper->reAttachToPin();
  // Done
}

void setup() {
  Serial.begin(115200);
  Serial.print("StepperDemo Version ");
  Serial.println(VERSION);
  Serial.print("    F_CPU=");
  Serial.println(F_CPU);
  Serial.print("    TICKS_PER_S=");
  Serial.println(TICKS_PER_S);

  // If you are not sure, that the stepper hardware is working,
  // then try first direct port manipulation and uncomment the next line.
  // Alternatively use e.g. M1 T by serial command
  // test_direct_drive(&stepper_config[0]);

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
bool verbose = true;
bool usage_info = true;
uint32_t last_time = 0;
int selected = -1;

void info(FastAccelStepper *s) {
  Serial.print(s->isRampGeneratorActive() ? "AUTO" : "MANU");
  Serial.print(" Curr=");
  Serial.print(s->getCurrentPosition());
  Serial.print(" QueueEnd=");
  Serial.print(s->getPositionAfterCommandsCompleted());
  Serial.print("/");
  Serial.print(s->getPeriodAfterCommandsCompleted());
  Serial.print("us");
  if (s->isRunningContinuously()) {
    Serial.print(" nonstop");
  } else {
    Serial.print(" Target=");
    Serial.print(s->targetPos());
  }
  if (s->isRunning()) {
    Serial.print(" RUN ");
  } else {
    Serial.print(" STOP ");
  }
  Serial.print(" ");
  switch (s->rampState() & RAMP_STATE_MASK) {
    case RAMP_STATE_IDLE:
      Serial.print("IDLE ");
      break;
    case RAMP_STATE_ACCELERATE:
      Serial.print("ACC  ");
      break;
    case RAMP_STATE_DECELERATE_TO_STOP:
      Serial.print("DEC ");
      break;
    case RAMP_STATE_DECELERATE:
      Serial.print("RED  ");  // Reduce
      break;
    case RAMP_STATE_COAST:
      Serial.print("COAST ");
      break;
    case RAMP_STATE_REVERSE:
      Serial.print("REV   ");
      break;
    default:
      Serial.print(s->rampState());
  }
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
    "Enter commands separated by space, carriage return or newline:\n"
    "     M1/M2/..  ... to select stepper\n"
    "     A<accel>  ... Set selected stepper's acceleration\n"
    "     V<speed>  ... Set selected stepper's speed\n"
    "     U         ... Update selected stepper's speed/acceleration while "
    "running\n"
    "     P<pos>    ... Move selected stepper to position (can be "
    "negative)\n"

    "     R<n>      ... Move selected stepper by n steps (can be "
    "negative)\n"
    "     f         ... Run forward (counting up)\n"
    "     b         ... Run backward (counting down)\n"
    "     K         ... Keep selected stepper running in current direction\n"
    "     @<pos>    ... Set selected stepper to position (can be "
    "negative)\n"
    "     E<us>     ... Set selected stepper's delay from enable to steps\n"
    "     D<ms>     ... Set selected stepper's delay from steps to disable\n"
    "     N         ... Turn selected stepper output on (disable auto enable)\n"
    "     F         ... Turn selected stepper output off (disable auto "
    "enable)\n"
    "     O         ... Put selected stepper into auto enable mode\n"
    "     S         ... Stop selected stepper with deceleration\n"
    "     X         ... Immediately stop motor and set zero position\n"
    "     I         ... Toggle motor info, while any motor is running\n"
    "     W         ... Blocking wait until selected motor is stopped (will "
    "deadlock if the motor will never stop)\n"
    "     +         ... Perform one step forward of the selected motor\n"
    "     -         ... Perform one step backward of the selected motor\n"
    "     T         ... Test selected motor with direct port access\n"
#if defined(ARDUINO_ARCH_ESP32)
    "     r         ... Call ESP.restart()\n"
#endif
    "     t         ... Enter test mode\n"
    "     Q         ... Toggle print usage on motor stop\n"
    "     ?         ... Print this usage\n"
    "\n";

const static char test_usage_str[] PROGMEM =
    "Enter commands separated by space, carriage return or newline:\n"
    "     M1/M2/..  ... to select stepper\n"
    "     R         ... start all selected tests\n"
    "     01        ... select test sequence 01 for selected stepper\n"
    "     :\n"
    "     04        ... select test sequence 04 for selected stepper\n"
#if defined(ARDUINO_ARCH_ESP32)
    "     r         ... Call ESP.restart()\n"
#endif
    "\n";

void stepper_info() {
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    if (stepper[i]) {
      if (i == selected) {
        Serial.print(">> ");
      } else {
        Serial.print("   ");
      }
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(": ");
      info(stepper[i]);
      Serial.println();
    }
  }
}

void usage() {
#if defined(ARDUINO_ARCH_AVR)
  char ch;
  PGM_P s;
  if (!test_mode) {
    s = usage_str;
  } else {
    s = test_usage_str;
  }
  for (;;) {
    ch = pgm_read_byte(s++);
    if (ch == 0) {
      break;
    }
    Serial.print(ch);
  }
#elif defined(ARDUINO_ARCH_ESP32)
  if (!test_mode) {
    Serial.print(usage_str);
  } else {
    Serial.print(test_usage_str);
  }
#endif
  stepper_info();
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
    } else if ((ch == ' ') || (ch == '\n') || (ch == '\r')) {
      long val;
      in_buffer[in_ptr] = 0;
      if ((strcmp(in_buffer, "M1") == 0) && stepper[0]) {
        Serial.println("Select stepper 1");
        selected = 0;
      } else if ((strcmp(in_buffer, "M2") == 0) && stepper[1]) {
        Serial.println("Select stepper 2");
        selected = 1;
      } else if ((strcmp(in_buffer, "M3") == 0) && stepper[2]) {
        Serial.println("Select stepper 3");
        selected = 2;
      } else if ((strcmp(in_buffer, "M4") == 0) && stepper[3]) {
        Serial.println("Select stepper 4");
        selected = 3;
      } else if ((strcmp(in_buffer, "M5") == 0) && stepper[4]) {
        Serial.println("Select stepper 5");
        selected = 4;
      } else if ((strcmp(in_buffer, "M6") == 0) && stepper[5]) {
        Serial.println("Select stepper 6");
        selected = 5;
      }
#if defined(ARDUINO_ARCH_ESP32)
      else if (strcmp(in_buffer, "r") == 0) {
        Serial.println("ESP restart");
        ESP.restart();
      }
#endif
      else if (strcmp(in_buffer, "I") == 0) {
        Serial.println("Toggle motor info");
        verbose = !verbose;
      } else if (strcmp(in_buffer, "Q") == 0) {
        Serial.println("Toggle usage info");
        usage_info = !usage_info;
      } else if (strcmp(in_buffer, "?") == 0) {
        usage();
      } else if (strcmp(in_buffer, "t") == 0) {
        Serial.println("Enter test mode");
        test_mode = true;
        usage();
      } else if (selected >= 0) {
        FastAccelStepper *stepper_selected = stepper[selected];
        if (!test_mode) {
          if (sscanf(in_buffer, "A%ld", &val) == 1) {
            Serial.print("Set acceleration to ");
            Serial.println(val);
            stepper_selected->setAcceleration(val);
          } else if (sscanf(in_buffer, "V%ld", &val) == 1) {
            Serial.print("Set speed (us) to ");
            Serial.println(val);
            stepper_selected->setSpeed(val);
          } else if (sscanf(in_buffer, "R%ld", &val) == 1) {
            Serial.print("Move steps ");
            Serial.println(val);
            int res = stepper_selected->move(val);
            Serial.print("returncode=");
            Serial.println(res);
          } else if (sscanf(in_buffer, "P%ld", &val) == 1) {
            Serial.print("Move to position ");
            Serial.println(val);
            int res = stepper_selected->moveTo(val);
            Serial.print("returncode=");
            Serial.println(res);
          } else if (sscanf(in_buffer, "@%ld", &val) == 1) {
            Serial.print("Set position ");
            Serial.println(val);
            stepper_selected->setCurrentPosition(val);
          } else if (sscanf(in_buffer, "E%ld", &val) == 1) {
            Serial.print("Set enable time to ");
            Serial.println(val);
            int res = stepper_selected->setDelayToEnable(val);
            Serial.print("returncode=");
            Serial.println(res);
          } else if (sscanf(in_buffer, "D%ld", &val) == 1) {
            Serial.print("Set disable time to ");
            Serial.println(val);
            stepper_selected->setDelayToDisable(val);
          } else if (strcmp(in_buffer, "N") == 0) {
            Serial.println("Output driver on");
            stepper_selected->setAutoEnable(false);
            stepper_selected->enableOutputs();
          } else if (strcmp(in_buffer, "F") == 0) {
            Serial.println("Output driver off");
            stepper_selected->setAutoEnable(false);
            stepper_selected->disableOutputs();
          } else if (strcmp(in_buffer, "O") == 0) {
            Serial.println("Output driver off");
            stepper_selected->setAutoEnable(true);
          } else if (strcmp(in_buffer, "S") == 0) {
            Serial.println("Stop");
            stepper_selected->stopMove();
          } else if (strcmp(in_buffer, "K") == 0) {
            Serial.println("Keep running");
            stepper_selected->keepRunning();
          } else if (strcmp(in_buffer, "f") == 0) {
            Serial.println("run forward");
            int res = stepper_selected->runForward();
            Serial.print("returncode=");
            Serial.println(res);
          } else if (strcmp(in_buffer, "b") == 0) {
            Serial.println("run backward");
            int res = stepper_selected->runBackward();
            Serial.print("returncode=");
            Serial.println(res);
          } else if (strcmp(in_buffer, "X") == 0) {
            Serial.println("Immediate Stop");
            stepper_selected->forceStopAndNewPosition(0);
          } else if (strcmp(in_buffer, "U") == 0) {
            Serial.println("Update speed/acceleration");
            stepper_selected->applySpeedAcceleration();
          } else if (strcmp(in_buffer, "W") == 0) {
            Serial.println("Blocking wait for running stepper to stop");
            if (!stepper_selected->isRunningContinuously()) {
              // Wait for stepper stop
              while (stepper_selected->isRunning()) {
                // do nothing
              }
            }
          } else if (strcmp(in_buffer, "T") == 0) {
            if (!stepper_selected->isRunning()) {
              Serial.println("Test direct drive");
              test_direct_drive(stepper_selected, &stepper_config[selected]);
            }
          } else if (strcmp(in_buffer, "+") == 0) {
            if (!stepper_selected->isRunning()) {
              stepper_selected->forwardStep(true);
              Serial.println("Stepped forward");
            }
          }
        } else {
          if (strcmp(in_buffer, "R") == 0) {
            Serial.println("Run tests");
            test_ongoing = true;
          } else if (strcmp(in_buffer, "01") == 0) {
            Serial.println("Select test_seq_01");
            test_seq[selected].test = test_seq_01;
            test_seq[selected].state = 0;
          } else if (strcmp(in_buffer, "02") == 0) {
            Serial.println("Select test_seq_02");
            test_seq[selected].test = test_seq_02;
            test_seq[selected].state = 0;
          } else if (strcmp(in_buffer, "03") == 0) {
            Serial.println("Select test_seq_03");
            test_seq[selected].test = test_seq_03;
            test_seq[selected].state = 0;
          } else if (strcmp(in_buffer, "04") == 0) {
            Serial.println("Select test_seq_04");
            test_seq[selected].test = test_seq_04;
            test_seq[selected].state = 0;
          }
        }
      }
      in_ptr = 0;
    } else {
      in_buffer[in_ptr++] = ch;
    }
  }

  if (test_mode) {
    if (test_ongoing) {
      bool finished = true;
      uint32_t ms = millis();
      for (uint8_t i = 0; i < MAX_STEPPER; i++) {
        if (test_seq[i].test != NULL) {
          finished &= test_seq[i].test(stepper[i], &test_seq[i], ms);
        }
      }
      if (finished) {
        test_ongoing = false;
        Serial.println("finished");
        stepper_info();
      }
    }
  } else {
    bool running = false;
    for (uint8_t i = 0; i < MAX_STEPPER; i++) {
      if (stepper[i]) {
        running |= stepper[i]->isRunning();
      }
    }
    if (running) {
      uint32_t now = millis();
      if (now - last_time >= 100) {
        if (verbose) {
          output_info();
        }
        last_time = now;
      }
    }
    if (!stopped && !running) {
      output_info();
      if (usage_info) {
        usage();
      }
    }
    stopped = !running;
  }
}
