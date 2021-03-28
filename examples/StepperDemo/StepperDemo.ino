#include "AVRStepperPins.h"
#include "FastAccelStepper.h"
#include "test_seq.h"

#ifdef SIM_TEST_INPUT
#include <avr/sleep.h>
#endif
#if defined(ARDUINO_ARCH_ESP32)
#include <esp_task_wdt.h>
#endif

#define VERSION "post-c58d3f7"

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
#if defined(__AVR_ATmega328P__)
// Example hardware configuration for Arduino Nano
// Please adapt to your configuration
const uint8_t led_pin = 13;  // turn off with PIN_UNDEFINED
const struct stepper_config_s stepper_config[MAX_STEPPER] = {
    {
      // stepper 1 shall be connected to OC1A
      step : stepPinStepper1A,
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
      step : stepPinStepper1B,
      enable_low_active : 8,
      enable_high_active : PIN_UNDEFINED,
      direction : 7,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    }};
#elif defined(__AVR_ATmega2560__)
// Example hardware configuration for Arduino ATmega2560
// Please adapt to your configuration
const uint8_t led_pin = PIN_UNDEFINED;  // turn off with PIN_UNDEFINED
const struct stepper_config_s stepper_config[MAX_STEPPER] = {
    {
      step : stepPinStepperA,
      enable_low_active : 19,
      enable_high_active : PIN_UNDEFINED,
      direction : 21,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 500000,
      off_delay_ms : 5000
    },
    {
      step : stepPinStepperB,
      enable_low_active : 18,
      enable_high_active : PIN_UNDEFINED,
      direction : 20,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    },
    {
      // stepper 3 shall be connected to OC4C
      step : stepPinStepperC,
      enable_low_active : 43,
      enable_high_active : PIN_UNDEFINED,
      direction : 42,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    }};
#endif
#elif defined(ARDUINO_ARCH_ESP32)
// Example hardware configuration for esp32 board.
// Please adapt to your configuration
const uint8_t led_pin = PIN_UNDEFINED;
const struct stepper_config_s stepper_config[MAX_STEPPER] = {
    // Test-HW
    // Position 01 linked to atmega nano
    // 2: Enable Left Pin 13 GPIO13   , DIR Right Pin 7 GPIO18,    Step Right
    // Pin 13 GPIO15
    // 3: Enable Left Pin 12 GPIO12   , DIR Right Pin 6 GPIO19,    Step Right
    // Pin 12 GPIO2  blue LED
    // 4: Enable Left Pin 11 GPIO14   , DIR Right Pin 5 GPIO21,    Step Right
    // Pin 11 GPIO4
    // 5: Enable Left Pin 10 GPIO27   , DIR Right Pin 4 GPIO3 RX0, Step Right
    // Pin 10 GPIO16 RX2
    // 6: Enable Left Pin 9  GPIO26 A9, DIR Right Pin 3 GPIO1 TX0, Step Right
    // Pin 9  GPIO17 TX2
    // 7: Enable Left Pin 8  GPIO25 A8, DIR Right Pin 2 GPIO22,    Step Right
    // Pin 8  GPIO5
    //                          ALL Enable: Right Pin 1 GPIO23
    // Left Pin 15: +5V
    {
      // position 01.234567 => 2
      step : 17,
      enable_low_active : 26,
      enable_high_active : PIN_UNDEFINED,
      direction : 18,  // was GPIO 1 in conflict with TXD, via wire to Dir of
                       // next stepper
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 50,
      off_delay_ms : 1000
    },
    {
      // position 01.234567 => 3
      step : 15,
      enable_low_active : 13,
      enable_high_active : PIN_UNDEFINED,
      direction : 18,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 500,
      off_delay_ms : 1000
    },
    {
      // position 01.234567 => 4, step is linked to blue LED
      step : 2,
      enable_low_active : 12,
      enable_high_active : PIN_UNDEFINED,
      direction : 19,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 500,
      off_delay_ms : 1000
    },
    {
      // position 01.234567 => 5
      step : 5,
      enable_low_active : 25,
      enable_high_active : PIN_UNDEFINED,
      direction : 22,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    },
    {
      // position 01.234567 => 6
      step : 16,
      enable_low_active : 27,
      enable_high_active : PIN_UNDEFINED,
      direction : 21,  // was GPIO 3 in conflict with RXD, via wire to GPIO21
                       // (Dir next stepper)
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    },
    {
      // position 01.234567 => 7
      step : 4,
      enable_low_active : 14,
      enable_high_active : PIN_UNDEFINED,
      direction : 21,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    }};
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

const static char messages[] PROGMEM =
#define MSG_SELECT_STEPPER 0
    "Select stepper |"
#define MSG_TOGGLE_MOTOR_INFO 1
    "Toggle motor info\n|"
#define MSG_TOGGLE_USAGE_INFO 2
    "Toggle usage info\n|"
#define MSG_ENTER_TEST_MODE 3
    "Enter test mode\n|"
#define MSG_SET_ACCELERATION_TO 4
    "Set acceleration to |"
#define MSG_SET_SPEED_TO_US 5
    "Set speed (us/step) to |"
#define MSG_MOVE_STEPS 6
    "Move steps |"
#define MSG_MOVE_TO_POSITION 7
    "Move to position |"
#define MSG_RETURN_CODE 8
    "returncode = |"
#define MSG_SET_POSITION 9
    "set position |"
#define MSG_SET_ENABLE_TIME 10
    "set enable time to |"
#define MSG_SET_DISABLE_TIME 11
    "set disable time to |"
#define MSG_OUTPUT_DRIVER_ON 12
    "Output driver on\n|"
#define MSG_OUTPUT_DRIVER_OFF 13
    "Output driver off\n|"
#define MSG_OUTPUT_DRIVER_AUTO 14
    "Output driver automatic mode\n|"
#define MSG_STOP 15
    "Stop\n|"
#define MSG_KEEP_RUNNING 16
    "Keep running\n|"
#define MSG_RUN_FORWARD 17
    "Run forward\n|"
#define MSG_RUN_BACKWARD 18
    "Run backward\n|"
#define MSG_IMMEDIATE_STOP 19
    "Immediate Stop\n|"
#define MSG_UPDATE_SPEED_ACCELERATION 20
    "Update speed/acceleration\n|"
#define MSG_BLOCKING_WAIT 21
    "Blocking wait for running stepper to stop\n|"
#define MSG_TEST_DIRECT_DRIVE 22
    "Test direct drive\n|"
#define MSG_STEPPED_FORWARD 23
    "Stepped forward\n|"
#define MSG_STEPPED_BACKWARD 24
    "Stepped backward\n|"
#define MSG_WAIT_MS 25
    " ms wait\n|"
#define MSG_SELECT_TEST_SEQUENCE 26
    "Select test_seq: |"
#define MSG_EXIT_TO_MAIN_MENU 27
    "Exit to main menu\n|"
#define MSG_RUN_TESTS 28
    "Run tests\n|"
#define MSG_WAIT_COMPLETE 29
    "Wait complete\n|"
#define MSG_FAILED_STATUS 30
    "Failed status from test\n|"
#define MSG_ENABLE_LOW_PIN_IS_NOT_LOW 31
    "Cannot set enable low pin to LOW\n|"
#define MSG_ENABLE_LOW_PIN_IS_NOT_HIGH 32
    "Cannot set enable low pin to HIGH\n|"
#define MSG_ENABLE_HIGH_PIN_IS_NOT_LOW 33
    "Cannot set enable high pin to LOW\n|"
#define MSG_ENABLE_HIGH_PIN_IS_NOT_HIGH 34
    "Cannot set enable high pin to HIGH\n|"
#define MSG_STEP_PIN_IS_NOT_LOW 35
    "Cannot set step pin to LOW\n|"
#define MSG_STEP_PIN_IS_NOT_HIGH 36
    "Cannot set step pin to HIGH\n|"
#define MSG_CANNOT_SET_DIRECTION_PIN 37
    "Cannot set direction pin to |"
#define MSG_STEPPER_VERSION 38
    "StepperDemo Version " VERSION
    "\n|"
#define MSG_ATTACH_PULSE_COUNTER 39
    "Attach pulse counter |"
#define MSG_ERROR_ATTACH_PULSE_COUNTER 40
    "ERROR attaching pulse counter\n|"
#define MSG_ERROR_INVALID_VALUE 41
    "ERROR invalid value\n|"
#define MSG_ERROR_MOVE_ERR_ACCELERATION_IS_UNDEFINED__MINUS_3 42
    "ERROR acceleration is not defined\n|"
#define MSG_ERROR_MOVE_ERR_SPEED_IS_UNDEFINED__MINUS_2 43
    "ERROR speed is not defined\n|"
#define MSG_ERROR_MOVE_ERR_NO_DIRECTION_PIN__MINUS_1 44
    "ERROR no direction Pin => impossible move\n|"
#define MSG_MOVE_OK 45
    "OK\n|"
#define MSG_STRAY_DIGITAL_READ_TOGGLE 46
    "Toggle erroneous digitalRead() to step pin\n|"
#define MSG_STRAY_DIGITAL_READ_ENABLED 47
    "ERRONEOUS digitalRead() TO STEP PIN IS ON !!!\n|"
#define MSG_LONG_INTERRUPT_BLOCK_TOGGLE 48
    "Toggle erroneous 100 µs ISR block\n|"
#define MSG_LONG_INTERRUPT_BLOCK_ENABLED 49
    "ERRONEOUS 100 µs ISR BLOCK IS ON\n|"
#define MSG_SET_UNIDIRECTIONAL_STEPPER 50
    "Set unidirectional stepper\n|"
#define MSG_CLEAR_PULSE_COUNTER 51
    "Clear pulse counter\n|"
#define MSG_SET_SPEED_TO_HZ 52
    "Set speed (steps/s) to |"
#define MSG_PASS_STATUS 53
    "Test passed\n|"
#define MSG_TEST_COMPLETED 54
    "Test completed\n|";

void output_msg(int8_t i) {
  char ch;
#if defined(ARDUINO_ARCH_AVR)
  PGM_P p = messages;
  while (i >= 0) {
    ch = pgm_read_byte(p++);
    if (ch == '|') {
      i--;
    } else if (i == 0) {
      Serial.print(ch);
    }
  }
#elif defined(ARDUINO_ARCH_ESP32)
  const char *p = messages;
  while (i >= 0) {
    ch = *p++;
    if (ch == '|') {
      i--;
    } else if (i == 0) {
      Serial.print(ch);
    }
  }
#endif
}

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
    delayMicroseconds(10);
    if (digitalRead(enableLow) != LOW) {
      output_msg(MSG_ENABLE_LOW_PIN_IS_NOT_LOW);
    }
  }
  if (enableHigh != PIN_UNDEFINED) {
    digitalWrite(enableHigh, HIGH);
    pinMode(enableHigh, OUTPUT);
    delayMicroseconds(10);
    if (digitalRead(enableHigh) != HIGH) {
      output_msg(MSG_ENABLE_HIGH_PIN_IS_NOT_HIGH);
    }
  }
  if (direction != PIN_UNDEFINED) {
    digitalWrite(direction, direction_high_count_up);
    pinMode(direction, OUTPUT);
    delayMicroseconds(10);
    if (digitalRead(direction) != direction_high_count_up) {
      output_msg(MSG_CANNOT_SET_DIRECTION_PIN);
      Serial.println(direction_high_count_up ? "HIGH" : "LOW");
    }
  }
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(step, HIGH);
    delayMicroseconds(10);
    if (digitalRead(step) != HIGH) {
      output_msg(MSG_STEP_PIN_IS_NOT_HIGH);
    }
    digitalWrite(step, LOW);
    delayMicroseconds(190);
    if (digitalRead(step) != LOW) {
      output_msg(MSG_STEP_PIN_IS_NOT_LOW);
    }
  }
  if (direction != PIN_UNDEFINED) {
    digitalWrite(direction, !direction_high_count_up);
    delayMicroseconds(10);
    delayMicroseconds(10);
    if (digitalRead(direction) != !direction_high_count_up) {
      output_msg(MSG_CANNOT_SET_DIRECTION_PIN);
      Serial.println(!direction_high_count_up ? "HIGH" : "LOW");
    }
  }
  for (uint16_t i = 0; i < 3200; i++) {
    digitalWrite(step, HIGH);
    delayMicroseconds(10);
    if (digitalRead(step) != HIGH) {
      output_msg(MSG_STEP_PIN_IS_NOT_HIGH);
    }
    digitalWrite(step, LOW);
    delayMicroseconds(190);
    if (digitalRead(step) != LOW) {
      output_msg(MSG_STEP_PIN_IS_NOT_LOW);
    }
  }
  if (enableLow != PIN_UNDEFINED) {
    digitalWrite(enableLow, HIGH);
    delayMicroseconds(10);
    if (digitalRead(enableLow) != HIGH) {
      output_msg(MSG_ENABLE_LOW_PIN_IS_NOT_HIGH);
    }
  }
  if (enableHigh != PIN_UNDEFINED) {
    digitalWrite(enableHigh, LOW);
    delayMicroseconds(10);
    if (digitalRead(enableHigh) != LOW) {
      output_msg(MSG_ENABLE_HIGH_PIN_IS_NOT_LOW);
    }
  }
  // Done
}

void setup() {
  Serial.begin(115200);
  output_msg(MSG_STEPPER_VERSION);
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

#ifdef SIM_TEST_INPUT
const char *input = SIM_TEST_INPUT
    " ";  // final space is too easy forgotten in platformio.ini test
#else
const char *input = NULL;
#endif
uint8_t write_ptr = 0;
uint8_t read_ptr = 0;
char in_buffer[256];  // This allows to let in/out_ptr wrap around
uint8_t out_ptr = 0;
char out_buffer[256];
bool stopped = true;
bool verbose = true;
bool usage_info = true;
bool speed_in_milli_hz = false;
uint32_t last_time = 0;
int selected = -1;
uint32_t pause_ms = 0;
uint32_t pause_start = 0;
#if defined(ARDUINO_ARCH_AVR)
bool simulate_digitalRead_error = false;
#endif
bool simulate_blocked_ISR = false;

void info(FastAccelStepper *s, bool long_info) {
  Serial.print('@');
#if defined(ARDUINO_ARCH_ESP32)
  if (s->pulseCounterAttached()) {
    int16_t pcnt_pos_1 = s->readPulseCounter();
    int32_t pos = s->getCurrentPosition();
    int16_t pcnt_pos_2 = s->readPulseCounter();
    Serial.print(pos);
    Serial.print(" [");
    Serial.print(pcnt_pos_1);
    Serial.print(']');
    if (pcnt_pos_1 != pcnt_pos_2) {
      Serial.print(" [");
      Serial.print(pcnt_pos_2);
      Serial.print(']');
    }
  } else {
    int32_t pos = s->getCurrentPosition();
    Serial.print(pos);
  }
#else
  int32_t pos = s->getCurrentPosition();
  Serial.print(pos);
#endif
  if (s->isRunning()) {
    if (s->isRunningContinuously()) {
      Serial.print(" nonstop");
    } else {
      Serial.print(" => ");
      Serial.print(s->targetPos());
    }
    Serial.print(" QueueEnd=");
    Serial.print(s->getPositionAfterCommandsCompleted());
    if (speed_in_milli_hz) {
      Serial.print(" v=");
      Serial.print(s->getCurrentSpeedInMilliHz());
      Serial.print("mSteps/s");
    } else {
      Serial.print('/');
      Serial.print(s->getPeriodInUsAfterCommandsCompleted());
      Serial.print("us/");
      Serial.print(s->getPeriodInTicksAfterCommandsCompleted());
      Serial.print("ticks");
    }
    if (s->isRampGeneratorActive()) {
      switch (s->rampState() & RAMP_STATE_MASK) {
        case RAMP_STATE_IDLE:
          Serial.print(" IDLE ");
          break;
        case RAMP_STATE_ACCELERATE:
          Serial.print(" ACC  ");
          break;
        case RAMP_STATE_DECELERATE_TO_STOP:
          Serial.print(" DEC  ");
          break;
        case RAMP_STATE_DECELERATE:
          Serial.print(" RED  ");  // Reduce
          break;
        case RAMP_STATE_COAST:
          Serial.print(" COAST");
          break;
        case RAMP_STATE_REVERSE:
          Serial.print(" REV  ");
          break;
        default:
          Serial.print(s->rampState());
      }
    } else {
      Serial.print(" MANU");
    }
  } else {
    if (long_info) {
      Serial.print(" Acceleration [Steps/s^2]=");
      Serial.print(s->getAcceleration());
      if (speed_in_milli_hz) {
        Serial.print(" Speed [mStep/s]=");
        Serial.print(s->getSpeedInMilliHz());
      } else {
        Serial.print(" Speed [us/step]=");
        Serial.print(s->getSpeedInUs());
      }
    }
  }
  Serial.print(' ');
}

const static char usage_str[] PROGMEM =
    "Enter commands separated by space, carriage return or newline:\n"
    "     M1/M2/..  ... to select stepper\n"
    "     V<speed>  ... Set selected stepper's speed in us/step\n"
    "     H<speed>  ... Set selected stepper's speed in steps/s\n"
    "     A<accel>  ... Set selected stepper's acceleration\n"
    "     a<accel>  ... Acceleration control with +/-acceleration values\n"
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
    "     w<ms>     ... Wait time in ms\n"
    "     +         ... Perform one step forward of the selected motor\n"
    "     -         ... Perform one step backward of the selected motor\n"
    "     T         ... Test selected motor with direct port access\n"
#if defined(ARDUINO_ARCH_ESP32)
    "     r         ... Call ESP.restart()\n"
    "     reset     ... Perform reset\n"
    "     p<n>      ... Attach pulse counter n<=7\n"
    "     p<n>,l,h  ... Attach pulse counter n<=7 with low and high limits\n"
    "     pc        ... Clear pulse counter\n"
#endif
    "     t         ... Enter test mode\n"
    "     u         ... Unidirectional mode (need reset to restore)\n"
#if defined(ARDUINO_ARCH_AVR)
    "     r         ... Toggle erroneous digitalRead() of stepper pin\n"
#endif
    "     e         ... Toggle erroneous long 100us interrupt block\n"
    "     Q         ... Toggle print usage on motor stop\n"
    "     ?         ... Print this usage\n"
    "\n";

const static char test_usage_str[] PROGMEM =
    "Enter commands separated by space, carriage return or newline:\n"
    "     M1/M2/..  ... to select stepper\n"
    "     R         ... start all selected tests\n"
    "     I         ... Toggle motor info, while test sequence is running\n"
    "     01        ... select test sequence 01 for selected stepper\n"
    "     :\n"
    "     11        ... select test sequence 11 for selected stepper\n"
#ifdef SIM_TEST_INPUT
    "     W         ... Blocking wait until test is finished\n"
#endif
#if defined(ARDUINO_ARCH_ESP32)
    "     r         ... Call ESP.restart()\n"
    "     x         ... Exit test mode\n"
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
      Serial.print('M');
      Serial.print(i + 1);
      Serial.print(": ");
      info(stepper[i], true);
      Serial.println();
    }
  }
  if (simulate_blocked_ISR) {
    output_msg(MSG_LONG_INTERRUPT_BLOCK_ENABLED);
  }
#if defined(ARDUINO_ARCH_AVR)
  if (simulate_digitalRead_error) {
    output_msg(MSG_STRAY_DIGITAL_READ_ENABLED);
  }
#endif
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

void output_info(bool only_running) {
  bool need_ln = false;
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    if (stepper[i]) {
      if (!only_running || stepper[i]->isRunning()) {
        need_ln = true;
        Serial.print('M');
        Serial.print(i + 1);
        Serial.print(": ");
        info(stepper[i], false);
      }
    }
  }
  if (need_ln) {
    Serial.println();
  }
}

void loop() {
  char ch = 0;
  if (input == NULL) {
    if (Serial.available()) {
      ch = Serial.read();
    }
  } else {
    ch = *input;
    if (ch == 0) {
      if (read_ptr == write_ptr) {
        input = NULL;
#ifdef SIM_TEST_INPUT
        delay(1000);
        noInterrupts();
        sleep_cpu();
#endif
      }
    } else {
      input++;
    }
  }

  if (ch != 0) {
    if ((uint8_t)(write_ptr + 1) == read_ptr) {
      read_ptr++;
    }
    in_buffer[write_ptr++] = ch;
  }
  if (pause_ms > 0) {
    if ((uint32_t)(millis() - pause_start) >= pause_ms) {
      pause_ms = 0;
      output_msg(MSG_WAIT_COMPLETE);
    }
  } else if (read_ptr != write_ptr) {
    ch = in_buffer[read_ptr++];
    if ((ch != ' ') && (ch != '\n') && (ch != '\r')) {
      if (out_ptr == 255) {
        out_ptr = 0;
      }
      out_buffer[out_ptr++] = ch;
    } else if ((ch == ' ') || (ch == '\n') || (ch == '\r')) {
      long val;
#if defined(ARDUINO_ARCH_ESP32)
      long val2, val3;
#endif
      out_buffer[out_ptr] = 0;
      if ((strcmp(out_buffer, "M1") == 0) && stepper[0]) {
        output_msg(MSG_SELECT_STEPPER);
        Serial.println(1);
        selected = 0;
      } else if ((strcmp(out_buffer, "M2") == 0) && stepper[1]) {
        output_msg(MSG_SELECT_STEPPER);
        Serial.println(2);
        selected = 1;
      } else if ((strcmp(out_buffer, "M3") == 0) && stepper[2]) {
        output_msg(MSG_SELECT_STEPPER);
        Serial.println(3);
        selected = 2;
      } else if ((strcmp(out_buffer, "M4") == 0) && stepper[3]) {
        output_msg(MSG_SELECT_STEPPER);
        Serial.println(4);
        selected = 3;
      } else if ((strcmp(out_buffer, "M5") == 0) && stepper[4]) {
        output_msg(MSG_SELECT_STEPPER);
        Serial.println(5);
        selected = 4;
      } else if ((strcmp(out_buffer, "M6") == 0) && stepper[5]) {
        output_msg(MSG_SELECT_STEPPER);
        Serial.println(6);
        selected = 5;
      }
#if defined(ARDUINO_ARCH_ESP32)
      else if (strcmp(out_buffer, "r") == 0) {
        Serial.println("ESP restart");
        ESP.restart();
      } else if (strcmp(out_buffer, "reset") == 0) {
        Serial.println("ESP reset");
        esp_task_wdt_init(1, true);
        esp_task_wdt_add(NULL);
        while (true)
          ;
      }
#endif
#if defined(ARDUINO_ARCH_AVR)
      else if (strcmp(out_buffer, "r") == 0) {
        output_msg(MSG_STRAY_DIGITAL_READ_TOGGLE);
        simulate_digitalRead_error ^= true;
      }
#endif
      else if (strcmp(out_buffer, "e") == 0) {
        output_msg(MSG_LONG_INTERRUPT_BLOCK_TOGGLE);
        simulate_blocked_ISR ^= true;
      } else if (strcmp(out_buffer, "I") == 0) {
        output_msg(MSG_TOGGLE_MOTOR_INFO);
        verbose = !verbose;
      } else if (strcmp(out_buffer, "Q") == 0) {
        output_msg(MSG_TOGGLE_USAGE_INFO);
        usage_info = !usage_info;
      } else if (strcmp(out_buffer, "?") == 0) {
        usage();
      } else if (strcmp(out_buffer, "t") == 0) {
        output_msg(MSG_ENTER_TEST_MODE);
        test_mode = true;
        usage();
      } else if (selected >= 0) {
        FastAccelStepper *stepper_selected = stepper[selected];
        if (!test_mode) {
          if (sscanf(out_buffer, "A%lu", &val) == 1) {
            output_msg(MSG_SET_ACCELERATION_TO);
            Serial.println(val);
            int8_t res = stepper_selected->setAcceleration(val);
            if (res < 0) {
              output_msg(MSG_ERROR_INVALID_VALUE);
            }
          } else if (sscanf(out_buffer, "V%lu", &val) == 1) {
            speed_in_milli_hz = false;
            output_msg(MSG_SET_SPEED_TO_US);
            Serial.println(val);
            int8_t res = stepper_selected->setSpeedInUs(val);
            if (res < 0) {
              output_msg(MSG_ERROR_INVALID_VALUE);
            }
          } else if (sscanf(out_buffer, "H%lu", &val) == 1) {
            speed_in_milli_hz = true;
            output_msg(MSG_SET_SPEED_TO_HZ);
            Serial.println(val);
            int8_t res = stepper_selected->setSpeedInHz(val);
            if (res < 0) {
              output_msg(MSG_ERROR_INVALID_VALUE);
            }
          } else if (sscanf(out_buffer, "a%ld", &val) == 1) {
            output_msg(MSG_SET_ACCELERATION_TO);
            Serial.println(val);
            int8_t res = stepper_selected->moveByAcceleration(val);
            output_msg(MSG_MOVE_OK + res);
          } else if (sscanf(out_buffer, "R%ld", &val) == 1) {
            output_msg(MSG_MOVE_STEPS);
            Serial.println(val);
            int8_t res = stepper_selected->move(val);
            output_msg(MSG_MOVE_OK + res);
          } else if (sscanf(out_buffer, "P%ld", &val) == 1) {
            output_msg(MSG_MOVE_TO_POSITION);
            Serial.println(val);
            int8_t res = stepper_selected->moveTo(val);
            output_msg(MSG_MOVE_OK + res);
          } else if (sscanf(out_buffer, "@%ld", &val) == 1) {
            output_msg(MSG_SET_POSITION);
            Serial.println(val);
            stepper_selected->setCurrentPosition(val);
          } else if (sscanf(out_buffer, "E%lu", &val) == 1) {
            output_msg(MSG_SET_ENABLE_TIME);
            Serial.println(val);
            int res = stepper_selected->setDelayToEnable(val);
            output_msg(MSG_RETURN_CODE);
            Serial.println(res);
          } else if (sscanf(out_buffer, "D%lu", &val) == 1) {
            output_msg(MSG_SET_DISABLE_TIME);
            Serial.println(val);
            stepper_selected->setDelayToDisable(val);
          } else if (strcmp(out_buffer, "N") == 0) {
            output_msg(MSG_OUTPUT_DRIVER_ON);
            stepper_selected->setAutoEnable(false);
            stepper_selected->enableOutputs();
          } else if (strcmp(out_buffer, "F") == 0) {
            output_msg(MSG_OUTPUT_DRIVER_OFF);
            stepper_selected->setAutoEnable(false);
            stepper_selected->disableOutputs();
          } else if (strcmp(out_buffer, "O") == 0) {
            output_msg(MSG_OUTPUT_DRIVER_AUTO);
            stepper_selected->setAutoEnable(true);
          } else if (strcmp(out_buffer, "S") == 0) {
            output_msg(MSG_STOP);
            stepper_selected->stopMove();
          } else if (strcmp(out_buffer, "K") == 0) {
            output_msg(MSG_KEEP_RUNNING);
            stepper_selected->keepRunning();
          } else if (strcmp(out_buffer, "f") == 0) {
            output_msg(MSG_RUN_FORWARD);
            int res = stepper_selected->runForward();
            output_msg(MSG_RETURN_CODE);
            Serial.println(res);
          } else if (strcmp(out_buffer, "b") == 0) {
            output_msg(MSG_RUN_BACKWARD);
            int res = stepper_selected->runBackward();
            output_msg(MSG_RETURN_CODE);
            Serial.println(res);
          } else if (strcmp(out_buffer, "X") == 0) {
            output_msg(MSG_IMMEDIATE_STOP);
            stepper_selected->forceStopAndNewPosition(0);
          } else if (strcmp(out_buffer, "U") == 0) {
            output_msg(MSG_UPDATE_SPEED_ACCELERATION);
            stepper_selected->applySpeedAcceleration();
          } else if (strcmp(out_buffer, "u") == 0) {
            output_msg(MSG_SET_UNIDIRECTIONAL_STEPPER);
            stepper_selected->setDirectionPin(PIN_UNDEFINED);
          } else if (strcmp(out_buffer, "W") == 0) {
#ifdef SIM_TEST_INPUT
            if (stepper_selected->isRunning()) {
              read_ptr -= 2;
            }
#else
            output_msg(MSG_BLOCKING_WAIT);
            if (!stepper_selected->isRunningContinuously() ||
                stepper_selected->isStopping()) {
              // Wait for stepper stop
              while (stepper_selected->isRunning()) {
                // do nothing
              }
              Serial.println("STOPPED");
            }
#endif
          } else if (sscanf(out_buffer, "w%lu", &val) == 1) {
            Serial.print(val);
            output_msg(MSG_WAIT_MS);
            pause_ms = val;
            pause_start = millis();
          } else if (strcmp(out_buffer, "T") == 0) {
            if (!stepper_selected->isRunning()) {
              output_msg(MSG_TEST_DIRECT_DRIVE);
              stepper_selected->detachFromPin();
              test_direct_drive(&stepper_config[selected]);
              stepper_selected->reAttachToPin();
            }
          } else if (strcmp(out_buffer, "+") == 0) {
            if (!stepper_selected->isRunning()) {
              stepper_selected->forwardStep(true);
              output_msg(MSG_STEPPED_FORWARD);
            }
          } else if (strcmp(out_buffer, "-") == 0) {
            if (!stepper_selected->isRunning()) {
              stepper_selected->backwardStep(true);
              output_msg(MSG_STEPPED_BACKWARD);
            }
          }
#if defined(ARDUINO_ARCH_ESP32)
          else if (sscanf(out_buffer, "p%lu,%ld,%ld", &val, &val2, &val3) ==
                   3) {
            output_msg(MSG_ATTACH_PULSE_COUNTER);
            Serial.println(val);
            if (!stepper_selected->attachToPulseCounter(val, val2, val3)) {
              output_msg(MSG_ERROR_ATTACH_PULSE_COUNTER);
            }
          } else if (sscanf(out_buffer, "p%lu", &val) == 1) {
            output_msg(MSG_ATTACH_PULSE_COUNTER);
            Serial.println(val);
            if (!stepper_selected->attachToPulseCounter(val)) {
              output_msg(MSG_ERROR_ATTACH_PULSE_COUNTER);
            }
          } else if (strcmp(out_buffer, "pc") == 0) {
            output_msg(MSG_CLEAR_PULSE_COUNTER);
            stepper_selected->clearPulseCounter();
          }
#endif
        } else {
          if (strcmp(out_buffer, "x") == 0) {
            output_msg(MSG_EXIT_TO_MAIN_MENU);
            test_ongoing = false;
            test_mode = false;
          } else if (strcmp(out_buffer, "R") == 0) {
            output_msg(MSG_RUN_TESTS);
            test_ongoing = true;
          } else if (strcmp(out_buffer, "I") == 0) {
            output_msg(MSG_TOGGLE_MOTOR_INFO);
            verbose = !verbose;
          }
#ifdef SIM_TEST_INPUT
          else if (strcmp(out_buffer, "W") == 0) {
            if (test_ongoing) {
              read_ptr -= 2;
            }
          }
#endif
          else if (strcmp(out_buffer, "01") == 0) {
            output_msg(MSG_SELECT_TEST_SEQUENCE);
            Serial.println(out_buffer);
            test_seq[selected].test = test_seq_01;
            test_seq[selected].state = 0;
          } else if (strcmp(out_buffer, "02") == 0) {
            output_msg(MSG_SELECT_TEST_SEQUENCE);
            Serial.println(out_buffer);
            test_seq[selected].test = test_seq_02;
            test_seq[selected].state = 0;
          } else if (strcmp(out_buffer, "03") == 0) {
            output_msg(MSG_SELECT_TEST_SEQUENCE);
            Serial.println(out_buffer);
            test_seq[selected].test = test_seq_03;
            test_seq[selected].state = 0;
          } else if (strcmp(out_buffer, "04") == 0) {
            output_msg(MSG_SELECT_TEST_SEQUENCE);
            Serial.println(out_buffer);
            test_seq[selected].test = test_seq_04;
            test_seq[selected].state = 0;
          } else if (strcmp(out_buffer, "05") == 0) {
            output_msg(MSG_SELECT_TEST_SEQUENCE);
            Serial.println(out_buffer);
            test_seq[selected].test = test_seq_05;
            test_seq[selected].state = 0;
          } else if (strcmp(out_buffer, "06") == 0) {
            output_msg(MSG_SELECT_TEST_SEQUENCE);
            Serial.println(out_buffer);
            test_seq[selected].test = test_seq_06;
            test_seq[selected].state = 0;
          } else if (strcmp(out_buffer, "07") == 0) {
            output_msg(MSG_SELECT_TEST_SEQUENCE);
            Serial.println(out_buffer);
            test_seq[selected].test = test_seq_07;
            test_seq[selected].state = 0;
          } else if (strcmp(out_buffer, "08") == 0) {
            output_msg(MSG_SELECT_TEST_SEQUENCE);
            Serial.println(out_buffer);
            test_seq[selected].test = test_seq_08;
            test_seq[selected].state = 0;
          } else if (strcmp(out_buffer, "09") == 0) {
            output_msg(MSG_SELECT_TEST_SEQUENCE);
            Serial.println(out_buffer);
            test_seq[selected].test = test_seq_09;
            test_seq[selected].state = 0;
          } else if (strcmp(out_buffer, "10") == 0) {
            output_msg(MSG_SELECT_TEST_SEQUENCE);
            Serial.println(out_buffer);
            test_seq[selected].test = test_seq_10;
            test_seq[selected].state = 0;
          } else if (strcmp(out_buffer, "11") == 0) {
            output_msg(MSG_SELECT_TEST_SEQUENCE);
            Serial.println(out_buffer);
            test_seq[selected].test = test_seq_11;
            test_seq[selected].state = 0;
          }
        }
      }
      out_ptr = 0;
    }
  }

  uint32_t ms = millis();

#if defined(ARDUINO_ARCH_AVR)
  if (simulate_digitalRead_error) {
    digitalRead(stepPinStepperA);
    digitalRead(stepPinStepperB);
#ifdef stepPinStepperC
    digitalRead(stepPinStepperC);
#endif
  }
#endif
  if (simulate_blocked_ISR) {
    if ((ms & 0xff) < 0x40) {
      noInterrupts();
      delayMicroseconds(100);
      interrupts();
    }
  }

  if (test_mode) {
    if (test_ongoing) {
      bool finished = true;
      for (uint8_t i = 0; i < MAX_STEPPER; i++) {
        struct test_seq_s *s = &test_seq[i];
        if ((s->test != NULL) && (s->state != TEST_STATE_ERROR)) {
          bool res = s->test(stepper[i], &test_seq[i], ms);
          if (res) {
            s->test = NULL;
          }
          finished &= res;
        }
      }
      if (finished) {
        test_ongoing = false;
        bool test_failed = false;
        stepper_info();
        for (uint8_t i = 0; i < MAX_STEPPER; i++) {
          struct test_seq_s *s = &test_seq[i];
          s->test = NULL;
          if (s->state == TEST_STATE_ERROR) {
            test_failed = true;
          }
        }
        if (test_failed) {
          output_msg(MSG_FAILED_STATUS);
        } else {
          output_msg(MSG_PASS_STATUS);
        }
        output_msg(MSG_TEST_COMPLETED);
      } else {
        uint32_t now = millis();
        if (now - last_time >= 100) {
          if (verbose) {
            output_info(true);
          }
          last_time = now;
        }
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
      if (ms - last_time >= 100) {
        if (verbose) {
          output_info(true);
        }
        last_time = ms;
      }
    }
    if (!stopped && !running) {
      output_info(true);
      if (usage_info) {
        usage();
      }
    }
    stopped = !running;
  }
}
