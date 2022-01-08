#include "AVRStepperPins.h"
#include "FastAccelStepper.h"
#include "test_seq.h"

#ifdef SIM_TEST_INPUT
#include <avr/sleep.h>
#endif
#if defined(ARDUINO_ARCH_ESP32)
#include <esp_task_wdt.h>
#endif

// Code Optimization
//   Start   30170 Bytes


#define VERSION "post-c3019eb"

struct stepper_config_s {
  uint8_t step;
  uint8_t enable_low_active;
  uint8_t enable_high_active;
  uint8_t direction;
  uint16_t dir_change_delay;
  bool direction_high_count_up;
  bool auto_enable;
  uint32_t on_delay_us;
  uint16_t off_delay_ms;
};

#if defined(ARDUINO_ARCH_AVR)
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__)
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
      dir_change_delay : 0,
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
      dir_change_delay : 0,
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
      dir_change_delay : 0,
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
      dir_change_delay : 0,
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
      dir_change_delay : 0,
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
      dir_change_delay : 0,
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
      dir_change_delay : 0,
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
      dir_change_delay : 0,
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
      dir_change_delay : 0,
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
      dir_change_delay : 0,
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
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    }};
#elif defined(ARDUINO_ARCH_SAM)
// Hardware configuration copied from esp32 board. Not used on due board
// Please adapt to your configuration
const uint8_t led_pin = PIN_UNDEFINED;
const struct stepper_config_s stepper_config[MAX_STEPPER] = {
    {
      step : 17,
      enable_low_active : 26,
      enable_high_active : PIN_UNDEFINED,
      direction : 18,  // was GPIO 1 in conflict with TXD, via wire to Dir of
                       // next stepper
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 50,
      off_delay_ms : 1000
    },
    {
      step : 15,
      enable_low_active : 13,
      enable_high_active : PIN_UNDEFINED,
      direction : 18,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 500,
      off_delay_ms : 1000
    },
    {
      step : 2,
      enable_low_active : 12,
      enable_high_active : PIN_UNDEFINED,
      direction : 19,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 500,
      off_delay_ms : 1000
    },
    {
      step : 5,
      enable_low_active : 25,
      enable_high_active : PIN_UNDEFINED,
      direction : 22,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    },
    {
      step : 16,
      enable_low_active : 27,
      enable_high_active : PIN_UNDEFINED,
      direction : 21,  // was GPIO 3 in conflict with RXD, via wire to GPIO21
                       // (Dir next stepper)
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    },
    {
      step : 4,
      enable_low_active : 14,
      enable_high_active : PIN_UNDEFINED,
      direction : 21,
      dir_change_delay : 0,
      direction_high_count_up : true,
      auto_enable : true,
      on_delay_us : 5000,
      off_delay_ms : 10
    }};
#endif

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper[MAX_STEPPER];

enum { normal, test, config } mode = normal;
bool test_ongoing = false;
struct test_seq_s test_seq[MAX_STEPPER] = {
#if defined(ARDUINO_ARCH_ESP32)
    {.test = NULL}, {.test = NULL}, {.test = NULL}, {.test = NULL},
#endif
    {.test = NULL}, {.test = NULL}};

#define _NL_ "\n"
#define _SEP_ "|"

const static char messages[] PROGMEM =
#define _Move_ "\200"
    "Move " _SEP_
#define _Output_driver_ "\201"
    "Output driver " _SEP_
#define _Cannot_set_ "\202"
    "Cannot set " _SEP_
#define s_pin_ "\203"
    "pin " _SEP_
#define _step_pin_ "\204"
    "step " s_pin_ "" _SEP_
#define s_to_ "\205"
    "to " _SEP_
#define _LOW_ "\206"
    "LOW " _SEP_
#define _HIGH_ "\207"
    "HIGH " _SEP_
#define _to_LOW_nl "\210"
     s_to_  _LOW_ "" _NL_ _SEP_
#define _to_HIGH_nl "\211"
     s_to_  _HIGH_ "" _NL_ _SEP_
#define _Toggle_ "\212"
    "Toggle " _SEP_
#define _ERROR_ "\213"
    "ERROR " _SEP_
#define _high_counts_ "\214"
    "high counts " _SEP_
#define _enable_ "\215"
    "enable " _SEP_
#define _direction_ "\216"
    "direction " _SEP_
#define _pulse_counter_ "\217"
    "pulse counter " _SEP_
#define _attach "\220"
    "attach" _SEP_
#define s_set_ "\221"
    "set " _SEP_
#define _time_to_ "\222"
    "time" s_to_ _SEP_
#define _is_not_defined_nl "\223"
    "is not defined" _NL_ _SEP_
#define _run_ "\224"
    "run " _SEP_
#define MSG_OFFSET 21
#define MSG_SELECT_STEPPER 0+MSG_OFFSET
    "Select stepper " _SEP_
#define MSG_TOGGLE_MOTOR_INFO 1+MSG_OFFSET
    _Toggle_ "motor info" _NL_ _SEP_
#define MSG_TOGGLE_USAGE_INFO 2+MSG_OFFSET
    _Toggle_ "usage info" _NL_ _SEP_
#define MSG_ENTER_TEST_MODE 3+MSG_OFFSET
    "Enter test mode" _NL_ _SEP_
#define MSG_SET_ACCELERATION_TO 4+MSG_OFFSET
    "Set acceleration to " _SEP_
#define MSG_SET_SPEED_TO_US 5+MSG_OFFSET
    "Set speed (us/step) to " _SEP_
#define MSG_MOVE_STEPS 6+MSG_OFFSET
    _Move_ "steps " _SEP_
#define MSG_MOVE_TO_POSITION 7+MSG_OFFSET
    _Move_ "to position " _SEP_
#define MSG_RETURN_CODE 8+MSG_OFFSET
    "returncode = " _SEP_
#define MSG_SET_POSITION 9+MSG_OFFSET
    s_set_ "position " _SEP_
#define MSG_SET_ENABLE_TIME 10+MSG_OFFSET
    s_set_ _enable_ _time_to_ _SEP_
#define MSG_SET_DISABLE_TIME 11+MSG_OFFSET
    s_set_ "disable " _time_to_ _SEP_
#define MSG_OUTPUT_DRIVER_ON 12+MSG_OFFSET
    _Output_driver_ "on" _NL_ _SEP_
#define MSG_OUTPUT_DRIVER_OFF 13+MSG_OFFSET
    _Output_driver_ "off" _NL_ _SEP_
#define MSG_OUTPUT_DRIVER_AUTO 14+MSG_OFFSET
    _Output_driver_ "automatic mode" _NL_ _SEP_
#define MSG_STOP 15+MSG_OFFSET
    "Stop" _NL_ _SEP_
#define MSG_KEEP_RUNNING 16+MSG_OFFSET
    "Keep running" _NL_ _SEP_
#define MSG_RUN_FORWARD 17+MSG_OFFSET
    _run_ "forward" _NL_ _SEP_
#define MSG_RUN_BACKWARD 18+MSG_OFFSET
    _run_ "backward" _NL_ _SEP_
#define MSG_IMMEDIATE_STOP 19+MSG_OFFSET
    "Immediate Stop" _NL_ _SEP_
#define MSG_UPDATE_SPEED_ACCELERATION 20+MSG_OFFSET
    "Update speed/acceleration" _NL_ _SEP_
#define MSG_BLOCKING_WAIT 21+MSG_OFFSET
    "Blocking wait for running stepper to stop" _NL_ _SEP_
#define MSG_TEST_DIRECT_DRIVE 22+MSG_OFFSET
    "Test direct drive" _NL_ _SEP_
#define MSG_STEPPED_FORWARD 23+MSG_OFFSET
    "Stepped forward" _NL_ _SEP_
#define MSG_STEPPED_BACKWARD 24+MSG_OFFSET
    "Stepped backward" _NL_ _SEP_
#define MSG_WAIT_MS 25+MSG_OFFSET
    " ms wait" _NL_ _SEP_
#define MSG_SELECT_TEST_SEQUENCE 26+MSG_OFFSET
    "Select test_seq: " _SEP_
#define MSG_EXIT_TO_MAIN_MENU 27+MSG_OFFSET
    "Exit to main menu" _NL_ _SEP_
#define MSG_RUN_TESTS 28+MSG_OFFSET
    _run_ "tests" _NL_ _SEP_
#define MSG_WAIT_COMPLETE 29+MSG_OFFSET
    "Wait complete" _NL_ _SEP_
#define MSG_FAILED_STATUS 30+MSG_OFFSET
    "Failed status from test" _NL_ _SEP_
#define MSG_ENABLE_LOW_PIN_IS_NOT_LOW 31+MSG_OFFSET
    _Cannot_set_ _enable_  _LOW_  s_pin_ _to_LOW_nl _SEP_
#define MSG_ENABLE_LOW_PIN_IS_NOT_HIGH 32+MSG_OFFSET
    _Cannot_set_ _enable_  _LOW_  s_pin_ _to_HIGH_nl _SEP_
#define MSG_ENABLE_HIGH_PIN_IS_NOT_LOW 33+MSG_OFFSET
    _Cannot_set_ _enable_  _HIGH_  s_pin_ _to_LOW_nl _SEP_
#define MSG_ENABLE_HIGH_PIN_IS_NOT_HIGH 34+MSG_OFFSET
    _Cannot_set_ _enable_  _HIGH_  s_pin_ _to_HIGH_nl _SEP_
#define MSG_STEP_PIN_IS_NOT_LOW 35+MSG_OFFSET
    _Cannot_set_ _step_pin_ _to_LOW_nl _SEP_
#define MSG_STEP_PIN_IS_NOT_HIGH 36+MSG_OFFSET
    _Cannot_set_ _step_pin_ _to_HIGH_nl _SEP_
#define MSG_CANNOT_SET_DIRECTION_PIN 37+MSG_OFFSET
    _Cannot_set_ _direction_ s_pin_ s_to_ _SEP_
#define MSG_STEPPER_VERSION 38+MSG_OFFSET
    "StepperDemo Version " VERSION
    "" _NL_ _SEP_
#define MSG_ATTACH_PULSE_COUNTER 39+MSG_OFFSET
    _attach " " _pulse_counter_ _SEP_
#define MSG_ERROR_ATTACH_PULSE_COUNTER 40+MSG_OFFSET
    _ERROR_ _attach "ing "  _pulse_counter_ "" _NL_ _SEP_
#define MSG_ERROR_INVALID_VALUE 41+MSG_OFFSET
    _ERROR_ "invalid value" _NL_ _SEP_
#define MSG_ERROR_MOVE_ERR_ACCELERATION_IS_UNDEFINED__MINUS_3 42+MSG_OFFSET
    _ERROR_ "acceleration" _is_not_defined_nl _SEP_
#define MSG_ERROR_MOVE_ERR_SPEED_IS_UNDEFINED__MINUS_2 43+MSG_OFFSET
    _ERROR_ "speed " _is_not_defined_ _SEP_
#define MSG_ERROR_MOVE_ERR_NO_DIRECTION_PIN__MINUS_1 44+MSG_OFFSET
    _ERROR_ "no " _direction_  s_pin_ "=> impossible move" _NL_ _SEP_
#define MSG_MOVE_OK 45+MSG_OFFSET
    "OK" _NL_ _SEP_
#define MSG_STRAY_DIGITAL_READ_TOGGLE 46+MSG_OFFSET
    _Toggle_ "erroneous digitalRead() " s_to_ _step_pin_ "" _NL_ _SEP_
#define MSG_STRAY_DIGITAL_READ_ENABLED 47+MSG_OFFSET
    "ERRONEOUS digitalRead() TO STEP PIN IS ON !!!" _NL_ _SEP_
#define MSG_LONG_INTERRUPT_BLOCK_TOGGLE 48+MSG_OFFSET
    _Toggle_ "erroneous 100 µs ISR block" _NL_ _SEP_
#define MSG_LONG_INTERRUPT_BLOCK_ENABLED 49+MSG_OFFSET
    "ERRONEOUS 100 µs ISR BLOCK IS ON" _NL_ _SEP_
#define MSG_SET_UNIDIRECTIONAL_STEPPER 50+MSG_OFFSET
    _set_ "unidirectional stepper" _NL_ _SEP_
#define MSG_CLEAR_PULSE_COUNTER 51+MSG_OFFSET
    "Clear " _pulse_counter_ "" _NL_ _SEP_
#define MSG_SET_SPEED_TO_HZ 52+MSG_OFFSET
    s_set_ "speed (steps/s) to " _SEP_
#define MSG_PASS_STATUS 53+MSG_OFFSET
    "Test passed" _NL_ _SEP_
#define MSG_TEST_COMPLETED 54+MSG_OFFSET
    "Test completed" _NL_ _SEP_
#define MSG_ENTER_CONFIG_MODE 55+MSG_OFFSET
    "Enter config mode" _NL_ _SEP_
#define MSG_DIRECTION_PIN 56+MSG_OFFSET
    _direction_ s_pin_ _SEP_
#define MSG_SET_TO_PIN 57+MSG_OFFSET
    s_set_  s_to_  s_pin_ _SEP_
#define MSG_DISABLED 58+MSG_OFFSET
    "disabled" _NL_ _SEP_
#define MSG_HIGH_COUNT_UP 59+MSG_OFFSET
    _high_counts_ "up" _NL_ _SEP_
#define MSG_HIGH_COUNT_DOWN 60+MSG_OFFSET
    _high_counts_ "down" _NL_ _SEP_
#define MSG_DELAY 61+MSG_OFFSET
    "delay in us = " _SEP_;

void output_msg(int8_t i) {
  char ch;
#if defined(ARDUINO_ARCH_AVR)
  PGM_P p = messages;
  while (i >= 0) {
    ch = pgm_read_byte(p++);
    if (ch == '|') {
      i--;
    } else if (i == 0) {
	  if (ch >= 128) {
          output_msg(ch - 128);
      }
      else {
		  Serial.print(ch);
	  }
    }
  }
#elif defined(ARDUINO_ARCH_ESP32)
  const char *p = messages;
  while (i >= 0) {
    ch = *p++;
    if (ch == '|') {
      i--;
    } else if (i == 0) {
	  if (ch >= 128) {
          output_msg(ch - 128);
      }
      else {
		  Serial.print(ch);
	  }
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
        s->setDirectionPin(config->direction, config->direction_high_count_up,
                           config->dir_change_delay);
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
int selected = 0;
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
    "     c         ... Enter configuration mode\n"
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
    "     c         ... Enter configuration mode\n"
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
    "     reset     ... Perform reset\n"
#endif
    "     t         ... Enter test mode\n"
    "     Q         ... Toggle print usage on motor stop\n"
    "     ?         ... Print this usage\n"
    "     x         ... Exit test mode\n"
    "\n";

const static char config_usage_str[] PROGMEM =
    "Enter commands separated by space, carriage return or newline:\n"
    "     M1/M2/..  ... to select stepper\n"
    "     c         ... Enter configuration mode\n"
    "     d<p>      ... Set direction pin\n"
    "     d<p,n>\n"
    "     d<p,n,t>\n"
    "                       p ... pin number\n"
    "                       n ... 1: high counts up 0: high counts down\n"
    "                       t ... delay from dir change to step in us, 0 means "
    "off\n"
    "     dc        ... Clear direction pin (unidirectional)\n"
#if defined(ARDUINO_ARCH_ESP32)
    "     r         ... Call ESP.restart()\n"
    "     reset     ... Perform reset\n"
#endif
    "     t         ... Enter test mode\n"
    "     Q         ... Toggle print usage on motor stop\n"
    "     ?         ... Print this usage\n"
    "     x         ... Exit config mode\n"
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
  switch (mode) {
    case normal:
      s = usage_str;
      break;
    case test:
      s = test_usage_str;
      break;
    case config:
      s = config_usage_str;
      break;
  }
  for (;;) {
    ch = pgm_read_byte(s++);
    if (ch == 0) {
      break;
    }
    Serial.print(ch);
  }
#elif defined(ARDUINO_ARCH_ESP32)
  switch (mode) {
    case normal:
      Serial.print(usage_str);
      break;
    case test:
      Serial.print(test_usage_str);
      break;
    case config:
      Serial.print(config_usage_str);
      break;
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

#define MODE(mode,CMD) ((mode<<8)+CMD)

void process_cmd(char *cmd) {
  FastAccelStepper *stepper_selected = stepper[selected];
  uint16_t s = *cmd++;
  char *endptr;
  int8_t res;
  long val1,val2,val3;
  switch(MODE(mode, s)) {
	case MODE(normal, 'M'):
		if ((cmd[1] >= '0') && (cmd[1] <= '0' + MAX_STEPPER)) {
			output_msg(MSG_SELECT_STEPPER);
			selected = cmd[1] - '0';
			Serial.println(selected);
		}
		break;
	case MODE(normal, 'r'):
#if defined(ARDUINO_ARCH_ESP32)
		if (strcmp(cmd, "reset") == 0) {
		    Serial.println("ESP reset");
			esp_task_wdt_init(1, true);
			esp_task_wdt_add(NULL);
		    while (true)
				;
		}
        Serial.println("ESP restart");
        ESP.restart();
#endif
#if defined(ARDUINO_ARCH_AVR)
        output_msg(MSG_STRAY_DIGITAL_READ_TOGGLE);
        simulate_digitalRead_error ^= true;
#endif
		break;
	case MODE(normal, 'I'):
        output_msg(MSG_TOGGLE_MOTOR_INFO);
        verbose = !verbose;
		break;
	case MODE(normal, 'Q'):
        output_msg(MSG_TOGGLE_USAGE_INFO);
        usage_info = !usage_info;
		break;
	case MODE(normal, '?'):
        usage();
		break;
	case MODE(normal, 't'):
        output_msg(MSG_ENTER_TEST_MODE);
        mode = test;
        usage();
		break;
	case MODE(normal, 'c'):
        output_msg(MSG_ENTER_CONFIG_MODE);
        mode = config;
        usage();
		break;
	case MODE(normal, 'e'):
		output_msg(MSG_LONG_INTERRUPT_BLOCK_TOGGLE);
		simulate_blocked_ISR ^= true;
		break;
	case MODE(normal, 'A'):
		val1 = strtol(cmd, &endptr, 10);
		output_msg(MSG_SET_ACCELERATION_TO);
		Serial.println(val1);
		res = stepper_selected->setAcceleration(val1);
		if (res < 0) {
			output_msg(MSG_ERROR_INVALID_VALUE);
		}
		break;
	case MODE(normal, 'V'):
		val1 = strtol(cmd, &endptr, 10);
		speed_in_milli_hz = false;
		output_msg(MSG_SET_SPEED_TO_US);
		Serial.println(val1);
		res = stepper_selected->setSpeedInUs(val1);
		if (res < 0) {
		  output_msg(MSG_ERROR_INVALID_VALUE);
		}
		break;
	case MODE(normal, 'H'):
		val1 = strtol(cmd, &endptr, 10);
              speed_in_milli_hz = true;
              output_msg(MSG_SET_SPEED_TO_HZ);
              Serial.println(val1);
              res = stepper_selected->setSpeedInHz(val1);
              if (res < 0) {
                output_msg(MSG_ERROR_INVALID_VALUE);
              }
		break;
	case MODE(normal, 'a'):
		val1 = strtol(cmd, &endptr, 10);
              output_msg(MSG_SET_ACCELERATION_TO);
              Serial.println(val1);
              res = stepper_selected->moveByAcceleration(val1);
              output_msg(MSG_MOVE_OK + res);
		break;
	case MODE(normal, 'R'):
		val1 = strtol(cmd, &endptr, 10);
              output_msg(MSG_MOVE_STEPS);
              Serial.println(val1);
              res = stepper_selected->move(val1);
              output_msg(MSG_MOVE_OK + res);
		break;
	case MODE(normal, 'P'):
		val1 = strtol(cmd, &endptr, 10);
              output_msg(MSG_MOVE_TO_POSITION);
              Serial.println(val1);
              res = stepper_selected->moveTo(val1);
              output_msg(MSG_MOVE_OK + res);
		break;
	case MODE(normal, '@'):
		val1 = strtol(cmd, &endptr, 10);
              output_msg(MSG_SET_POSITION);
              Serial.println(val1);
              stepper_selected->setCurrentPosition(val1);
		break;
	case MODE(normal, 'E'):
		val1 = strtol(cmd, &endptr, 10);
              output_msg(MSG_SET_ENABLE_TIME);
              Serial.println(val1);
              res = stepper_selected->setDelayToEnable(val1);
              output_msg(MSG_RETURN_CODE);
              Serial.println(res);
		break;
	case MODE(normal, 'D'):
		val1 = strtol(cmd, &endptr, 10);
              output_msg(MSG_SET_DISABLE_TIME);
              Serial.println(val1);
              stepper_selected->setDelayToDisable(val1);
		break;
	case MODE(normal, 'w'):
		val1 = strtol(cmd, &endptr, 10);
              Serial.print(val1);
              output_msg(MSG_WAIT_MS);
              pause_ms = val1;
              pause_start = millis();
		break;
	case MODE(config, 'd'):
		val1 = strtol(cmd, &endptr, 10);
		cmd = endptr;
		if (*cmd == ',') {
			cmd++;
			val2 = strtol(cmd, &endptr, 10);
			cmd = endptr;
			if (*cmd == ',') {
				cmd++;
				val3 = strtol(cmd, &endptr, 10);
				  output_msg(MSG_DIRECTION_PIN);
				  output_msg(MSG_SET_TO_PIN);
				  Serial.println(val1);
				  output_msg(MSG_DIRECTION_PIN);
				  if (val2 != 0) {
					output_msg(MSG_HIGH_COUNT_DOWN);
				  } else {
					output_msg(MSG_HIGH_COUNT_UP);
				  }
				  output_msg(MSG_DELAY);
				  Serial.println(val3);
				  stepper_selected->setDirectionPin(val1, val2, val3);
				}
				else {
              output_msg(MSG_DIRECTION_PIN);
              output_msg(MSG_SET_TO_PIN);
              Serial.println(val1);
              output_msg(MSG_DIRECTION_PIN);
              if (val2 != 0) {
                output_msg(MSG_HIGH_COUNT_DOWN);
              } else {
                output_msg(MSG_HIGH_COUNT_UP);
              }
              stepper_selected->setDirectionPin(val1, val2);
				}
			}
			else {
              output_msg(MSG_DIRECTION_PIN);
              output_msg(MSG_SET_TO_PIN);
              Serial.println(val1);
              stepper_selected->setDirectionPin(val1);
		}
		break;
#if defined(ARDUINO_ARCH_ESP32)
	case MODE(normal, 'p'):
		val1 = strtol(cmd, &endptr, 10);
		cmd = endptr;
		if (*cmd == ',') {
			cmd++;
			val2 = strtol(cmd, &endptr, 10);
			cmd = endptr;
			if (*cmd == ',') {
				cmd++;
				val3 = strtol(cmd, &endptr, 10);
				cmd = endptr;
				if (*cmd == ',') {
					cmd++;
				  output_msg(MSG_ATTACH_PULSE_COUNTER);
				  Serial.println(val1);
				  if (!stepper_selected->attachToPulseCounter(val1, val2, val3)) {
					output_msg(MSG_ERROR_ATTACH_PULSE_COUNTER);
				  }
				}
			}
		}
		break;
#endif
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
      out_buffer[out_ptr] = 0;

      process_cmd(out_buffer);

      if (selected >= 0) {
        FastAccelStepper *stepper_selected = stepper[selected];
        switch (mode) {
          case normal:
            if (strcmp(out_buffer, "N") == 0) {
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
            break;
          case test:
            if (strcmp(out_buffer, "x") == 0) {
              output_msg(MSG_EXIT_TO_MAIN_MENU);
              test_ongoing = false;
              mode = normal;
              usage();
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
            else {
              for (uint8_t i = 0; i < NUM_TEST_SEQUENCE; i++) {
                const struct test_seq_def_s *ts = &test_sequence[i];
                if (strcmp(out_buffer, ts->code) == 0) {
                  output_msg(MSG_SELECT_TEST_SEQUENCE);
                  Serial.println(out_buffer);
                  test_seq[selected].test = ts->test;
                  test_seq[selected].state = 0;
                }
              }
            }
            break;
          case config:
            if (strcmp(out_buffer, "x") == 0) {
              output_msg(MSG_EXIT_TO_MAIN_MENU);
              test_ongoing = false;
              mode = normal;
              usage();
            } else if (strcmp(out_buffer, "dc") == 0) {
              output_msg(MSG_DISABLED);
              stepper_selected->setDirectionPin(PIN_UNDEFINED);
            }
            break;
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

  if (mode == test) {
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
