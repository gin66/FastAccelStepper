#include <stdint.h>

#include "PoorManFloat.h"

//
// This file can be renamed to a .ino and compiled as sketch to be run on the
// target e.g. arduino nano.
//

#include <Arduino.h>
#ifdef SIMULATOR
#include <avr/sleep.h>
#endif

uint16_t error_cnt = 0;
char buffer[256];
#define trace(s) Serial.println(s)
#define xprintf(args...)
#define test(x, msg)          \
  if (!(x)) {                 \
    error_cnt++;              \
    Serial.print("ERROR: ");  \
    Serial.println(__LINE__); \
  };

#include "test_03.h"

void setup() {
  Serial.begin(115200);
  Serial.println("Start test...");
  bool result = perform_test();
  if (result) {
    Serial.println("TEST PASSED");
  } else {
    Serial.print("TEST FAILED: ");
    Serial.print(error_cnt);
    Serial.println(" errors");
  }
#ifdef SIMULATOR
  // if result is Ok. Toggle port twice, otherwise once
#define PIN 10
#define DIRPIN 7
  pinMode(DIRPIN, OUTPUT);
  digitalWrite(DIRPIN, HIGH);
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN, HIGH);
  digitalWrite(PIN, LOW);
  if (result) {
    digitalWrite(PIN, HIGH);
    digitalWrite(PIN, LOW);
  }

  delay(1000);
  noInterrupts();
  sleep_cpu();
#endif
}

void loop() {}
