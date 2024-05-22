#ifndef TEST_PROBE_H
#define TEST_PROBE_H
//#define TEST_MODE
// test mode in rmt:
//   code to investigate rmt module functioning

//#define ESP32_TEST_PROBE
//#define ESP32C3_TEST_PROBE

// in rmt:
//   TEST_PROBE_1  on startQueue and queue stop, with double toggle at
//   startQueue TEST_PROBE_2  end interrupt, when rmt transmission hits buffer
//   end TEST_PROBE_3  threshold interrupt, after first buffer half transmission
//   is complete

#ifdef ESP32_TEST_PROBE
#define TEST_PROBE_1 18
#define TEST_PROBE_2 5
#define TEST_PROBE_3 4
#endif

#ifdef ESP32C3_TEST_PROBE
#define TEST_PROBE_1 1
#define TEST_PROBE_2 2
#define TEST_PROBE_3 3
#endif

#ifdef TEST_PROBE_1
#define PROBE_1(x) digitalWrite(TEST_PROBE_1, x)
#define PROBE_1_TOGGLE           \
  pinMode(TEST_PROBE_1, OUTPUT); \
  digitalWrite(TEST_PROBE_1, digitalRead(TEST_PROBE_1) == HIGH ? LOW : HIGH)
#else
#define PROBE_1(x)
#define PROBE_1_TOGGLE
#endif
#ifdef TEST_PROBE_2
#define PROBE_2(x) digitalWrite(TEST_PROBE_2, x)
#define PROBE_2_TOGGLE           \
  pinMode(TEST_PROBE_2, OUTPUT); \
  digitalWrite(TEST_PROBE_2, digitalRead(TEST_PROBE_2) == HIGH ? LOW : HIGH)
#else
#define PROBE_2(x)
#define PROBE_2_TOGGLE
#endif
#ifdef TEST_PROBE_3
#define PROBE_3(x) digitalWrite(TEST_PROBE_3, x)
#define PROBE_3_TOGGLE           \
  pinMode(TEST_PROBE_3, OUTPUT); \
  digitalWrite(TEST_PROBE_3, digitalRead(TEST_PROBE_3) == HIGH ? LOW : HIGH)
#else
#define PROBE_3(x)
#define PROBE_3_TOGGLE
#endif

#endif
