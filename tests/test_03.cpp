#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "FastAccelStepper.h"
#include "StepperISR.h"

char TCCR1A;
char TCCR1B;
char TCCR1C;
char TIMSK1;
char TIFR1;
unsigned short OCR1A;
unsigned short OCR1B;

uint8_t fas_q_readptr_A = 0;  // ISR stops if readptr == next_writeptr
uint8_t fas_q_next_writeptr_A = 0;
uint8_t fas_q_readptr_B = 0;
uint8_t fas_q_next_writeptr_B = 0;
struct queue_entry fas_queue_A[QUEUE_LEN], fas_queue_B[QUEUE_LEN];

uint8_t fas_autoEnablePin_A = 255;
uint8_t fas_autoEnablePin_B = 255;
uint8_t fas_dirPin_A = 255;
uint8_t fas_dirPin_B = 255;

int main() {

  upm_float x;

  x = upm_from((uint8_t) 1);
  test(x == 0x0080,"conversion error from uint8_t 1");
  x = upm_from((uint8_t) 2);
  test(x == 0x0180,"conversion error from uint8_t 2");
  x = upm_from((uint8_t) 4);
  test(x == 0x0280,"conversion error from uint8_t 4");
  x = upm_from((uint8_t) 8);
  test(x == 0x0380,"conversion error from uint8_t 8");
  x = upm_from((uint8_t) 16);
  test(x == 0x0480,"conversion error from uint8_t 16");
  x = upm_from((uint8_t) 32);
  test(x == 0x0580,"conversion error from uint8_t 32");
  x = upm_from((uint8_t) 64);
  test(x == 0x0680,"conversion error from uint8_t 64");
  x = upm_from((uint8_t) 128);
  test(x == 0x0780,"conversion error from uint8_t 128");

  x = upm_from((uint16_t) 1);
  test(x == 0x0080,"conversion error from uint16_t 1");
  x = upm_from((uint16_t) 256);
  test(x == 0x0880,"conversion error from uint16_t 256");
  x = upm_from((uint16_t) 512);
  test(x == 0x0980,"conversion error from uint16_t 512");
  x = upm_from((uint16_t) 1024);
  test(x == 0x0a80,"conversion error from uint16_t 1024");
  x = upm_from((uint16_t) 2048);
  test(x == 0x0b80,"conversion error from uint16_t 2048");
  x = upm_from((uint16_t) 4096);
  test(x == 0x0c80,"conversion error from uint16_t 4096");
  x = upm_from((uint16_t) 8192);
  test(x == 0x0d80,"conversion error from uint16_t 8192");
  x = upm_from((uint16_t) 16384);
  test(x == 0x0e80,"conversion error from uint16_t 16384");

  x = upm_from((uint32_t) 1);
  test(x == 0x0080,"conversion error from uint32_t 1");

  printf("TEST_03 PASSED\n");
}
