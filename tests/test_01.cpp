#include "FastAccelStepper.h"
#include "StepperISR.h"
#include <stdio.h>
#include <stdlib.h>

char TCCR1A;
char TCCR1B;
char TCCR1C;
char TIMSK1;
char TIFR1;
unsigned short OCR1A;
unsigned short OCR1B;

uint8_t fas_q_readptr_A;   // ISR stops if readptr == next_writeptr
uint8_t fas_q_next_writeptr_A;
uint8_t fas_q_readptr_B;
uint8_t fas_q_next_writeptr_B;
struct queue_entry fas_queue_A[QUEUE_LEN],fas_queue_B[QUEUE_LEN];

uint8_t fas_autoEnablePin_A;
uint8_t fas_autoEnablePin_B;
uint8_t fas_dirPin_A;
uint8_t fas_dirPin_B;


int main() {
   printf("HI\n");
}
