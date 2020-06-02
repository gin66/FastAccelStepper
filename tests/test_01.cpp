#include "FastAccelStepper.h"
#include "StepperISR.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

char TCCR1A;
char TCCR1B;
char TCCR1C;
char TIMSK1;
char TIFR1;
unsigned short OCR1A;
unsigned short OCR1B;

uint8_t fas_q_readptr_A = 0;   // ISR stops if readptr == next_writeptr
uint8_t fas_q_next_writeptr_A = 0;
uint8_t fas_q_readptr_B = 0;
uint8_t fas_q_next_writeptr_B = 0;
struct queue_entry fas_queue_A[QUEUE_LEN],fas_queue_B[QUEUE_LEN];

uint8_t fas_autoEnablePin_A = 255;
uint8_t fas_autoEnablePin_B = 255;
uint8_t fas_dirPin_A = 255;
uint8_t fas_dirPin_B = 255;


int main() {

	FastAccelStepper s = FastAccelStepper(true);
	assert(0 == s.getCurrentPosition());
	assert(s.isQueueEmpty());
	assert(s.isQueueEmpty());
	s.add_queue_entry(1,10000,100,true,0);
	assert(!s.isQueueEmpty());

	printf("TEST PASSED\n");
}
