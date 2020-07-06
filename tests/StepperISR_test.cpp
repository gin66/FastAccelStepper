#include "StepperISR.h"

#include "FastAccelStepper.h"

// Here are the global variables to interface with the interrupts
//StepperQueue fas_queue[NUM_QUEUES];

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
	_initVars();
}
bool StepperQueue::startQueue(struct queue_entry *e) {
	isRunning = true;
	return false;
}
