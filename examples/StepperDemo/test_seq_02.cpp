#include "test_seq.h"

// u32_1 shall be number of steps

bool test_seq_02(FastAccelStepper *stepper, struct test_seq_s *seq, uint32_t time_ms) {
	int32_t steps = seq->u32_1;
	switch (seq->state & 15) {
		case 0: // INIT
			stepper->setSpeed(30);
			stepper->setAcceleration(1000000);
			seq->u32_1 = 1;
			seq->state++;
			break;
		case 1:
		case 5:
			// Turn 
			stepper->move(steps);
			seq->state = 2;
			break;
		case 3:
			// Turn back
			stepper->move(-steps);
			seq->u32_1 += 5;
			seq->state++;
			break;
		case 2:
		case 4:
			if (!stepper->isRunning()) {
				if (seq->u32_1 >= 3200) {
				return true; // finished
				}
			}
			seq->state++;
			break;
	}
	return false;
}
