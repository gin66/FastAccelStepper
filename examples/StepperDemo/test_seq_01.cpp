#include "test_seq.h"

// u32_1 shall be milliseconds from last tick

bool test_seq_01(FastAccelStepper *stepper, struct test_seq_s *seq, uint32_t time_ms) {
	switch (seq->state & 15) {
		case 0: // INIT
			stepper->setSpeed(30);
			stepper->setAcceleration(1000000);
			seq->u32_1 = time_ms;
			seq->state++;
			break;
		case 1: // Wait 1s pass
		case 3:
		case 5:
			if (time_ms - seq->u32_1 >= 1000) {
				if ((seq->state & 0xfff0) == (16*60)) {
					seq->state = 7;
			stepper->setSpeed(1000);
			stepper->setAcceleration(10000);
			stepper->move(-3200);
					break;
				}
			   seq->state++;
			}
			break;
		case 2:
		case 4:
			// second pass, run motor
			//    3200steps = 40*53steps + 20*54steps
			seq->u32_1 += 1000;
			stepper->move(53);
			seq->state += 16 + 1;
			break;
		case 6:
			seq->u32_1 += 1000;
			stepper->move(54);
			seq->state += 16 + 1 - 6; // instead of state 7, go to 1
			break;
		case 7:
			if (!stepper->isRunning()) {
					return true; // finished
			}
			break;
	}
	return false;
}
