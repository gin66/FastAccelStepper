#include "test_seq.h"

// u32_1 shall be milliseconds from last tick
//
// Run 320000 steps with speed changes every 100ms
// in order to reproduce issue #24

bool test_seq_07(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms) {
  switch (seq->state) {
    case 0:  // INIT
      stepper->setSpeedInUs(40);
      stepper->moveByAcceleration(30000);
      seq->u32_1 = time_ms;
      seq->state++;
      break;
    case 1:
      if ((stepper->rampState() & RAMP_STATE_MASK) == RAMP_STATE_COAST) {
        int32_t dt = time_ms - seq->u32_1;
        Serial.println(dt);
        if (abs(dt - 792) > 16) {  // 779 esp, 805 avr
          seq->state = TEST_STATE_ERROR;
          return true;
        }
        stepper->moveByAcceleration(-30000);
        seq->u32_1 = time_ms;
        seq->state++;
      }
      break;
    case 2:
      if (time_ms - seq->u32_1 >= 3000) {
        stepper->moveByAcceleration(40000);
        seq->u32_1 = time_ms;
        seq->state++;
      }
      break;
    case 3:  // Return to start position
      if (time_ms - seq->u32_1 >= 1000) {
        stepper->moveTo(0);
        seq->u32_1 = time_ms;
        seq->state++;
      }
      break;
    case 4:
      if (!stepper->isRunning()) {
        int32_t dt = time_ms - seq->u32_1;
        Serial.println(dt);
        if (abs(dt - 1495) > 30) {
          seq->state = TEST_STATE_ERROR;
        }
        Serial.println(stepper->getPositionAfterCommandsCompleted());
        if (stepper->getPositionAfterCommandsCompleted() != 0) {
          seq->state = TEST_STATE_ERROR;
        }
        return true;  // finished
      }
      break;
  }
  return false;
}
