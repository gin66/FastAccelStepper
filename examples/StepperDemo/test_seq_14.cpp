#include "test_seq.h"

#if defined(SUPPORT_ESP32_PULSE_COUNTER)
#include "test_seq_14_pattern.h"

#define SEQ14_TIME_LIMIT_MS 1000

static bool seq14_retry(MoveTimedResultCode rc) {
  switch (rc) {
    case MOVE_TIMED_BUSY:
    case MoveTimedResultCode::QueueFull:
    case MoveTimedResultCode::DirPinIsBusy:
    case MoveTimedResultCode::DirPin2msPauseAdded:
    case MoveTimedResultCode::DirChangePauseInjected:
    case MoveTimedResultCode::WaitForEnablePinActive:
    case MoveTimedResultCode::DeviceNotReady:
      return true;
    default:
      return false;
  }
}

static bool seq14_feed(FastAccelStepper* stepper, int16_t steps, uint32_t ticks,
                       int32_t* drift) {
  uint32_t duration = ticks + *drift;
  for (;;) {
    uint32_t actual = 0;
    MoveTimedResultCode rc = stepper->moveTimed(steps, duration, &actual, true);
    if ((rc == MOVE_TIMED_OK) || (rc == MOVE_TIMED_EMPTY)) {
      *drift = (int32_t)(duration - actual);
      return true;
    }
    if (!seq14_retry(rc)) {
      PRINTLN(toString(rc));
      return false;
    }
  }
}

bool test_seq_14(FastAccelStepper* stepper, struct test_seq_s* seq,
                 uint32_t time_ms) {
  switch (seq->state) {
    case 0:
      if (!stepper->attachToPulseCounter(7)) {
        PRINTLN("Error attaching to pulse counter");
        seq->state = TEST_STATE_ERROR;
        return true;
      }
      stepper->setAutoEnable(false);
      stepper->enableOutputs();
      stepper->setCurrentPosition(0);
      stepper->clearPulseCounter();
      seq->u32_1 = MILLIS();
      seq->state++;
      // fall through
    case 1: {
      int32_t drift = 0;
      for (uint16_t i = 0; i < REPLAY_PATTERN_LEN; i++) {
        if (!seq14_feed(stepper, REPLAY_PATTERN[i].steps,
                        REPLAY_PATTERN[i].ticks, &drift)) {
          seq->state = TEST_STATE_ERROR;
          return true;
        }
      }
      seq->state++;
      break;
    }
    case 2:
      if (!stepper->isRunning()) {
        uint32_t dt = MILLIS() - seq->u32_1;
        int16_t pcnt = stepper->readPulseCounter();
        PRINT("pcnt=");
        PRINTI16(pcnt);
        PRINT(" pos=");
        PRINTI32(stepper->getCurrentPosition());
        PRINT(" ramp_ms=");
        PRINTU32(dt);
        PRINTLN("");
        if ((pcnt != REPLAY_PATTERN_NET_STEPS) || (dt >= SEQ14_TIME_LIMIT_MS)) {
          seq->state = TEST_STATE_ERROR;
        }
        return true;
      }
      break;
  }
  (void)time_ms;
  return false;
}

#else
bool test_seq_14(FastAccelStepper* stepper, struct test_seq_s* seq,
                 uint32_t time_ms) {
  (void)stepper;
  (void)seq;
  (void)time_ms;
  return true;
}
#endif
