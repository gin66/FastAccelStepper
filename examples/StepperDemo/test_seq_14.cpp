#include "test_seq.h"

#if defined(SUPPORT_ESP32_PULSE_COUNTER)
#include "test_seq_14_pattern.h"

#define SEQ14_TIME_LIMIT_MS 1000

// Feed one command to the stepper. A successful append (MOVE_TIMED_OK /
// MOVE_TIMED_EMPTY) updates the drift; every other result is handed back to the
// caller so it can decide what to do on the next loop() tick.
static MoveTimedResultCode seq14_feed(FastAccelStepper* stepper, int16_t steps,
                                      uint32_t ticks, int32_t* drift,
                                      bool start) {
  uint32_t duration = ticks + *drift;
  uint32_t actual = 0;
  MoveTimedResultCode rc = stepper->moveTimed(steps, duration, &actual, start);
  if ((rc == MOVE_TIMED_OK) || (rc == MOVE_TIMED_EMPTY)) {
    *drift = (int32_t)(duration - actual);
  }
  return rc;
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
      PRINTLN("Attached to pulse counter 7");
      stepper->setAutoEnable(false);
      stepper->enableOutputs();
      stepper->setCurrentPosition(0);
      stepper->clearPulseCounter();
      seq->u32_1 = MILLIS();
      seq->s16_1 = 0;  // next command index into REPLAY_PATTERN
      seq->s16_2 = 0;  // 0: filling (start=false), 1: running top-up
      seq->s32_1 = 0;  // drift
      seq->state++;
      break;
    case 1: {
      // Replay the recorded #370 stroke one command per loop() tick. The
      // command index and the fill/run mode live in the sequence state, so
      // nothing ever blocks: a busy or transient result simply leaves the
      // state unchanged and the main loop feeds the next command on the
      // following tick, keeping the task watchdog happy.
      int16_t idx = seq->s16_1;
      if (idx < REPLAY_PATTERN_LEN) {
        bool start = (seq->s16_2 != 0);
        MoveTimedResultCode rc =
            seq14_feed(stepper, REPLAY_PATTERN[idx].steps,
                       REPLAY_PATTERN[idx].ticks, &seq->s32_1, start);
        switch (rc) {
          case MOVE_TIMED_OK:
          case MOVE_TIMED_EMPTY:
            seq->s16_1++;
            break;
          case MOVE_TIMED_BUSY:
          case MoveTimedResultCode::QueueFull:
            // Queue is full: the move was not appended. Switch from the
            // pure fill phase (start=false) to running top-up mode
            // (start=true, which auto-restarts the queue) and retry the
            // same command on the next tick without advancing.
            if (seq->s16_2 == 0) {
              stepper->moveTimed(0, 0, NULL, true);  // start the queue
            }
            seq->s16_2 = 1;
            break;
          case MoveTimedResultCode::DirPinIsBusy:
          case MoveTimedResultCode::DirPin2msPauseAdded:
          case MoveTimedResultCode::DirChangePauseInjected:
          case MoveTimedResultCode::WaitForEnablePinActive:
          case MoveTimedResultCode::DeviceNotReady:
            // Transient: keep the index and mode for the next tick.
            break;
          default:
            PRINTLN(toString(rc));
            seq->state = TEST_STATE_ERROR;
            break;
        }
        if (seq->s16_1 >= REPLAY_PATTERN_LEN) {
          seq->state++;
        }
      }
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
