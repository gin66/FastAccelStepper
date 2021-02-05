#include "test_seq.h"

#if defined(ARDUINO_ARCH_ESP32)
struct command_s {
  uint32_t accel;
  uint32_t speed;
  uint32_t move;
} commands[] = {{.accel = 0, .speed = 0, .move = 0}};

int16_t old = 0;

bool test_seq_08(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms) {
  char buf[100];
  switch (seq->state) {
    case 0:  // INIT
      srand(135);
      if (!stepper->attachToPulseCounter(7)) {
        Serial.println("Error attaching to pulse counter");
        seq->state = TEST_STATE_ERROR;
        return true;
      }
      seq->u32_1 = time_ms;
      seq->state++;
      break;
    case 1:
      if (!stepper->isRunning()) {
        int16_t pcnt = stepper->readPulseCounter();
        int32_t spos = stepper->getPositionAfterCommandsCompleted();
        if ((pcnt & 0x3fff) != (spos & 0x3fff)) {
          sprintf(buf, "stepper pos=%d  real pos=%d", spos, pcnt);
          Serial.println(buf);

          seq->state = TEST_STATE_ERROR;
          return true;
        }
#define VMIN 40
#define VMAX 16384
        uint16_t speed = rand() % (VMAX * 4);
        speed = speed >> ((speed % 4) + 2);
        speed = speed + VMIN;

#define AMIN 10
#define AMAX (4 * 65536)
        uint32_t accel = rand() % (AMAX * 4);
        accel = accel >> ((accel % 4) + 2);
        accel = accel + AMIN;

        // s = a * tr² + v * tc  with t = 2*tr+tc
        //
        // v = a * tr => tr = v/a
        //
        // s = v²/a + v * (t-2*tr)
        //
        //   = v²/a + v * (t-2*v/a)
        //
        //   = v²/a + v * t - 2*v²/a
        //
        //   = v * t - v²/a
        //
        //   = v * (t - v/a)
        //
        //   = t/tv - 1/(tv²*a)
        //
        // With t = 1s
        int32_t move = 1000000 / speed;
        if (move & 2) {
          move = -move;
        }

        sprintf(buf, "speed=%d accel=%d move=%d", speed, accel, move);
        Serial.println(buf);
        stepper->setSpeedInUs(speed);
        stepper->setAcceleration(accel);
        stepper->move(move);
        seq->u32_1 = time_ms;
      }
      break;
  }
  return false;
}

#else
bool test_seq_08(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms) {
  return true;  // finished
}
#endif
