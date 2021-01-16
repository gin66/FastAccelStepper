#include "test_seq.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <driver/gpio.h>
#include <driver/pcnt.h>
#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>

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
        if (pcnt != spos) {
          sprintf(buf, "stepper pos=%d  real pos=%d", spos, pcnt);
          Serial.println(buf);

          seq->state = TEST_STATE_ERROR;
          return true;
        }
#define VMIN 20
#define VMAX 10000
        uint32_t speed = rand() % (VMAX * 4);
        speed = speed >> ((speed % 4) + 2);
        speed = speed + VMIN;

#define AMIN 10
#define AMAX 1000000
        uint32_t accel = rand() % (AMAX * 4);
        accel = accel >> ((accel % 4) + 2);
        accel = accel + AMIN;

        int32_t pos = rand() % 32766;
        if (pos & 1) {
          pos = -(pos >> 1);
        } else {
          pos = (pos >> 1);
        }

        sprintf(buf, "speed=%d accel=%d pos=%d", speed, accel, pos);
        Serial.println(buf);
        stepper->setSpeed(speed);
        stepper->setAcceleration(accel);
        stepper->moveTo(pos);
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
