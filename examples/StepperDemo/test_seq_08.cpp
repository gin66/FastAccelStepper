#include "test_seq.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <driver/gpio.h>
#include <driver/pcnt.h>
#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>

void connect_to_step_pin(FastAccelStepper *stepper) {
  pcnt_config_t cfg;
  // if step_pin is not set here (or 0x30), then it does not work
  cfg.pulse_gpio_num = stepper->getStepPin();
  cfg.ctrl_gpio_num = stepper->getDirectionPin();
  cfg.lctrl_mode = PCNT_MODE_REVERSE;
  cfg.hctrl_mode = PCNT_MODE_KEEP;
  cfg.pos_mode = PCNT_COUNT_INC;  // increment on rising edge
  cfg.neg_mode = PCNT_COUNT_DIS;  // ignore falling edge
  cfg.counter_h_lim = 16384;
  cfg.counter_l_lim = -16384;
  cfg.unit = PCNT_UNIT_7;
  cfg.channel = PCNT_CHANNEL_0;
  pcnt_unit_config(&cfg);

  PCNT.conf_unit[cfg.unit].conf0.thr_h_lim_en = 0;
  PCNT.conf_unit[cfg.unit].conf0.thr_l_lim_en = 0;

  pcnt_counter_clear(cfg.unit);
  pcnt_counter_resume(cfg.unit);

  stepper->detachFromPin();
  stepper->reAttachToPin();
  gpio_matrix_out(stepper->getDirectionPin(), 0x100, false, false);
  gpio_iomux_in(stepper->getStepPin(), PCNT_SIG_CH0_IN7_IDX);
  gpio_iomux_in(stepper->getDirectionPin(), PCNT_CTRL_CH0_IN7_IDX);
  pinMode(stepper->getDirectionPin(), OUTPUT);
}

struct command_s {
  uint32_t accel;
  uint32_t speed;
  uint32_t move;
} commands[] = {{.accel = 0, .speed = 0, .move = 0}};

int16_t old = 0;

bool test_seq_08(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms) {
  char buf[100];
  int16_t pcnt;
  switch (seq->state) {
    case 0:  // INIT
      srand(135);
      connect_to_step_pin(stepper);
      seq->u32_1 = time_ms;
      seq->state++;
      break;
    case 1:
      if (!stepper->isRunning()) {
        pcnt = PCNT.cnt_unit[PCNT_UNIT_7].cnt_val;
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

        int32_t pos = rand() % 65534;
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
