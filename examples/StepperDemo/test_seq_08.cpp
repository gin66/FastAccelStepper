#include "test_seq.h"

// u32_1 shall be milliseconds from last tick
//
// Run 320000 steps with speed changes every 100ms
// in order to reproduce issue #24

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
  cfg.lctrl_mode = PCNT_MODE_KEEP;
  cfg.hctrl_mode = PCNT_MODE_REVERSE;
  cfg.pos_mode = PCNT_COUNT_INC;    // increment on rising edge
  cfg.neg_mode = PCNT_COUNT_DIS;	// ignore falling edge
  cfg.counter_h_lim = 32767;
  cfg.counter_l_lim = -32768;
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
#endif

int16_t old=0; 

bool test_seq_08(FastAccelStepper *stepper, struct test_seq_s *seq,
                 uint32_t time_ms) {
#if defined(ARDUINO_ARCH_ESP32)
  int16_t pcnt;
#endif
#if defined(ARDUINO_ARCH_ESP32)
	  pcnt = PCNT.cnt_unit[PCNT_UNIT_7].cnt_val;
	  if (pcnt != old) {
	  Serial.print("pnct=");
	  Serial.println(pcnt);
	  old=pcnt;
	  }
#endif
  switch (seq->state) {
    case 0:  // INIT
	  connect_to_step_pin(stepper);
      stepper->setSpeed(30);
      stepper->setAcceleration(10);
      stepper->move(30);
      seq->u32_1 = time_ms;
      seq->state++;
      break;
    case 1:
      if (!stepper->isRunning()) {
        stepper->move(-30);
        seq->u32_1 = time_ms;
        seq->state++;
      }
      break;
    case 2:
      if (!stepper->isRunning()) {
        int32_t dt = time_ms - seq->u32_1;
        Serial.println(dt);
        if (stepper->getPositionAfterCommandsCompleted() != 0) {
          seq->state = TEST_STATE_ERROR;
        }
#if defined(ARDUINO_ARCH_ESP32)
	  pcnt = PCNT.cnt_unit[PCNT_UNIT_7].cnt_val;
	  Serial.print("pnct=");
	  Serial.println(pcnt);
#endif
        return true;  // finished
      }
      break;
  }
  return false;
}
