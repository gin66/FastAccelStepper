#include "StepperISR.h"
#if defined(SUPPORT_ESP32_PULSE_COUNTER) && (ESP_IDF_VERSION_MAJOR == 4)

uint32_t sig_idx[SUPPORT_ESP32_PULSE_COUNTER] = {
    PCNT_SIG_CH0_IN0_IDX, PCNT_SIG_CH0_IN1_IDX,
    PCNT_SIG_CH0_IN2_IDX, PCNT_SIG_CH0_IN3_IDX,
#if SUPPORT_ESP32_PULSE_COUNTER == 8
    PCNT_SIG_CH0_IN4_IDX, PCNT_SIG_CH0_IN5_IDX,
    PCNT_SIG_CH0_IN6_IDX, PCNT_SIG_CH0_IN7_IDX,
#endif
};
uint32_t ctrl_idx[SUPPORT_ESP32_PULSE_COUNTER] = {
    PCNT_CTRL_CH0_IN0_IDX, PCNT_CTRL_CH0_IN1_IDX, PCNT_CTRL_CH0_IN2_IDX,
    PCNT_CTRL_CH0_IN3_IDX,
#if SUPPORT_ESP32_PULSE_COUNTER == 8
    PCNT_CTRL_CH0_IN4_IDX, PCNT_CTRL_CH0_IN5_IDX, PCNT_CTRL_CH0_IN6_IDX,
    PCNT_CTRL_CH0_IN7_IDX
#endif
};

bool FastAccelStepper::attachToPulseCounter(uint8_t pcnt_unit,
                                            int16_t low_value,
                                            int16_t high_value) {
  if (pcnt_unit >= SUPPORT_ESP32_PULSE_COUNTER) {
    return false;
  }

  pcnt_config_t cfg;
  uint8_t dir_pin = getDirectionPin();
  uint8_t step_pin = getStepPin();
  cfg.pulse_gpio_num = PCNT_PIN_NOT_USED;
  if (dir_pin == PIN_UNDEFINED) {
    cfg.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    cfg.hctrl_mode = PCNT_MODE_KEEP;
    cfg.lctrl_mode = PCNT_MODE_KEEP;
  } else {
    cfg.ctrl_gpio_num = dir_pin;
    if (directionPinHighCountsUp()) {
      cfg.lctrl_mode = PCNT_MODE_REVERSE;
      cfg.hctrl_mode = PCNT_MODE_KEEP;
    } else {
      cfg.lctrl_mode = PCNT_MODE_KEEP;
      cfg.hctrl_mode = PCNT_MODE_REVERSE;
    }
  }
  cfg.pos_mode = PCNT_COUNT_INC;  // increment on rising edge
  cfg.neg_mode = PCNT_COUNT_DIS;  // ignore falling edge
  cfg.counter_h_lim = high_value;
  cfg.counter_l_lim = low_value;
  cfg.unit = (pcnt_unit_t)pcnt_unit;
  cfg.channel = PCNT_CHANNEL_0;
  pcnt_unit_config(&cfg);

#ifndef HAVE_ESP32S3_PULSE_COUNTER
  PCNT.conf_unit[cfg.unit].conf0.thr_h_lim_en = 0;
  PCNT.conf_unit[cfg.unit].conf0.thr_l_lim_en = 0;
#else
  PCNT.conf_unit[cfg.unit].conf0.thr_h_lim_en_un = 0;
  PCNT.conf_unit[cfg.unit].conf0.thr_l_lim_en_un = 0;
#endif

  //  detachFromPin();
  //  reAttachToPin();
  gpio_matrix_in(step_pin, sig_idx[pcnt_unit], 0);
  gpio_iomux_in(step_pin,
                sig_idx[pcnt_unit]);  // test failure without this call
  if (dir_pin != PIN_UNDEFINED) {
    pinMode(dir_pin, OUTPUT);
    gpio_matrix_out(dir_pin, 0x100, false, false);
    gpio_matrix_in(dir_pin, ctrl_idx[pcnt_unit], 0);
    gpio_iomux_in(dir_pin, ctrl_idx[pcnt_unit]);
  }

  pcnt_counter_clear(cfg.unit);
  pcnt_counter_resume(cfg.unit);

  _attached_pulse_cnt_unit = pcnt_unit;
  return true;
}
void FastAccelStepper::clearPulseCounter() {
  if (pulseCounterAttached()) {
    pcnt_counter_clear((pcnt_unit_t)_attached_pulse_cnt_unit);
  }
}
int16_t FastAccelStepper::readPulseCounter() {
  if (pulseCounterAttached()) {
#ifndef HAVE_ESP32S3_PULSE_COUNTER
    return PCNT.cnt_unit[(pcnt_unit_t)_attached_pulse_cnt_unit].cnt_val;
#else
    return PCNT.cnt_unit[(pcnt_unit_t)_attached_pulse_cnt_unit].pulse_cnt_un;
#endif
  }
  return 0;
}
#endif
