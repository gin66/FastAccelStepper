#include <driver/mcpwm.h>
#include <driver/pcnt.h>
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>
#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>

#include "StepperISR.h"
#include "FastAccelStepper.h"

#if defined(ARDUINO_ARCH_ESP32)
// Here are the global variables to interface with the interrupts
StepperQueue fas_queue[NUM_QUEUES];

void StepperQueue::init(uint8_t step_pin) {
  _initVars();
  digitalWrite(step_pin, LOW);
  pinMode(step_pin, OUTPUT);
}

#define LED_PIN 2

static void IRAM_ATTR pcnt_isr(void *arg) {
  Serial.print("PCNT interrupt: ");
  Serial.print(PCNT.int_st.val);
  Serial.print(" ");
  Serial.println(PCNT.status_unit[0].val);
  PCNT.int_clr.val = PCNT.int_st.val;
  uint8_t x = REG_READ(MCPWM_CLK_CFG_REG(0));
  x = x - 1;
  REG_WRITE(MCPWM_CLK_CFG_REG(0), x);
}

void pwm_setup() {
  pcnt_config_t cfg;
  cfg.pulse_gpio_num = LED_PIN;
  cfg.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  cfg.lctrl_mode = PCNT_MODE_KEEP;
  cfg.hctrl_mode = PCNT_MODE_KEEP;
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DIS;
  cfg.counter_h_lim = 1;
  cfg.counter_l_lim = 0;
  cfg.unit = PCNT_UNIT_0;
  cfg.channel = PCNT_CHANNEL_0;
  pcnt_unit_config(&cfg);
  PCNT.conf_unit[0].conf0.thr_h_lim_en = 1;  // update only on zero
  PCNT.conf_unit[0].conf0.thr_l_lim_en = 0;

  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
  pcnt_set_event_value(PCNT_UNIT_0, PCNT_EVT_THRES_0, 5);
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_THRES_0);
  pcnt_set_event_value(PCNT_UNIT_0, PCNT_EVT_THRES_1, 10);
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_THRES_1);
  // pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
  pcnt_isr_service_install(ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM);
  pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_isr, 0);
  pcnt_intr_enable(PCNT_UNIT_0);

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LED_PIN);
  MCPWM0.clk_cfg.prescale = 160 - 1;    // 160 MHz/160  => 1 us
  MCPWM0.timer[0].period.upmethod = 0;  // 0 = immediate update, 1 = TEZ
  MCPWM0.timer[0].period.period = 4000;
  MCPWM0.timer[0].period.prescale = 250 - 1;  // => 1 Hz
  MCPWM0.timer[0].mode.start = 2;             // free run
  MCPWM0.timer[0].mode.mode = 1;              // increase mod
  MCPWM0.timer[0].sync.val = 0;               // no sync
  MCPWM0.timer_sel.operator0_sel = 0;
  MCPWM0.timer_sel.operator1_sel = 1;  // timer 1
  MCPWM0.timer_sel.operator2_sel = 2;  // timer 2
  MCPWM0.channel[0].cmpr_cfg.a_upmethod =
      0;  // timer 0 compare A update method: immediate
  MCPWM0.channel[0].cmpr_value[0].cmpr_val = 2000;
  MCPWM0.channel[0].generator[0].val = 0;
  MCPWM0.channel[0].generator[0].utez = 2;  // high at zero
  MCPWM0.channel[0].generator[0].utea = 1;  // low at compare A match

  int input_sig_index = PCNT_SIG_CH0_IN0_IDX;
  gpio_iomux_in(LED_PIN, input_sig_index);
}
int StepperQueue::addQueueEntry(uint32_t start_delta_ticks, uint8_t steps,
                                    bool dir_high, int16_t change_ticks) {
  int32_t c_sum = 0;
  if (steps >= 128) {
    return AQE_STEPS_ERROR;
  }
  if (start_delta_ticks > ABSOLUTE_MAX_TICKS) {
    return AQE_TOO_HIGH;
  }
  if ((change_ticks != 0) && (steps > 1)) {
    c_sum = change_ticks * (steps - 1);
  }
  if (change_ticks > 0) {
    if (c_sum > 32768) {
      return AQE_CHANGE_TOO_HIGH;
    }
  } else if (change_ticks < 0) {
    if (c_sum < -32768) {
      return AQE_CHANGE_TOO_LOW;
    }
    if (start_delta_ticks + c_sum < MIN_DELTA_TICKS) {
      return AQE_CUMULATED_CHANGE_TOO_LOW;
    }
  }

  uint16_t msb = start_delta_ticks >> 14;
  uint16_t lsw;
  if (msb > 1) {
    msb--;
    lsw = start_delta_ticks & 0x3fff;
    lsw |= 0x4000;
  } else {
    msb = 0;
    lsw = start_delta_ticks;
  }

  uint8_t wp = next_write_ptr;
  uint8_t rp = read_ptr;
  struct queue_entry* e = &entry[wp];

  uint8_t next_wp = (wp + 1) & QUEUE_LEN_MASK;
  if (next_wp != rp) {
    pos_at_queue_end += dir_high ? steps : -steps;
    ticks_at_queue_end = change_ticks * (steps - 1) + start_delta_ticks;
    steps <<= 1;
    e->delta_msb = msb;
    e->delta_lsw = lsw;
    e->delta_change = change_ticks;
    e->steps = (dir_high != dir_high_at_queue_end) ? steps | 0x01 : steps;
    dir_high_at_queue_end = dir_high;
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
    {
      unsigned char* x = (unsigned char*)e;
      for (uint8_t i = 0; i < sizeof(struct queue_entry); i++) {
        if (checksum & 0x80) {
          checksum <<= 1;
          checksum ^= 0xde;
        } else {
          checksum <<= 1;
        }
        checksum ^= *x++;
      }
    }
#endif
    next_write_ptr = next_wp;
    return AQE_OK;
  }
  return AQE_FULL;
}
#endif
