#include <driver/mcpwm.h>
#include <driver/pcnt.h>
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>
#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>

#include "StepperISR.h"

#if defined(ARDUINO_ARCH_ESP32)

#define TIMER_H_L_TRANSITION 200

#define TIMER_PRESCALER 0  // cannot be updated will timer is running

// Here are the global variables to interface with the interrupts
StepperQueue fas_queue[NUM_QUEUES];

void IRAM_ATTR next_command(StepperQueue *queue, struct queue_entry *e) {
  uint8_t queueNum = queue->queueNum;
  mcpwm_dev_t *mcpwm = queueNum < 3 ? &MCPWM0 : &MCPWM1;
  uint8_t timer = queueNum % 3;
  mcpwm->timer[timer].period.period = e->period;
  uint8_t steps = e->steps;
  PCNT.conf_unit[timer].conf2.cnt_h_lim = steps >> 1;  // update only on zero
  if ((steps & 0x01) != 0) {
    uint8_t dirPin = queue->dirPin;
    digitalWrite(dirPin, digitalRead(dirPin) == HIGH ? LOW : HIGH);
  }
  uint8_t n_periods = e->n_periods;
  if (n_periods == 1) {
    mcpwm->channel[timer].generator[0].utez = 2;  // high at zero
    switch (timer) {
      case 0:
        mcpwm->int_ena.timer0_tez_int_ena = 0;
        break;
      case 1:
        mcpwm->int_ena.timer1_tez_int_ena = 0;
        break;
      case 2:
        mcpwm->int_ena.timer2_tez_int_ena = 0;
        break;
    }
  } else {
    queue->current_n_periods = n_periods;
    queue->current_period = 1;  // mcpwm is already running in period 1
    mcpwm->channel[timer].generator[0].utez = 1;  // low at zero
    switch (timer) {
      case 0:
        mcpwm->int_clr.timer0_tez_int_clr = 1;
        mcpwm->int_ena.timer0_tez_int_ena = 1;
        break;
      case 1:
        mcpwm->int_clr.timer1_tez_int_clr = 1;
        mcpwm->int_ena.timer1_tez_int_ena = 1;
        break;
      case 2:
        mcpwm->int_clr.timer2_tez_int_clr = 1;
        mcpwm->int_ena.timer2_tez_int_ena = 1;
        break;
    }
  }
}

static void IRAM_ATTR pcnt_isr_service(void *arg) {
  StepperQueue *q = (StepperQueue *)arg;
  uint8_t rp = q->read_ptr;
  if (rp != q->next_write_ptr) {
    struct queue_entry *e = &q->entry[rp];
    rp = (rp + 1) & QUEUE_LEN_MASK;
    q->read_ptr = rp;
    next_command(q, e);
  } else {
    // no more commands: stop timer at period end
    mcpwm_dev_t *mcpwm = q->queueNum < 3 ? &MCPWM0 : &MCPWM1;
    uint8_t timer = q->queueNum % 3;
    mcpwm->timer[timer].mode.start = 1;           // stop at TEP
    mcpwm->channel[timer].generator[0].utez = 1;  // low at zero
    q->isRunning = false;
    if (q->autoEnablePin != 255) {
      digitalWrite(q->autoEnablePin, HIGH);
    }
  }
}

#define MCPWM_SERVICE(mcpwm, TIMER, pcnt)                            \
  if (mcpwm.int_st.timer##TIMER##_tez_int_st != 0) {                 \
    mcpwm.int_clr.timer##TIMER##_tez_int_clr = 1;                    \
    uint8_t cp = fas_queue[pcnt].current_period + 1;                 \
    if (fas_queue[pcnt].current_n_periods == cp) {                   \
      mcpwm.channel[TIMER].generator[0].utez = 2; /* high at zero */ \
      fas_queue[pcnt].current_period = 0;                            \
    } else {                                                         \
      mcpwm.channel[TIMER].generator[0].utez = 1; /* low at zero */  \
      fas_queue[pcnt].current_period = cp;                           \
    }                                                                \
  }

static void IRAM_ATTR mcpwm0_isr_service(void *arg) {
  MCPWM_SERVICE(MCPWM0, 0, 0);
  MCPWM_SERVICE(MCPWM0, 1, 1);
  MCPWM_SERVICE(MCPWM0, 2, 2);
}
static void IRAM_ATTR mcpwm1_isr_service(void *arg) {
  MCPWM_SERVICE(MCPWM1, 0, 3);
  MCPWM_SERVICE(MCPWM1, 1, 4);
  MCPWM_SERVICE(MCPWM1, 2, 5);
}

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
  _initVars();

  digitalWrite(step_pin, LOW);
  pinMode(step_pin, OUTPUT);
  queueNum = queue_num;
  isRunning = false;

  mcpwm_dev_t *mcpwm = queue_num < 3 ? &MCPWM0 : &MCPWM1;
  mcpwm_unit_t mcpwm_unit = queue_num < 3 ? MCPWM_UNIT_0 : MCPWM_UNIT_1;
  pcnt_unit_t pcnt_unit = (pcnt_unit_t)queue_num;

  pcnt_config_t cfg;
  cfg.pulse_gpio_num = step_pin;
  cfg.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  cfg.lctrl_mode = PCNT_MODE_KEEP;
  cfg.hctrl_mode = PCNT_MODE_KEEP;
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DIS;
  cfg.counter_h_lim = 1;
  cfg.counter_l_lim = 0;
  cfg.unit = pcnt_unit;
  cfg.channel = PCNT_CHANNEL_0;
  pcnt_unit_config(&cfg);

  pcnt_counter_clear(pcnt_unit);
  pcnt_counter_resume(pcnt_unit);
  pcnt_event_enable(pcnt_unit, PCNT_EVT_H_LIM);
  if (queue_num == 0) {
    // isr_service_install apparently enables the interrupt
    PCNT.int_clr.val = PCNT.int_st.val;
    pcnt_isr_service_install(ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM);
    pcnt_isr_handler_add(pcnt_unit, pcnt_isr_service, (void *)this);
  }

  uint8_t timer = queue_num % 3;
  switch (timer) {
    case 0:
      mcpwm_gpio_init(mcpwm_unit, MCPWM0A, step_pin);
      break;
    case 1:
      mcpwm_gpio_init(mcpwm_unit, MCPWM1A, step_pin);
      break;
    case 2:
      mcpwm_gpio_init(mcpwm_unit, MCPWM2A, step_pin);
      break;
  }
  if (timer == 0) {
    // Init mcwpm module for use
    //
    periph_module_enable(queue_num < 3 ? PERIPH_PWM0_MODULE
                                       : PERIPH_PWM1_MODULE);
    mcpwm->int_ena.val = 0;  // disable all interrupts
    mcpwm_isr_register(mcpwm_unit,
                       queueNum < 3 ? mcpwm0_isr_service : mcpwm1_isr_service,
                       NULL, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_EDGE, NULL);

    mcpwm->clk_cfg.prescale = 10 - 1;  // 160 MHz/10  => 16 MHz

    mcpwm->timer_sel.operator0_sel = 0;  // timer 0 is input for operator 0
    mcpwm->timer_sel.operator1_sel = 1;  // timer 1 is input for operator 1
    mcpwm->timer_sel.operator2_sel = 2;  // timer 2 is input for operator 2
  }
  mcpwm->timer[timer].period.upmethod = 0;  // 0 = immediate update, 1 = TEZ
  mcpwm->timer[timer].period.prescale = TIMER_PRESCALER;
  mcpwm->timer[timer].period.period = 400;
  mcpwm->timer[timer].mode.val = 0;  // freeze
  mcpwm->timer[timer].sync.val = 0;  // no sync
  mcpwm->channel[timer].cmpr_cfg.a_upmethod = 0;
  mcpwm->channel[timer].cmpr_value[0].cmpr_val = TIMER_H_L_TRANSITION;
  mcpwm->channel[timer].generator[0].val = 0;
  mcpwm->channel[timer].generator[1].val = 0;
  // mcpwm->channel[timer].generator[0].utez = 2;  // high at zero
  mcpwm->channel[timer].generator[0].utez = 1;  // low at zero
  mcpwm->channel[timer].generator[0].utea = 1;  // low at compare A match
  mcpwm->channel[timer].db_cfg.val = 0;
  mcpwm->channel[timer].carrier_cfg.val = 0;  // carrier disabled

  // at last link the output to pcnt input
  int input_sig_index;
  switch (queue_num) {
    case 0:
      input_sig_index = PCNT_SIG_CH0_IN0_IDX;
      break;
    case 1:
      input_sig_index = PCNT_SIG_CH0_IN1_IDX;
      break;
    case 2:
      input_sig_index = PCNT_SIG_CH0_IN2_IDX;
      break;
    case 3:
      input_sig_index = PCNT_SIG_CH0_IN3_IDX;
      break;
    case 4:
      input_sig_index = PCNT_SIG_CH0_IN4_IDX;
      break;
    case 5:
      input_sig_index = PCNT_SIG_CH0_IN5_IDX;
      break;
  }
  gpio_iomux_in(step_pin, input_sig_index);
}

// Mechanism is like this, starting from stopped motor:
//
//		init counter
//		init mcpwm
//		start mcpwm
//		-- mcpwm counter counts every L->H-transition at mcpwm.timer = 0
//		-- if counter reaches planned steps, then counter is reset and
//interrupt is created 		pcnt interrupt: available time is from mcpwm.timer = 0+x
//to period 			read next commmand: store period in counter shadow and steps in pcnt
//			without next command: set mcpwm to stop mode on reaching
//period
//

bool StepperQueue::startQueue(struct queue_entry *e) {
  // TODO steps and direction update
  mcpwm_dev_t *mcpwm = queueNum < 3 ? &MCPWM0 : &MCPWM1;
  pcnt_unit_t pcnt_unit = (pcnt_unit_t)queueNum;
  uint8_t timer = queueNum % 3;
  // timer should be either a TEP or at zero
  mcpwm->channel[timer].generator[0].utez = 1;  // low at zero
  mcpwm->timer[timer].period.upmethod = 0;      // 0 = immediate update, 1 = TEZ
  mcpwm->timer[timer].period.period =
      65535;  // will be overwritten in next_command
  isRunning = true;

  mcpwm->timer[timer].mode.val = 10;  // free run incrementing

  // busy wait period for timer zero
  uint32_t i;
  while (mcpwm->timer[timer].status.value >= TIMER_H_L_TRANSITION) {
    i++;
  }
  // Serial.print("Loops=");
  // Serial.println(i);

  next_command(this, e);

  if (autoEnablePin != 255) {
    digitalWrite(autoEnablePin, LOW);
  }
  return true;
}
#endif
