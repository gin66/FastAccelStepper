#include "FastAccelStepper.h"

#define dirPinStepper 5
#define enablePinStepper 6
#define stepPinStepper 9 


#define LED_PIN 2

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = engine.stepperConnectToPin(stepPinStepper);

#include <driver/pcnt.h>
#include <driver/mcpwm.h>
#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>
static void IRAM_ATTR pcnt_isr(void *arg) {
    Serial.print("PCNT interrupt: ");
    Serial.print(PCNT.int_st.val);
    Serial.print(" ");
    Serial.println(PCNT.status_unit[0].val);
    PCNT.int_clr.val = PCNT.int_st.val;
}
static void IRAM_ATTR mcpwm_isr(void *arg) {
  // only needed to add change
  uint32_t mcpwm_intr_status = MCPWM0.int_st.val;
  if (mcpwm_intr_status & MCPWM_TIMER0_TEZ_INT_CLR) {
    uint8_t x = REG_READ(MCPWM_CLK_CFG_REG(0));
    x = x - 1;
    REG_WRITE(MCPWM_CLK_CFG_REG(0), x);
    //MCPWM0.int_clr.timer0_tez_int_clr = 1;
  }
  else {
    Serial.print("Spurious interrupt: ");
    Serial.println(mcpwm_intr_status);
  }
  MCPWM0.int_clr.val = mcpwm_intr_status;
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
  PCNT.conf_unit[0].conf0.thr_h_lim_en = 1;
  PCNT.conf_unit[0].conf0.thr_l_lim_en = 0;

  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
  pcnt_set_event_value(PCNT_UNIT_0, PCNT_EVT_THRES_0, 5);
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_THRES_0);
  pcnt_set_event_value(PCNT_UNIT_0, PCNT_EVT_THRES_1, 10);
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_THRES_1);
  //pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
  pcnt_isr_service_install(ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM);
  pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_isr, 0);
  pcnt_intr_enable(PCNT_UNIT_0);

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LED_PIN);
  REG_WRITE(MCPWM_CLK_CFG_REG(0), 160-1);  // 160 MHz/160  => 1 us
  REG_WRITE(MCPWM_TIMER0_CFG0_REG(0), (250-1) + (4000L << 8));  // => 1 Hz bei Up
  REG_WRITE(MCPWM_TIMER0_CFG1_REG(0), 0x0a);
  REG_WRITE(MCPWM_GEN0_TSTMP_A_REG(0), 2000);
  REG_WRITE(MCPWM_GEN0_A_REG(0), (2<<0) | (1<<4));
  mcpwm_isr_register(MCPWM_UNIT_0, mcpwm_isr, 0, ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM, NULL);
  MCPWM0.int_ena.val = 0;
  MCPWM0.int_ena.timer0_tez_int_ena = 1;
  MCPWM0.int_clr.timer0_tez_int_clr = 1;

  int input_sig_index = PCNT_SIG_CH0_IN0_IDX;
  gpio_iomux_in(LED_PIN, input_sig_index);
}

void setup() {
  Serial.begin(115200);

  engine.init();
//  engine.setDebugLed(LED_PIN);

  pwm_setup();

  if (stepper) {
	  stepper->setDirectionPin(dirPinStepper);
	  stepper->setEnablePin(enablePinStepper);
	  stepper->setAutoEnable(true);
  }
}

uint8_t in_ptr = 0;
char in_buffer[256];
uint8_t in_val_ptr = 0;
long in_vals[8];
bool stopped = false;

void loop() {
  int16_t cnt;
  Serial.print(pcnt_get_counter_value(PCNT_UNIT_0, &cnt));
  Serial.print(" ");
  Serial.print(cnt);
  Serial.print(" ");
  Serial.println(digitalRead(0));
  delay(100);

  bool cmd_ok = false;
  bool queue_ok = false;

  if (Serial.available()) {
    char ch = Serial.read();
    if ((ch == '\n') || (ch == ' ')) {
      if (in_ptr > 0) {
        in_buffer[in_ptr] = 0;
        in_ptr = 0;
        if (in_val_ptr < 8) {
          in_vals[in_val_ptr++] = atol(in_buffer);
        }
      }
    } else {
      in_buffer[in_ptr++] = ch;
    }
    if (ch == '\n') {
      if (in_val_ptr == 3) {
        cmd_ok = true;
      }
      if (in_val_ptr == 1) {
        queue_ok = true;
      }
      if (in_val_ptr == 0) {
        stopped = false;
      }
      in_val_ptr = 0;
      in_ptr = 0;
    }
  }
  if (stepper) {

  if (queue_ok) {
      Serial.println(
          stepper->addQueueEntry(5L * 16384, 120, true, -16384 / 119));
      Serial.println(
          stepper->addQueueEntry(4L * 16384, 120, true, -16384 / 119));
      Serial.println(
          stepper->addQueueEntry(3L * 16384, 120, true, -16384 / 119));
      Serial.println(
          stepper->addQueueEntry(2L * 16384, 120, true, -8192 / 119));
      Serial.println(stepper->addQueueEntry(6L * 4096, 120, true, -4096 / 119));
      Serial.println(stepper->addQueueEntry(5L * 4096, 120, true, -4096 / 119));
      Serial.println(stepper->addQueueEntry(4L * 4096, 120, true, -4096 / 119));
      Serial.println(stepper->addQueueEntry(3L * 4096, 120, true, -4096 / 119));
      Serial.println(stepper->addQueueEntry(2L * 4096, 120, true, 0));
      Serial.println(stepper->addQueueEntry(2L * 4096, 120, true, 0));
      Serial.println(stepper->addQueueEntry(2L * 4096, 120, true, 0));
      Serial.println(stepper->addQueueEntry(2L * 4096, 120, true, 0));
      Serial.println(stepper->addQueueEntry(2L * 4096, 120, true, 0));
      Serial.println(stepper->addQueueEntry(2L * 4096, 120, true, 0));
      Serial.println(stepper->addQueueEntry(2L * 4096, 120, true, 0));
  }

  if (cmd_ok) {
    long move = in_vals[1];
    long ticks = in_vals[2];
    long accel = in_vals[3];
    if (move) {
      Serial.print("ticks=");
      Serial.print(ticks);
      Serial.print("  accel=");
      Serial.print(accel);
      Serial.print("  move=");
      Serial.print(move);
      stopped = false;
      stepper->setSpeed(ticks);
      stepper->setAcceleration(accel);
      stepper->move(move);
      Serial.print("  Start stepper: ");
      Serial.println(stepper->getCurrentPosition());
    }
  }

  if (!stopped) {
    Serial.print("Stepper: ");
    Serial.print(stepper->isr_speed_control_enabled ? " AUTO " : " MANU ");
    Serial.print(stepper->getCurrentPosition());
    if (stepper->isRunning()) {
      Serial.print("  RUNNING");
    } else {
      Serial.print("  PAUSED ");
    }
    Serial.print("  state=");
    Serial.print(stepper->ramp_state);
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
    Serial.print("  max/us=");
    Serial.print(stepper->max_micros);
#endif
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
    Serial.print("  checksum=");
    Serial.print(stepper->checksum);
#endif

    stopped = !stepper->isRunning();
    if (stopped) {
      Serial.println(
          "Please enter one line with <steps> <speed> <acceleration> "
          "e.g.");
      Serial.println("10000 1000 100");
    }
  } else {
    stopped = !stepper->isRunning();
  }
}
}
