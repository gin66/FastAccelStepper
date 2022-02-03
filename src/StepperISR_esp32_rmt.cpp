#include "StepperISR.h"

#ifdef SUPPORT_ESP32_RMT

#ifdef __ESP32_IDF_V44__
#include <driver/periph_ctrl.h>
#include <soc/periph_defs.h>
#endif /* __ESP32_IDF_V44__ */

//#define TEST_PROBE 18

// The following concept is in use:
//
//    The buffer of 64Bytes is split into two parts à  30 words.
//    Each part will hold on command (or part of).
//    After the 2*30 words an end marker is placed.
//    The threshold is set to 30.
//
//    This way, the threshold interrupt occurs after the first part
//    and the end interrupt after the second part.
//
//
// Currently with V>=1000us is ok, but V=500us has step loss. Faster yields more step loss
//
//  A=100000, V=40, 16000 Steps => 22 steps lost
//  A=100000, V=40, 13000 Steps => 22 steps lost
//  A=100000, V=40, 10000 Steps => 22 steps lost
//  => Ramp up phase is problematic
//
//  A=100000, V=50, 16000 Steps => 7 steps lost
//  A=100000, V=50, 13000 Steps => 8 steps lost
//  A=100000, V=50, 10000 Steps => 18 steps lost
//
#define PART_SIZE 30

static void IRAM_ATTR apply_command(StepperQueue *q, bool fill_part_one) {
  // ignore double threshold interrupts
  // ==> reason need to be understood
  if (q->last_was_first == fill_part_one) {
    return;
  }
  q->last_was_first = fill_part_one;
  if (RMT.conf_ch[q->channel].conf1.tx_conti_mode == 0) {
    return;
  }
  uint32_t *data = (uint32_t *)RMT_CHANNEL_MEM(q->channel);
  if (!fill_part_one) {
    data += PART_SIZE;
  }

  uint8_t rp = q->read_idx;
  if (rp == q->next_write_idx) {
    *data = 0;
    q->_isRunning = false;
    RMT.conf_ch[q->channel].conf1.tx_conti_mode = 0;
  } else {
    struct queue_entry *e_curr = &q->entry[rp & QUEUE_LEN_MASK];

    uint8_t steps = e_curr->steps;
    uint16_t ticks = e_curr->ticks;
    if (steps == 0) {
      for (uint8_t i = 0; i < PART_SIZE - 1; i++) {
        // two pauses à 16 ticks
        *data++ = 0x00100010;
        ticks -= 32;
      }
      uint16_t ticks_l = ticks >> 1;
      uint16_t ticks_r = ticks - ticks_l;
      uint32_t rmt_entry = ticks_l;
      rmt_entry <<= 16;
      rmt_entry |= ticks_r;
      *data = rmt_entry;
    } else if (ticks == 0xffff) {
      // special treatment for this case, because an rmt entry can only cover up
      // to 0xfffe ticks every step must be minimum split into two rmt entries,
      // so at max PART/2 steps can be done.
      if (steps < PART_SIZE / 2) {
        for (uint8_t i = 1; i < steps; i++) {
          // steps-1 iterations
          *data++ = 0x80ff0100;
          *data++ = 0x7f007f00;
        }
        *data++ = 0x80ff0100;
        uint32_t remaining = 0x7f007f00;
        // 2*(steps - 1) + 1 already stored => 2*steps - 1
        // and after this for loop one entry added => 2*steps
        for (uint8_t i = 2 * steps; i < PART_SIZE; i++) {
          *data++ = 0x01000100;
          remaining -= 0x01000100;
        }
        *data = remaining;
        steps = 0;
      } else {
        steps -= PART_SIZE / 2;
        for (uint8_t i = 0; i < PART_SIZE / 2; i++) {
          *data++ = 0x80ff0100;
          *data++ = 0x7f007f00;
        }
      }
    } else if ((steps < 2 * PART_SIZE) && (steps != PART_SIZE)) {
      uint8_t steps_to_do = steps;
      if (steps > PART_SIZE) {
        steps_to_do = steps / 2;
      }

      uint16_t ticks_l = ticks >> 1;
      uint16_t ticks_r = ticks - ticks_l;
      uint32_t rmt_entry = ticks_l | 0x8000;  // with step
      rmt_entry <<= 16;
      rmt_entry |= ticks_r;
      for (uint8_t i = 1; i < steps_to_do; i++) {
        *data++ = rmt_entry;
      }
      uint32_t *last_step_data = data++;
      for (uint8_t i = steps_to_do; i < PART_SIZE; i++) {
        *data++ = 0x00040004;
        rmt_entry -= 0x00040004;
      }
      *last_step_data = rmt_entry;
      steps -= steps_to_do;
    } else {
      // every entry one step
      uint16_t ticks_l = ticks >> 1;
      uint16_t ticks_r = ticks - ticks_l;
      uint32_t rmt_entry = ticks_l | 0x8000;  // with step
      rmt_entry <<= 16;
      rmt_entry |= ticks_r;
      for (uint8_t i = 0; i < PART_SIZE; i++) {
        *data++ = rmt_entry;
      }
      // This could lead to single step at high speed in next part .... so need
      // to rework The previous part tries to address this
      steps -= PART_SIZE;
    }
    if (steps == 0) {
      // The command has been completed
      rp++;
      q->read_idx = rp;
      struct queue_entry *e_next = &q->entry[rp & QUEUE_LEN_MASK];
      // The dir pin toggle at this place is problematic, but if the last
      // command contains only one step, it could work
      if (e_next->toggle_dir) {
        gpio_num_t dirPin = (gpio_num_t)q->dirPin;
        gpio_set_level(dirPin, gpio_get_level(dirPin) ^ 1);
      }
    } else {
      e_curr->steps = steps;
    }
  }
}

static void IRAM_ATTR init_stop(StepperQueue *q) {
  rmt_tx_stop(q->channel);
  q->_isRunning = false;
}

static void IRAM_ATTR tx_intr_handler(void *arg) {
  uint32_t mask = RMT.int_st.val;
  RMT.int_clr.val = mask;
  if (mask & RMT_CH0_TX_END_INT_ST) {
    apply_command(&fas_queue[6], false);
  }
  if (mask & RMT_CH1_TX_END_INT_ST) {
    apply_command(&fas_queue[7], false);
  }
  if (mask & RMT_CH0_TX_THR_EVENT_INT_ST) {
    apply_command(&fas_queue[6], true);
  }
  if (mask & RMT_CH1_TX_THR_EVENT_INT_ST) {
    apply_command(&fas_queue[7], true);
  }
}

void StepperQueue::init_rmt(uint8_t channel_num, uint8_t step_pin) {
#ifdef TEST_PROBE
  pinMode(TEST_PROBE, OUTPUT);
#endif

  _initVars();
  _step_pin = step_pin;

  channel = (rmt_channel_t)channel_num;

  if (channel_num == 0) {
    periph_module_enable(PERIPH_RMT_MODULE);
  }

  // 80 MHz/5 = 16 MHz
  rmt_set_source_clk(channel, RMT_BASECLK_APB);
  rmt_set_clk_div(channel, 5);
  rmt_set_mem_block_num(channel, 1);
  rmt_set_tx_carrier(channel, false, 0, 0, RMT_CARRIER_LEVEL_LOW);
  rmt_tx_stop(channel);
  rmt_rx_stop(channel);
  if (channel_num == 0) {
    rmt_isr_register(tx_intr_handler, NULL,
                     ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM, NULL);
    RMT.apb_conf.fifo_mask = 1;
    RMT.apb_conf.mem_tx_wrap_en = 1;
  }

  digitalWrite(step_pin, LOW);
  pinMode(step_pin, OUTPUT);

  connect();
}

void StepperQueue::connect_rmt() {
#ifndef __ESP32_IDF_V44__
  rmt_set_pin(channel, RMT_MODE_TX, (gpio_num_t)_step_pin);
#else
  rmt_set_gpio(channel, RMT_MODE_TX, (gpio_num_t)_step_pin, false);
#endif
}

void StepperQueue::disconnect_rmt() {
  rmt_set_tx_intr_en(channel, false);
  rmt_set_tx_thr_intr_en(channel, false, 0);
#ifndef __ESP32_IDF_V44__
//  rmt_set_pin(channel, RMT_MODE_TX, (gpio_num_t)-1);
#else
  rmt_set_gpio(channel, RMT_MODE_TX, GPIO_NUM_NC, false);
#endif
}

void StepperQueue::startQueue_rmt() {
  Serial.println("START");
#ifdef TEST_PROBE
  // The time used by this command can have an impact
  digitalWrite(TEST_PROBE, digitalRead(TEST_PROBE) == HIGH ? LOW : HIGH);
#endif

  _isRunning = true;
  rmt_memory_rw_rst(channel);
  for (uint8_t i = 0; i < 2 * PART_SIZE; i++) {
    uint32_t *mem = (uint32_t *)RMT_CHANNEL_MEM(channel);
    mem[i] = 0x7fff7fff;
    mem[2 * PART_SIZE] = 0;
  }
  rmt_set_tx_intr_en(channel, true);
  rmt_set_tx_thr_intr_en(channel, true, PART_SIZE);
  RMT.conf_ch[channel].conf1.tx_conti_mode = 1;
  last_was_first = false;
  apply_command(this, true);
  apply_command(this, false);
  rmt_tx_start(this->channel, true);
}
void StepperQueue::forceStop_rmt() {
  init_stop(this);
  read_idx = next_write_idx;
}
uint16_t StepperQueue::_getPerformedPulses_rmt() { return 0; }
#endif
