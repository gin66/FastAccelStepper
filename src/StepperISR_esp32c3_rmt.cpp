#include "StepperISR.h"
#ifdef SUPPORT_ESP32C3_RMT

//#define TEST_MODE
//#define TRACE

#include "test_probe.h"

// The following concept is in use:
//
//    The rmt buffer is split into two parts.
//    Each part will hold one command (or part of).
//    After the two parts an end marker is placed.
//
// Of these 32 bits, the low 16-bit entry is sent first and the high entry
// second.
// Every 16 bit entry defines with MSB the output level and the lower 15 bits
// the ticks.
//
// Important difference of esp32c3 (compared to esp32):
// - configuration updates need an conf_update strobe
//   (apparently the manual is not correct by mentioning to set conf_update
//   first)
// - if the end marker is hit in continuous mode, there is no end interrupt
// - there is no tick lost with the end marker
// - minimum periods as per relation 1 and 2 to be adhered to
// 
//
#ifdef SUPPORT_ESP32C3_RMT
#define PART_SIZE 23
#define RMT_MEM_SIZE 48
#else
#error
#define PART_SIZE 31
#define RMT_MEM_SIZE 64
#endif

// In order to avoid threshold/end interrupt on end, add one
#define enable_rmt_interrupts(channel) {          \
    RMT.tx_lim[channel].RMT_LIMIT = PART_SIZE + 2;\
    RMT.tx_conf[channel].conf_update = 1;         \
    RMT.tx_conf[channel].conf_update = 0;         \
	RMT.int_clr.val |= 0x101 << channel;          \
	RMT.int_ena.val |= 0x101 << channel;          \
}
#define disable_rmt_interrupts(channel) { \
	RMT.int_ena.val &= ~(0x101 << channel); \
}

void IRAM_ATTR StepperQueue::stop_rmt(bool both) {
  // We are stopping the rmt by letting it run into the end at high speed.
  //
  // disable the interrupts
  //  rmt_set_tx_intr_en(channel, false);
  //  rmt_set_tx_thr_intr_en(channel, false, 0);

  // stop esp32 rmt, by let it hit the end
  RMT.tx_conf[channel].tx_conti_mode = 0;
  RMT.tx_conf[channel].conf_update = 1;
  RMT.tx_conf[channel].conf_update = 0;

  // replace second part of buffer with pauses
  uint32_t *data = FAS_RMT_MEM(channel);
  uint8_t start = both ? 0 : PART_SIZE;
  data = &data[start];
  for (uint8_t i = start; i < 2 * PART_SIZE; i++) {
    // two pauses à n ticks to achieve MIN_CMD_TICKS
    *data++ = 0x00010001 * ((MIN_CMD_TICKS + 61) / 62);
  }
  *data = 0;

  // as the rmt is not running anymore, mark it as stopped
  _rmtStopped = true;
}

static void IRAM_ATTR apply_command(StepperQueue *q, bool fill_part_one,
                                    uint32_t *data) {
  if (!fill_part_one) {
    data += PART_SIZE;
  }
  uint8_t rp = q->read_idx;
  if (rp == q->next_write_idx) {
    // no command in queue
    if (fill_part_one) {
      q->bufferContainsSteps[0] = false;
      for (uint8_t i = 0; i < PART_SIZE; i++) {
        // make a pause with approx. 1ms
        //    258 ticks * 2 * 31 = 15996 @ 16MHz
        //    347 ticks * 2 * 23 = 15962 @ 16MHz
        *data++ = 0x010001 * (16000/2/PART_SIZE);
      }
    } else {
      q->stop_rmt(false);
    }
    return;
  }
  // Process command
  struct queue_entry *e_curr = &q->entry[rp & QUEUE_LEN_MASK];

  if (e_curr->toggle_dir) {
    // the command requests dir pin toggle
    // This is ok only, if the ongoing command does not contain steps
    if (q->bufferContainsSteps[fill_part_one ? 1 : 0]) {
      // So we need a pause. change the finished read entry into a pause
      q->bufferContainsSteps[fill_part_one ? 0 : 1] = false;
      for (uint8_t i = 0; i < PART_SIZE; i++) {
        // two pauses à n ticks to achieve MIN_CMD_TICKS
        *data++ = 0x00010001 *
                  ((MIN_CMD_TICKS + 2 * PART_SIZE - 1) / (2 * PART_SIZE));
      }
      return;
    }
    // The ongoing command does not contain steps, so change dir here should be
    // ok
    gpio_num_t dirPin = (gpio_num_t)q->dirPin;
    gpio_set_level(dirPin, gpio_get_level(dirPin) ^ 1);
    // and delete the request
    e_curr->toggle_dir = 0;
  }

  uint8_t steps = e_curr->steps;
  uint16_t ticks = e_curr->ticks;
  //  if (steps != 0) {
  //  	PROBE_1_TOGGLE;
  //}
  uint32_t last_entry;
  if (steps == 0) {
    q->bufferContainsSteps[fill_part_one ? 0 : 1] = false;
    // Perhaps the rmt performs look ahead
	ticks -= (PART_SIZE-2) * 4 + 8;
    uint16_t ticks_l = ticks >> 1;
    uint16_t ticks_r = ticks - ticks_l;
    last_entry = ticks_l;
    last_entry <<= 16;
    last_entry |= ticks_r;
	*data++ = last_entry;
    for (uint8_t i = 1; i < PART_SIZE - 1; i++) {
      *data++ = 0x00020002;
    }
	last_entry = 0x00040004;
  } else {
    q->bufferContainsSteps[fill_part_one ? 0 : 1] = true;
    if (ticks == 0xffff) {
      // special treatment for this case, because an rmt entry can only cover up
      // to 0xfffe ticks every step must be minimum split into two rmt entries,
      // so at max PART/2 steps can be done.
      if (steps < PART_SIZE / 2) {
        for (uint8_t i = 1; i < steps; i++) {
          // steps-1 iterations
          *data++ = 0x40007fff | 0x8000;
          *data++ = 0x20002000;
        }
        *data++ = 0x40007fff | 0x8000;
        uint16_t delta = PART_SIZE - 2 * steps;
        delta <<= 5;
        *data++ = 0x20002000 - delta;
        // 2*(steps - 1) + 1 already stored => 2*steps - 1
        // and after this for loop one entry added => 2*steps
        for (uint8_t i = 2 * steps; i < PART_SIZE - 1; i++) {
          *data++ = 0x00100010;
        }
        last_entry = 0x00100010;
        steps = 0;
      } else {
        steps -= PART_SIZE / 2;
        for (uint8_t i = 0; i < PART_SIZE / 2 - 1; i++) {
          *data++ = 0x40007fff | 0x8000;
          *data++ = 0x20002000;
        }
        *data++ = 0x40007fff | 0x8000;
        last_entry = 0x20002000;
      }
    } else if ((steps < 2 * PART_SIZE) && (steps != PART_SIZE)) {
      uint8_t steps_to_do = steps;
      if (steps > PART_SIZE) {
        steps_to_do /= 2;
      }

      uint16_t ticks_l = ticks >> 1;
      uint16_t ticks_r = ticks - ticks_l;
      uint32_t rmt_entry = ticks_l;
      rmt_entry <<= 16;
      rmt_entry |= ticks_r | 0x8000;  // with step
      for (uint8_t i = 1; i < steps_to_do; i++) {
        *data++ = rmt_entry;
      }
      uint32_t delta = (PART_SIZE - steps_to_do) * 4 + 8;
      delta <<= 16;  // shift in upper 16bit
      *data++ = rmt_entry - delta;
      for (uint8_t i = steps_to_do; i < PART_SIZE - 1; i++) {
        *data++ = 0x00020002;
      }
      last_entry = 0x00040004;
      steps -= steps_to_do;
    } else {
      // either >= 2*PART_SIZE or = PART_SIZE
      // every entry one step
      uint16_t ticks_l = ticks >> 1;
      uint16_t ticks_r = ticks - ticks_l;
      uint32_t rmt_entry = ticks_l;
      rmt_entry <<= 16;
      rmt_entry |= ticks_r | 0x8000;  // with step
      for (uint8_t i = 0; i < PART_SIZE - 1; i++) {
        *data++ = rmt_entry;
      }
      last_entry = rmt_entry;
      steps -= PART_SIZE;
    }
  }
  // No tick lost mentioned for esp32c3
  // if (!fill_part_one) {
    // Note: When enabling the continuous transmission mode by setting
    // RMT_REG_TX_CONTI_MODE, the transmitter will transmit the data on the
    // channel continuously, that is, from the first byte to the last one,
    // then from the first to the last again, and so on. In this mode, there
    // will be an idle level lasting one clk_div cycle between N and N+1
    // transmissions.
    // last_entry -= 1;
  // }
  *data = last_entry;

  // Data is complete
  if (steps == 0) {
    // The command has been completed
    if (e_curr->repeat_entry == 0) {
      q->read_idx = rp + 1;
    }
  } else {
    e_curr->steps = steps;
  }
}

#if !defined(RMT_CHANNEL_MEM) && !defined(SUPPORT_ESP32C3_RMT)
#define RMT_LIMIT tx_lim
#define RMT_FIFO apb_fifo_mask
#else
#define RMT_LIMIT limit
#define RMT_FIFO fifo_mask
#endif

// The threshold interrupts are happening in the "middle" of the previous entry.
// Best strategy:
// 1. enable threshold interrupt with PART_SIZE+2
// 2. On every threshold interrupt toggle PART_SIZE and PART_SIZE+1
// This way the first interrupt happens on the first entry of second half,
// and the second interrupt on the first entry of the first half.
// Afterwards alternating. This way the end interrupt is always "half buffer
// away" from the threshold interrupt
#ifdef TEST_MODE
#define PROCESS_CHANNEL(ch)                       \
  if (mask & RMT_CH##ch##_TX_END_INT_ST) {        \
    PROBE_1_TOGGLE;                               \
  }                                               \
  if (mask & RMT_CH##ch##_TX_THR_EVENT_INT_ST) {  \
    uint8_t old_limit = RMT.tx_lim[ch].RMT_LIMIT; \
    if (old_limit == PART_SIZE + 1) {             \
      /* second half of buffer sent */            \
      PROBE_2_TOGGLE;                             \
      /* demonstrate modification of RAM */       \
      uint32_t *mem = FAS_RMT_MEM(ch);            \
	  mem[PART_SIZE] = 0x33ff33ff;                \
      RMT.tx_lim[ch].RMT_LIMIT = PART_SIZE;       \
    } else {                                      \
      /* first half of buffer sent */             \
      PROBE_3_TOGGLE;                             \
      RMT.tx_lim[ch].RMT_LIMIT = PART_SIZE + 1;   \
    }                                             \
    RMT.tx_conf[ch].conf_update = 1;              \
    RMT.tx_conf[ch].conf_update = 0;              \
  }
#else
#define PROCESS_CHANNEL(ch)                                      \
  if (mask & RMT_CH##ch##_TX_END_INT_ST) {        \
    StepperQueue *q = &fas_queue[QUEUES_MCPWM_PCNT + ch];        \
	disable_rmt_interrupts(q->channel);                        \
    q->_isRunning = false;                                     \
    PROBE_1_TOGGLE;                                            \
    PROBE_2_TOGGLE;                                            \
    PROBE_3_TOGGLE;                                            \
  }                                               \
  if (mask & RMT_CH##ch##_TX_THR_EVENT_INT_ST) {  \
    uint8_t old_limit = RMT.tx_lim[ch].RMT_LIMIT; \
    StepperQueue *q = &fas_queue[QUEUES_MCPWM_PCNT + ch];        \
      uint32_t *mem = FAS_RMT_MEM(ch);            \
    if (old_limit == PART_SIZE + 1) {             \
      /* second half of buffer sent */            \
      PROBE_2_TOGGLE;                             \
      apply_command(q, false, mem);                  \
      /* demonstrate modification of RAM */       \
	  /*mem[PART_SIZE] = 0x33fff3ff;       */         \
      RMT.tx_lim[ch].RMT_LIMIT = PART_SIZE;       \
    } else {                                      \
      /* first half of buffer sent */             \
      PROBE_3_TOGGLE;                             \
      apply_command(q, true, mem);                   \
      RMT.tx_lim[ch].RMT_LIMIT = PART_SIZE + 1;   \
    }                                             \
    RMT.tx_conf[ch].conf_update = 1;              \
    RMT.tx_conf[ch].conf_update = 0;              \
  }
#endif

static void IRAM_ATTR tx_intr_handler(void *arg) {
  uint32_t mask = RMT.int_st.val;
  RMT.int_clr.val = mask;
  PROCESS_CHANNEL(0);
  PROCESS_CHANNEL(1);
#if QUEUES_RMT >= 4
  PROCESS_CHANNEL(2);
  PROCESS_CHANNEL(3);
#endif
#if QUEUES_RMT == 8
  PROCESS_CHANNEL(4);
  PROCESS_CHANNEL(5);
  PROCESS_CHANNEL(6);
  PROCESS_CHANNEL(7);
#endif
}

void StepperQueue::init_rmt(uint8_t channel_num, uint8_t step_pin) {
#ifdef TEST_PROBE_1
  if (channel_num == 0) {
    pinMode(TEST_PROBE_1, OUTPUT);
    PROBE_1(HIGH);
    delay(10);
    PROBE_1(LOW);
    delay(5);
    PROBE_1(HIGH);
    delay(5);
    PROBE_1(LOW);
    delay(5);
  }
#endif
#ifdef TEST_PROBE_2
  pinMode(TEST_PROBE_2, OUTPUT);
  PROBE_2(LOW);
#endif
#ifdef TEST_PROBE_3
  pinMode(TEST_PROBE_3, OUTPUT);
  PROBE_3(LOW);
#endif

  _initVars();
  _step_pin = step_pin;
  //  digitalWrite(step_pin, LOW);
  //  pinMode(step_pin, OUTPUT);

  channel = (rmt_channel_t)channel_num;

  if (channel_num == 0) {
    periph_module_enable(PERIPH_RMT_MODULE);
  }
  // APB_CLOCK=80 MHz
  // CLK_DIV = APB_CLOCK/5 = 16 MHz
  //
  // Relation 1 in esp32c3 technical reference:
  //      3 * T_APB + 5 * T_RMT_CLK < period * T_CLK_DIV
  //      => 8 * T_APB < period * T_APB*5
  //      => period > 8/5
  //      => period >= 2 
  // 
  // Relation 2 in esp32c3 technical reference before end marker:
  //      6 * T_APB + 12 * T_RMT_CLK < period * T_CLK_DIV
  //      => 18 * T_APB < period * T_APB*5
  //      => period > 18/5
  //      => period >= 4 
  // 
  disable_rmt_interrupts(channel);
  rmt_set_source_clk(channel, RMT_BASECLK_APB);
  rmt_set_clk_div(channel, 5);
  rmt_set_mem_block_num(channel, 1);
  rmt_set_tx_carrier(channel, false, 0, 0, RMT_CARRIER_LEVEL_LOW);
  rmt_tx_stop(channel);
  rmt_memory_rw_rst(channel);
  // rmt_rx_stop(channel); TX only channel !
  //  rmt_tx_memory_reset is not defined in arduino V340 and based on test
  //  result not needed rmt_tx_memory_reset(channel);
  if (channel_num == 0) {
    rmt_isr_register(tx_intr_handler, NULL,
                     ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM, NULL);
    RMT.sys_conf.fifo_mask = 1;  // disable fifo mode
  }

  _isRunning = false;
  _rmtStopped = true;

  connect_rmt();

#ifdef TEST_MODE
  if (channel == 0) {
    RMT.tx_conf[channel].mem_rd_rst = 1;
    RMT.tx_conf[channel].mem_rd_rst = 0;
    uint32_t *mem = FAS_RMT_MEM(channel);
    // Fill the buffer with a significant pattern for debugging
    for (uint8_t i = 0; i < PART_SIZE; i++) {
      *mem++ = 0x7fffffff;
    }
    for (uint8_t i = PART_SIZE; i < 2 * PART_SIZE; i++) {
      *mem++ = 0x3fffdfff;
    }
    // without end marker it does not loop
    *mem++ = 0;
    *mem++ = 0;

    // conti mode is accepted with the conf_update 1 strobe
    RMT.tx_conf[channel].tx_conti_mode = 1;
    RMT.tx_conf[channel].conf_update = 1;
    RMT.tx_conf[channel].conf_update = 0;
    RMT.tx_conf[channel].mem_tx_wrap_en = 0;
    RMT.tx_conf[channel].conf_update = 1;
    RMT.tx_conf[channel].conf_update = 0;
	enable_rmt_interrupts(channel);
    // tx_start does not need conf_update
    PROBE_1_TOGGLE;  // end interrupt will toggle again PROBE_1
    RMT.tx_conf[channel].tx_start = 1;

    delay(1000);
    if (false) {
      mem--;
      mem--;
      // destroy end marker => no end interrupt, no repeat
      *mem++ = 0x00010001;
      *mem = 0x00010001;
    }
    if (true) {
      // just clear conti mode => causes end interrupt, no repeat
      RMT.tx_conf[channel].tx_conti_mode = 0;
      RMT.tx_conf[channel].conf_update = 1;
      RMT.tx_conf[channel].conf_update = 0;
    }
    delay(1000);
    // actually no need to enable/disable interrupts.
    // and this seems to avoid some pitfalls

    // This runs the RMT buffer once
    RMT.tx_conf[channel].tx_conti_mode = 0;
    RMT.tx_conf[channel].conf_update = 1;
    RMT.tx_conf[channel].conf_update = 0;
    RMT.tx_conf[channel].tx_start = 1;
    while (true) {
      delay(1000);
      PROBE_1_TOGGLE;
    }
  }
#endif
}

void StepperQueue::connect_rmt() {
  RMT.tx_conf[channel].idle_out_lv = 0;
  RMT.tx_conf[channel].idle_out_en = 1;
  RMT.tx_conf[channel].conf_update = 1;
  RMT.tx_conf[channel].conf_update = 0;
  // RMT.tx_conf[channel].mem_tx_wrap_en = 0;
#ifndef __ESP32_IDF_V44__
  rmt_set_pin(channel, RMT_MODE_TX, (gpio_num_t)_step_pin);
#else
  rmt_set_gpio(channel, RMT_MODE_TX, (gpio_num_t)_step_pin, false);
#endif

#ifdef TEST_MODE
  // here gpio is 0
  delay(1);
  PROBE_3_TOGGLE;
  RMT.tx_conf[channel].idle_out_lv = 1;
  RMT.tx_conf[channel].conf_update = 1;
  RMT.tx_conf[channel].conf_update = 0;
  // here gpio is 1
  delay(2);
  PROBE_3_TOGGLE;
  RMT.tx_conf[channel].idle_out_lv = 0;
  RMT.tx_conf[channel].conf_update = 1;
  RMT.tx_conf[channel].conf_update = 0;
  // here gpio is 0
  delay(2);
  PROBE_3_TOGGLE;
  RMT.tx_conf[channel].idle_out_en = 0;
  RMT.tx_conf[channel].conf_update = 1;
  RMT.tx_conf[channel].conf_update = 0;
  // here gpio is 0
  delay(2);
  PROBE_3_TOGGLE;
  RMT.tx_conf[channel].idle_out_lv = 1;
  RMT.tx_conf[channel].idle_out_en = 1;
  RMT.tx_conf[channel].conf_update = 1;
  RMT.tx_conf[channel].conf_update = 0;
  // here gpio is 1
  delay(2);
  PROBE_3_TOGGLE;
  RMT.tx_conf[channel].idle_out_lv = 0;
  RMT.tx_conf[channel].conf_update = 1;
  RMT.tx_conf[channel].conf_update = 0;
  // here gpio is 0
  delay(2);
  PROBE_3_TOGGLE;
#endif
}

void StepperQueue::disconnect_rmt() {
  disable_rmt_interrupts(channel);
#ifndef __ESP32_IDF_V44__
//  rmt_set_pin(channel, RMT_MODE_TX, (gpio_num_t)-1);
#else
//  rmt_set_gpio(channel, RMT_MODE_TX, GPIO_NUM_NC, false);
#endif
  RMT.tx_conf[channel].idle_out_en = 0;
  RMT.tx_conf[channel].conf_update = 1;
  RMT.tx_conf[channel].conf_update = 0;
}

void StepperQueue::startQueue_rmt() {
#ifdef TRACE
  USBSerial.println("START");
#endif
#ifdef TEST_PROBE_2
  PROBE_2(LOW);
#endif
#ifdef TEST_PROBE_3
  PROBE_3(LOW);
#endif
#if defined(TEST_PROBE_1) && defined(TEST_PROBE_2) && defined(TEST_PROBE_3)
  delay(1);
  PROBE_1_TOGGLE;
  delay(1);
  PROBE_1_TOGGLE;
  delay(1);
  PROBE_1_TOGGLE;
  delay(1);
  PROBE_2_TOGGLE;
  delay(2);
  PROBE_2_TOGGLE;
  delay(2);
  PROBE_3_TOGGLE;
  delay(3);
  PROBE_3_TOGGLE;
  delay(3);
#endif
  rmt_tx_stop(channel);
  // rmt_rx_stop(channel);
  RMT.tx_conf[channel].mem_rd_rst = 1;
  RMT.tx_conf[channel].mem_rd_rst = 0;
  uint32_t *mem = FAS_RMT_MEM(channel);
//#define TRACE
#ifdef TRACE
  // Fill the buffer with a significant pattern for debugging
  // Keep it for now
  for (uint8_t i = 0; i < 2 * PART_SIZE; i += 2) {
//    mem[i] = 0x0fff8fff;
//    mem[i + 1] = 0x7fff8fff;
  }
#endif
  // Write end marker
  mem[2 * PART_SIZE] = 0;
  mem[2 * PART_SIZE + 1] = 0;
  _isRunning = true;
  _rmtStopped = false;
  disable_rmt_interrupts(channel);
  RMT.tx_conf[channel].mem_tx_wrap_en = 0;
  RMT.tx_conf[channel].conf_update = 1;
  RMT.tx_conf[channel].conf_update = 0;

#ifdef TRACE
  USBSerial.print("Queue:");
  USBSerial.print(read_idx);
  USBSerial.print('/');
  USBSerial.println(next_write_idx);
  for (uint8_t i = 0; i < RMT_MEM_SIZE; i++) {
    USBSerial.print(i);
    USBSerial.print(' ');
    USBSerial.println(mem[i], HEX);
  }
#endif

  // set dirpin toggle here
  uint8_t rp = read_idx;
  if (rp == next_write_idx) {
    // nothing to do ?
    // Should not happen, so bail
    return;
  }
  if (entry[rp & QUEUE_LEN_MASK].toggle_dir) {
    gpio_set_level((gpio_num_t)dirPin, gpio_get_level((gpio_num_t)dirPin) ^ 1);
    entry[rp & QUEUE_LEN_MASK].toggle_dir = false;
  }

  bufferContainsSteps[0] = true;
  bufferContainsSteps[1] = true;
  apply_command(this, true, mem);

#ifdef TRACE
  USBSerial.print(RMT.tx_conf[channel].val, BIN);
  USBSerial.println(' ');
  USBSerial.print(RMT.tx_conf[channel].mem_tx_wrap_en);
  USBSerial.println(' ');
  for (uint8_t i = 0; i < RMT_MEM_SIZE; i++) {
    USBSerial.print(i);
    USBSerial.print(' ');
    USBSerial.println(mem[i], HEX);
  }
  if (!isRunning()) {
    USBSerial.println("STOPPED 1");
  }
#endif

  apply_command(this, false, mem);

#ifdef TRACE
  USBSerial.print(RMT.tx_conf[channel].val, BIN);
  USBSerial.print(' ');
  USBSerial.print(RMT.tx_conf[channel].mem_tx_wrap_en);
  USBSerial.println(' ');

  for (uint8_t i = 0; i < RMT_MEM_SIZE; i++) {
    USBSerial.print(i);
    USBSerial.print(' ');
    USBSerial.println(mem[i], HEX);
  }
#endif
  enable_rmt_interrupts(channel);

#ifdef TRACE
  USBSerial.print("Interrupt enable:");
  USBSerial.println(RMT.int_ena.val, BIN);
  USBSerial.print("Interrupt status:");
  USBSerial.println(RMT.int_st.val, BIN);
  USBSerial.print("Threshold: 0x");
  USBSerial.println(RMT.tx_lim[channel].val, HEX);
#endif

  // This starts the rmt module
  RMT.tx_conf[channel].tx_conti_mode = 1;
  RMT.tx_conf[channel].conf_update = 1;
  RMT.tx_conf[channel].conf_update = 0;
    RMT.tx_conf[channel].mem_tx_wrap_en = 0;
    RMT.tx_conf[channel].conf_update = 1;
    RMT.tx_conf[channel].conf_update = 0;

  PROBE_1_TOGGLE;
  RMT.tx_conf[channel].tx_start = 1;
}
void StepperQueue::forceStop_rmt() {
  stop_rmt(true);

  // and empty the buffer
  read_idx = next_write_idx;
}
bool StepperQueue::isReadyForCommands_rmt() {
  if (_isRunning) {
    return !_rmtStopped;
  }
  return true;
}
uint16_t StepperQueue::_getPerformedPulses_rmt() { return 0; }
#endif
