#include "StepperISR.h"
#if defined(HAVE_ESP32S3_RMT) && (ESP_IDF_VERSION_MAJOR == 4)

// #define TEST_MODE
// #define TRACE

#include "fas_arch/test_probe.h"

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
// Important difference of esp32s3 (compared to esp32):
// - configuration updates need an conf_update strobe
//   (apparently the manual is not correct by mentioning to set conf_update
//   first)
// - if the end marker is hit in continuous mode, there is no end interrupt
// - there is no tick lost with the end marker
// - minimum periods as per relation 1 and 2 to be adhered to
//
//

// In order to avoid threshold/end interrupt on end, add one
#define enable_rmt_interrupts(channel)                 \
  {                                                    \
    RMT.chn_tx_lim[channel].RMT_LIMIT = PART_SIZE + 2; \
    RMT.chnconf0[channel].conf_update_n = 1;           \
    RMT.chnconf0[channel].conf_update_n = 0;           \
    RMT.int_clr.val |= 0x101 << channel;               \
    RMT.int_ena.val |= 0x101 << channel;               \
  }
#define disable_rmt_interrupts(channel)     \
  {                                         \
    RMT.int_ena.val &= ~(0x101 << channel); \
  }

void IRAM_ATTR StepperQueue::stop_rmt(bool both) {
  // We are stopping the rmt by letting it run into the end at high speed.
  //
  // disable the interrupts
  //  rmt_set_tx_intr_en(channel, false);
  //  rmt_set_tx_thr_intr_en(channel, false, 0);

  // stop esp32 rmt, by let it hit the end
  RMT.chnconf0[channel].tx_conti_mode_n = 0;
  RMT.chnconf0[channel].conf_update_n = 1;
  RMT.chnconf0[channel].conf_update_n = 0;

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

#if !defined(RMT_CHANNEL_MEM) && !defined(HAVE_ESP32S3_RMT)
#define RMT_LIMIT tx_lim
#define RMT_FIFO apb_fifo_mask
#else
#define RMT_LIMIT tx_lim_chn
#define RMT_FIFO apb_fifo_mask
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
#define PROCESS_CHANNEL(ch)                           \
  if (mask & RMT_CH##ch##_TX_END_INT_ST) {            \
    PROBE_1_TOGGLE;                                   \
  }                                                   \
  if (mask & RMT_CH##ch##_TX_THR_EVENT_INT_ST) {      \
    uint8_t old_limit = RMT.chn_tx_lim[ch].RMT_LIMIT; \
    if (old_limit == PART_SIZE + 1) {                 \
      /* second half of buffer sent */                \
      PROBE_2_TOGGLE;                                 \
      /* demonstrate modification of RAM */           \
      uint32_t *mem = FAS_RMT_MEM(ch);                \
      mem[PART_SIZE] = 0x33ff33ff;                    \
      RMT.chn_tx_lim[ch].RMT_LIMIT = PART_SIZE;       \
    } else {                                          \
      /* first half of buffer sent */                 \
      PROBE_3_TOGGLE;                                 \
      RMT.chn_tx_lim[ch].RMT_LIMIT = PART_SIZE + 1;   \
    }                                                 \
    RMT.chnconf0[ch].conf_update_n = 1;               \
    RMT.chnconf0[ch].conf_update_n = 0;               \
  }
#else
#define PROCESS_CHANNEL(ch)                               \
  if (mask & RMT_CH##ch##_TX_END_INT_ST) {                \
    StepperQueue *q = &fas_queue[QUEUES_MCPWM_PCNT + ch]; \
    disable_rmt_interrupts(q->channel);                   \
    q->_isRunning = false;                                \
    PROBE_1_TOGGLE;                                       \
    PROBE_2_TOGGLE;                                       \
    PROBE_3_TOGGLE;                                       \
  }                                                       \
  if (mask & RMT_CH##ch##_TX_THR_EVENT_INT_ST) {          \
    uint8_t old_limit = RMT.chn_tx_lim[ch].RMT_LIMIT;     \
    StepperQueue *q = &fas_queue[QUEUES_MCPWM_PCNT + ch]; \
    uint32_t *mem = FAS_RMT_MEM(ch);                      \
    if (old_limit == PART_SIZE + 1) {                     \
      /* second half of buffer sent */                    \
      PROBE_2_TOGGLE;                                     \
      rmt_apply_command(q, false, mem);                   \
      /* demonstrate modification of RAM */               \
      /*mem[PART_SIZE] = 0x33fff3ff;       */             \
      RMT.chn_tx_lim[ch].RMT_LIMIT = PART_SIZE;           \
    } else {                                              \
      /* first half of buffer sent */                     \
      PROBE_3_TOGGLE;                                     \
      rmt_apply_command(q, true, mem);                    \
      RMT.chn_tx_lim[ch].RMT_LIMIT = PART_SIZE + 1;       \
    }                                                     \
    RMT.chnconf0[ch].conf_update_n = 1;                   \
    RMT.chnconf0[ch].conf_update_n = 0;                   \
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

bool StepperQueue::init_rmt(uint8_t channel_num, uint8_t step_pin) {
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
  // Relation 1 in esp32s3 technical reference:
  //      3 * T_APB + 5 * T_RMT_CLK < period * T_CLK_DIV
  //      => 8 * T_APB < period * T_APB*5
  //      => period > 8/5
  //      => period >= 2
  //
  // Relation 2 in esp32s3 technical reference before end marker:
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
  rmt_tx_memory_reset(channel);
  // rmt_rx_stop(channel); TX only channel !
  //  rmt_tx_memory_reset is not defined in arduino V340 and based on test
  //  result not needed rmt_tx_memory_reset(channel);
  if (channel_num == 0) {
    rmt_isr_register(tx_intr_handler, NULL,
                     ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM, NULL);
    RMT.sys_conf.apb_fifo_mask = 1;  // disable fifo mode
  }

  _isRunning = false;
  _rmtStopped = true;

  connect_rmt();

#ifdef TEST_MODE
  if (channel == 0) {
    RMT.chnconf0[channel].mem_rd_rst_n = 1;
    RMT.chnconf0[channel].mem_rd_rst_n = 0;
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
    RMT.chnconf0[channel].tx_conti_mode_n = 1;
    RMT.chnconf0[channel].conf_update_n = 1;
    RMT.chnconf0[channel].conf_update_n = 0;
    RMT.chnconf0[channel].mem_tx_wrap_en_n = 0;
    RMT.chnconf0[channel].conf_update_n = 1;
    RMT.chnconf0[channel].conf_update_n = 0;
    enable_rmt_interrupts(channel);
    // tx_start does not need conf_update
    PROBE_1_TOGGLE;  // end interrupt will toggle again PROBE_1
    RMT.chnconf0[channel].tx_start_n = 1;

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
      RMT.chnconf0[channel].tx_conti_mode_n = 0;
      RMT.chnconf0[channel].conf_update_n = 1;
      RMT.chnconf0[channel].conf_update_n = 0;
    }
    delay(1000);
    // actually no need to enable/disable interrupts.
    // and this seems to avoid some pitfalls

    // This runs the RMT buffer once
    RMT.chnconf0[channel].tx_conti_mode_n = 0;
    RMT.chnconf0[channel].conf_update_n = 1;
    RMT.chnconf0[channel].conf_update_n = 0;
    RMT.chnconf0[channel].tx_start_n = 1;
    while (true) {
      delay(1000);
      PROBE_1_TOGGLE;
    }
  }
#endif
  return true;
}

void StepperQueue::connect_rmt() {
  RMT.chnconf0[channel].idle_out_lv_n = 0;
  RMT.chnconf0[channel].idle_out_en_n = 1;
  RMT.chnconf0[channel].conf_update_n = 1;
  RMT.chnconf0[channel].conf_update_n = 0;
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
  RMT.tx_conf[channel].idle_out_lv_n = 1;
  RMT.tx_conf[channel].conf_update_n = 1;
  RMT.tx_conf[channel].conf_update_n = 0;
  // here gpio is 1
  delay(2);
  PROBE_3_TOGGLE;
  RMT.chnconf0[channel].idle_out_lv_n = 0;
  RMT.chnconf0[channel].conf_update_n = 1;
  RMT.chnconf0[channel].conf_update_n = 0;
  // here gpio is 0
  delay(2);
  PROBE_3_TOGGLE;
  RMT.chnconf0[channel].idle_out_en_n = 0;
  RMT.chnconf0[channel].conf_update_n = 1;
  RMT.chnconf0[channel].conf_update_n = 0;
  // here gpio is 0
  delay(2);
  PROBE_3_TOGGLE;
  RMT.chnconf0[channel].idle_out_lv_n = 1;
  RMT.chnconf0[channel].idle_out_en_n = 1;
  RMT.chnconf0[channel].conf_update_n = 1;
  RMT.chnconf0[channel].conf_update_n = 0;
  // here gpio is 1
  delay(2);
  PROBE_3_TOGGLE;
  RMT.chnconf0[channel].idle_out_lv_n = 0;
  RMT.chnconf0[channel].conf_update_n = 1;
  RMT.chnconf0[channel].conf_update_n = 0;
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
  RMT.chnconf0[channel].idle_out_en_n = 0;
  RMT.chnconf0[channel].conf_update_n = 1;
  RMT.chnconf0[channel].conf_update_n = 0;
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
  RMT.chnconf0[channel].mem_rd_rst_n = 1;
  RMT.chnconf0[channel].mem_rd_rst_n = 0;
  uint32_t *mem = FAS_RMT_MEM(channel);
// #define TRACE
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
  RMT.chnconf0[channel].mem_tx_wrap_en_n = 0;
  RMT.chnconf0[channel].conf_update_n = 1;
  RMT.chnconf0[channel].conf_update_n = 0;

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
    LL_TOGGLE_PIN(dirPin);
    entry[rp & QUEUE_LEN_MASK].toggle_dir = false;
  }

  lastChunkContainsSteps = true;
  rmt_apply_command(this, true, mem);

#ifdef TRACE
  USBSerial.print(RMT.chnconf0[channel].val, BIN);
  USBSerial.println(' ');
  USBSerial.print(RMT.chnconf0[channel].mem_tx_wrap_en_n);
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

  rmt_apply_command(this, false, mem);

#ifdef TRACE
  USBSerial.print(RMT.chnconf0[channel].val, BIN);
  USBSerial.print(' ');
  USBSerial.print(RMT.chnconf0[channel].mem_tx_wrap_en_n);
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
  RMT.chnconf0[channel].tx_conti_mode_n = 1;
  RMT.chnconf0[channel].conf_update_n = 1;
  RMT.chnconf0[channel].conf_update_n = 0;
  RMT.chnconf0[channel].mem_tx_wrap_en_n = 0;
  RMT.chnconf0[channel].conf_update_n = 1;
  RMT.chnconf0[channel].conf_update_n = 0;

  PROBE_1_TOGGLE;
  RMT.chnconf0[channel].tx_start_n = 1;
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
