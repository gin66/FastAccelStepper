#include "StepperISR.h"
#if defined(HAVE_ESP32_RMT) && (ESP_IDF_VERSION_MAJOR == 5)

// #define TEST_MODE

#include "fas_arch/test_probe.h"

#define PART_SIZE (RMT_SIZE / 2)

static bool IRAM_ATTR queue_done(rmt_channel_handle_t tx_chan,
                                 const rmt_tx_done_event_data_t *edata,
                                 void *user_ctx) {
  StepperQueue *q = (StepperQueue *)user_ctx;
  q->_isRunning = false;
  return false;
}

#define ENTER_PAUSE(ticks)                                          \
  {                                                                 \
    uint16_t remaining_ticks = ticks;                               \
    uint16_t half_ticks_per_symbol = ticks / (2 * PART_SIZE);       \
    uint32_t main_symbol = 0x00010001 * half_ticks_per_symbol;      \
    for (uint8_t i = 0; i < PART_SIZE - 1; i++) {                   \
      (*symbols++).val = main_symbol;                               \
    }                                                               \
    remaining_ticks -= 2 * (PART_SIZE - 1) * half_ticks_per_symbol; \
    uint16_t first_ticks = remaining_ticks / 2;                     \
    remaining_ticks -= first_ticks;                                 \
    last_entry = 0x00010000 * first_ticks + remaining_ticks;        \
  }

static size_t IRAM_ATTR encode_commands(const void *data, size_t data_size,
                                        size_t symbols_written,
                                        size_t symbols_free,
                                        rmt_symbol_word_t *symbols, bool *done,
                                        void *arg) {
  // this printf causes Guru Meditation
  // printf("encode commands\n");

  StepperQueue *q = (StepperQueue *)arg;

  *done = false;
  if (symbols_free < PART_SIZE) {
    // not sufficient space for the symbols
    return 0;
  }

  uint8_t rp = q->read_idx;
  if (q->_rmtStopped) {
    *done = true;
    return 0;
  }
  if ((rp == q->next_write_idx) || q->_rmtStopped) {
    // if we return done already here, then single stepping fails
    q->_rmtStopped = true;
    // Not sure if this pause is really needed
    uint16_t last_entry;
    ENTER_PAUSE(MIN_CMD_TICKS);
    symbols->val = last_entry;
    return PART_SIZE;
  }

  // Process command
  struct queue_entry *e_curr = &q->entry[rp & QUEUE_LEN_MASK];

  if (e_curr->toggle_dir) {
    // the command requests dir pin toggle
    // This is ok only, if the ongoing command does not contain steps
    if (q->lastChunkContainsSteps) {
      // So we need a pause. change the finished read entry into a pause
      q->lastChunkContainsSteps = false;
      uint16_t last_entry;
      ENTER_PAUSE(MIN_CMD_TICKS);
      symbols->val = last_entry;
      return PART_SIZE;
    }
    // The ongoing command does not contain steps, so change dir here should be
    // ok
    LL_TOGGLE_PIN(q->dirPin);
    // and delete the request
    e_curr->toggle_dir = 0;
  }

  uint8_t steps = e_curr->steps;
  uint16_t ticks = e_curr->ticks;
  //  if (steps != 0) {
  //  	PROBE_2_TOGGLE;
  //}
  uint32_t last_entry;
  if (steps == 0) {
    q->lastChunkContainsSteps = false;
    ENTER_PAUSE(ticks);
  } else {
    q->lastChunkContainsSteps = true;
    if (ticks == 0xffff) {
      // special treatment for this case, because an rmt entry can only cover up
      // to 0xfffe ticks every step must be minimum split into two rmt entries,
      // so at max PART/2 steps can be done.
      if (steps < PART_SIZE / 2) {
        for (uint8_t i = 1; i < steps; i++) {
          // steps-1 iterations
          (*symbols++).val = 0x40007fff | 0x8000;
          (*symbols++).val = 0x20002000;
        }
        // the last step needs to be stretched to fill PART_SIZE entries
        (*symbols++).val = 0x40007fff | 0x8000;
        uint16_t delta = PART_SIZE - 2 * steps;
        delta <<= 5;
        (*symbols++).val = 0x20002000 - delta;
        // 2*(steps - 1) + 1 already stored => 2*steps - 1
        // and after this for loop one entry added => 2*steps
        for (uint8_t i = 2 * steps; i < PART_SIZE - 1; i++) {
          (*symbols++).val = 0x00100010;
        }
        last_entry = 0x00100010;
        steps = 0;
      } else {
        steps -= PART_SIZE / 2;
        for (uint8_t i = 0; i < PART_SIZE / 2 - 1; i++) {
          (*symbols++).val = 0x40007fff | 0x8000;
          (*symbols++).val = 0x20002000;
        }
        (*symbols++).val = 0x40007fff | 0x8000;
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
        (*symbols++).val = rmt_entry;
      }
      // the last step needs to be stretched to fill PART_SIZE entries
      uint32_t delta = PART_SIZE - steps_to_do;
      delta <<= 18;  // shift in upper 16bit and multiply with 4
      (*symbols++).val = rmt_entry - delta;
      for (uint8_t i = steps_to_do; i < PART_SIZE - 1; i++) {
        (*symbols++).val = 0x00020002;
      }
      last_entry = 0x00020002;
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
        (*symbols++).val = rmt_entry;
      }
      last_entry = rmt_entry;
      steps -= PART_SIZE;
    }
  }

  // if (!fill_part_one) {
  //  Note: When enabling the continuous transmission mode by setting
  //  RMT_REG_TX_CONTI_MODE, the transmitter will transmit the data on the
  //  channel continuously, that is, from the first byte to the last one,
  //  then from the first to the last again, and so on. In this mode, there
  //  will be an idle level lasting one clk_div cycle between N and N+1
  //  transmissions.
  // last_entry -= 1;
  //}
  symbols->val = last_entry;

  // Data is complete
  if (steps == 0) {
    // The command has been completed
    if (e_curr->repeat_entry == 0) {
      q->read_idx = rp + 1;
    }
  } else {
    e_curr->steps = steps;
  }

  return PART_SIZE;
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
  pinMode(step_pin, OUTPUT);
  digitalWrite(step_pin, LOW);

  rmt_simple_encoder_config_t enc_config = {
      .callback = encode_commands, .arg = this, .min_chunk_size = PART_SIZE};
  esp_err_t rc;
  rc = rmt_new_simple_encoder(&enc_config, &_tx_encoder);
  ESP_ERROR_CHECK_WITHOUT_ABORT(rc);

  connect_rmt();
  _isRunning = false;
  _rmtStopped = true;
}

void StepperQueue::connect_rmt() {
  rmt_tx_channel_config_t config{};
  config.gpio_num = (gpio_num_t)_step_pin;
  config.clk_src = RMT_CLK_SRC_DEFAULT;
  config.resolution_hz = TICKS_PER_S;
  config.mem_block_symbols = 2 * PART_SIZE;
  config.trans_queue_depth = 1;
  config.intr_priority = 0;
  config.flags.invert_out = 0;
  config.flags.with_dma = 0;
  config.flags.io_loop_back = 0;
  config.flags.io_od_mode = 0;
  esp_err_t rc = rmt_new_tx_channel(&config, &channel);
  ESP_ERROR_CHECK_WITHOUT_ABORT(rc);

  rmt_tx_event_callbacks_t callbacks = {.on_trans_done = queue_done};
  rmt_tx_register_event_callbacks(channel, &callbacks, this);

  _channel_enabled = false;
}

void StepperQueue::disconnect_rmt() {
  if (_channel_enabled || _isRunning || !_rmtStopped) {
    return;
  }
  rmt_del_channel(channel);
  channel = NULL;
}

void StepperQueue::startQueue_rmt() {
// #define TRACE
#ifdef TRACE
  Serial.println("START");
#endif
#ifdef TEST_PROBE_2
  PROBE_2(LOW);
#endif
#ifdef TEST_PROBE_3
  PROBE_3(LOW);
#endif
#ifdef TEST_PROBE_1
  delay(1);
  PROBE_1_TOGGLE;
  delay(1);
  PROBE_1_TOGGLE;
  delay(1);
  PROBE_1_TOGGLE;
  delay(1);
#endif

// #define TRACE
#ifdef TRACE
  printf("Queue: %d/%d %s\n", read_idx, next_write_idx,
         _isRunning ? "Running" : "Stopped");
#endif

  if (channel == NULL) {
    return;
  }

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

#ifdef TRACE
  queue_entry *e = &entry[rp & QUEUE_LEN_MASK];
  printf("first command: ticks=%u steps=%u %s %s\n", e->ticks, e->steps,
         e->countUp ? "up" : "down", e->toggle_dir ? "toggle" : "");
#endif

  if (_channel_enabled) {
    //	rmt_disable(channel);
    //	_channel_enabled = false;
  }

  lastChunkContainsSteps = false;
  _isRunning = true;
  _rmtStopped = false;

  // payload and payload bytes must not be 0
  int payload = 0;
  rmt_transmit_config_t tx_config;
  tx_config.loop_count = 0;
  tx_config.flags.eot_level = 0;  // output level at end of transmission
  tx_config.flags.queue_nonblocking = 1;
  _tx_encoder->reset(_tx_encoder);

  if (!_channel_enabled) {
    rmt_enable(channel);
    _channel_enabled = true;
  }

  esp_err_t rc = rmt_transmit(channel, _tx_encoder, &payload, 1, &tx_config);
  ESP_ERROR_CHECK_WITHOUT_ABORT(rc);
#ifdef TRACE
  printf("Transmission started\n");
#endif
}
void StepperQueue::forceStop_rmt() {
  rmt_disable(channel);
  _channel_enabled = false;
  _isRunning = false;
  _rmtStopped = true;

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
