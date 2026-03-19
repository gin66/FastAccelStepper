#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_MCPWM_PCNT) && (ESP_IDF_VERSION_MAJOR >= 5)

#include <esp_intr_alloc.h>

#define TIMER_BIT(timer) (1 << (timer + 15))

struct pcnt_unit_internal {
  void* group;
  portMUX_TYPE spinlock;
  int unit_id;
};

struct mapping_s {
  mcpwm_timer_handle_t timer;
  mcpwm_oper_handle_t oper;
  mcpwm_cmpr_handle_t cmpr;
  mcpwm_gen_handle_t gen;
  pcnt_unit_handle_t pcnt_unit;
  uint8_t pcnt_unit_id;
  mcpwm_dev_t* mcpwm;
};

static struct mapping_s channel2mapping[NUM_QUEUES];
static StepperQueue* pcnt_unit_to_queue[QUEUES_MCPWM_PCNT] = {};
static bool pcnt_isr_installed = false;

#define isr_pcnt_counter_clear(pcnt_unit_id)             \
  REG_SET_BIT(PCNT_CTRL_REG, (1 << (2 * pcnt_unit_id))); \
  REG_CLR_BIT(PCNT_CTRL_REG, (1 << (2 * pcnt_unit_id)))

static void IRAM_ATTR prepare_for_next_command(
    StepperQueue* queue, const struct queue_entry* e_next) {
  uint8_t next_steps = e_next->steps;
  if (next_steps > 0) {
    const struct mapping_s* mapping =
        (const struct mapping_s*)queue->driver_data;
    uint8_t pcnt_unit_id = mapping->pcnt_unit_id;
    PCNT.conf_unit[pcnt_unit_id].conf2.cnt_h_lim = next_steps;
  }
}

static void IRAM_ATTR apply_command(StepperQueue* queue,
                                    const struct queue_entry* e) {
  const struct mapping_s* mapping = (const struct mapping_s*)queue->driver_data;
  mcpwm_dev_t* mcpwm = mapping->mcpwm;
  uint8_t pcnt_unit_id = mapping->pcnt_unit_id;
  uint8_t timer = mapping->pcnt_unit_id;
  uint8_t steps = e->steps;
  if (e->toggle_dir) {
    LL_TOGGLE_PIN(queue->dirPin);
  }
  uint16_t ticks = e->ticks;
  if (mcpwm->timer[timer].timer_status.timer_value <= 1) {
    mcpwm->timer[timer].timer_cfg0.timer_period_upmethod = 0;
  } else {
    mcpwm->timer[timer].timer_cfg0.timer_period_upmethod = 1;
  }
  mcpwm->timer[timer].timer_cfg0.timer_period = ticks;
  if (steps == 0) {
    mcpwm->operators[timer].generator[0].gen_utea = 1;
    mcpwm->int_clr.val = TIMER_BIT(timer);
    mcpwm->int_ena.val |= TIMER_BIT(timer);
  } else {
    bool disable_mcpwm_interrupt = true;
    if (PCNT.conf_unit[pcnt_unit_id].conf2.cnt_h_lim != steps) {
      uint16_t val1 = steps - PCNT.cnt_unit[pcnt_unit_id].cnt_val;
      mcpwm->int_clr.val = TIMER_BIT(timer);
      uint16_t val2 = steps - PCNT.cnt_unit[pcnt_unit_id].cnt_val;
      if (val1 != val2) {
        mcpwm->int_clr.val = TIMER_BIT(timer);
      }
      PCNT.conf_unit[pcnt_unit_id].conf2.cnt_h_lim = val2;
      isr_pcnt_counter_clear(pcnt_unit_id);
      if ((mcpwm->int_raw.val & TIMER_BIT(timer)) != 0) {
        if (PCNT.cnt_unit[pcnt_unit_id].cnt_val == 0) {
          PCNT.conf_unit[pcnt_unit_id].conf2.cnt_h_lim = val2 - 1;
          isr_pcnt_counter_clear(pcnt_unit_id);
          disable_mcpwm_interrupt = val2 > 0;
        }
      }
    }
    if (disable_mcpwm_interrupt) {
      mcpwm->int_ena.val &= ~TIMER_BIT(timer);
    }
    mcpwm->operators[timer].generator[0].gen_utea = 2;
  }
}

static void IRAM_ATTR init_stop(StepperQueue* q) {
  const struct mapping_s* mapping = (const struct mapping_s*)q->driver_data;
  mcpwm_dev_t* mcpwm = mapping->mcpwm;
  uint8_t timer = mapping->pcnt_unit_id;
  mcpwm->timer[timer].timer_cfg1.timer_start = 0;
  mcpwm->int_ena.val &= ~TIMER_BIT(timer);
  q->_isRunning = false;
}

static void IRAM_ATTR what_is_next(StepperQueue* q) {
  bool isPrepared = q->_nextCommandIsPrepared;
  q->_nextCommandIsPrepared = false;
  uint8_t rp = q->read_idx;
  if (rp != q->next_write_idx) {
    rp++;
    q->read_idx = rp;
    if (rp != q->next_write_idx) {
      struct queue_entry* e_curr = &q->entry[rp & QUEUE_LEN_MASK];
      if (!isPrepared) {
        prepare_for_next_command(q, e_curr);
        const struct mapping_s* mapping =
            (const struct mapping_s*)q->driver_data;
        isr_pcnt_counter_clear(mapping->pcnt_unit_id);
      }
      apply_command(q, e_curr);
      rp++;
      if (rp != q->next_write_idx) {
        struct queue_entry* e_next = &q->entry[rp & QUEUE_LEN_MASK];
        q->_nextCommandIsPrepared = true;
        prepare_for_next_command(q, e_next);
      }
      return;
    }
  }
  init_stop(q);
}

static void IRAM_ATTR pcnt_isr_handler(void* arg) {
  (void)arg;
  uint32_t int_st = PCNT.int_st.val;
  uint32_t mask = (1 << QUEUES_MCPWM_PCNT) - 1;
  uint32_t pending = int_st & mask;
  if (pending == 0) return;
  PCNT.int_clr.val = pending;
  for (int i = 0; i < QUEUES_MCPWM_PCNT; i++) {
    if (pending & (1 << i)) {
      StepperQueue* q = pcnt_unit_to_queue[i];
      if (q) {
        what_is_next(q);
      }
    }
  }
}

static bool IRAM_ATTR mcpwm_on_reach(mcpwm_cmpr_handle_t cmpr,
                                     const mcpwm_compare_event_data_t* edata,
                                     void* user_ctx) {
  what_is_next((StepperQueue*)user_ctx);
  return false;
}

static mcpwm_dev_t* _mcpwm_for_timer(int timer_num) {
  return timer_num < 3 ? &MCPWM0 : &MCPWM1;
}

static uint8_t _group_for_timer(int timer_num) { return timer_num < 3 ? 0 : 1; }

static int _pcnt_unit_for_timer(int timer_num) { return timer_num; }

void StepperQueue::init_mcpwm_pcnt(uint8_t channel_num, uint8_t step_pin) {
  _step_pin = step_pin;

  struct mapping_s* mapping = &channel2mapping[channel_num];
  driver_data = (void*)mapping;

  int timer_num = channel_num;
  uint8_t group_id = _group_for_timer(timer_num);
  uint8_t pcnt_unit_id = _pcnt_unit_for_timer(timer_num);

  pcnt_unit_config_t pcnt_cfg = {.low_limit = -32768,
                                 .high_limit = 32767,
                                 .intr_priority = 1,
                                 .flags = {.accum_count = 0}};
  ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_new_unit(&pcnt_cfg, &mapping->pcnt_unit));

  pcnt_chan_config_t chan_cfg = {.edge_gpio_num = step_pin,
                                 .level_gpio_num = -1,
                                 .flags = {.invert_edge_input = 0,
                                           .invert_level_input = 0,
                                           .virt_edge_io_level = 0,
                                           .virt_level_io_level = 0,
                                           .io_loop_back = 0}};
  pcnt_channel_handle_t pcnt_chan;
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      pcnt_new_channel(mapping->pcnt_unit, &chan_cfg, &pcnt_chan));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_HOLD));

  mapping->pcnt_unit_id =
      ((struct pcnt_unit_internal*)mapping->pcnt_unit)->unit_id;

  PCNT.conf_unit[pcnt_unit_id].conf2.cnt_h_lim = 1;
  PCNT.conf_unit[pcnt_unit_id].conf0.thr_h_lim_en = 1;
  PCNT.conf_unit[pcnt_unit_id].conf0.thr_l_lim_en = 0;

  pcnt_unit_to_queue[pcnt_unit_id] = this;
  if (!pcnt_isr_installed) {
    PCNT.int_clr.val = PCNT.int_st.val;
    PCNT.int_ena.val |= ((1 << QUEUES_MCPWM_PCNT) - 1);
    esp_intr_alloc(
        ETS_PCNT_INTR_SOURCE,
        ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1,
        pcnt_isr_handler, NULL, NULL);
    pcnt_isr_installed = true;
  }

  ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_clear_count(mapping->pcnt_unit));
  ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_enable(mapping->pcnt_unit));
  ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_start(mapping->pcnt_unit));

  mcpwm_timer_config_t timer_cfg = {
      .group_id = group_id,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = TICKS_PER_S,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
      .period_ticks = 400,
      .intr_priority = 1,
      .flags = {.update_period_on_empty = 0, .update_period_on_sync = 0}};
  ESP_ERROR_CHECK_WITHOUT_ABORT(mcpwm_new_timer(&timer_cfg, &mapping->timer));

  mcpwm_operator_config_t oper_cfg = {.group_id = group_id,
                                      .intr_priority = 1,
                                      .flags = {.update_gen_action_on_tez = 1,
                                                .update_gen_action_on_tep = 1,
                                                .update_dead_time_on_tez = 0,
                                                .update_dead_time_on_tep = 0}};
  ESP_ERROR_CHECK_WITHOUT_ABORT(mcpwm_new_operator(&oper_cfg, &mapping->oper));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      mcpwm_operator_connect_timer(mapping->oper, mapping->timer));

  mcpwm_comparator_config_t cmpr_cfg = {.intr_priority = 1,
                                        .flags = {.update_cmp_on_tez = 0,
                                                  .update_cmp_on_tep = 0,
                                                  .update_cmp_on_sync = 0}};
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      mcpwm_new_comparator(mapping->oper, &cmpr_cfg, &mapping->cmpr));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      mcpwm_comparator_set_compare_value(mapping->cmpr, 1));

  mcpwm_generator_config_t gen_cfg = {.gen_gpio_num = step_pin,
                                      .flags = {.invert_pwm = 0}};
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      mcpwm_new_generator(mapping->oper, &gen_cfg, &mapping->gen));

  mcpwm_comparator_event_callbacks_t cmpr_cbs = {.on_reach = mcpwm_on_reach};
  ESP_ERROR_CHECK_WITHOUT_ABORT(mcpwm_comparator_register_event_callbacks(
      mapping->cmpr, &cmpr_cbs, (void*)this));

  mcpwm_dev_t* mcpwm = _mcpwm_for_timer(timer_num);
  mapping->mcpwm = mcpwm;
  int timer_in_group = timer_num % 3;

  mcpwm->operators[timer_in_group].generator[0].val = 0;
  mcpwm->operators[timer_in_group].generator[1].val = 0;
  mcpwm->operators[timer_in_group].generator[0].gen_dtep = 1;
  mcpwm->operators[timer_in_group].gen_stmp_cfg.gen_a_upmethod = 0;
  mcpwm->operators[timer_in_group].dt_cfg.val = 0;
  mcpwm->operators[timer_in_group].carrier_cfg.val = 0;
  mcpwm->operators[timer_in_group].gen_force.val = 0;
  mcpwm->clk_cfg.clk_prescale = 5 - 1;
  mcpwm->operator_timersel.val = 0;
  mcpwm->operator_timersel.operator0_timersel = 0;
  mcpwm->operator_timersel.operator1_timersel = 1;
  mcpwm->operator_timersel.operator2_timersel = 2;

  GPIO.func_out_sel_cfg[step_pin].func_sel =
      (timer_num < 3) ? PWM0_OUT0A_IDX + timer_num * 2
                      : PWM0_OUT0A_IDX + (timer_num - 3) * 2;
  GPIO.func_out_sel_cfg[step_pin].oen_sel = 0;

  ESP_ERROR_CHECK_WITHOUT_ABORT(mcpwm_timer_enable(mapping->timer));

  connect();
}

void StepperQueue::connect_mcpwm_pcnt() {
  const struct mapping_s* mapping = (const struct mapping_s*)driver_data;
  uint8_t step_pin = _step_pin;
  uint8_t pcnt_unit_id = mapping->pcnt_unit_id;

  int signal =
      pcnt_periph_signals.groups[0].units[pcnt_unit_id].channels[0].pulse_sig;
  gpio_iomux_in(step_pin, signal);

  GPIO.func_out_sel_cfg[step_pin].func_sel =
      (mapping->mcpwm == &MCPWM0) ? PWM0_OUT0A_IDX + (pcnt_unit_id) * 2
                                  : PWM0_OUT0A_IDX + (pcnt_unit_id - 3) * 2;
  GPIO.func_out_sel_cfg[step_pin].oen_sel = 0;
}

void StepperQueue::disconnect_mcpwm_pcnt() {
  GPIO.func_out_sel_cfg[_step_pin].func_sel = 0x100;
}

void StepperQueue::startQueue_mcpwm_pcnt() {
  const struct mapping_s* mapping = (const struct mapping_s*)driver_data;
  ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_clear_count(mapping->pcnt_unit));

  _isRunning = true;
  _nextCommandIsPrepared = false;
  struct queue_entry* e = &entry[read_idx & QUEUE_LEN_MASK];
  apply_command(this, e);

  mcpwm_timer_start_stop(mapping->timer, MCPWM_TIMER_START_NO_STOP);
}

void StepperQueue::forceStop_mcpwm_pcnt() {
  init_stop(this);
  read_idx = next_write_idx;
}

bool StepperQueue::isReadyForCommands_mcpwm_pcnt() const {
  if (isRunning()) {
    return true;
  }
  const struct mapping_s* mapping = (const struct mapping_s*)driver_data;
  uint8_t timer = mapping->pcnt_unit_id;
  if (mapping->mcpwm->timer[timer].timer_status.timer_value > 1) {
    return false;
  }
  return true;
}

uint16_t StepperQueue::_getPerformedPulses_mcpwm_pcnt() const {
  const struct mapping_s* mapping = (const struct mapping_s*)driver_data;
  return PCNT.cnt_unit[mapping->pcnt_unit_id].cnt_val;
}

#endif
