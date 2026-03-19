#include <Arduino.h>

#include <driver/mcpwm_timer.h>
#include <driver/mcpwm_oper.h>
#include <driver/mcpwm_cmpr.h>
#include <driver/mcpwm_gen.h>
#include <driver/pulse_cnt.h>
#include <soc/mcpwm_struct.h>
#include <soc/pcnt_struct.h>
#include <soc/pcnt_periph.h>
#include <soc/gpio_sig_map.h>
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_struct.h>

#define STEP_PIN GPIO_NUM_27
#define RESOLUTION_HZ 16000000
#define TEST_PULSE_COUNT 100

static volatile int pcnt_watch_fired = 0;
static volatile int pcnt_watch_value = 0;
static SemaphoreHandle_t test_sem = NULL;

static bool IRAM_ATTR pcnt_watch_cb(pcnt_unit_handle_t unit,
                                    const pcnt_watch_event_data_t* edata,
                                    void* user_ctx) {
  pcnt_watch_fired++;
  pcnt_watch_value = edata->watch_point_value;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(test_sem, &xHigherPriorityTaskWoken);
  return (xHigherPriorityTaskWoken == pdTRUE);
}

static void wait_for_pcnt_watch(int timeout_ms) {
  xSemaphoreTake(test_sem, pdMS_TO_TICKS(timeout_ms));
}

struct pcnt_unit_internal {
  void* group;
  portMUX_TYPE spinlock;
  int unit_id;
};

static void setup_mcpwm(mcpwm_timer_handle_t* timer, mcpwm_oper_handle_t* oper,
                        mcpwm_cmpr_handle_t* cmpr, mcpwm_gen_handle_t* gen) {
  mcpwm_timer_config_t tcfg = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = RESOLUTION_HZ,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
      .period_ticks = 400,
      .intr_priority = 1,
      .flags = {.update_period_on_empty = 0, .update_period_on_sync = 0}};
  ESP_ERROR_CHECK(mcpwm_new_timer(&tcfg, timer));

  mcpwm_operator_config_t ocfg = {.group_id = 0,
                                  .intr_priority = 1,
                                  .flags = {.update_gen_action_on_tez = 1,
                                            .update_gen_action_on_tep = 1,
                                            .update_dead_time_on_tez = 0,
                                            .update_dead_time_on_tep = 0}};
  ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, oper));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(*oper, *timer));

  mcpwm_comparator_config_t ccfg = {.intr_priority = 1,
                                    .flags = {.update_cmp_on_tez = 0,
                                              .update_cmp_on_tep = 0,
                                              .update_cmp_on_sync = 0}};
  ESP_ERROR_CHECK(mcpwm_new_comparator(*oper, &ccfg, cmpr));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*cmpr, 1));

  mcpwm_generator_config_t gcfg = {.gen_gpio_num = STEP_PIN,
                                   .flags = {.invert_pwm = 0}};
  ESP_ERROR_CHECK(mcpwm_new_generator(*oper, &gcfg, gen));

  MCPWM0.operators[0].generator[0].val = 0;
  MCPWM0.operators[0].generator[0].gen_utea = 2;
  MCPWM0.operators[0].generator[0].gen_dtep = 1;
  MCPWM0.operators[0].gen_stmp_cfg.gen_a_upmethod = 0;
  MCPWM0.operators[0].dt_cfg.val = 0;
  MCPWM0.operators[0].carrier_cfg.val = 0;
  MCPWM0.operators[0].gen_force.val = 0;
}

static int setup_pcnt_with_input(pcnt_unit_handle_t* pcnt_unit,
                                 pcnt_channel_handle_t* pcnt_chan,
                                 bool add_watch0) {
  pcnt_unit_config_t ucfg = {.low_limit = -32768,
                             .high_limit = 32767,
                             .intr_priority = 1,
                             .flags = {.accum_count = 1}};
  ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, pcnt_unit));

  pcnt_chan_config_t ccfg = {.edge_gpio_num = STEP_PIN,
                             .level_gpio_num = -1,
                             .flags = {.invert_edge_input = 0,
                                       .invert_level_input = 0,
                                       .virt_edge_io_level = 0,
                                       .virt_level_io_level = 0,
                                       .io_loop_back = 0}};
  ESP_ERROR_CHECK(pcnt_new_channel(*pcnt_unit, &ccfg, pcnt_chan));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
      *pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
      PCNT_CHANNEL_EDGE_ACTION_HOLD));

  if (add_watch0) {
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*pcnt_unit, 0));
  }

  int unit_id = ((struct pcnt_unit_internal*)(*pcnt_unit))->unit_id;

  GPIO.func_out_sel_cfg[STEP_PIN].func_sel = PWM0_OUT0A_IDX;
  GPIO.func_out_sel_cfg[STEP_PIN].oen_sel = 0;

  return unit_id;
}

static void teardown_all(mcpwm_timer_handle_t timer, mcpwm_oper_handle_t oper,
                         mcpwm_cmpr_handle_t cmpr, mcpwm_gen_handle_t gen,
                         pcnt_unit_handle_t pcnt_unit,
                         pcnt_channel_handle_t pcnt_chan) {
  mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
  if (pcnt_unit) {
    pcnt_unit_stop(pcnt_unit);
    pcnt_unit_disable(pcnt_unit);
  }
  if (pcnt_chan) pcnt_del_channel(pcnt_chan);
  if (pcnt_unit) pcnt_del_unit(pcnt_unit);
  mcpwm_timer_disable(timer);
  mcpwm_del_generator(gen);
  mcpwm_del_comparator(cmpr);
  mcpwm_del_operator(oper);
  mcpwm_del_timer(timer);
}

static bool test_mcpwm_pulses() {
  printf("=== Test 1: MCPWM generates pulses ===\n");

  mcpwm_timer_handle_t timer;
  mcpwm_oper_handle_t oper;
  mcpwm_cmpr_handle_t cmpr;
  mcpwm_gen_handle_t gen;
  setup_mcpwm(&timer, &oper, &cmpr, &gen);

  pcnt_unit_handle_t pcnt_unit;
  pcnt_channel_handle_t pcnt_chan;
  (void)setup_pcnt_with_input(&pcnt_unit, &pcnt_chan, false);

  pcnt_unit_enable(pcnt_unit);
  pcnt_unit_clear_count(pcnt_unit);
  pcnt_unit_start(pcnt_unit);

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_set_period(timer, 400));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

  printf("Running for 2s at 20kHz pulses...\n");
  vTaskDelay(pdMS_TO_TICKS(2000));

  int count = 0;
  pcnt_unit_get_count(pcnt_unit, &count);
  printf("PCNT count after 2s: %d (expected ~40000)\n", count);

  teardown_all(timer, oper, cmpr, gen, pcnt_unit, pcnt_chan);

  if (count > 10000 && count < 50000) {
    printf("PASS: Test 1 (count=%d, wraps expected at 32767)\n\n", count);
    return true;
  }
  printf("FAIL: Test 1 - count=%d, expected ~14464 (80k mod 32768)\n\n", count);
  return false;
}

static bool test_pcnt_watchpoint() {
  printf("=== Test 2: PCNT watch point fires at expected count ===\n");

  mcpwm_timer_handle_t timer;
  mcpwm_oper_handle_t oper;
  mcpwm_cmpr_handle_t cmpr;
  mcpwm_gen_handle_t gen;
  setup_mcpwm(&timer, &oper, &cmpr, &gen);

  pcnt_unit_handle_t pcnt_unit;
  pcnt_channel_handle_t pcnt_chan;
  (void)setup_pcnt_with_input(&pcnt_unit, &pcnt_chan, false);

  pcnt_watch_fired = 0;
  pcnt_watch_value = 0;
  test_sem = xSemaphoreCreateBinary();

  pcnt_event_callbacks_t pcnt_cbs = {.on_reach = pcnt_watch_cb};
  ESP_ERROR_CHECK(
      pcnt_unit_register_event_callbacks(pcnt_unit, &pcnt_cbs, NULL));

  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, TEST_PULSE_COUNT));
  pcnt_unit_enable(pcnt_unit);
  pcnt_unit_clear_count(pcnt_unit);
  pcnt_unit_start(pcnt_unit);

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_set_period(timer, 400));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

  printf("Waiting for PCNT watch point at count=%d...\n", TEST_PULSE_COUNT);
  wait_for_pcnt_watch(2000);

  printf("Watch fired=%d, value=%d\n", pcnt_watch_fired, pcnt_watch_value);

  teardown_all(timer, oper, cmpr, gen, pcnt_unit, pcnt_chan);
  vSemaphoreDelete(test_sem);
  test_sem = NULL;

  if (pcnt_watch_fired == 1 && pcnt_watch_value == TEST_PULSE_COUNT) {
    printf("PASS: Test 2\n\n");
    return true;
  }
  printf("FAIL: Test 2 - fired=%d value=%d (expected 1, %d)\n\n",
         pcnt_watch_fired, pcnt_watch_value, TEST_PULSE_COUNT);
  return false;
}

static bool test_pcnt_dynamic_hlimit() {
  printf("=== Test 3: PCNT high-limit change via direct register ===\n");

  mcpwm_timer_handle_t timer;
  mcpwm_oper_handle_t oper;
  mcpwm_cmpr_handle_t cmpr;
  mcpwm_gen_handle_t gen;
  setup_mcpwm(&timer, &oper, &cmpr, &gen);

  pcnt_unit_handle_t pcnt_unit;
  pcnt_channel_handle_t pcnt_chan;
  int pcnt_unit_id = setup_pcnt_with_input(&pcnt_unit, &pcnt_chan, true);

  pcnt_watch_fired = 0;
  pcnt_watch_value = 0;
  test_sem = xSemaphoreCreateBinary();

  pcnt_event_callbacks_t pcnt_cbs = {.on_reach = pcnt_watch_cb};
  ESP_ERROR_CHECK(
      pcnt_unit_register_event_callbacks(pcnt_unit, &pcnt_cbs, NULL));

  PCNT.conf_unit[pcnt_unit_id].conf0.thr_h_lim_en = 1;
  PCNT.conf_unit[pcnt_unit_id].conf2.cnt_h_lim = 50;

  pcnt_unit_enable(pcnt_unit);
  pcnt_unit_clear_count(pcnt_unit);
  pcnt_unit_start(pcnt_unit);

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_set_period(timer, 400));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

  printf("First: wait for watch at h_lim=50...\n");
  wait_for_pcnt_watch(2000);
  printf("Watch 1: fired=%d value=%d\n", pcnt_watch_fired, pcnt_watch_value);
  int first_fired = pcnt_watch_fired;

  pcnt_unit_clear_count(pcnt_unit);
  PCNT.conf_unit[pcnt_unit_id].conf2.cnt_h_lim = 200;
  pcnt_watch_fired = 0;

  printf("Second: wait for watch at dynamically set h_lim=200...\n");
  wait_for_pcnt_watch(2000);
  printf("Watch 2: fired=%d value=%d\n", pcnt_watch_fired, pcnt_watch_value);
  int second_fired = pcnt_watch_fired;

  teardown_all(timer, oper, cmpr, gen, pcnt_unit, pcnt_chan);
  vSemaphoreDelete(test_sem);
  test_sem = NULL;

  bool pass = (first_fired >= 1 && second_fired >= 1);
  if (pass) {
    printf("PASS: Test 3\n\n");
  } else {
    printf("FAIL: Test 3 - first=%d second=%d (expected >=1 each)\n\n",
           first_fired, second_fired);
  }
  return pass;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  delay(1000);
  printf("\n\n====== MCPWM+PCNT Test (IDF 5.3 / Arduino) ======\n\n");
  printf("ESP_IDF_VERSION: %d.%d.%d\n\n", ESP_IDF_VERSION_MAJOR,
         ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH);
}

void loop() {
  int passed = 0;
  int total = 3;

  if (test_mcpwm_pulses()) passed++;
  if (test_pcnt_watchpoint()) passed++;
  if (test_pcnt_dynamic_hlimit()) passed++;

  printf("\n====== RESULTS: %d/%d passed ======\n", passed, total);
  if (passed == total) {
    printf("ALL TESTS PASSED\n");
  } else {
    printf("SOME TESTS FAILED\n");
  }
  printf("\nRestarting in 10s...\n");
  delay(10000);
  ESP.restart();
}
