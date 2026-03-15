#include "fas_queue/stepper_queue.h"

#if defined(SUPPORT_RP_PICO)
#include <FreeRTOS.h>
#include <task.h>

#include "hardware/irq.h"
#include "pico_pio.h"

static stepper_pio_program* program;

static void pio0_fifo_irq_handler();
static void pio1_fifo_irq_handler();
#if NUM_PIOS > 2
static void pio2_fifo_irq_handler();
#endif

static const irq_handler_t pio_fifo_irq_handlers[] = {
    pio0_fifo_irq_handler,
    pio1_fifo_irq_handler,
#if NUM_PIOS > 2
    pio2_fifo_irq_handler,
#endif
};

bool StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
  (void)queue_num;
  _step_pin = step_pin;
  _isActive = false;
  dirPin = PIN_UNDEFINED;
  pos_offset = 0;
  max_speed_in_ticks = 80;
  _initVars();
  return true;
}

void StepperQueue::attachDirPinToStatemachine() {
  if ((dirPin != PIN_UNDEFINED) && ((dirPin & PIN_EXTERNAL_FLAG) == 0)) {
    pio_sm_set_set_pins(pio, sm, dirPin, 1);
    pio_sm_set_consecutive_pindirs(pio, sm, dirPin, 1, true);
    pio_gpio_init(pio, dirPin);
  }
}

void StepperQueue::setDirPinState(bool high) {
  if ((dirPin != PIN_UNDEFINED) && ((dirPin & PIN_EXTERNAL_FLAG) == 0)) {
    pio_sm_exec(pio, sm, pio_encode_set(pio_pins, high ? 1 : 0));
  }
}

uint8_t StepperQueue::s_claimed_pios = 0;
PIO StepperQueue::s_pio[NUM_PIOS];

bool StepperQueue::claim_pio_resources(FastAccelStepperEngine* engine,
                                       uint8_t step_pin, PioResources* out) {
  (void)engine;
  for (uint8_t i = 0; i < s_claimed_pios; i++) {
    int claimed_sm = pio_claim_unused_sm(s_pio[i], false);
    if (claimed_sm >= 0) {
      out->pio = s_pio[i];
      out->sm = claimed_sm;
      return true;
    }
  }
  pio_program_t pio_program;
  pio_program.instructions = program->code;
  pio_program.length = program->pc;
  pio_program.origin = 0;
  pio_program.pio_version = 0;
#if defined(PICO_RP_2350)
  pio_program.used_gpio_ranges = 0;
#endif
  uint offset;
  uint8_t pio_index = s_claimed_pios;
  if (pio_index == NUM_PIOS) {
    return false;
  }
  bool rc = pio_claim_free_sm_and_add_program_for_gpio_range(
      &pio_program, &out->pio, &out->sm, &offset, step_pin, 1, true);
  if (!rc) {
    rc = pio_claim_free_sm_and_add_program_for_gpio_range(
        &pio_program, &out->pio, &out->sm, &offset, step_pin, 1, true);
  }
  if (rc) {
    s_pio[pio_index] = out->pio;
    s_claimed_pios = pio_index + 1;
    uint pio_idx = pio_get_index(out->pio);
    irq_set_exclusive_handler(PIO_IRQ_NUM(out->pio, 0),
                              pio_fifo_irq_handlers[pio_idx]);
    irq_set_enabled(PIO_IRQ_NUM(out->pio, 0), true);
  }
  return rc;
}

void StepperQueue::setupSM() {
  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_jmp_pin(&c, _step_pin);
  sm_config_set_out_pins(&c, _step_pin, 1);
  sm_config_set_wrap(&c, program->wrap_target, program->wrap_at);

  uint offset = 0;
  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_consecutive_pindirs(pio, sm, _step_pin, 1, true);
  pio_sm_set_enabled(pio, sm, true);
  pio_sm_clear_fifos(pio, sm);
  pio_sm_restart(pio, sm);
}

void StepperQueue::connect() {
  pio_gpio_init(pio, _step_pin);
  if ((dirPin != PIN_UNDEFINED) && ((dirPin & PIN_EXTERNAL_FLAG) == 0)) {
    pio_gpio_init(pio, dirPin);
  }
}

void StepperQueue::disconnect() {
  gpio_init(_step_pin);
  if ((dirPin != PIN_UNDEFINED) && ((dirPin & PIN_EXTERNAL_FLAG) == 0)) {
    gpio_init(dirPin);
  }
}

bool StepperQueue::isReadyForCommands() const { return true; }

static bool push_command(StepperQueue* q) {
  uint8_t rp = q->read_idx;
  if (rp == q->next_write_idx) {
    return false;
  }
  if (pio_sm_is_tx_fifo_full(q->pio, q->sm)) {
    return false;
  }
  struct queue_entry* e_curr = &q->entry[rp & QUEUE_LEN_MASK];
  uint8_t steps = e_curr->steps;
  uint16_t ticks = e_curr->ticks;
  bool dirHigh = e_curr->dirPinState == 1;
  bool countUp = e_curr->countUp == 1;
  uint32_t loops = pio_calc_loops(steps, ticks, &q->adjust_80MHz);
  uint32_t entry = pio_make_fifo_entry(dirHigh, countUp, steps, loops);
  pio_sm_put(q->pio, q->sm, entry);
  rp++;
  q->read_idx = rp;
  return true;
}

void StepperQueue::startQueue() {
  if (_isActive) {
    return;
  }
  pio_set_irq0_source_enabled(
      pio, pio_get_tx_fifo_not_full_interrupt_source(sm), false);
  _isActive = false;
  while (push_command(this)) {
  };
  _isActive = true;
  pio_set_irq0_source_enabled(
      pio, pio_get_tx_fifo_not_full_interrupt_source(sm), true);
}

void StepperQueue::forceStop() {
  _isActive = false;
  pio_set_irq0_source_enabled(
      pio, pio_get_tx_fifo_not_full_interrupt_source(sm), false);
  pio_sm_set_enabled(pio, sm, false);
  pio_sm_clear_fifos(pio, sm);
  pio_sm_restart(pio, sm);
  pio_sm_set_enabled(pio, sm, true);
  pio_sm_exec(pio, sm, pio_encode_jmp(0));
  pio_sm_exec(pio, sm, pio_encode_mov(pio_pins, pio_null));
  setDirPinState(queue_end.dir);

  read_idx = next_write_idx;

  pos_offset = 0;
}

bool StepperQueue::isRunning() const {
  if (!pio_sm_is_tx_fifo_empty(pio, sm)) {
    return true;
  }
  uint8_t pc = pio_sm_get_pc(pio, sm);
  return (pc != 0);
}

int32_t StepperQueue::getCurrentStepCount() const {
  bool running = isRunning();
  uint32_t pos;
  if (!running) {
    for (uint8_t i = 0; i <= 4; i++) {
      if (pio_sm_is_rx_fifo_empty(pio, sm)) {
        break;
      }
      pio_sm_get(pio, sm);
    }
    uint32_t entry = pio_make_fifo_entry(queue_end.dir, 0, 0, LOOPS_FOR_1US);
    pio_sm_put(pio, sm, entry);

    while (isRunning()) {
      if (!pio_sm_is_tx_fifo_empty(pio, sm)) {
        break;
      }
    }
  }
  for (uint8_t i = 0; i <= 4; i++) {
    pos = pio_sm_get(pio, sm);
    if (pio_sm_is_rx_fifo_empty(pio, sm)) {
      break;
    }
  }
  return (int32_t)pos;
}

bool StepperQueue::isValidStepPin(uint8_t step_pin) { return (step_pin < 32); }

static uint8_t stepper_allocated_count = 0;

StepperQueue* StepperQueue::tryAllocateQueue(FastAccelStepperEngine* engine,
                                             uint8_t step_pin) {
  if (!isValidStepPin(step_pin)) {
    return nullptr;
  }

  if (stepper_allocated_count >= MAX_STEPPER) {
    return nullptr;
  }

  PioResources res;
  if (!claim_pio_resources(engine, step_pin, &res)) {
    return nullptr;
  }

  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    if (fas_queue[i]._step_pin == PIN_UNDEFINED) {
      fas_queue[i].pio = res.pio;
      fas_queue[i].sm = res.sm;
      fas_queue[i].init(i, step_pin);
      fas_queue[i].setupSM();
      fas_queue[i].connect();
      stepper_allocated_count++;
      return &fas_queue[i];
    }
  }

  return nullptr;
}

static void pio_fifo_irq_handler(PIO pio) {
  uint32_t ints = pio->ints0;

  for (uint8_t i = 0; i < NUM_QUEUES; i++) {
    StepperQueue* q = &fas_queue[i];
    if (!q->_isActive) continue;
    if (q->_step_pin == PIN_UNDEFINED) {
      continue;
    }
    if (q->pio != pio) continue;

    if (!(ints & (1u << (pis_sm0_tx_fifo_not_full + q->sm)))) continue;

    while (push_command(q)) {
    }

    if (q->read_idx == q->next_write_idx) {
      q->_isActive = false;
      pio_set_irq0_source_enabled(
          pio, pio_get_tx_fifo_not_full_interrupt_source(q->sm), false);
    }
  }
}

static void pio0_fifo_irq_handler() { pio_fifo_irq_handler(pio0); }
static void pio1_fifo_irq_handler() { pio_fifo_irq_handler(pio1); }
#if NUM_PIOS > 2
static void pio2_fifo_irq_handler() { pio_fifo_irq_handler(pio2); }
#endif

void StepperTask(void* parameter) {
  FastAccelStepperEngine* engine = (FastAccelStepperEngine*)parameter;
  while (true) {
    engine->manageSteppers();
    const TickType_t delay_time =
        (engine->_delay_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
    vTaskDelay(delay_time);
  }
}

int32_t StepperQueue::getCurrentPosition() const {
  return getCurrentStepCount() + pos_offset;
}

void fas_init_engine(FastAccelStepperEngine* engine) {
  for (uint8_t i = 0; i < NUM_QUEUES; i++) {
    fas_queue[i]._step_pin = PIN_UNDEFINED;
    fas_queue[i]._isActive = true;
  }
  StepperQueue::s_claimed_pios = 0;
#define STACK_SIZE 3000
#define PRIORITY (configMAX_PRIORITIES - 1)
  engine->_delay_ms = DELAY_MS_BASE;
  xTaskCreate(StepperTask, "StepperTask", STACK_SIZE, engine, PRIORITY, NULL);

  program = stepper_make_program();
}
#endif
