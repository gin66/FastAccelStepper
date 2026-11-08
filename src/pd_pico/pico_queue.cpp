#include "fas_queue/stepper_queue.h"

#if defined(SUPPORT_RP_PICO)
#if defined(__FREERTOS)
#include <FreeRTOS.h>
#include <task.h>
#else
#include <pico/time.h>
#endif

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
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

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
  (void)queue_num;
  _step_pin = step_pin;
}

static bool real_dir_pin(const StepperQueue* q) {
  return (q->dirPin != PIN_UNDEFINED) && ((q->dirPin & PIN_EXTERNAL_FLAG) == 0);
}

void StepperQueue::attachDirPinToStatemachine() {
  if (!real_dir_pin(this)) {
    return;
  }
  // PIN_OUTPUT already set the level. gpio_init here would clear it.
  // The pin stays a CPU GPIO: the program no longer has a SET of DIR.
  gpio_set_function(dirPin, GPIO_FUNC_SIO);
  gpio_set_dir(dirPin, GPIO_OUT);
  if (dir_gpio_valid) {
    gpio_put(dirPin, dir_gpio_high);
  } else {
    dir_gpio_high = dirHighCountsUp;
    dir_gpio_valid = true;
  }
}

void StepperQueue::setDirPinState(bool high) {
  if (!real_dir_pin(this)) {
    return;
  }
  gpio_put(dirPin, high);
  dir_gpio_high = high;
  dir_gpio_valid = true;
}

// One entry per PIO block whose program loaded. `sm[]` is every state
// machine this driver claimed on that block (1 to 4). A later connect takes
// the next one. The machines stay claimed after a successful load so another
// caller cannot be told the block is empty.
struct PioBlock {
  PIO pio;
  uint8_t offset;
  uint8_t claimed;
  uint8_t used;
  uint8_t sm[NUM_PIO_STATE_MACHINES];
};

static PioBlock s_block[NUM_PIOS];
static uint8_t s_block_count = 0;

static bool pio_drives_pin(PIO pio, uint8_t pin) {
  uint base = pio_get_gpio_base(pio);
  return (pin >= base) && (pin < base + 32);
}

static bool block_loaded(PIO pio) {
  for (uint8_t i = 0; i < s_block_count; i++) {
    if (s_block[i].pio == pio) {
      return true;
    }
  }
  return false;
}

bool StepperQueue::claim_pio_resources(FastAccelStepperEngine* engine,
                                       uint8_t step_pin, PioResources* out) {
  (void)engine;
  for (uint8_t i = 0; i < s_block_count; i++) {
    PioBlock* b = &s_block[i];
    if (b->used < b->claimed && pio_drives_pin(b->pio, step_pin)) {
      out->pio = b->pio;
      out->sm = b->sm[b->used];
      out->offset = b->offset;
      b->used++;
      return true;
    }
  }

  pio_program_t pio_program = {};
  pio_program.instructions = program->code;
  pio_program.length = program->pc;
  // Relocatable. Origin 0 cannot sit beside the 2-word cycle counter.
  pio_program.origin = -1;
  pio_program.pio_version = 0;
#if PICO_PIO_VERSION > 0
  pio_program.used_gpio_ranges = 0;
#endif

  for (uint8_t pi = 0; pi < NUM_PIOS; pi++) {
    PIO pio = pio_get_instance(pi);
    if (block_loaded(pio) || !pio_drives_pin(pio, step_pin)) {
      continue;
    }
    uint8_t taken[NUM_PIO_STATE_MACHINES];
    uint8_t n = 0;
    for (uint sm = 0; sm < NUM_PIO_STATE_MACHINES; sm++) {
      if (!pio_sm_is_claimed(pio, sm)) {
        pio_sm_claim(pio, sm);
        taken[n] = (uint8_t)sm;
        n++;
      }
    }
    // Nothing free here (the cycle counter may already hold one). The
    // program is not written, so there is nothing to undo.
    if (n == 0) {
      continue;
    }
    int offset = pio_add_program(pio, &pio_program);
    if (offset < 0) {
      for (uint8_t k = 0; k < n; k++) {
        pio_sm_unclaim(pio, taken[k]);
      }
      continue;
    }
    PioBlock* b = &s_block[s_block_count];
    s_block_count++;
    b->pio = pio;
    b->offset = (uint8_t)offset;
    b->claimed = n;
    b->used = 1;
    for (uint8_t k = 0; k < n; k++) {
      b->sm[k] = taken[k];
    }
    irq_set_exclusive_handler(PIO_IRQ_NUM(pio, 0), pio_fifo_irq_handlers[pi]);
    irq_set_enabled(PIO_IRQ_NUM(pio, 0), true);
    out->pio = pio;
    out->sm = taken[0];
    out->offset = (uint8_t)offset;
    return true;
  }
  return false;
}

void StepperQueue::setupSM() {
  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_jmp_pin(&c, _step_pin);
  sm_config_set_out_pins(&c, _step_pin, 1);
  // Wrap addresses are program-relative until the load offset is added.
  // The loader places a relocatable program at the top of the block, not at 0.
  uint offset = prog_offset;
  sm_config_set_wrap(&c, (uint)program->wrap_target + offset,
                     (uint)program->wrap_at + offset);

  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_consecutive_pindirs(pio, sm, _step_pin, 1, true);
  pio_sm_set_enabled(pio, sm, true);
  pio_sm_clear_fifos(pio, sm);
  pio_sm_restart(pio, sm);
}

void StepperQueue::connect() {
  pio_gpio_init(pio, _step_pin);
  attachDirPinToStatemachine();
}

void StepperQueue::disconnect() {
  gpio_init(_step_pin);
  if (real_dir_pin(this)) {
    // Back to an input. dir_gpio_high is kept so the next connect restores it.
    gpio_init(dirPin);
  }
}

bool StepperQueue::isReadyForCommands() const { return true; }

// Put one software-queue word into the TX FIFO. One command is already in
// flight and the FIFO holds four more, so a direction level is applied only
// after this put has succeeded. A pause keeps the old level. The command
// after the four drain pauses is the one whose level differs: getting it in
// requires a pull, and that pull takes a pause. A full FIFO returns false
// with the pin unchanged. adjust_80MHz is not consumed on that refusal.
static bool push_command(StepperQueue* q) {
  uint8_t rp = q->read_idx;
  if (rp == q->next_write_idx) {
    return false;
  }
  if (pio_sm_is_tx_fifo_full(q->pio, q->sm)) {
    return false;
  }
  struct queue_entry* e = &q->entry[rp & QUEUE_LEN_MASK];
  uint8_t steps = e->steps;
  bool count_up = e->countUp == 1;
  uint32_t loops = pio_calc_loops(steps, e->ticks, &q->adjust_80MHz);
  uint32_t word = pio_make_fifo_entry(count_up, steps, loops);
  pio_sm_put(q->pio, q->sm, word);
  if (real_dir_pin(q)) {
    bool want_high = e->dirPinState == 1;
    if (!q->dir_gpio_valid || (q->dir_gpio_high != want_high)) {
      q->setDirPinState(want_high);
    }
  }
  e->toggle_dir = 0;
  rp++;
  q->read_idx = rp;
  return true;
}

// Push until the software queue is empty or the TX FIFO is full. Leave the
// level IRQ enabled only while a word is still waiting: it is level
// triggered, and an empty queue would spin.
static void feed_queue(StepperQueue* q) {
  while (push_command(q)) {
  }
  bool more = q->read_idx != q->next_write_idx;
  q->_isActive = more;
  pio_set_irq0_source_enabled(
      q->pio, pio_get_tx_fifo_not_full_interrupt_source(q->sm), more);
}

void StepperQueue::startQueue() {
  if (_isActive) {
    return;
  }
  uint32_t save = save_and_disable_interrupts();
  if (_isActive) {
    restore_interrupts(save);
    return;
  }
  feed_queue(this);
  restore_interrupts(save);
}

void StepperQueue::forceStop() {
  uint32_t save = save_and_disable_interrupts();
  _isActive = false;
  pio_set_irq0_source_enabled(
      pio, pio_get_tx_fifo_not_full_interrupt_source(sm), false);
  pio_sm_set_enabled(pio, sm, false);
  pio_sm_clear_fifos(pio, sm);
  pio_sm_restart(pio, sm);
  pio_sm_set_enabled(pio, sm, true);
  // pull is the first word of the loaded program, not instruction 0.
  pio_sm_exec(pio, sm, pio_encode_jmp(prog_offset));
  pio_sm_exec(pio, sm, pio_encode_mov(pio_pins, pio_null));
  setDirPinState(queue_end.dir);

  read_idx = next_write_idx;
  pos_offset = 0;
  // The FIFO and the software queue no longer hold those pauses.
  clear_pause_stats();
  restore_interrupts(save);
}

bool StepperQueue::isRunning() const {
  if (!pio_sm_is_tx_fifo_empty(pio, sm)) {
    return true;
  }
  uint8_t pc = pio_sm_get_pc(pio, sm);
  return (pc != prog_offset);
}

int32_t StepperQueue::getCurrentStepCount() const {
  bool running = isRunning();
  uint32_t pos = 0;
  if (running) {
    // A running queue cannot use the samples already in the RX FIFO. The PIO
    // pushes the position with a *non-blocking* push, so once the FIFO is full
    // the push stops updating it: a queue nobody has read since it started
    // holds samples from the run's beginning -- 0 before the first step -- and
    // reading one of those reports a position the stepper left long ago.
    //
    // Measured on RP2350, SR_30 (XSTOP ~25 % into a 4080-step fill): `POS`
    // and `DONE` were 0 in 3 of 8 runs while the wire carried the full run,
    // and the FIFO was full (`rxl=4`) at the read with all four entries stale.
    // The read-order fix only made the empty case deterministic; it does not
    // make the position current.
    //
    // So discard the stale samples and wait for the SM to push a current one.
    // The wait is short: the period loop reaches its push every 3 cycles, and
    // the longest gap before it (the step and position-update section) is tens
    // of cycles. If the SM is instead stalled on a `pull` there is no
    // meaningful position to report, and the move's own target is the honest
    // fallback.
    while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
      pio_sm_get(pio, sm);
    }
    for (uint16_t spin = 0; spin < 256; spin++) {
      if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        return (int32_t)pio_sm_get(pio, sm);
      }
    }
    // Nothing new arrived, so the SM is stalled on pull (the period loop
    // pushes every 3 cycles). Nothing is in the TX FIFO. The executed
    // position is the queue end minus commands not yet pushed. Caught up,
    // that difference is zero.
    if (pio_sm_is_tx_fifo_empty(pio, sm)) {
      int32_t pos = queue_end.pos;
      uint8_t rp = read_idx;
      uint8_t wp = next_write_idx;
      while (rp != wp) {
        const struct queue_entry* e = &entry[rp & QUEUE_LEN_MASK];
        if (e->countUp) {
          pos -= (int32_t)e->steps;
        } else {
          pos += (int32_t)e->steps;
        }
        rp++;
      }
      return pos - pos_offset;
    }
    return queue_end.pos - pos_offset;
  }
  for (uint8_t i = 0; i <= 4; i++) {
    if (pio_sm_is_rx_fifo_empty(pio, sm)) {
      break;
    }
    pio_sm_get(pio, sm);
  }
  uint32_t entry = pio_make_fifo_entry(0, 0, LOOPS_FOR_1US);
  pio_sm_put(pio, sm, entry);

  while (isRunning()) {
    if (!pio_sm_is_tx_fifo_empty(pio, sm)) {
      break;
    }
  }
  for (uint8_t i = 0; i <= 4; i++) {
    // Test before read, like the drain loop above. Reading first consumed a
    // value from an empty RX FIFO and then used it, so on a stopped, drained
    // queue -- the normal state after forceStop() -- the count reported was
    // whatever the read returned rather than nothing.
    if (pio_sm_is_rx_fifo_empty(pio, sm)) {
      break;
    }
    pos = pio_sm_get(pio, sm);
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
      fas_queue[i]._initVars();
      fas_queue[i].pio = res.pio;
      fas_queue[i].sm = res.sm;
      // _initVars clears prog_offset. The loaded pull is not at 0.
      fas_queue[i].prog_offset = res.offset;
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

    uint32_t save = save_and_disable_interrupts();
    feed_queue(q);
    restore_interrupts(save);
  }
}

static void pio0_fifo_irq_handler() { pio_fifo_irq_handler(pio0); }
static void pio1_fifo_irq_handler() { pio_fifo_irq_handler(pio1); }
#if NUM_PIOS > 2
static void pio2_fifo_irq_handler() { pio_fifo_irq_handler(pio2); }
#endif

#if defined(__FREERTOS)
void StepperTask(void* parameter) {
  FastAccelStepperEngine* engine =
      static_cast<FastAccelStepperEngine*>(parameter);
  while (true) {
    engine->manageSteppers();
    const TickType_t delay_time =
        (engine->_task_rate() + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
    vTaskDelay(delay_time);
  }
}
#else
static bool stepper_timer_callback(repeating_timer_t* timer) {
  static_cast<FastAccelStepperEngine*>(timer->user_data)->manageSteppers();
  return true;
}
#endif

int32_t StepperQueue::getCurrentPosition() const {
  // A stopped queue has no live hardware count worth reading. forceStop()
  // restarts the SM, which clears the input shift register the PIO keeps the
  // position in, so the sample path then reads 0 -- and it sets `pos_offset` to
  // 0 as well, so the whole thing reports 0 even though
  // forceStopAndNewPosition() just stored the real position in queue_end.pos.
  // The queue's own bookkeeping already has it: for a stopped queue
  // queue_end.pos *is* the position, which is what the generic
  // getCurrentPosition() returns for an empty queue.
  if (!isRunning()) {
    return queue_end.pos;
  }
  return getCurrentStepCount() + pos_offset;
}

void fas_init_engine(FastAccelStepperEngine* engine) {
  for (uint8_t i = 0; i < NUM_QUEUES; i++) {
    fas_queue[i]._step_pin = PIN_UNDEFINED;
    fas_queue[i]._isActive = true;
  }
  s_block_count = 0;
  engine->task_rate(DELAY_MS_BASE);
#if defined(__FREERTOS)
#define STACK_SIZE 3000
#define PRIORITY (configMAX_PRIORITIES - 1)
  xTaskCreate(StepperTask, "StepperTask", STACK_SIZE, engine, PRIORITY, NULL);
#else
  static repeating_timer_t stepper_timer;
  add_repeating_timer_us(-(int64_t)engine->_task_rate() * 1000,
                         stepper_timer_callback, engine, &stepper_timer);
#endif

  program = stepper_make_program();
}

AqeResultCode FastAccelStepperEngine::synchronizedStart(
    FastAccelStepper** const steppers, uint8_t cnt) {
  AqeResultCode rc = AqeResultCode::OK;
  fasDisableInterrupts();
  for (uint8_t i = 0; i < cnt; i++) {
    AqeResultCode e = steppers[i]->addQueueEntry(NULL, true);
    if (rc == AqeResultCode::OK && e != AqeResultCode::OK) {
      rc = e;
    }
  }
  fasEnableInterrupts();
  return rc;
}

#endif
