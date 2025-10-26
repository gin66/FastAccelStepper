#include "StepperISR.h"

#if defined(SUPPORT_RP_PICO)
#include <FreeRTOS.h>
#include <task.h>

#include "pico_pio.h"

// Here are the global variables to interface with the interrupts
StepperQueue fas_queue[NUM_QUEUES];
static stepper_pio_program *program;

bool StepperQueue::init(FastAccelStepperEngine *engine, uint8_t queue_num,
                        uint8_t step_pin) {
  (void)queue_num; // silence compiler for unused parameter
  _step_pin = step_pin;
  _isStarting = false;
  dirPin = PIN_UNDEFINED;
  pos_offset = 0;
  max_speed_in_ticks = 80;  // This equals 200kHz @ 16MHz
  bool ok = claim_pio_sm(engine);
  if (ok) {
    setupSM();
    connect();
  }
  return ok;
}

void StepperQueue::attachDirPinToStatemachine() {
  if ((dirPin != PIN_UNDEFINED) && ((dirPin & PIN_EXTERNAL_FLAG) == 0)) {
    pio_sm_set_set_pins(pio, sm, dirPin, 1);  // Direction pin via set
    pio_sm_set_consecutive_pindirs(pio, sm, dirPin, 1, true);
    pio_gpio_init(pio, dirPin);
  }
}

bool StepperQueue::claim_pio_sm(FastAccelStepperEngine *engine) {
  // We have two or three PIO modules. If we need one sm from a pio,
  // the whole PIO need to be claimed due to the size of our pio code.
  // Let's check first, if there is any PIO claimed.
  // If yes, check if we can claim a sm from that PIO.
  for (uint8_t i = 0; i < engine->claimed_pios; i++) {
    // pio has been claimed, so our program is valid
    int claimed_sm = pio_claim_unused_sm(engine->pio[i], false);
    if (claimed_sm >= 0) {
      // successfully claimed
      pio = engine->pio[i];
      sm = claimed_sm;
      // Serial.print("claim pio=");
      // Serial.print(i);
      // Serial.print(" sm=");
      // Serial.println(sm);
      return true;
    }
  }
  // claim a new pio
  pio_program_t pio_program;
  pio_program.instructions = program->code;
  pio_program.length = program->pc;
  pio_program.origin = 0;
  pio_program.pio_version = 0;
#if defined(PICO_RP_2350)
  pio_program.used_gpio_ranges = 0;
#endif
  uint offset;
  uint8_t pio_index = engine->claimed_pios;
  if (pio_index == NUM_PIOS) {
      return false;
  }
  bool rc = pio_claim_free_sm_and_add_program_for_gpio_range(
      &pio_program, &pio, &sm, &offset, _step_pin, 1, true);
  if (!rc) {
      // try again. for whatever reason this may fail on first attempt
      // Serial.println("retry claim");
      rc = pio_claim_free_sm_and_add_program_for_gpio_range(
      	&pio_program, &pio, &sm, &offset, _step_pin, 1, true);
  }
      // Serial.print("claim new pio=");
      // Serial.print(engine->claimed_pios);
      // Serial.print(" sm=");
      // Serial.print(sm);
      // Serial.print(" result=");
      // Serial.println(rc);
  if (rc) {
    engine->pio[pio_index] = pio;
    engine->claimed_pios = pio_index+1;
  }
  return rc;
}

void StepperQueue::setupSM() {
  pio_sm_config c = pio_get_default_sm_config();
  // Map the state machine's OUT pin group to one pin, namely the `pin`
  // parameter to this function.
  sm_config_set_jmp_pin(&c, _step_pin);  // Step pin read back 
  sm_config_set_out_pins(&c, _step_pin, 1);  // Step pin via out
  sm_config_set_wrap(&c, program->wrap_target, program->wrap_at);

  // Load our configuration, and jump to the start of the program
  uint offset = 0;
  pio_sm_init(pio, sm, offset, &c);
  // Set the pin direction to output at the PIO
  pio_sm_set_consecutive_pindirs(pio, sm, _step_pin, 1, true);
  pio_sm_set_enabled(pio, sm, true);  // sm is running, otherwise loop() stops
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
  pio_sm_set_enabled(pio, sm, false);
  gpio_init(_step_pin);
  if ((dirPin != PIN_UNDEFINED) && ((dirPin & PIN_EXTERNAL_FLAG) == 0)) {
    gpio_init(dirPin);
  }
}

bool StepperQueue::isReadyForCommands() { return true; }

static bool push_command(StepperQueue *q) {
  uint8_t rp = q->read_idx;
  if (rp == q->next_write_idx) {
    // no command in queue
    return false;
  }
  if (pio_sm_is_tx_fifo_full(q->pio, q->sm)) {
    // Serial.println("TX FIFO full, cannot push command");
    return false;
  }
  // Process command
  struct queue_entry *e_curr = &q->entry[rp & QUEUE_LEN_MASK];
  uint8_t steps = e_curr->steps;
  uint16_t ticks = e_curr->ticks;
  bool dirHigh = e_curr->dirPinState == 1;
  bool countUp = e_curr->countUp == 1;
  //char out[200];
  //sprintf(out, "push_command %d: dirHigh: %d, countUp: %d, steps: %d, ticks: %d",
  //        rp, dirHigh, countUp, steps, ticks);
  //Serial.println(out);
  uint32_t loops = pio_calc_loops(steps, ticks, &q->adjust_80MHz);
  uint32_t entry = pio_make_fifo_entry(dirHigh, countUp, steps, loops);
  pio_sm_put(q->pio, q->sm, entry);
  // Serial.println((entry & 512) != 0 ? "HIGH":"LOW");
  rp++;
  q->read_idx = rp;
  return true;
}

void StepperQueue::startQueue() {
  // These commands would clear isr and consequently the sm state's position is
  // lost
  //  pio_sm_set_enabled(pio, sm, true); // sm is running, otherwise loop()
  //  stops pio_sm_clear_fifos(pio, sm); pio_sm_restart(pio, sm);
  _isStarting = true;
  while (push_command(this)) {
  };
  _isStarting = false;
}
void StepperQueue::forceStop() {
  pio_sm_clear_fifos(pio, sm);
  pio_sm_restart(pio, sm);
  // ensure step is zero
  uint32_t entry = pio_make_fifo_entry(queue_end.dir, 0, 0, LOOPS_FOR_1US); // no steps and 1us cycle
  pio_sm_put(pio, sm, entry);
}
bool StepperQueue::isRunning() {
  if (!pio_sm_is_tx_fifo_empty(pio, sm)) {
    return true;
  }
  // Still the sm can process a command
  uint8_t pc = pio_sm_get_pc(pio, sm);
  // if pc > 0, then sm is not waiting for fifo entry
  return (pc != 0);
}
int32_t StepperQueue::getCurrentStepCount() {
  bool running = isRunning();
  uint32_t pos;
  if (!running) {
    // Empty queue
    for (uint8_t i = 0; i <= 4; i++) {
      if (pio_sm_is_rx_fifo_empty(pio, sm)) {
         break;
      }
      pio_sm_get(pio,sm);
    }
    // kick off loop to probe position
    uint32_t entry = pio_make_fifo_entry(queue_end.dir, 0, 0, LOOPS_FOR_1US); // no steps and 1us cycle
    pio_sm_put(pio, sm, entry);
  
    // wait for pc reaching 0 again to ensure isRunning() returns false
    while(isRunning()) {
       if (!pio_sm_is_tx_fifo_empty(pio, sm)) {
          break; // apparently the stepper is now getting commands
       }
    }
  }
  // use last value
  for (uint8_t i = 0; i <= 4; i++) {
    pos = pio_sm_get(pio,sm);
    if (pio_sm_is_rx_fifo_empty(pio, sm)) {
       break;
    }
  }
  return (int32_t)pos;
}

//*************************************************************************************************

bool StepperQueue::isValidStepPin(uint8_t step_pin) {
  // for now we do only support lower 32 gpios
  return (step_pin < 32);
}

//*************************************************************************************************
void StepperTask(void *parameter) {
  FastAccelStepperEngine *engine = (FastAccelStepperEngine *)parameter;
  while (true) {
    engine->manageSteppers();
    const TickType_t delay_time =
        (engine->_delay_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
    vTaskDelay(delay_time);
  }
}
void FastAccelStepperEngine::pushCommands() {
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    FastAccelStepper *s = _stepper[i];
    if (s) {
      StepperQueue *q = &fas_queue[s->_queue_num];
      if (q->_isStarting) {
        continue;
      }
      if (q->isRunning()) {
        // sm is running, so we can push commands
        // without this, any new command in queue would be pushed immediately
        while (push_command(q)) {
        };
      }
    }
  }
}
void StepperTaskQueue(void *parameter) {
  FastAccelStepperEngine *engine = (FastAccelStepperEngine *)parameter;
  while (true) {
    engine->pushCommands();
    const TickType_t delay_time = 1;
    vTaskDelay(delay_time);
  }
}

void fas_init_engine(FastAccelStepperEngine *engine) {
#define STACK_SIZE 3000
#define PRIORITY (configMAX_PRIORITIES - 1)
  engine->_delay_ms = DELAY_MS_BASE;
  xTaskCreate(StepperTask, "StepperTask", STACK_SIZE, engine, PRIORITY, NULL);

  xTaskCreate(StepperTaskQueue, "StepperTaskQueue", STACK_SIZE, engine,
              PRIORITY, NULL);

  program = stepper_make_program();
}
#endif
