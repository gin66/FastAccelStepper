#include "StepperISR.h"

#if defined(SUPPORT_RP_PICO)
#include <FreeRTOS.h>
#include <task.h>

#include "pico_pio.h"

// Here are the global variables to interface with the interrupts
StepperQueue fas_queue[NUM_QUEUES];
static stepper_pio_program *program;

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
  uint8_t channel = queue_num;
  _step_pin = step_pin;
}

bool StepperQueue::claim_pio_sm(FastAccelStepperEngine *engine) {
  // We have two or three PIO modules. If we need one sm from a pio,
  // the whole PIO need to be claimed due to the size of our pio code.
  // Let's check first, if there is any PIO claimed.
  // If yes, check if we can claim a sm from that PIO.
  for (uint8_t i = 0;i < engine->claimed_pios;i++) {
    // pio has been claimed, so our program is valid
    int claimed_sm = pio_claim_unused_sm(engine->pio[i], false);
    if (claimed_sm >= 0) {
      // successfully claimed
      pio = engine->pio[i];
      sm = claimed_sm;
      return true;
    }
  }
  pio_program_t pio_program;
  pio_program.instructions = program->code;
  pio_program.length = program->pc;
  pio_program.origin = 0;
  pio_program.pio_version = 0;
  pio_program.used_gpio_ranges = 0;
  uint offset;
  bool rc = pio_claim_free_sm_and_add_program_for_gpio_range(&pio_program, &pio, &sm, &offset, _step_pin, 1, true);
  return rc;
}

void StepperQueue::setupSM() {
  pio_sm_config c = pio_get_default_sm_config();
  // Map the state machine's OUT pin group to one pin, namely the `pin`
  // parameter to this function.
  sm_config_set_jmp_pin(&c, _step_pin);    // Step pin read back for double period
  if ((dirPin != PIN_UNDEFINED) && ((dirPin & PIN_EXTERNAL_FLAG) == 0)) {
    sm_config_set_out_pins(&c, dirPin, 1);// Direction pin via out
  }
  sm_config_set_set_pins(&c, _step_pin, 1);// Step pin via set
  sm_config_set_wrap(&c, program->wrap_target, program->wrap_at);

  // Load our configuration, and jump to the start of the program
  uint offset = 0;
  pio_sm_init(pio, sm, offset, &c);
  // Set the pin direction to output at the PIO
  pio_sm_set_consecutive_pindirs(pio, sm, _step_pin, 1, true);
  if ((dirPin != PIN_UNDEFINED) && ((dirPin & PIN_EXTERNAL_FLAG) == 0)) {
    pio_sm_set_consecutive_pindirs(pio, sm, dirPin, 1, true);
  }
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

bool StepperQueue::isReadyForCommands() {
  return true;
}

static void push_command(StepperQueue *q) {
  Serial.println("PUSH");
  uint8_t rp = q->read_idx;
  if (rp == q->next_write_idx) {
    // no command in queue
    return;
  }
  if (pio_sm_is_tx_fifo_full(q->pio,q->sm)) {
    return;
  }
  // Process command
  struct queue_entry *e_curr = &q->entry[rp & QUEUE_LEN_MASK];
  uint8_t steps = e_curr->steps;
  uint16_t ticks = e_curr->ticks;
 
  bool forward = true;
  uint32_t period = stepper_calc_period(forward, steps, 64000); // 4ms
  uint32_t entry = (period<<9) | (forward ? 0:256) | steps;
  pio_sm_put(q->pio, q->sm, entry);
  Serial.println(entry);
  rp++;
}

void StepperQueue::startQueue() {
  pio_sm_set_enabled(pio, sm, true); // sm is running, otherwise loop() stops
  pio_sm_clear_fifos(pio, sm);
  pio_sm_restart(pio, sm);
  push_command(this);
  push_command(this);
  push_command(this);
  push_command(this);
  push_command(this);
}
void StepperQueue::forceStop() {
  pio_sm_set_enabled(pio, sm, false);
}
int32_t StepperQueue::getCurrentPosition() {
  return 0;
}

//*************************************************************************************************

bool StepperQueue::isValidStepPin(uint8_t step_pin) {
  // for now we do only support lower 32 gpios
  return (step_pin < 32);
}
int8_t StepperQueue::queueNumForStepPin(uint8_t step_pin) { return -1; }

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

void StepperQueue::adjustSpeedToStepperCount(uint8_t steppers) {
  max_speed_in_ticks = 80;  // This equals 200kHz @ 16MHz
}

void fas_init_engine(FastAccelStepperEngine *engine) {
#define STACK_SIZE 3000
#define PRIORITY (configMAX_PRIORITIES - 1)
  engine->_delay_ms = DELAY_MS_BASE;
  xTaskCreate(StepperTask, "StepperTask", STACK_SIZE, engine, PRIORITY, NULL);

  program = stepper_make_program();
}

FastAccelStepper fas_stepper[MAX_STEPPER];
FastAccelStepper* FastAccelStepperEngine::stepperConnectToPin(uint8_t step_pin)
{
  // Check if already connected
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    FastAccelStepper* s = _stepper[i];
    if (s) {
      if (s->getStepPin() == step_pin) {
        return NULL;
      }
    }
  }
  if (!_isValidStepPin(step_pin)) {
    return NULL;
  }
  int8_t fas_stepper_num = StepperQueue::queueNumForStepPin(step_pin);
  if (fas_stepper_num < 0) {  // flexible, so just choose next
    if (_stepper_cnt >= MAX_STEPPER) {
      return NULL;
    }
    fas_stepper_num = _stepper_cnt;
  }

  FastAccelStepper* s = &fas_stepper[fas_stepper_num];
  s->init(this, fas_stepper_num, step_pin);

  _stepper_cnt++;
  _stepper[fas_stepper_num] = s;
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    FastAccelStepper* sx = _stepper[i];
    if (sx) {
      fas_queue[sx->_queue_num].adjustSpeedToStepperCount(_stepper_cnt);
    }
  }
  return s;
}
#endif
