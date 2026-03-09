#include "FastAccelStepperEngine.h"
#include "FastAccelStepper.h"
#include "fas_queue/stepper_queue.h"
#if defined(SUPPORT_ESP32_I2S)
#include "pd_esp32/i2s_manager.h"
#endif

#ifndef TEST
#define printf DO_NOT_USE_PRINTF
#define puts DO_NOT_USE_PUTS
#endif

static uint8_t fas_ledPin = PIN_UNDEFINED;
static uint16_t fas_debug_led_cnt = 0;

#if !defined(SUPPORT_DYNAMIC_ALLOCATION)
FastAccelStepper fas_stepper[MAX_STEPPER];
#endif

#if defined(SUPPORT_CPU_AFFINITY)
void FastAccelStepperEngine::init(uint8_t cpu_core) {
  _externalCallForPin = NULL;
  _stepper_cnt = 0;
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    _stepper[i] = NULL;
  }
  fas_init_engine(this, cpu_core);
}
#else
void FastAccelStepperEngine::init() {
  _externalCallForPin = NULL;
  _stepper_cnt = 0;
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    _stepper[i] = NULL;
  }
#if defined(SUPPORT_DYNAMIC_ALLOCATION)
  for (uint8_t i = 0; i < NUM_QUEUES; i++) {
    fas_queue[i] = NULL;
  }
#endif

#if defined(SUPPORT_RP_PICO)
  claimed_pios = 0;
#endif
  fas_init_engine(this);
}
#endif

void FastAccelStepperEngine::setExternalCallForPin(
    bool (*func)(uint8_t pin, uint8_t value)) {
  _externalCallForPin = func;
}

bool FastAccelStepperEngine::isDirPinBusy(uint8_t dir_pin,
                                          uint8_t except_stepper) {
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    if (i != except_stepper) {
      FastAccelStepper* s = _stepper[i];
      if (s) {
        if (s->getDirectionPin() == dir_pin) {
          if (s->isQueueRunning()) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

#if defined(SUPPORT_ESP32_I2S) && defined(SUPPORT_DYNAMIC_ALLOCATION)
bool FastAccelStepperEngine::initI2sMux(uint8_t data_pin, uint8_t bclk_pin,
                                        uint8_t ws_pin) {
  if (StepperQueue::_i2s_mux_initialized) {
    return false;
  }
  I2sManager* mgr = I2sManager::create(
      (gpio_num_t)data_pin, (gpio_num_t)bclk_pin, (gpio_num_t)ws_pin);
  if (mgr == nullptr) {
    return false;
  }
  mgr->_is_mux = true;
  StepperQueue::_i2s_mux_manager = mgr;
  StepperQueue::_i2s_mux_initialized = true;
  return true;
}

void FastAccelStepperEngine::i2sMuxSetBit(uint8_t slot, bool value) {
  if (StepperQueue::_i2s_mux_manager != nullptr) {
    StepperQueue::_i2s_mux_manager->i2sMuxSetBit(slot, value);
  }
}

bool FastAccelStepperEngine::i2sMuxGetBit(uint8_t slot) {
  if (StepperQueue::_i2s_mux_manager != nullptr) {
    return StepperQueue::_i2s_mux_manager->i2sMuxGetBit(slot);
  }
  return false;
}
#endif

#if defined(SUPPORT_DYNAMIC_ALLOCATION)
// Dynamic allocation (ESP32 with IDF >= 5.3)
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
FastAccelStepper* FastAccelStepperEngine::stepperConnectToPin(
    uint8_t step_pin, FasDriver driver_type) {
  StepperQueue* q = StepperQueue::tryAllocateQueue(driver_type, step_pin);
#else
FastAccelStepper* FastAccelStepperEngine::stepperConnectToPin(
    uint8_t step_pin) {
  StepperQueue* q = StepperQueue::tryAllocateQueue(step_pin);
#endif
  if (q == nullptr) {
    return nullptr;
  }

  uint8_t fas_stepper_num = _stepper_cnt;
  _stepper_cnt++;
  fas_queue[fas_stepper_num] = q;

  FastAccelStepper* s = new FastAccelStepper();
  if (!s->init(this, fas_stepper_num, step_pin)) {
    return nullptr;
  }
  _stepper[fas_stepper_num] = s;
  return s;
}
#else
// Static allocation (AVR, SAM, Pico, TEST, ESP32 IDF 4.x)
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
FastAccelStepper* FastAccelStepperEngine::stepperConnectToPin(
    uint8_t step_pin, FasDriver driver_type) {
  StepperQueue* q = StepperQueue::tryAllocateQueue(driver_type, step_pin);
#else
FastAccelStepper* FastAccelStepperEngine::stepperConnectToPin(
    uint8_t step_pin) {
  StepperQueue* q = StepperQueue::tryAllocateQueue(step_pin);
#endif
  if (q == nullptr) {
    return nullptr;
  }

  uint8_t fas_stepper_num = (uint8_t)(q - fas_queue);
  _stepper_cnt++;

  static FastAccelStepper fas_stepper[MAX_STEPPER];
  FastAccelStepper* s = &fas_stepper[fas_stepper_num];
  if (!s->init(this, fas_stepper_num, step_pin)) {
    return nullptr;
  }
  _stepper[fas_stepper_num] = s;

#if defined(NEED_ADJUSTABLE_MAX_SPEED_DEPENDING_ON_STEPPER_COUNT)
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    const FastAccelStepper* sx = _stepper[i];
    if (sx) {
      FAS_QUEUE(sx->_queue_num).adjustSpeedToStepperCount(_stepper_cnt);
    }
  }
#endif

  return s;
}
#endif

void FastAccelStepperEngine::setDebugLed(uint8_t ledPin) {
  fas_ledPin = ledPin;
  PIN_OUTPUT(fas_ledPin, LOW);
}

void FastAccelStepperEngine::manageSteppers() {
#ifdef DEBUG_LED_HALF_PERIOD
  if (fas_ledPin != PIN_UNDEFINED) {
    fas_debug_led_cnt++;
    if (fas_debug_led_cnt == DEBUG_LED_HALF_PERIOD) {
      digitalWrite(fas_ledPin, HIGH);
    }
    if (fas_debug_led_cnt == 2 * DEBUG_LED_HALF_PERIOD) {
      digitalWrite(fas_ledPin, LOW);
      fas_debug_led_cnt = 0;
    }
  }
#endif
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    FastAccelStepper* s = _stepper[i];
    if (s) {
      if (s->externalDirPinChangeCompletedIfNeeded()) {
        s->fill_queue();
      }
    }
  }

  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    FastAccelStepper* s = _stepper[i];
    if (s) {
      if (s->needAutoDisable()) {
        uint8_t high_active_pin = s->getEnablePinHighActive();
        uint8_t low_active_pin = s->getEnablePinLowActive();

        bool agree = true;
        for (uint8_t j = 0; j < MAX_STEPPER; j++) {
          if (i != j) {
            FastAccelStepper* other = _stepper[j];
            if (other) {
              if (other->usesAutoEnablePin(high_active_pin) ||
                  other->usesAutoEnablePin(low_active_pin)) {
                if (!other->agreeWithAutoDisable()) {
                  agree = false;
                  break;
                }
              }
            }
          }
        }
        if (agree) {
          for (uint8_t j = 0; j < MAX_STEPPER; j++) {
            FastAccelStepper* current = _stepper[j];
            if (current) {
              if (current->usesAutoEnablePin(high_active_pin) ||
                  current->usesAutoEnablePin(low_active_pin)) {
                current->disableOutputs();
              }
            }
          }
        }
      }
    }
  }

  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    FastAccelStepper* s = _stepper[i];
    if (s) {
      fasDisableInterrupts();
      s->updateAutoDisable();
      fasEnableInterrupts();
    }
  }
}
