#include "FastAccelStepperEngine.h"
#include "FastAccelStepper.h"
#include "fas_queue/stepper_queue.h"

#ifndef TEST
#define printf DO_NOT_USE_PRINTF
#define puts DO_NOT_USE_PUTS
#endif

static uint8_t fas_ledPin = PIN_UNDEFINED;
static uint16_t fas_debug_led_cnt = 0;

FastAccelStepper fas_stepper[MAX_STEPPER];

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
#if defined(SUPPORT_RP_PICO)
  claimed_pios = 0;
#endif
  fas_init_engine(this);
}
#endif

#if defined(SUPPORT_ESP32_I2S)
#include "pd_esp32/i2s_manager.h"
#include "pd_esp32/i2s_constants.h"

bool FastAccelStepperEngine::initI2sSingleStepper(
    const I2sSingleStepperConfig& cfg) {
  I2sManager& mgr = I2sManager::instance();
  if (!mgr.init(cfg.data_pin, cfg.bclk_pin, -1)) {
    return false;
  }
  return mgr.startDma();
}

bool FastAccelStepperEngine::isI2sInitialized() const {
  return I2sManager::instance().isInitialized();
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

#if defined(SUPPORT_DYNAMIC_ALLOCATION)
// dynamic allocation is currently only supported for esp32
FastAccelStepper* FastAccelStepperEngine::stepperConnectToPin(uint8_t step_pin) {
  return stepperConnectToPin(step_pin, DRIVER_DONT_CARE);
}
FastAccelStepper* FastAccelStepperEngine::stepperConnectToPin(
    uint8_t step_pin, FasDriver driver_type) {
  return NULL;
}
#else
#if !defined(SUPPORT_SELECT_DRIVER_TYPE)
FastAccelStepper* FastAccelStepperEngine::stepperConnectToPin(uint8_t step_pin)
#else
FastAccelStepper* FastAccelStepperEngine::stepperConnectToPin(
    uint8_t step_pin, FasDriver driver_type)
#endif
{
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    FastAccelStepper* s = _stepper[i];
    if (s) {
      if (s->getStepPin() == step_pin) {
        return NULL;
      }
    }
  }
  if (_stepper_cnt >= MAX_STEPPER) {
    return NULL;
  }
  if (!StepperQueue::isValidStepPin(step_pin)) {
    return NULL;
  }
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  uint8_t queue_from = 0;
  uint8_t queue_to = QUEUES_MCPWM_PCNT + QUEUES_RMT + QUEUES_I2S;
  if (driver_type == DRIVER_MCPWM_PCNT) {
    queue_to = QUEUES_MCPWM_PCNT;
  } else if (driver_type == DRIVER_RMT) {
    queue_from = QUEUES_MCPWM_PCNT;
  }
  int8_t fas_stepper_num = -1;
  for (uint8_t i = queue_from; i < queue_to; i++) {
    FastAccelStepper* s = _stepper[i];
    if (s == NULL) {
      fas_stepper_num = i;
      break;
    }
  }
  if (fas_stepper_num < 0) {
    return NULL;
  }
#else
#if defined(NEED_FIXED_QUEUE_TO_PIN_MAPPING)
  int8_t fas_stepper_num = StepperQueue::queueNumForStepPin(step_pin);
  if (fas_stepper_num < 0) {
    fas_stepper_num = _stepper_cnt;
  }
#else
  int8_t fas_stepper_num = _stepper_cnt;
#endif
#endif
  _stepper_cnt++;

  FastAccelStepper* s = &fas_stepper[fas_stepper_num];
  bool success = s->init(this, fas_stepper_num, step_pin);
  if (!success) {
    return NULL;
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
#ifdef SUPPORT_EXTERNAL_DIRECTION_PIN
      if (s->externalDirPinChangeCompletedIfNeeded()) {
        s->fill_queue();
      }
#else
      s->fill_queue();
#endif
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
