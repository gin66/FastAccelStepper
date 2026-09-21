#include "FastAccelStepperEngine.h"
#include <math.h>
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
  for (uint8_t i = 0; i < NUM_QUEUES; i++) {
#if defined(SUPPORT_DYNAMIC_ALLOCATION)
    fas_queue[i] = NULL;
#else
    fas_queue[i]._initVars();
#endif
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
  for (uint8_t i = 0; i < NUM_QUEUES; i++) {
#if defined(SUPPORT_DYNAMIC_ALLOCATION)
    fas_queue[i] = NULL;
#else
    fas_queue[i]._initVars();
#endif
  }

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
  StepperQueue* q = StepperQueue::tryAllocateQueue(this, driver_type, step_pin);
#else
FastAccelStepper* FastAccelStepperEngine::stepperConnectToPin(
    uint8_t step_pin) {
  StepperQueue* q = StepperQueue::tryAllocateQueue(this, step_pin);
#endif
  if (q == nullptr) {
    return nullptr;
  }

  uint8_t fas_stepper_num = _stepper_cnt;
  _stepper_cnt++;
  fas_queue[fas_stepper_num] = q;

  FastAccelStepper* s = new FastAccelStepper();
  s->init(this, fas_stepper_num, step_pin);
  _stepper[fas_stepper_num] = s;
  return s;
}
#else
// Static allocation (AVR, SAM, Pico, TEST, ESP32 IDF 4.x)
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
FastAccelStepper* FastAccelStepperEngine::stepperConnectToPin(
    uint8_t step_pin, FasDriver driver_type) {
  StepperQueue* q = StepperQueue::tryAllocateQueue(this, driver_type, step_pin);
#else
FastAccelStepper* FastAccelStepperEngine::stepperConnectToPin(
    uint8_t step_pin) {
  StepperQueue* q = StepperQueue::tryAllocateQueue(this, step_pin);
#endif
  if (q == nullptr) {
    return nullptr;
  }

  uint8_t fas_stepper_num = (uint8_t)(q - fas_queue);
  _stepper_cnt++;

  FastAccelStepper* s = &fas_stepper[fas_stepper_num];
  s->init(this, fas_stepper_num, step_pin);
  _stepper[fas_stepper_num] = s;

#if defined(NEED_ADJUSTABLE_MAX_SPEED_DEPENDING_ON_STEPPER_COUNT)
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    const FastAccelStepper* sx = _stepper[i];
    if (sx) {
      sx->_queue()->adjustSpeedToStepperCount(_stepper_cnt);
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

MoveResultCode FastAccelStepperEngine::moveAllToSync(
    FastAccelStepper* const* steppers, const int32_t* targetPositions,
    uint8_t count) {
  if (count > MAX_STEPPER) {
    count = MAX_STEPPER;
  }

  // Pass 1: find how long the slowest axis would take at its own
  // currently configured speed/acceleration. That duration becomes the
  // common target duration for all axes.
  float distance[MAX_STEPPER];
  float target_time = 0;
  for (uint8_t i = 0; i < count; i++) {
    distance[i] = 0;
    FastAccelStepper* s = steppers[i];
    if (s == NULL) {
      continue;
    }
    int32_t d = targetPositions[i] - s->getCurrentPosition();
    distance[i] = (d < 0) ? (float)(-d) : (float)d;
    float v = s->getSpeedInMilliHz() / 1000.0f;
    float a = (float)s->getAcceleration();
    if ((distance[i] <= 0) || (v <= 0) || (a <= 0)) {
      continue;
    }
    // symmetric ramp up/down: distance covered while not at constant
    // speed is v^2/a, taking time 2*v/a
    float t_ramp = v / a;
    float d_ramp = v * t_ramp;
    float t = (distance[i] >= d_ramp)
                  ? 2.0f * t_ramp + (distance[i] - d_ramp) / v
                  : 2.0f * sqrt(a * distance[i]) / a;
    if (t > target_time) {
      target_time = t;
    }
  }

  // Pass 2: slow every other axis down (speed first, and - for moves too
  // short to ever reach that reduced speed - acceleration too) so its own
  // move takes target_time, then start the move.
  MoveResultCode first_error = MOVE_OK;
  for (uint8_t i = 0; i < count; i++) {
    FastAccelStepper* s = steppers[i];
    if (s == NULL) {
      continue;
    }
    float v = s->getSpeedInMilliHz() / 1000.0f;
    float a = (float)s->getAcceleration();
    if ((distance[i] > 0) && (target_time > 0) && (v > 0) && (a > 0)) {
      float v_new = v;
      float a_new = a;
      float disc =
          (a * target_time) * (a * target_time) - 4.0f * a * distance[i];
      bool trapezoid = false;
      if (disc >= 0) {
        v_new = (a * target_time - sqrt(disc)) / 2.0f;
        trapezoid = (distance[i] - (v_new * v_new) / a) >= 0;
      }
      if (!trapezoid) {
        // move too short to ever cruise at a constant speed for
        // target_time: shrink acceleration, so the triangular ramp itself
        // takes exactly target_time
        a_new = 4.0f * distance[i] / (target_time * target_time);
        v_new = 2.0f * distance[i] / target_time;
      }
      // never exceed the axis' own configured maximum
      if (v_new > v) v_new = v;
      if (v_new < 1.0f) v_new = 1.0f;
      if (a_new > a) a_new = a;
      if (a_new < 1.0f) a_new = 1.0f;
      s->setSpeedInHz((uint32_t)(v_new + 0.5f));
      s->setAcceleration((int32_t)(a_new + 0.5f));
    }
    MoveResultCode res = s->moveTo(targetPositions[i]);
    if ((first_error == MOVE_OK) && (res != MOVE_OK)) {
      first_error = res;
    }
  }
  return first_error;
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
      s->fill_queue();
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
