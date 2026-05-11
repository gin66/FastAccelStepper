#ifndef PD_STM32_QUEUE_H
#define PD_STM32_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"
#include "fas_arch/result_codes.h"

// ---- Default pin mapping (overridable in sketch) ----
#ifndef STEP_PIN_STEPPER_0
#define STEP_PIN_STEPPER_0 PA0
#endif
#ifndef STEP_PIN_STEPPER_1
#define STEP_PIN_STEPPER_1 PA1
#endif
#ifndef STEP_PIN_STEPPER_2
#define STEP_PIN_STEPPER_2 PA2
#endif
#ifndef STEP_PIN_STEPPER_3
#define STEP_PIN_STEPPER_3 PA3
#endif

// ---- CC interrupt bit helpers ----
#define CCXIE_BIT(ch)   (TIM_DIER_CC1IE << (ch))
#define CCXIF_BIT(ch)   (TIM_SR_CC1IF << (ch))

// BSRR register layout (identical on ALL STM32 families):
//   Bits [0:15]  = set bits  (write 1 → pin HIGH)
//   Bits [16:31] = reset bits (write 1 → pin LOW)
// Clear is always done via BSRR high-half (mask << 16).
// Separate BRR register is NOT used — BSRR reset-half works everywhere.

// ====================================================================
// StepperQueue class — STM32-specific implementation
// ====================================================================
class StepperQueue : public StepperQueueBase {
 public:
#include "../fas_queue/protocol.h"

  volatile bool _isRunning;
  bool _initialized;

  // Step pin GPIO
  uint8_t         _step_pin;
  GPIO_TypeDef*   _step_port;
  uint32_t        _step_set_mask;   // BSRR set mask (write to BSRR low = set HIGH)
  uint32_t        _step_clr_mask;   // BSRR clear mask (write to BSRR high = set LOW) = mask<<16

  // Direction pin (atomic via BSRR)
  volatile uint32_t* _dir_bsrr;     // &GPIOx->BSRR
  uint32_t        _dir_set_mask;    // BSRR low bits = set HIGH
  uint32_t        _dir_clr_mask;    // BSRR high bits = set LOW (mask << 16)

  // Timer
  volatile uint32_t* _ccr_reg;      // &TIM2->CCR1/2/3/4
  uint8_t         _timer_ch;        // 0..3

  // Pulse tracking
  volatile bool   _pulse_high;
  volatile bool   _dir_delay_active;  // Direction settling in progress

  // Channel-to-queue mapping (static)
  static StepperQueue* _ch_to_queue[4];

  // ---- Inline methods ----
  inline void _pd_initVars() {
    _step_pin       = PIN_UNDEFINED;
    _step_port      = NULL;
    _step_set_mask  = 0;
    _step_clr_mask  = 0;
    _dir_bsrr       = NULL;
    _dir_set_mask   = 0;
    _dir_clr_mask   = 0;
    _ccr_reg        = NULL;
    _timer_ch       = 0;
    _isRunning      = false;
    _initialized    = false;
    _pulse_high     = false;
    _dir_delay_active = false;
    max_speed_in_ticks = STEP_PULSE_WIDTH_TICKS * 4;
  }

  inline bool isRunning() const            { return _isRunning; }
  inline bool isReadyForCommands() const   { return true; }

  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
    if ((dir_pin != PIN_UNDEFINED) && ((dir_pin & PIN_EXTERNAL_FLAG) == 0)) {
      GPIO_TypeDef* port = digitalPinToPort(dir_pin);
      uint32_t mask = digitalPinToBitMask(dir_pin);
      _dir_bsrr = &port->BSRR;
      _dir_set_mask = mask;
      _dir_clr_mask = mask << 16;   // BSRR high half = reset (works on ALL families)
    }
  }

  void adjustSpeedToStepperCount(uint8_t steppers);

 private:
  static StepperQueue* allocateSlot(uint8_t step_pin, uint8_t timer_ch);
};

// ---- Direction pin: atomic via BSRR/BRR ----
#define SET_DIRECTION_PIN_STATE(q, high)                      \
  do {                                                        \
    if ((q)->_dir_bsrr) {                                    \
      *(q->_dir_bsrr) = (high) ? (q)->_dir_set_mask          \
                               : (q)->_dir_clr_mask;         \
    }                                                         \
  } while (0)

// ---- Enable pin: simple digitalWrite ----
#define SET_ENABLE_PIN_STATE(q, pin, high) \
  digitalWrite((pin), (high) ? HIGH : LOW)

// ---- Direction-to-pulse delay ----
#define AFTER_SET_DIR_PIN_DELAY_US 30

#endif /* PD_STM32_QUEUE_H */