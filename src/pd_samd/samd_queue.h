#ifndef PD_SAMD_QUEUE_H
#define PD_SAMD_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"

// SAMD51 pulse driver.
//
// Each stepper claims the TCC instance its step pin muxes to (discovered via
// g_APinDescription). The TCC runs in normal PWM: PER = step interval in
// ticks, CC[channel] = pulse width (CC = 0 => pause period, output low).
// The overflow interrupt fires once per step and stages the following period
// into the buffered registers (PERBUF/CCBUF), which the hardware latches at
// the next period boundary. Pulse edges are therefore hardware-timed and
// immune to interrupt latency; only the staging has a full period of slack.
//
// See docs/adr/0001-tcc-npwm-per-step-isr.md for the design decision.

class StepperQueue : public StepperQueueBase {
 public:
#include "../fas_queue/protocol.h"

  Tcc* _tcc;
  uint8_t _tcc_num;      // TCC instance index (also ISR dispatch key)
  uint8_t _tcc_channel;  // CC channel / WO index of the step pin
  uint8_t _queue_num;
  uint8_t _step_pin;
  uint16_t _pulse_ticks;  // step pulse high time in ticks
  volatile bool _isRunning;
  // One dead (pulse-less) period is inserted after the queue drains before
  // the TCC is stopped, so a command arriving late is picked up seamlessly.
  volatile bool _noMoreCommands;
  bool _connected;

  PortGroup* _dirPinPortGrp;
  uint32_t _dirPinMask;

  inline void _pd_initVars() {
    _tcc = NULL;
    _step_pin = PIN_UNDEFINED;
    _dirPinPortGrp = NULL;
    _dirPinMask = 0;
    _isRunning = false;
    _noMoreCommands = false;
    _connected = false;
    // 10 us/step = 100k steps/s default ceiling; the per-step ISR costs
    // ~1 us, so this keeps interrupt load below ~10% per running stepper.
    max_speed_in_ticks = TICKS_PER_S / 100000;
  }

  inline bool isRunning() const { return _isRunning; }
  inline bool isReadyForCommands() const { return true; }

  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
    if ((dir_pin != PIN_UNDEFINED) && ((dir_pin & PIN_EXTERNAL_FLAG) == 0)) {
      const PinDescription& pd = g_APinDescription[dir_pin];
      _dirPinPortGrp = &PORT->Group[pd.ulPort];
      _dirPinMask = (1ul << pd.ulPin);
    }
  }

  // ISR-side helpers (defined in samd_queue.cpp)
  bool stageNextPeriod();
  void handleOverflow();

 private:
  static bool isValidStepPin(uint8_t step_pin);
};

#define SET_DIRECTION_PIN_STATE(q, high)                    \
  do {                                                      \
    if ((q)->_dirPinPortGrp != NULL) {                      \
      if (high) {                                           \
        (q)->_dirPinPortGrp->OUTSET.reg = (q)->_dirPinMask; \
      } else {                                              \
        (q)->_dirPinPortGrp->OUTCLR.reg = (q)->_dirPinMask; \
      }                                                     \
    }                                                       \
  } while (0)

#define SET_ENABLE_PIN_STATE(q, pin, high) \
  digitalWrite((pin), (high) ? HIGH : LOW)

#define AFTER_SET_DIR_PIN_DELAY_US 30

inline AqeResultCode StepperQueue::addDirChangePauseToQueue(
    const struct stepper_command_s* cmd, bool start,
    uint16_t dir_change_delay_ticks) {
  if (dir_change_delay_ticks == 0) {
    return AQE_OK;
  }
  if ((cmd->steps == 0) && (cmd->ticks >= dir_change_delay_ticks)) {
    return AQE_OK;
  }
  uint16_t pause_ticks = dir_change_delay_ticks;
  if (cmd->steps == 0) {
    pause_ticks = dir_change_delay_ticks - cmd->ticks;
  }
  struct stepper_command_s pause_cmd = {
      .ticks = (uint16_t)fas_max(pause_ticks, MIN_CMD_TICKS),
      .steps = 0,
      .count_up = cmd->count_up};
  AqeResultCode res = addQueueEntry(&pause_cmd, start);
  if (res != AQE_OK) {
    return res;
  }
  return AQE_DIR_CHANGE_PAUSE_INJECTED;
}

#endif  // PD_SAMD_QUEUE_H
