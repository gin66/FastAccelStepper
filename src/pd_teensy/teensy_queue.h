#ifndef PD_TEENSY_QUEUE_H
#define PD_TEENSY_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"
#include "imxrt.h"

// Teensy 4.x (i.MX RT1062) pulse driver. EXPERIMENTAL, not yet verified on
// real hardware - see pd_config.h for context and pd_teensy/teensy_queue.cpp
// for the implementation and its references.
//
// Each stepper claims one channel of one of the 4 QuadTimer (TMR) modules.
// Unlike SAMD/SAM, the channel's own output pin is not used: the step (and,
// on direction changes, the dir) pin is toggled directly in the ISR via
// digitalWriteFast(), so any digital pin works, not just specific muxed
// ones. The QuadTimer channel is only used to time the two ISR calls a step
// needs (rising edge, then falling edge after the pulse width) precisely,
// the same way this library's AVR backend uses a hardware compare match to
// time each edge instead of busy-waiting.

class StepperQueue : public StepperQueueBase {
 public:
#include "../fas_queue/protocol.h"

  IMXRT_TMR_CH_t* _regs;  // this channel's QuadTimer register block
  uint8_t _tmr_module;    // 0..3 (TMR1..TMR4) - ISR dispatch key
  uint8_t _tmr_channel;   // 0..3 within the module
  uint8_t _queue_num;
  uint8_t _step_pin;
  uint16_t _pulse_ticks;      // step pulse high time, in ticks
  uint16_t _remaining_ticks;  // ticks from falling edge to next rising edge
  bool _pulse_phase;          // true: pin is HIGH, next event lowers it
  volatile bool _isRunning;
  // One dead (pulse-less) period is inserted after the queue drains before
  // the timer is stopped, so a command arriving late is picked up seamlessly
  // (same grace-period idea as the SAMD backend).
  volatile bool _noMoreCommands;
  bool _connected;

  inline void _pd_initVars() {
    _regs = NULL;
    _step_pin = PIN_UNDEFINED;
    _pulse_phase = false;
    _isRunning = false;
    _noMoreCommands = false;
    _connected = false;
    // UNVERIFIED placeholder ceiling (50k steps/s <=> 20us/step). Each step
    // costs two ISR calls here (rising + falling edge), unlike SAMD's one.
    // Measure on real hardware and adjust via setAbsoluteSpeedLimit() if
    // needed - see SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING in pd_config.h.
    max_speed_in_ticks = TICKS_PER_S / 50000;
  }

  inline bool isRunning() const { return _isRunning; }
  inline bool isReadyForCommands() const { return true; }

  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
    if ((dir_pin != PIN_UNDEFINED) && ((dir_pin & PIN_EXTERNAL_FLAG) == 0)) {
      pinMode(dir_pin, OUTPUT);
    }
  }

  // ISR-side helpers (defined in teensy_queue.cpp)
  void primeFromQueue(uint8_t rp);
  void handleCompareMatch();

 private:
  static bool isValidStepPin(uint8_t step_pin);
};

// digitalWriteFast() accepts a runtime pin number on Teensyduino (it falls
// back to a fast table lookup, not a compile-time-only fast path) - the
// same call TeensyStep4 uses from its own step/dir ISRs.
#define SET_DIRECTION_PIN_STATE(q, high) \
  digitalWriteFast((q)->dirPin, (high) ? HIGH : LOW)

#define SET_ENABLE_PIN_STATE(q, pin, high) \
  digitalWrite((pin), (high) ? HIGH : LOW)

#define AFTER_SET_DIR_PIN_DELAY_US 5

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
  _injected_pause_ticks = pause_cmd.ticks;
  return AQE_DIR_CHANGE_PAUSE_INJECTED;
}

#endif  // PD_TEENSY_QUEUE_H
