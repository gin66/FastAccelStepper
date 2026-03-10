#ifndef FAS_QUEUE_PROTOCOL_H
#define FAS_QUEUE_PROTOCOL_H

// This file defines the protocol interface for StepperQueue implementations.
// Each platform-specific StepperQueue class (in pd_*/queue.h) should include
// this file to declare these methods. The implementations go in the respective
// .cpp files.
//
// The protocol methods are injected into the StepperQueue class via inclusion.
// This avoids circular dependencies since StepperQueue has no includes here.
//
// No includes in this file - types are provided by the including class.

//==========================================================================
// QUEUE ALLOCATION
//==========================================================================
// Unified allocation interface - all platforms must implement.
// Validates the step pin and returns an allocated queue, or nullptr on failure.
// - For SUPPORT_DYNAMIC_ALLOCATION: allocates queue dynamically
// - For static allocation: returns pointer to pre-allocated queue slot
// - For SUPPORT_SELECT_DRIVER_TYPE (ESP32): allows choosing driver type
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
static StepperQueue* tryAllocateQueue(FasDriver driver, uint8_t step_pin);
#else
static StepperQueue* tryAllocateQueue(uint8_t step_pin);
#endif

//==========================================================================
// QUEUE PROTOCOL METHODS
//==========================================================================

AqeResultCode addQueueEntry(const struct stepper_command_s* cmd, bool start);
int32_t getCurrentPosition() const;
uint32_t ticksInQueue() const;
bool hasTicksInQueue(uint32_t min_ticks) const;
bool getActualTicksWithDirection(struct actual_ticks_s* speed) const;

bool init(FastAccelStepperEngine* engine, uint8_t queue_num, uint8_t step_pin);
void startQueue();
void forceStop();
void _initVars();
void connect();
void disconnect();

//==========================================================================
// PREPROCESSOR PROTOCOL EXTENSIONS (defined in <arch>_queue.h)
//==========================================================================
//
// Setting direction/enable must be synchronized with step generation
// in order to avoid steps into wrong direction.
// Depending on the platform and driver capabilities, the direction/enable pin
// may be controlled in different ways, which are abstracted by the following
// macros. The platform-specific StepperQueue implementation defines these
// macros according to its capabilities, and the common addQueueEntry() logic
// uses them to control the direction/enable pins without needing to know the
// underlying implementation details.
//
// There are four possibilities for each pin (direction and enable):
//
// 1. Synchronized with commands
//    - Pin state changes are synchronized with command execution in ISR
//    - AVR, SAM, MCPWM, PICO for direction pin
//
// 2. Synchronized with buffer
//    - Driver controls the pin, but timing need to follow rules to avoid
//      race conditions with step pulses due to buffering in driver/ISR
//    - Direction/enable applied at buffer boundaries
//    - I2S-MUX: direction pin can be either in I2S bitstream or GPIO (same
//    situation)
//    - time delays depending on buffer size/buffer mechanism
//      BEFORE/AFTER_DIR_CHANGE_DELAY_TICKS
//
// 3. Queue controlled
//    - addQueueEntry() controls the pin directly as GPIO
//    - fixed to 4ms task rate and need to ensure no steps in command queue
//      (looking at the current or following command is not sufficient)
//
// 4. Externally controlled
//    - Pin controlled via callback by application (e.g. using I/O expander)
//    - Uses repeat_entry mechanism until pin change was applied by application
//    - Pin has PIN_EXTERNAL_FLAG set
//
// addQueueEntry() will use only these macros to control the direction pin,
// if defined. The macros are not used if dirPin is set to PIN_UNDEFINED
// or if PIN_EXTERNAL_FLAG is set (external control).
//
// === DIRECTION PIN HANDLING ===
//
// SET_DIRECTION_PIN_STATE(q, high)
//   Set direction pin to absolute HIGH or LOW state.
//   PRECONDITION: NOT called for CONTROLLED_EXTERNAL pins (PIN_EXTERNAL_FLAG)
//   Used by addQueueEntry() when queue is not running.
//   Must be defined
//
// AFTER_SET_DIR_PIN_DELAY_US
//   Microseconds to delay after directly setting direction pin state.
//   Gives the driver hardware time to register the pin change.
//   Only used when queue is empty and not running.
//   If undefined, no delay.
//
// BEFORE_DIR_CHANGE_DELAY_TICKS(q)
//   Ticks to pause BEFORE direction change entry appears in output.
//   Used by addQueueEntry() to insert pause for buffered drivers.
//   - Synchronized with commands: 0
//   - Synchronized with buffer (I2S-MUX): I2S_BLOCK_TICKS
//   - Queue controlled (RMT): MIN_CMD_TICKS
//   - Externally controlled: 0
//   If undefined, assume 0
//
// AFTER_DIR_CHANGE_DELAY_TICKS(q)
//   Minimum ticks for the direction change command itself.
//   Ensures dir change is alone in buffer for synchronized-with-buffer case.
//   - Synchronized with buffer (I2S-MUX): I2S_BLOCK_TICKS
//   - All others: 0 (or undefined)
//   If undefined, assume 0
//
// === ENABLE PIN HANDLING ===
//
// Enable pin control always happens in addQueueEntry() context (not ISR).
// The ~4ms processing delay is acceptable for enable/disable operations.
//
// SET_ENABLE_PIN_STATE_NEED_QUEUE
// SET_ENABLE_PIN_STATE(q, pin, high)
//   Set enable pin to HIGH or LOW state.
//   PRECONDITION: NOT called for CONTROLLED_EXTERNAL pins (PIN_EXTERNAL_FLAG)
//   Must be defined

#endif
