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

#if defined(SUPPORT_SELECT_DRIVER_TYPE)
static FasDriver selectAvailableDriverForPin(uint8_t step_pin,
                                             FasDriver preferred_driver);
#endif

static bool isValidStepPin(uint8_t step_pin);

AqeResultCode addQueueEntry(const struct stepper_command_s* cmd, bool start);
int32_t getCurrentPosition();
uint32_t ticksInQueue();
bool hasTicksInQueue(uint32_t min_ticks);
bool getActualTicksWithDirection(struct actual_ticks_s* speed);

bool init(FastAccelStepperEngine* engine, uint8_t queue_num, uint8_t step_pin);
void startQueue();
void forceStop();
void _initVars();
void connect();
void disconnect();

#endif
