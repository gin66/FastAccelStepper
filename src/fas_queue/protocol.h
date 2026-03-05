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
static FasDriver getAvailableDriverForPin(uint8_t step_pin,
                                          FasDriver preferred_driver);
#endif

static bool isValidStepPin(uint8_t step_pin);

#endif
