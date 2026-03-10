#ifndef FAS_QUEUE_STEPPER_QUEUE_H
#define FAS_QUEUE_STEPPER_QUEUE_H

#include <stdint.h>

#include "FastAccelStepper.h"
#include "fas_arch/common.h"
#include "fas_queue/queue.h"

// The pd_ prefix in directory names stands for "pulse driver" (though "platform
// driver" is also suitable).
#if defined(SUPPORT_AVR)
#include "pd_avr/avr_queue.h"
#elif defined(TEST)
#include "pd_test/test_queue.h"
#elif defined(SUPPORT_SAM)
#include "pd_sam/sam_queue.h"
#elif defined(SUPPORT_RP_PICO)
#include "pd_pico/pico_queue.h"
#elif defined(SUPPORT_ESP32)
#include "pd_esp32/esp32_queue.h"
#else
#error "Unsupported architecture"
#endif

#if defined(SUPPORT_CPU_AFFINITY)
void fas_init_engine(FastAccelStepperEngine* engine, uint8_t cpu_core);
#else
void fas_init_engine(FastAccelStepperEngine* engine);
#endif

#endif  // FAS_QUEUE_STEPPER_QUEUE_H
