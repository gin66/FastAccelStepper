#include <stdint.h>

#include "FastAccelStepper.h"
#include "fas_arch/common.h"
#include "fas_queue/base.h"

// Here are the global variables to interface with the interrupts

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

extern StepperQueue fas_queue[NUM_QUEUES];

#if defined(SUPPORT_CPU_AFFINITY)
void fas_init_engine(FastAccelStepperEngine* engine, uint8_t cpu_core);
#else
void fas_init_engine(FastAccelStepperEngine* engine);
#endif
