#ifndef FAS_QUEUE_QUEUE_H
#define FAS_QUEUE_QUEUE_H

#include "fas_arch/common.h"
#include "fas_queue/base.h"

class StepperQueue;

#if defined(SUPPORT_DYNAMIC_ALLOCATION)
extern StepperQueue* fas_queue[NUM_QUEUES];
#else
extern StepperQueue fas_queue[NUM_QUEUES];
#endif

#endif
