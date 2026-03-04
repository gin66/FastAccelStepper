#ifndef FAS_QUEUE_QUEUE_H
#define FAS_QUEUE_QUEUE_H

#include "fas_arch/common.h"
#include "fas_queue/base.h"

class StepperQueue;

#if defined(SUPPORT_DYNAMIC_ALLOCATION)
extern StepperQueue* fas_queue[NUM_QUEUES];
#define FAS_QUEUE(idx) (*fas_queue[idx])
#define FAS_QUEUE_PTR(idx) (fas_queue[idx])
#else
extern StepperQueue fas_queue[NUM_QUEUES];
#define FAS_QUEUE(idx) (fas_queue[idx])
#define FAS_QUEUE_PTR(idx) (&fas_queue[idx])
#endif

#endif
