#include "StepperISR.h"

#if defined(ARDUINO_ARCH_SAM)
const bool enabled = false;
uint32_t junk = 0;

inline void disconnectPWMPeriphal(Pio* port, uint8_t pin, uint32_t channelMask);

typedef struct _PWMCHANNELMAPPING {
  uint8_t pin;
  uint32_t channel;
  Pio* port;
  uint32_t channelMask;
} PWMCHANNELMAPPING;

#define NUM_PWM_CHANNELS 8
uint8_t TimerChannel_Map[NUM_PWM_CHANNELS];

uint8_t numChannels = 0;

static FastAccelStepperEngine* fas_engine = NULL;

#if defined(KEEP_SCORE)
volatile uint32_t totalPulsesDetected[NUM_QUEUES];
volatile uint32_t totalSteps[NUM_QUEUES];
inline void IncrementQueue(int queue) { totalPulsesDetected[queue]++; }
inline void AddToTotalSteps(int queue, uint32_t steps) {
  totalSteps[queue] += steps;
}
#else
#define IncrementQueue(Q)
#define AddToTotalSteps(Q, N)
#endif
bool channelsUsed[8] = {false, false, false, false, false, false, false, false};
StepperQueue fas_queue[NUM_QUEUES];
PWMCHANNELMAPPING gChannelMap[NUM_QUEUES];

void TC5_Handler() {
  uint32_t SR0 = TC1->TC_CHANNEL[2].TC_SR;
  if (SR0 & TC_SR_CPCS) {
    fas_engine->manageSteppers();
    TC1->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;
  }
}

void strobePin(uint32_t pin, uint32_t desiredChannel = 0,
               uint32_t channel = 0) {
  if (channel == desiredChannel) {
    digitalWrite(pin, HIGH);
    digitalWrite(pin, LOW);
  }
}

void PWM_Handler(void) {
  uint32_t sr = PWM_INTERFACE->PWM_ISR1;
  uint8_t leading_zeros;
  sr = sr & 0xFF;
  while ((leading_zeros = __CLZ(sr)) < 32) {
    uint8_t channel = 32 - leading_zeros - 1;
    uint32_t mask = (1 << channel);
    sr = sr & (~mask);
    uint8_t queue_num = TimerChannel_Map[channel];
    if (channelsUsed[channel] == false) continue;
    StepperQueue* q = &fas_queue[queue_num];
    uint32_t t = micros();
    uint32_t timeElapsed = micros() - q->timePWMInterruptEnabled;
    if (t < q->timePWMInterruptEnabled) {
      timeElapsed = 0xFFFFFFFF - q->timePWMInterruptEnabled;
      timeElapsed += t;
    }
    if (timeElapsed < 100) {
      continue;
    }
    if (!q->_pauseCommanded) {
      PWM_INTERFACE->PWM_IDR1 = mask;
      continue;
    }
    PWMCHANNELMAPPING* mapping = &gChannelMap[queue_num];
    q->driver_data = (void*)mapping;

    Pio* port = mapping->port;

    uint8_t rp = q->read_idx;
    if (rp == q->next_write_idx) {
      continue;
    }
    rp = ++q->read_idx;
    struct queue_entry* e = &q->entry[rp & QUEUE_LEN_MASK];
    if (rp == q->next_write_idx) {
      q->_hasISRactive = false;
      q->_pauseCommanded = false;
      PWM_INTERFACE->PWM_IDR1 = mask;
      PWM_INTERFACE->PWM_DIS = mask;
      disconnectPWMPeriphal(port, mapping->pin, mapping->channelMask);
      PWM_INTERFACE->PWM_IDR1 = mask;
      q->read_idx = rp;
      pinMode(mapping->pin, OUTPUT);
      PIO_SetOutput(mapping->port, g_APinDescription[mapping->pin].ulPin, 0, 0,
                    0);

      continue;
    }
    if (e->steps > 0) {
      delayMicroseconds(10);
      PWM_INTERFACE->PWM_CH_NUM[channel].PWM_CPRDUPD = e->ticks;
      PWM_INTERFACE->PWM_IDR1 = mask;
      uint32_t dwSR = port->PIO_ABSR;
      port->PIO_ABSR = (g_APinDescription[mapping->pin].ulPin | dwSR);
      port->PIO_ODR |= g_APinDescription[mapping->pin].ulPin;
      port->PIO_PDR = g_APinDescription[mapping->pin].ulPin;
      port->PIO_IER = g_APinDescription[mapping->pin].ulPin;
      q->_pauseCommanded = false;
    } else {
      PWM_INTERFACE->PWM_CH_NUM[channel].PWM_CPRDUPD = e->ticks;
    }
    q->read_idx = rp;
  }
}

inline void disconnectPWMPeriphal(Pio* port, uint8_t pin,
                                  uint32_t channelMask) {
  PWM_INTERFACE->PWM_DIS = channelMask;
  PIO_SetOutput(port, g_APinDescription[pin].ulPin, 0, 0, 0);
}

inline void attachPWMPeripheral(Pio* port, uint8_t pin, uint32_t channelMask) {
  PWM_INTERFACE->PWM_IDR1 = (channelMask);
  uint32_t dwSR = port->PIO_ABSR;
  port->PIO_ABSR = (g_APinDescription[pin].ulPin | dwSR);
  port->PIO_ODR |= g_APinDescription[pin].ulPin;
  port->PIO_PDR |= g_APinDescription[pin].ulPin;
  port->PIO_IER |= g_APinDescription[pin].ulPin;
  PWM_INTERFACE->PWM_ENA |= channelMask;
}

#define ISRQueueName(Q) Queue##Q##ISR
#define ISRQueueNameStr(Q) "Queue" #Q "ISR"

#define DUE_STEPPER_ISR(Q)                                          \
  void Queue##Q##ISR() {                                            \
    static StepperQueue* const q = &fas_queue[Q];                   \
    const PWMCHANNELMAPPING* mapping =                              \
        (const PWMCHANNELMAPPING*)fas_queue[Q].driver_data;         \
    static uint32_t channel = mapping->channel;                     \
    static uint32_t samPin = g_APinDescription[mapping->pin].ulPin; \
    static Pio* port = mapping->port;                               \
    if (!q->_hasISRactive || q->_pauseCommanded) {                  \
      return;                                                       \
    }                                                               \
    IncrementQueue(Q);                                              \
    uint8_t rp = q->read_idx;                                       \
    if (rp == q->next_write_idx) {                                  \
      return;                                                       \
    }                                                               \
    struct queue_entry* e = &q->entry[rp & QUEUE_LEN_MASK];         \
    uint8_t s = --e->steps;                                         \
    if (s == 0) {                                                   \
      if (e->toggle_dir) {                                          \
        e->toggle_dir = 0;                                          \
        *q->_dirPinPort ^= q->_dirPinMask;                          \
      }                                                             \
      rp++;                                                         \
      if (rp == q->next_write_idx) {                                \
        delayMicroseconds(10);                                      \
        disconnectPWMPeriphal(mapping->port, mapping->pin,          \
                              mapping->channelMask);                \
        q->_hasISRactive = false;                                   \
        q->_connected = false;                                      \
        q->read_idx = rp;                                           \
        return;                                                     \
      }                                                             \
      e = &q->entry[rp & QUEUE_LEN_MASK];                           \
      q->read_idx = rp;                                             \
      if (e->steps == 0 || !e->hasSteps) {                          \
        delayMicroseconds(7);                                       \
        q->_pauseCommanded = true;                                  \
        q->timePWMInterruptEnabled = micros();                      \
        PWM_INTERFACE->PWM_IER1 = mapping->channelMask;             \
        port->PIO_CODR = samPin;                                    \
        port->PIO_OER = samPin;                                     \
        port->PIO_PER = samPin;                                     \
        port->PIO_IDR = samPin;                                     \
        PWM_INTERFACE->PWM_CH_NUM[channel].PWM_CPRDUPD = e->ticks;  \
        return;                                                     \
      }                                                             \
      AddToTotalSteps(Q, e->steps);                                 \
      PWM_INTERFACE->PWM_CH_NUM[channel].PWM_CPRDUPD = e->ticks;    \
      if (e->toggle_dir) {                                          \
        e->toggle_dir = 0;                                          \
        *q->_dirPinPort ^= q->_dirPinMask;                          \
        delayMicroseconds(30);                                      \
      }                                                             \
      return;                                                       \
    }                                                               \
  }

DUE_STEPPER_ISR(0)
DUE_STEPPER_ISR(1)
DUE_STEPPER_ISR(2)
DUE_STEPPER_ISR(3)
DUE_STEPPER_ISR(4)
DUE_STEPPER_ISR(5)

inline uint32_t pinToChannel(uint32_t pin) {
  switch (pin) {
    case 34:
    case 67:
    case 73:
    case 35:
      return 0;
    case 17:
    case 36:
    case 72:
    case 37:
    case 42:
      return 1;
    case 38:
    case 43:
    case 63:
    case 39:
      return 2;
    case 40:
    case 64:
    case 69:
    case 41:
      return 3;
    case 9:
      return 4;
    case 8:
    case 44:
      return 5;
    case 7:
    case 45:
      return 6;
    case 6:
      return 7;
  }
  return 0xFFFFFFFF;
}

bool StepperQueue::init(FastAccelStepperEngine* engine, uint8_t queue_num,
                        uint8_t step_pin) {
  _queue_num = queue_num;
  driver_data = (void*)&gChannelMap[queue_num];
  _initVars();
  _step_pin = step_pin;
  channelsUsed[pinToChannel(step_pin)] = true;
  numChannels++;
  max_speed_in_ticks = 420;
#if defined(KEEP_SCORE)
  totalPulsesDetected[queue_num] = 0;
  totalSteps[queue_num] = 0;
#endif
  gChannelMap[queue_num].pin = step_pin;
  gChannelMap[queue_num].channel = pinToChannel(step_pin);
  TimerChannel_Map[gChannelMap[queue_num].channel] = queue_num;
  gChannelMap[queue_num].port = g_APinDescription[step_pin].pPort;
  gChannelMap[queue_num].channelMask = 1 << gChannelMap[queue_num].channel;
  static bool isPWMEnabled = false;
  if (!isPWMEnabled) {
    pmc_enable_periph_clk(PWM_INTERFACE_ID);
    isPWMEnabled = true;
  }
  static bool isTCEnabled = false;
  if (!isTCEnabled) {
    pmc_enable_periph_clk(ID_TC5);
    TC1->TC_CHANNEL[2].TC_CCR = 1;
    TC1->TC_CHANNEL[2].TC_CMR = 0;
    TC1->TC_CHANNEL[2].TC_CMR = 0x80 | 0x40;
    TC1->TC_CHANNEL[2].TC_RA = 84000;
    TC1->TC_CHANNEL[2].TC_RB = 84000;
    TC1->TC_CHANNEL[2].TC_RC = 84000;
    TC1->TC_CHANNEL[2].TC_IER = 0;
    TC1->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;
    TC1->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;
    NVIC_SetPriority(TC5_IRQn, 1);
    NVIC_EnableIRQ(TC5_IRQn);
  }

  digitalWrite(step_pin, LOW);
  pinMode(step_pin, OUTPUT);
  NVIC_SetPriority(PIOA_IRQn, 0);
  NVIC_SetPriority(PIOB_IRQn, 0);
  NVIC_SetPriority(PIOC_IRQn, 0);
  NVIC_SetPriority(PIOD_IRQn, 0);
  PWM_INTERFACE->PWM_IDR2 = 0x00FFFF0F;
  PWM_INTERFACE->PWM_IDR1 = 0x00FFFF0F;
  NVIC_EnableIRQ(PWM_IRQn);
  PWM_INTERFACE->PWM_IDR2 = 0x00FFFF0F;
  PWM_INTERFACE->PWM_IDR1 = 0x00FFFF0F;

  connect();
  return true;
}

void StepperQueue::connect() {
  PIO_Configure(g_APinDescription[_step_pin].pPort,
                g_APinDescription[_step_pin].ulPinType,
                g_APinDescription[_step_pin].ulPin,
                g_APinDescription[_step_pin].ulPinConfiguration);
  const PWMCHANNELMAPPING* mapping = (const PWMCHANNELMAPPING*)driver_data;
  PWMC_ConfigureChannel(PWM_INTERFACE, mapping->channel, PWM_CMR_CPRE_MCK_DIV_4,
                        0, 0);

  PWM_INTERFACE->PWM_IDR2 = 0x00FFFF0F;
  PWM_INTERFACE->PWM_IDR1 = 0x00FFFF0F;
  PWM_INTERFACE->PWM_DIS = PWM_INTERFACE->PWM_DIS & (~mapping->channelMask);
  PWMC_SetPeriod(PWM_INTERFACE, mapping->channel, 65535);
  PWM_INTERFACE->PWM_DIS = PWM_INTERFACE->PWM_DIS & (~mapping->channelMask);
  PWM_INTERFACE->PWM_CH_NUM[mapping->channel].PWM_CDTY = 21 * 10;
  PWM_INTERFACE->PWM_DIS = PWM_INTERFACE->PWM_DIS & (~mapping->channelMask);
  PWM_INTERFACE->PWM_IDR2 = 0x00FFFF0F;
  PWM_INTERFACE->PWM_IDR1 = 0x00FFFF0F;

  switch (_queue_num) {
    case 0:
      attachInterrupt(digitalPinToInterrupt(_step_pin), ISRQueueName(0),
                      RISING);
      break;
    case 1:
      attachInterrupt(digitalPinToInterrupt(_step_pin), ISRQueueName(1),
                      RISING);
      break;
    case 2:
      attachInterrupt(digitalPinToInterrupt(_step_pin), ISRQueueName(2),
                      RISING);
      break;
    case 3:
      attachInterrupt(digitalPinToInterrupt(_step_pin), ISRQueueName(3),
                      RISING);
      break;
    case 4:
      attachInterrupt(digitalPinToInterrupt(_step_pin), ISRQueueName(4),
                      RISING);
      break;
    case 5:
      attachInterrupt(digitalPinToInterrupt(_step_pin), ISRQueueName(5),
                      RISING);
      break;
  }
  _connected = true;
  PWM_INTERFACE->PWM_IDR2 = 0x00FFFF0F;
  PWM_INTERFACE->PWM_IDR1 = 0x00FFFF0F;
}

void StepperQueue::disconnect() {
  const PWMCHANNELMAPPING* mapping = (const PWMCHANNELMAPPING*)driver_data;
  PWMC_DisableChannel(PWM_INTERFACE, mapping->channel);
  PWM_INTERFACE->PWM_DIS = PWM_INTERFACE->PWM_DIS & (~mapping->channelMask);
  _connected = false;
}

void StepperQueue::startQueue() {
  noInterrupts();
  struct queue_entry* e = &entry[read_idx & QUEUE_LEN_MASK];
  interrupts();
  _hasISRactive = true;

  const PWMCHANNELMAPPING* mapping = (const PWMCHANNELMAPPING*)driver_data;

  if (PWM_INTERFACE->PWM_SR & (1 << mapping->channel)) {
    PWM_INTERFACE->PWM_CH_NUM[mapping->channel].PWM_CPRDUPD = e->ticks;
  } else {
    PWM_INTERFACE->PWM_CH_NUM[mapping->channel].PWM_CPRD = e->ticks;
  }

  if (e->steps > 0 || e->hasSteps) {
    attachPWMPeripheral(mapping->port, mapping->pin, mapping->channelMask);
    return;
  } else {
    PIO_SetOutput(mapping->port, g_APinDescription[mapping->pin].ulPin, 0, 0,
                  0);
    mapping->port->PIO_IDR = g_APinDescription[mapping->pin].ulPin;
    PWM_INTERFACE->PWM_ENA |= mapping->channelMask;
    _pauseCommanded = true;
    PWM_INTERFACE->PWM_IER1 = PWM_INTERFACE->PWM_IMR1 | mapping->channelMask;
    return;
  }
}

void StepperQueue::forceStop() {
  noInterrupts();
  read_idx = next_write_idx;
  interrupts();
  const PWMCHANNELMAPPING* mapping = (const PWMCHANNELMAPPING*)driver_data;
  PWMC_DisableChannel(PWM_INTERFACE, mapping->channel);
  _hasISRactive = false;
}

bool StepperQueue::isValidStepPin(uint8_t step_pin) {
  if (pinToChannel(step_pin) < 0x08) {
    if (!channelsUsed[pinToChannel(step_pin)]) {
      return true;
    }
  }
  return false;
}

void fas_init_engine(FastAccelStepperEngine* engine) { fas_engine = engine; }
#endif
