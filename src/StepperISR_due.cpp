#include "StepperISR.h"
#if defined  (ARDUINO_ARCH_SAM)
const bool enabled = false;

/*
I went through a ton of iterations of trying to make this work sd desired.
It seems quite simple, set the pulse period of the PWM generator, start it,
stop it after e->ticks number of ticks.  Except for that its never easy.

We start with how to detect that e->ticks has happened.  The PWM peripheral
only has interrupts for synchronous pulses.  Thats obnoxious.  So thats out.
Instead I found that regaurdless of output mode, the PIO device is always
connected and receiving inputs!  This means I can use it to trigger on a pulse
being sent!  But this means I will get an interrupt for every single pulse.
Thats not ideal, and I wasn't sure this would be able to operate fast enough in
this mode.  I actually did what I could to get into the PWM shutdown as fast as
possible.  However, it seems like it would still send one more pulse every
time.  It wouldn't register a full rise on my scope, and it probably....
wouldn't have triggered a pulse on a stepper driver...but probably isn't good
enough.  No matter what I did, I wasn't fast enough.  So I found that using
the PIO to disconnect the output of the pulse generator was nearly
instantaneous.  So fast that to keep the interrupt on a rising edge, I need to
delay a few microseconds to keep from chopping off the pulse!  In fact, my
driver says it needs 5 microsecond pulses.  I'm currently sending 10
microsencond pulses.  With a 5 microsecond delay before shutting off, the pulse
is only 7 microseconds in length!  The performance of the code seems quite
good!  SO not only is the overhead of the interrupts low enough to chop pulses
off, its good enough to easily manage at least one stepper.  (I need to make
code more generic and make ISRs for the other ports and test with more).

The last "trick" that was necessary was a delay in starting the pulse generator
after setting the direction pin.  Currently my test just spins the motor 1
revolution then spins it the other way.  So every stop needs a delay.  Honestly
a 30 microsecond delay in starting all the time is more consistant than just
when the direction changes, so I may leave it as it is.  The delay is 100%
necessary for setting the direction pin, so it cannot simply be removed!

With that, I've spun this motor to about 1600 RPM.  No matter how slow the
acceleration, it seems to stop somewhere just above that.  So the code can do
more than my nema23 test motor :)

*/
typedef struct _INTERRUPTMAP
{
  Pio* pio;
  void (*ISR)(void);
  uint32_t pin;
} INTERRUPTMAP;

INTERRUPTMAP InterruptMap[6];
uint8_t numChannels = 0;

// Here are the global variables to interface with the interrupts
volatile uint32_t totalPulsesDetected[NUM_QUEUES];
volatile uint32_t totalSteps[NUM_QUEUES];
bool channelsUsed[NUM_QUEUES] = { false, false, false, false, false, false };
StepperQueue fas_queue[NUM_QUEUES];
PWMCHANNELMAPPING gChannelMap[NUM_QUEUES];

static void apply_command(StepperQueue* queue, const struct queue_entry* e) {
  const PWMCHANNELMAPPING* mapping = queue->mapping;
  //That should be it...We should start the next command on the next cycle
  PWM_INTERFACE->PWM_CH_NUM[mapping->channel].PWM_CPRDUPD = e->ticks;
}

inline void disconnectPWMPeriphal(Pio* port,
  uint8_t pin, uint32_t channelMask, void (*ISR)(void)) {
  //We actually got pretty good at shutting down the pulse generator VERY 
  //VERY VERY quickly.  Enough so that we need a delay (or to switch to a 
  //falling edge interrupt) to get the whole pulse out!
  delayMicroseconds(5);

  PWM_INTERFACE->PWM_DIS = PWM_INTERFACE->PWM_DIS & (~channelMask);


  PIO_SetOutput(port, g_APinDescription[pin].ulPin, 0, 0, 0);
}

inline void attachPWMPeripheral(Pio* port,
                        uint8_t pin, uint32_t channelMask, void (*ISR)(void)) {
  port->PIO_IDR = g_APinDescription[pin].ulPin;
  uint32_t dwSR = port->PIO_ABSR;
  port->PIO_ABSR = (g_APinDescription[pin].ulPin | dwSR);
  // Remove the pins from under the control of PIO
  port->PIO_PDR = g_APinDescription[pin].ulPin;
  port->PIO_IER |= g_APinDescription[pin].ulPin;
  PWM_INTERFACE->PWM_ENA |= channelMask;
}

#define ISRQueueName(Q) Queue##Q##ISR

#define DUE_STEPPER_ISR(Q)                                                     \
  void Queue##Q##ISR() {                                                       \
    static StepperQueue* const q = &fas_queue[Q];                              \
    totalPulsesDetected[q->_queue_num]++;                                       \
    uint8_t rp = q->read_idx;                                                  \
    uint8_t wp = q->next_write_idx;                                            \
    struct queue_entry* e = &q->entry[rp & QUEUE_LEN_MASK];                    \
    uint8_t s = --e->steps;                                                    \
    /*Obvious case, if we have done all of the steps in this queue entry....*/ \
    if (s == 0)  {                                                             \ 
/*Setup for the next move...The PWM Peripheral is pretty smart.  We set        \
  the period "update" register, and it will take effect at the end of          \
  the current period.  Since we arent changing the clock, there is no          \
  need to change the duty cycle, just the period...                            \
  Check if we need to toggle direction....If so, we will also need to          \
  delay...Does this ever happen?  I think the dirPinBusy function              \
  prevents this...*/                                                           \
  if (e->toggle_dir) {                                                         \
      e->toggle_dir = 0;                                                       \
      * q->_dirPinPort ^= q->_dirPinMask;                                      \
      delayMicroseconds(30);                                                   \
  }                                                                            \
    /*Check immediatly if we need to turn off the pulse generator.Time is      \
      critical! */                                                             \
      if (++rp == wp) {                                                        \
        disconnectPWMPeriphal(                                                 \
          q->mapping->port,                                                    \
          q->mapping->pin,                                                     \
          q->mapping->channelMask,                                             \
          Queue##Q##ISR);                                                      \
          q->_hasISRactive = false;                                            \
          q->_connected = false;                                               \
        /*Update the queue pointer...setup for the next move.*/                \
        q->read_idx = rp;                                                      \
        e = &q->entry[rp & QUEUE_LEN_MASK];                                    \
        /*Would be nice if we were told to set the dir pin here if we already  \
          knew.  Maybe it does, so we'll handle the case...*/                  \
        if (e->toggle_dir) {                                                   \
          e->toggle_dir = 0;                                                   \
          *q->_dirPinPort ^= q->_dirPinMask;                                   \
          delayMicroseconds(30);                                               \
        }                                                                      \
        return;                                                                \
      }                                                                        \
      /*Since we're done with e, e is now e_next...*/                          \
      e = &q->entry[rp & QUEUE_LEN_MASK];                                      \
      q->read_idx = rp;                                                        \
      /*That should be it...We should start the next command on the next       \
        cycle */                                                               \
      PWM_INTERFACE->PWM_CH_NUM[q->mapping->channel].PWM_CPRDUPD = e->ticks;   \
      if (e->toggle_dir) {                                                     \
         e->toggle_dir = 0;                                                    \
         *q->_dirPinPort ^= q->_dirPinMask;                                    \
         delayMicroseconds(30);                                                \
       }                                                                       \
       return;                                                                 \
    }                                                                          \
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
    return 0;
  case 17:
  case 36:
  case 72:
    return 1;
  case 38:
  case 43:
  case 63:
    return 2;
  case 40:
  case 64:
  case 69:
    return 3;
  case 9:
    return 4;
  case 8:
    return 5;
  case 7:
    return 6;
  case 6:
    return 7;
  }
  return 0xFFFFFFFF;
}

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
  _queue_num = queue_num;
  _initVars();
  _step_pin = step_pin;
  InterruptMap[queue_num].pin = step_pin;
  totalPulsesDetected[queue_num] = 0;
  totalSteps[queue_num] = 0;
  gChannelMap[queue_num].pin = step_pin;
  gChannelMap[queue_num].channel = pinToChannel(step_pin);
  gChannelMap[queue_num].port = g_APinDescription[step_pin].pPort;
  gChannelMap[queue_num].channelMask = 1 << gChannelMap[queue_num].channel;
  mapping = &gChannelMap[queue_num];

  static bool isPWMEnabled = false;
  if (!isPWMEnabled) {
    pmc_enable_periph_clk(PWM_INTERFACE_ID);
    //ESP32 operates at 2x the speed as the due, which is reasonable to say the
    // esp32 should be 2x as capable.  In the code for the esp, the comment 
    //says the clock is 160/5, but the prescaler is set to 5-1? so its at 40MHz.  
    //We'll pick a slower clock rate for for this 255 for the prescaler, and a 
    //mod clock of 4 for 84,000,000/4=21,000,000/255 is 82352.9....52 might be 
    //the better number...This puts us at roughly 1/2 the performance of the 
    //esp32...
    //remain compatible with due - We'll use MCK or clk B as necessary.
    PWMC_ConfigureClocks(
      PWM_FREQUENCY * PWM_MAX_DUTY_CYCLE, VARIANT_MCK / 4, VARIANT_MCK); 
    //PWM->PWM_CLK = (PWM->PWM_CLK & 0xFFFF0000) | 0x0AFF;
    isPWMEnabled = true;
  }
  PIO_Configure(g_APinDescription[step_pin].pPort, 
                g_APinDescription[step_pin].ulPinType, 
                g_APinDescription[step_pin].ulPin, 
                g_APinDescription[step_pin].ulPinConfiguration);
  PWMC_ConfigureChannel(
    PWM_INTERFACE, gChannelMap[queue_num].channel, PWM_CMR_CPRE_CLKB, 0, 0);

  //Rising edge, so we get the interrupt at the beginning of the pulse (gives 
  //us some extra time) and so we only get 1 int/pulse Change would give 2
  //fixme - use defines so I only have to write the ISR once.
  attachInterrupt(digitalPinToInterrupt(step_pin), ISRQueueName(0), RISING);

  digitalWrite(step_pin, LOW);
  pinMode(step_pin, OUTPUT);

  connect();
}

void StepperQueue::connect() {
  PIO_Configure(g_APinDescription[_step_pin].pPort, 
                g_APinDescription[_step_pin].ulPinType, 
                g_APinDescription[_step_pin].ulPin, 
                g_APinDescription[_step_pin].ulPinConfiguration);
  PWMC_ConfigureChannel(
    PWM_INTERFACE, mapping->channel, PWM_CMR_CPRE_CLKB, 0, 0);
  //21 pulses makes for a microsecond pulse.  I believe my drivers need 5us.  
  //I'll set this to 10 just to make sure the drivers receive this.  At 20us
  //pulse speed, this is still a 1/2 duty cycle PWM.  Should be easy for any
  //driver to pickup.
  PWM_INTERFACE->PWM_DIS = 0x80;
  PWMC_SetPeriod(PWM_INTERFACE, mapping->channel, 65535);
  PWM_INTERFACE->PWM_DIS = 0x80;
  PWMC_SetDutyCycle(PWM_INTERFACE, mapping->channel, 21 * 10);
  PWM_INTERFACE->PWM_DIS = 0x80;

  //Rising edge, so we get the interrupt at the beginning of the pulse (gives 
  //us some extra time) and so we only get 1 int/pulse Change would give 2.
  //clear any pending...
  uint32_t isr = g_APinDescription[_step_pin].pPort->PIO_ISR; 
  switch (_queue_num) {
    case 0:
      attachInterrupt(digitalPinToInterrupt(_step_pin), ISRQueueName(0), RISING);
      break;
    case 1:
      attachInterrupt(digitalPinToInterrupt(_step_pin), ISRQueueName(1), RISING);
      break;
    case 2:
      attachInterrupt(digitalPinToInterrupt(_step_pin), ISRQueueName(2), RISING);
      break;
    case 3:
      attachInterrupt(digitalPinToInterrupt(_step_pin), ISRQueueName(3), RISING);
      break;
    case 4:
      attachInterrupt(digitalPinToInterrupt(_step_pin), ISRQueueName(4), RISING);
      break;
    case 5:
      attachInterrupt(digitalPinToInterrupt(_step_pin), ISRQueueName(5), RISING);
      break;
  }
  _connected = true;
}

void StepperQueue::disconnect() {
  PWMC_DisableChannel(PWM_INTERFACE, mapping->channel);
  PWM_INTERFACE->PWM_DIS = 0x80;
  _connected = false;
}

void StepperQueue::commandAddedToQueue(bool start) {
  // If it is not the first command in the queue, then just return
  // Otherwise just prepare, what is possible for start (set direction pin)

  // apply_command() assumes the pcnt counter to contain executed steps
  // and deduct this from the new command. For a starting motor
  // need to make sure, that the counter is 0. (issue #33)
  noInterrupts();
  struct queue_entry* e = &entry[read_idx & QUEUE_LEN_MASK];
  interrupts();
  if (e->toggle_dir) {
    e->toggle_dir = 0;
    *_dirPinPort ^= _dirPinMask;
    delayMicroseconds(30);
  }

  apply_command(this, e);
  if (start) {
    bool first = (next_write_idx++ == read_idx);
    if (first) {
      if (PWM_INTERFACE->PWM_SR & (1 << mapping->channel)) {
        //That should be it...We should start the next command on the next cycle
        PWM_INTERFACE->PWM_CH_NUM[mapping->channel].PWM_CPRDUPD = e->ticks; 
        return;
      }
      if (e->toggle_dir) {
        e->toggle_dir = 0;
        *_dirPinPort ^= _dirPinMask;
        delay(50);
      }
    } else {
      if (e->toggle_dir) {
        e->toggle_dir = 0;
        *_dirPinPort ^= _dirPinMask;
        delayMicroseconds(30);
      } else {
        switch (_queue_num) {
        case 0:
          InterruptMap[_queue_num].ISR = Queue0ISR;
          attachPWMPeripheral(mapping->port, mapping->pin, mapping->channelMask,
                              Queue0ISR);
          break;
        case 1:
          InterruptMap[_queue_num].ISR = Queue1ISR;
          attachPWMPeripheral(mapping->port, mapping->pin, mapping->channelMask,
                              Queue1ISR);
          break;
        case 2:
          InterruptMap[_queue_num].ISR = Queue1ISR;
          attachPWMPeripheral(mapping->port, mapping->pin, mapping->channelMask,
                              Queue2ISR);
          break;
        case 3:
          InterruptMap[_queue_num].ISR = Queue3ISR;
          attachPWMPeripheral(mapping->port, mapping->pin, mapping->channelMask,
                              Queue3ISR);
          break;
        case 4:
          InterruptMap[_queue_num].ISR = Queue4ISR;
          attachPWMPeripheral(mapping->port, mapping->pin, mapping->channelMask,
                              Queue4ISR);
          break;
        case 5:
          InterruptMap[_queue_num].ISR = Queue5ISR;
          attachPWMPeripheral(mapping->port, mapping->pin, mapping->channelMask,
                              Queue5ISR);
          break;
        }
        _hasISRactive = true;
        _connected = true;
      }
    }
  }
}

int8_t StepperQueue::startPreparedQueue() {
  if (next_write_idx == read_idx) {
    return AQE_ERROR_EMPTY_QUEUE_TO_START;
  }
  if (PWM_INTERFACE->PWM_SR & (1 << mapping->channel)) {
    noInterrupts();
    struct queue_entry* e = &entry[read_idx & QUEUE_LEN_MASK];
    interrupts();
    //That should be it...We should start the next command on the next cycle
    PWM_INTERFACE->PWM_CH_NUM[mapping->channel].PWM_CPRDUPD = e->ticks; 
    _hasISRactive = true;
    return AQE_OK;
  }
  return AQE_OK;
}

void StepperQueue::forceStop() {
  read_idx = next_write_idx;
  PWMC_DisableChannel(PWM_INTERFACE, mapping->channel);
}

bool StepperQueue::isValidStepPin(uint8_t step_pin) {
  if (pinToChannel(step_pin) < 0x08) {
    if (!channelsUsed[pinToChannel(step_pin)]) {
      channelsUsed[pinToChannel(step_pin)] = true;
      numChannels++;
      return true;
    }
  }
  return false;
}

bool StepperQueue::isRunning() {
  if (_hasISRactive) {
    return true;
  }
}

int8_t StepperQueue::queueNumForStepPin(uint8_t step_pin) { return -1; }
#endif