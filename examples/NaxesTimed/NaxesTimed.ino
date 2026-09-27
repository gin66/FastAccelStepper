// NaxesTimed — a two-axis FasTimed example (extras/doc/FasTimed.md).
//
// The head draws a diamond in the XY plane. Each of the four sides is one
// motor-speed ladder: it accelerates from rest through the pitch ladder in
// NaxesTimed_ramp.h, then decelerates back to rest. Both axes move together
// on every chunk (the diagonal keeps |dx| == |dy|), so the two motors share
// one clock and arrive at every corner at the same time.
//
// FasTimed does not invent the ramp: the caller asks for a rate and a
// duration per chunk. The ladder here is the one extras/tests/pc_based/test_29
// verifies; its RampMap does not depend on the platform's getMaxSpeedInTicks()
// because the periods stay well above it, so it is legal on AVR, Pico, SAM
// and ESP32.
//
// Acceleration must stay kNaxesTimedAccel (100000 steps/s^2). A different
// acceleration changes the ramp map and addDelta then returns
// TimingNotAchievable.
//
// Reversals at the corners need the axis at ramp-step 0, which the ladder
// provides by ending each side back at rest. A buffered driver that injects
// its own direction-change pause (ESP32 RMT/MCPWM/I2S) reports that as
// TimedStatus::Error; the timer-based AVR and Pico drivers do not.

// FastAccelStepper.h must come first: it defines PIN_UNDEFINED / FasDriver,
// which StepperConfig.h (pulled in by every pin table) uses.
#include "FastAccelStepper.h"

#if defined(ARDUINO_ARCH_AVR)
#include "StepperPins_NaxesTimed_avr.h"
#elif defined(ARDUINO_ARCH_SAMD)
#include "StepperPins_NaxesTimed_sam.h"
#elif defined(ARDUINO_ARCH_SAM)
#include "StepperPins_NaxesTimed_sam.h"
#elif defined(ARDUINO_ARCH_RP2040)
#include "StepperPins_NaxesTimed_pico.h"
#elif defined(ARDUINO_ARCH_RP2350)
#include "StepperPins_NaxesTimed_pico.h"
#elif defined(ARDUINO_ARCH_ESP32)
#include "StepperPins_NaxesTimed_esp32.h"
#else
#error "NaxesTimed: no pin table for this platform"
#endif

#include "StepperConfig.h"
#include "generic.h"
#include "FasTimed.h"
#include "NaxesTimed_ramp.h"

#ifdef SIMULATOR
#include <avr/sleep.h>
#endif

#define NAXES_HW 2
#define NAXES_TIMED_HORIZON 8

// FastAccelStepperEngine owns the steppers and the cyclic fill ISR.
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* NaxesTimed_s[NAXES_HW] = {NULL};

// The planner. Each chunk is already one queue command, so the ring is small.
FasTimed<NAXES_HW, NAXES_TIMED_HORIZON> NaxesTimed_planner(engine);

// Diamond sides: (+1,+1), (-1,+1), (-1,-1), (+1,-1). Both axes use the same
// step count, so they share the ladder's ramp-step exactly.
static const int8_t kSideSign[4][2] = {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

static uint8_t g_side = 0;
static uint8_t g_phase = 0;
static uint8_t g_stage = 0;
static uint8_t g_hold = 0;
static bool g_done = false;

// One chunk of this period: as many steps as fit in 65535 ticks. The step
// count is shared by both axes so the diagonal stays at 45 degrees.
static void timed_chunk(uint16_t period, int16_t* steps, uint16_t* ticks) {
  uint32_t n = 65535u / period;
  if (n > 128) {
    n = 128;
  }
  if (n < 1) {
    n = 1;
  }
  *steps = (int16_t)n;
  *ticks = (uint16_t)(n * period);
}

// Submit the next chunk if the ring has room. Returns false when addDelta
// back-pressured (ring full): pump() drains and the next call retries.
static bool timed_next(void) {
  uint16_t period;
  uint8_t repeats;
  uint8_t stages;
  if (g_phase == 0) {
    stages =
        (uint8_t)(sizeof(kNaxesTimedUpPeriod) / sizeof(kNaxesTimedUpPeriod[0]));
    period = kNaxesTimedUpPeriod[g_stage];
    repeats = kNaxesTimedUpHold;
  } else {
    stages = (uint8_t)(sizeof(kNaxesTimedDownPeriod) /
                       sizeof(kNaxesTimedDownPeriod[0]));
    period = kNaxesTimedDownPeriod[g_stage];
    repeats = kNaxesTimedDownHold;
  }
  int16_t nsteps = 0;
  uint16_t ticks = 0;
  timed_chunk(period, &nsteps, &ticks);
  int16_t d[NAXES_HW];
  d[0] = (int16_t)(kSideSign[g_side][0] * nsteps);
  d[1] = (int16_t)(kSideSign[g_side][1] * nsteps);
  if (NaxesTimed_planner.addDelta(d, ticks) != TimedAdd::Ok) {
    return false;
  }
  g_hold++;
  if (g_hold >= repeats) {
    g_hold = 0;
    g_stage++;
    if (g_stage >= stages) {
      g_stage = 0;
      g_phase++;
      if (g_phase > 1) {
        g_phase = 0;
        g_side++;
        if (g_side >= 4) {
          g_side = 0;
          g_done = true;
        }
      }
    }
  }
  return true;
}

void setup() {
  engine.init();

  const struct stepper_config_s* cfg = NaxesTimed_config_0;
  for (uint8_t i = 0; i < NAXES_HW; i++) {
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
    NaxesTimed_s[i] =
        engine.stepperConnectToPin(cfg[i].step, cfg[i].driver_type);
#else
    NaxesTimed_s[i] = engine.stepperConnectToPin(cfg[i].step);
#endif
    if (NaxesTimed_s[i] == NULL) {
      // No Serial here: it would drag Print/println into the flash image.
      while (1) {
      }
    }
    NaxesTimed_s[i]->setDirectionPin(cfg[i].direction,
                                     cfg[i].direction_high_count_up);
    if (cfg[i].enable_low_active != PIN_UNDEFINED) {
      NaxesTimed_s[i]->setEnablePin(cfg[i].enable_low_active);
    }
    NaxesTimed_s[i]->setAutoEnable(cfg[i].auto_enable);
    // The ladder was chosen for this acceleration at 16 MHz ticks. FasTimed
    // ignores the configured speed limit; only the acceleration feeds the
    // ramp map.
    NaxesTimed_s[i]->setAcceleration(kNaxesTimedAccel);
  }

  // Enable and settle the outputs before the planner kicks off (whitepaper
  // section 4.5), so addQueueEntry() is not bounced off the enable wait.
  if (cfg[0].on_delay_us != 0) {
    DELAY_US(cfg[0].on_delay_us);
  }
  for (uint8_t i = 0; i < NAXES_HW; i++) {
    NaxesTimed_s[i]->enableOutputs();
  }
  DELAY_US(100);
  NaxesTimed_planner.addAxis(0, NaxesTimed_s[0]);
  NaxesTimed_planner.addAxis(1, NaxesTimed_s[1]);

  int32_t origin[NAXES_HW] = {0, 0};
  NaxesTimed_planner.setCurrentPosition(origin);
}

void loop() {
  if (!g_done) {
    timed_next();
  }

  NaxesTimed_planner.pump();

  if (g_done && !NaxesTimed_planner.isBusy()) {
#ifdef SIMULATOR
    noInterrupts();
    sleep_cpu();
#endif
  }
}
