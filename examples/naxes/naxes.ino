// naxes — a three-axis FasNAxis example (whitepaper
// extras/doc/n_axes_whitepaper.md). One FastAccelStepper per axis X/Y/Z, no
// gantry, driven in time-synchronized lockstep by a single FasNAxis planner.
//
// The path is fixed and deterministic so a run is comparable to the next:
//
//    helix     ->  hexagon     ->  square     ->  return to origin
//
// A compile-time switch (NAXES_HW) drops the third axis on the ATmega168/328/
// 328p, which expose only two step channels (OC1A / OC1B); every other
// platform wires three axes.
//
// The stepper pin table is selected by architecture, following the StepperDemo
// pattern: each platform header provides the naxes_config_0[] array.

// FastAccelStepper.h must come first: it defines PIN_UNDEFINED / FasDriver,
// which StepperConfig.h (pulled in by every pin table) uses.
#include "FastAccelStepper.h"

#if defined(ARDUINO_ARCH_AVR)
#include "StepperPins_naxes_avr.h"
#elif defined(ARDUINO_ARCH_SAMD)
#include "StepperPins_naxes_sam.h"
#elif defined(ARDUINO_ARCH_SAM)
#include "StepperPins_naxes_sam.h"
#elif defined(ARDUINO_ARCH_RP2040)
#include "StepperPins_naxes_pico.h"
#elif defined(ARDUINO_ARCH_RP2350)
#include "StepperPins_naxes_pico.h"
#elif defined(ARDUINO_ARCH_ESP32)
#include "StepperPins_naxes_esp32.h"
#else
#error "naxes: no pin table for this platform"
#endif

#include "StepperConfig.h"
#include "generic.h"
#include "FasNAxis.h"
#include "naxes_path.h"

#ifdef SIMULATOR
#include <avr/sleep.h>
#endif

// ATmega168/328/328p have two step channels, so the third axis is compiled
// out. Everything else wires three axes. The #undef-then-define form keeps
// PlatformIO's prototype pass (which preprocesses with -fpreprocessed and so
// does not evaluate the #if) from warning about a redefinition.
#define NAXES_HW 3
#define NAXES_HORIZON 64
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || \
    defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__)
#undef NAXES_HW
#undef NAXES_HORIZON
#define NAXES_HW 2
#define NAXES_HORIZON 64
#endif

// FastAccelStepperEngine owns the steppers and the cyclic fill ISR that keeps
// the planner-fed queues alive.
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* naxes_s[NAXES_HW] = {NULL};

// The planner. HORIZON must hold enough of the path for a smooth ramp while
// still fitting the small AVR parts; NAXES_HORIZON is 64 points on the
// two-axis build and 48 on the three-axis one.
FasNAxis<NAXES_HW, NAXES_HORIZON> naxes_planner(FasNAxisConfig{});

// Index of the waypoint currently being commanded. 0 is the first addLine after
// the origin; the run finishes when it reaches total_waypoints().
static uint16_t naxes_wp_index = 0;
// The path is fully emitted and the queues have drained.
static bool naxes_done = false;

// Quarter index q in 0..3, within-quarter sample k in 0..NAXES_QSAMPLES-1.
// mag = R*sin(k*90/Q), rad = R*cos(k*90/Q). The four quarters place a vertex
// on the circle of radius R at angle (q*90 + k*90/Q) degrees:
//   q0: x = R*sin, y =  R*cos
//   q1: x = R*cos, y = -R*sin
//   q2: x = -R*sin, y = -R*cos
//   q3: x = -R*cos, y = R*sin
// Vertex 0 is (R, 0), so the initial move from the origin is a straight line
// along +X at Z=0 (outside the helix-radius window).
static int32_t q_axis(int q, int16_t mag, int16_t rad, bool x_axis) {
  if (x_axis) {
    switch (q) {
      case 0:
        return rad;
      case 1:
        return -mag;
      case 2:
        return -rad;
      default:
        return mag;
    }
  }
  switch (q) {
    case 0:
      return mag;
    case 1:
      return rad;
    case 2:
      return -mag;
    default:
      return -rad;
  }
}

// Absolute target (one entry per axis) of the i-th waypoint of the fixed path,
// computed on the fly so the whole curve costs only a few words of RAM. The
// origin (index 0 of the addLine sequence) is established by setCurrentPosition
// in setup(); total_waypoints() counts the addLine targets that follow.
void waypoint_target(uint16_t i, int32_t* t) {
  t[0] = 0;
  t[1] = 0;
#if NAXES_HW >= 3
  t[2] = 0;
#endif
  const uint32_t per_turn = 4u * NAXES_QSAMPLES;
  const uint32_t helix_len = (uint32_t)NAXES_HELIX_TURNS * per_turn;

  if (i < helix_len) {
    uint32_t rem = i % per_turn;
    int q = (int)(rem / NAXES_QSAMPLES);
    int k = (int)(rem % NAXES_QSAMPLES);
    int16_t mag = NAXES_SIN_Q[k];
    int16_t rad = NAXES_COS_Q[k];
    t[0] = q_axis(q, mag, rad, true);
    t[1] = q_axis(q, mag, rad, false);
#if NAXES_HW >= 3
    // Climb Z smoothly over the whole helix, reaching Z_MAX at its end.
    t[2] = (int32_t)((uint32_t)i * NAXES_HELIX_Z_PER_TURN / per_turn);
#endif
    return;
  }

  i -= (uint16_t)helix_len;

  // ---- hexagon: six corners of a regular hexagon in the XY plane.
  const int32_t hex_x[6] = {NAXES_RADIUS,  NAXES_RADIUS / 2,  -NAXES_RADIUS / 2,
                            -NAXES_RADIUS, -NAXES_RADIUS / 2, NAXES_RADIUS / 2};
  const int32_t hex_y[6] = {0, NAXES_HEX_Y,  NAXES_HEX_Y,
                            0, -NAXES_HEX_Y, -NAXES_HEX_Y};
  if (i < 6) {
    t[0] = hex_x[i];
    t[1] = hex_y[i];
#if NAXES_HW >= 3
    t[2] = NAXES_HELIX_TURNS * NAXES_HELIX_Z_PER_TURN;
#endif
    return;
  }
  i -= 6;

  // ---- square: four corners of an axis-aligned square in the XY plane. On a
  // 2-axis build this is the cube footprint (four corners).
  const int32_t h = NAXES_SQUARE_HALF;
  static const int32_t sq[4][2] = {{h, h}, {-h, h}, {-h, -h}, {h, -h}};
  if (i < 4) {
    t[0] = sq[i][0];
    t[1] = sq[i][1];
#if NAXES_HW >= 3
    t[2] = NAXES_HELIX_TURNS * NAXES_HELIX_Z_PER_TURN;
#endif
    return;
  }
  i -= 4;

  // ---- return to origin.
  t[0] = 0;
  t[1] = 0;
#if NAXES_HW >= 3
  t[2] = 0;
#endif
}

uint16_t total_waypoints() {
  uint16_t n =
      (uint16_t)((uint32_t)NAXES_HELIX_TURNS * 4 * NAXES_QSAMPLES + 6 + 4 + 1);
  return n;
}

void setup() {
  engine.init();

  // Connect one stepper per axis. The pin table carries NAXES_HW valid
  // entries followed by the STEPPER_CONFIG_END sentinel.
  const struct stepper_config_s* cfg = naxes_config_0;
  for (uint8_t i = 0; i < NAXES_HW; i++) {
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
    naxes_s[i] = engine.stepperConnectToPin(cfg[i].step, cfg[i].driver_type);
#else
    naxes_s[i] = engine.stepperConnectToPin(cfg[i].step);
#endif
    if (naxes_s[i] == NULL) {
      // No Serial here: it would drag Print/println into the flash image and
      // the small AVR parts have no room for it.
      while (1) {
      }
    }
    naxes_s[i]->setDirectionPin(cfg[i].direction,
                                cfg[i].direction_high_count_up);
    if (cfg[i].enable_low_active != PIN_UNDEFINED) {
      naxes_s[i]->setEnablePin(cfg[i].enable_low_active);
    }
    naxes_s[i]->setAutoEnable(cfg[i].auto_enable);
    // A gentle but brisk profile so the whole path runs in a few seconds.
    naxes_s[i]->setSpeedInHz(400);
    naxes_s[i]->setAcceleration(1200);
  }

  // Without auto enable the outputs must be enabled and settled before the
  // planner kicks off (whitepaper section 4.5): a short delay lets the enable
  // pin reach its active state so addQueueEntry() is not bounced off the
  // WaitForEnablePinActive path.
  if (cfg[0].on_delay_us != 0) {
    DELAY_US(cfg[0].on_delay_us);
  }
  for (uint8_t i = 0; i < NAXES_HW; i++) {
    naxes_s[i]->enableOutputs();
  }
  DELAY_US(100);
  naxes_planner.addAxis(0, naxes_s[0]);
#if NAXES_HW >= 2
  naxes_planner.addAxis(1, naxes_s[1]);
#endif
#if NAXES_HW >= 3
  naxes_planner.addAxis(2, naxes_s[2]);
#endif
  naxes_planner.setLimitsFromSteppers();

  // Start at the origin and open the path.
  int32_t origin[NAXES_HW] = {0};
  naxes_planner.setCurrentPosition(origin);
}

void loop() {
  if (!naxes_done) {
    // Feed the next waypoint target, but only when the planner's ring has room.
    // addLine() drains as the queues run, so this one target per pass keeps the
    // ring from filling.
    if (naxes_wp_index < total_waypoints()) {
      int32_t t[NAXES_HW] = {0};
      waypoint_target(naxes_wp_index, t);
      if (naxes_planner.addLine(t)) {
        naxes_wp_index++;
        if (naxes_wp_index >= total_waypoints()) {
          naxes_planner.endPath();
        }
      }
    }
  }

  // Plan and feed the committed path into the stepper queues. The engine's
  // cyclic ISR (manageSteppers, via the timer) emits the step / dir pins;
  // pump() keeps the queues fed.
  naxes_planner.pump();

  if (!naxes_planner.isBusy() && naxes_wp_index >= total_waypoints()) {
#ifdef SIMULATOR
    noInterrupts();
    sleep_cpu();
#else
    naxes_done = true;
#endif
  }
}
