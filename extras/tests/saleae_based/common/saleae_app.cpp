/*
 * saleae_app.cpp — Host command channel and addQueueEntry() characterization.
 *
 * The purpose of this harness is to characterize the queue layer at the pin
 * level: what step/dir waveform actually comes out of addQueueEntry() plus the
 * driver. It deliberately does NOT use the ramp generator (move()) or
 * moveTimed() — those are covered by the PC-based and SimAVR tests, and a
 * logic analyzer adds nothing to them.
 *
 * Why the firmware owns the command program
 * ------------------------------------------
 * 115200 baud needs ~0.3 s to transfer a few hundred commands, and a stepper
 * would have moved long before the last one arrives, so streaming a plan over
 * serial is not an option. Nor is a large static plan: on a 328P the queue and
 * two stepper objects already dominate the 2 KB of RAM. So the firmware builds
 * the commands itself. The host sends one short line per segment
 * (`QSEG <steps> <ticks> <dir>`), and the firmware keeps a bounded program of
 * at most QE_MAX_SEG segments (QE_MAX_SEG * sizeof(segment), one shared copy)
 * plus a per-stepper cursor into it. A whole scenario is a handful of short
 * lines, the RAM cost is ~50 bytes regardless of scenario length, and sweeping
 * a parameter does not need a recompile.
 *
 * `ticks` is the raw queue period in timer ticks rather than microseconds so
 * the host can address the 16-bit boundaries exactly (1 and 65535). QINFO
 * reports TICKS_PER_S, MIN_CMD_TICKS, QUEUE_LEN and the per-stepper
 * max_speed_in_ticks floor so the host can compute valid values.
 *
 * Protocol
 * --------
 *   SR00                       SR_00 connection self-test (8 pins, 1 Hz)
 *   CONFIG <n> <drv>[,<drv>...] [dir|nodir]
 *                              connect <n> steppers, one driver NAMED per
 *                              stepper. There is no automatic choice: a result
 *                              that does not record which driver produced it
 *                              characterizes nothing. An unknown driver, an
 *                              absent one, a list whose length is not <n>, a
 *                              count this platform cannot provide and an
 *                              unimplemented pin mode are all refused --
 *                              nothing is ever silently substituted.
 *                              rmt | rmt_v2 | mcpwm | mcpwm_pcnt | i2s |
 *                              i2s_direct | i2s_mux on the ESP32 family, timer
 *                              on AVR/SAM/SAMD, pio on Pico; a driver the
 *                              running build has no queues for is refused.
 *   DRIVERS                    which drivers this build accepts, and whether
 *                              the I2S multiplexer is up. Read from the same
 *                              conditions CONFIG parses names with, so a
 *                              capability report cannot disagree with what
 *                              CONFIG will actually do. The host keeps no
 *                              table of its own: a host-side table was wrong
 *                              about this board (6 MCPWM queues where there is
 *                              one, 32 mux queues where there are none) and a
 *                              host cannot tell that it is wrong.
 *   IMUX <data> <bclk> <ws>    bring the I2S multiplexer up, at runtime.
 *                              initI2sMux() must precede any mux stepper and
 *                              cannot run twice, so it is a serial command
 *                              rather than a build flag: wiring a multiplexer
 *                              up is three pins on existing firmware instead
 *                              of a recompile. Refused on a non-I2S build.
 *   QINFO                      tick rate, MIN_CMD_TICKS, QUEUE_LEN and the
 *                              per-stepper speed floor
 *   QCLR                       drop the program and stop
 *   QSEG <steps> <ticks> <dir> append a segment to the shared program; steps=0
 *                              is a pause of <ticks> ticks, dir is 0 or 1.
 *   QSEG <idx> <steps> <ticks> <dir>
 *                              append to stepper <idx>'s own program instead,
 *                              for scenarios that need two steppers at
 *                              different speeds (SR_15). A stepper with its
 *                              own program ignores the shared one.
 *   QRUN <mask>                run the program on the steppers selected by the
 *                              bitmask, synchronized start
 *   POS                        reply positions of all steppers
 *   STOP                       stop move / self-test
 *
 * Characterization scenarios are assembled from segments, e.g.
 *   QCLR | QSEG 255 80 1 | QSEG 0 1600 1 | QSEG 1 80 1 | QRUN 1
 * is "255 steps at max speed, a pause, then a single step" — the case that
 * exposes MCPWM/PCNT counter-limit overrun handling.
 */

#include "saleae_app.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FastAccelStepper.h"
#include "FastAccelStepperEngine.h"
#include "saleae_hal.h"
#include "saleae_test.h"

#define SALEAE_SERIAL_BAUD 115200

// Analyzer channels and the pin stride, which together decide how many steppers
// fit. A stepper with a direction pin costs two channels (step + dir), a
// step-only one costs a single channel, so `dir` reaches 4 and `nodir` reaches
// 8 on the same eight channels (white paper 3.3/10.1).
#define SALEAE_CHANNELS 8
#define SALEAE_STRIDE_DIR 2
#define SALEAE_STRIDE_NODIR 1

// Bounded program: 8 segments is 8 * 6 bytes = 48 bytes of RAM, shared by all
// steppers. Every characterization scenario needs fewer.
#define QE_MAX_SEG 8

// The library cannot hand out more steppers than MAX_STEPPER, and each costs
// RAM, so never ask for more. AVR is the tight case (2 on a 328P).
#if defined(MAX_STEPPER)
#define SALEAE_MAX_STEPPERS (MAX_STEPPER < 8 ? MAX_STEPPER : 8)
#else
#define SALEAE_MAX_STEPPERS 8
#endif

// The longest argument is the CONFIG driver list: one driver name per stepper,
// at 12 bytes each (10 for the longest name, "mcpwm_pcnt" or "i2s_direct", plus
// the separating comma).
//
// An earlier revision used a flat 48 bytes, which truncated a legal
// `CONFIG 4 mcpwm_pcnt,...` at 31 characters and refused it with a misleading
// "no such driver" on the last, half-cut name. A flat 96 would fix that too and
// cost AVR 72 bytes of RAM it cannot spare, so the width steps with the stepper
// count.
//
// It has to be a *literal* rather than `12 * SALEAE_MAX_STEPPERS`, because it
// is also an sscanf field width and a format string cannot hold an expression.
// The static_assert below is what keeps the ladder honest: a platform that
// gains steppers and outgrows the top rung fails the build instead of silently
// truncating a driver list.
#if SALEAE_MAX_STEPPERS <= 2
#define SALEAE_ARG2_MAX 24
#elif SALEAE_MAX_STEPPERS <= 4
#define SALEAE_ARG2_MAX 48
#elif SALEAE_MAX_STEPPERS <= 6
#define SALEAE_ARG2_MAX 72
#else
#define SALEAE_ARG2_MAX 96
#endif

static_assert(SALEAE_ARG2_MAX >= 12 * SALEAE_MAX_STEPPERS,
              "SALEAE_ARG2_MAX too small for the driver list");

// Stringize SALEAE_ARG2_MAX so it can be used as an sscanf field width, which
// must be a literal in the format string.
#define SALEAE_STR_(x) #x
#define SALEAE_STR(x) SALEAE_STR_(x)

// Enough for "CONFIG " + count + the driver list + " nodir" + slack.
#define SALEAE_LINE_MAX (SALEAE_ARG2_MAX + 32)

// Reply buffers are sized from the stepper count too, for the same RAM reason.
// avr-gcc puts the stack in .data, so an oversized local buffer is not free on
// a 328P: a flat 256-byte CONFIG reply cost 112 bytes of RAM there and pushed
// the build to 93 % of SRAM, for no benefit, since a 328P has two steppers.
//
// The CONFIG reply is the larger of the two -- "OK CONFIG n=8 mode=nodir
// stride=1 drivers=" plus 8 driver names plus 8 "maxspeedN=<ticks>" fields
// needs about 26 bytes per stepper on top of a fixed 48. The short replies
// carry no driver names, so they get their own size.
// QINFO is the one reply that grows per stepper: a fixed prefix for
// tps/mincmd/qlen/maxall, then " maxspeedN=<ticks>" each. Sized per rung --
// 48 + 16 * SALEAE_MAX_STEPPERS rounded up, which is what the worst case
// (5-digit ticks on every stepper, one-character index) needs.
#define QINFO_REPLY_FOR(n) (48 + 16 * (n) + 8)

#if SALEAE_MAX_STEPPERS <= 2
#define SALEAE_CFG_REPLY_MAX 96
#define SALEAE_SHORT_REPLY_MAX 48
#define SALEAE_QINFO_REPLY_MAX QINFO_REPLY_FOR(2)
#elif SALEAE_MAX_STEPPERS <= 4
#define SALEAE_CFG_REPLY_MAX 192
#define SALEAE_SHORT_REPLY_MAX 64
#define SALEAE_QINFO_REPLY_MAX QINFO_REPLY_FOR(4)
#else
#define SALEAE_CFG_REPLY_MAX 288
#define SALEAE_SHORT_REPLY_MAX 80
#define SALEAE_QINFO_REPLY_MAX QINFO_REPLY_FOR(8)
#endif

// GPIO per analyzer channel, in channel order.
//
// This one table serves both pin modes, and the stride is what selects between
// them: in `dir` stepper j owns channels 2j (step) and 2j+1 (dir); in `nodir`
// it owns channel j alone. So the mode cannot disagree with the map -- there is
// nothing to keep in step but the stride. It is also the same order and the
// same pins as SR_00's eight (common/saleae_test.cpp), so a channel the
// self-test proved is the channel a scenario measures.
//
// On AVR the step pin MUST be the one the library maps to the timer compare
// output (stepPinStepperA/B), which depends on FAS_TIMER_MODULE, so use the
// library's own macros there instead of literals. A 328P has MAX_STEPPER == 2,
// so only the first two channels' step pins are ever reached; the rest are
// filled with the dir pins, which is inert.
#if defined(ARDUINO_ARCH_ESP32)
static constexpr uint8_t kChanPin[SALEAE_CHANNELS] = {2,  0, 4,  16,
                                                      17, 5, 18, 19};
#elif defined(ARDUINO_ARCH_AVR)
// 8 and 12 are the dir pins: pin 9 and 10 are the Timer1 compare outputs
// (OC1A/OC1B) and are already the step pins; 0 and 1 are the serial port; 13 is
// the LED. Anything left that is not a compare pin is fine for a plain output.
static constexpr uint8_t kChanPin[SALEAE_CHANNELS] = {
    stepPinStepperA, 8, stepPinStepperB, 12, 0, 0, 0, 0};
#else
static constexpr uint8_t kChanPin[SALEAE_CHANNELS] = {2, 3, 4, 5, 6, 7, 8, 9};
#endif

// A pin cannot be a step output and a direction output at once, and two
// steppers cannot share a pin. On AVR the step pins are the timer compare pins,
// which move with FAS_TIMER_MODULE, so this is checked rather than assumed -- a
// collision here compiles cleanly and then has two peripherals driving one pin.
//
// `nodir` needs one channel per stepper, so SALEAE_MAX_STEPPERS must fit in the
// channel budget: that is the binding limit for the 8-stepper case, ahead of
// any driver queue count. (A driver may bind first -- MCPWM/PCNT has 6 queues
// on IDF 5 -- and CONFIG reports the refusal per stepper rather than guessing.)
static_assert(SALEAE_MAX_STEPPERS <= SALEAE_CHANNELS,
              "SALEAE_MAX_STEPPERS does not fit one channel per stepper");

#if defined(ARDUINO_ARCH_AVR)
// Only channels 0..3 are reachable (MAX_STEPPER is 2); the rest are inert.
static_assert(kChanPin[0] != kChanPin[1], "step A collides with dir A");
static_assert(kChanPin[2] != kChanPin[3], "step B collides with dir B");
static_assert(kChanPin[0] != kChanPin[2], "both steppers on one compare pin");
#else
static_assert(kChanPin[0] != kChanPin[1], "step A collides with dir A");
static_assert(kChanPin[2] != kChanPin[3], "step B collides with dir B");
static_assert(kChanPin[0] != kChanPin[2], "step A collides with step B");
static_assert(kChanPin[4] != kChanPin[6], "step C collides with step D");
static_assert(kChanPin[1] != kChanPin[7], "dir A collides with dir D");
static_assert(kChanPin[5] != kChanPin[7], "dir C collides with dir D");
#endif

// Portable driver selector (FasDriver only exists on platforms with driver
// selection, so keep our own enum and map it at connect time).
//
// There is deliberately no "automatic" member. The harness never sends an
// unspecified driver and the firmware never substitutes one: a result that does
// not record which driver produced it characterizes nothing. An earlier
// revision resolved 1ch/2ch to the library's automatic choice and tagged
// seventeen of twenty-five recorded results `auto`, which recorded whatever the
// firmware happened to pick and said nothing about any driver.
enum saleae_driver { SA_RMT, SA_MCPWM, SA_I2S, SA_I2S_MUX, SA_TIMER, SA_PIO };

// One entry of the command program. steps == 0 means "pause for ticks ticks".
struct segment {
  uint16_t ticks;
  uint16_t steps;  // split into <=255 step commands by the pump
  bool count_up;
};

// Per-stepper cursor. `list`/`len` resolve which program this stepper walks:
// its own if it was given one, otherwise the shared one. A pointer rather than
// an index so the shared and per-stepper cases cost the same.
struct qe_cursor {
  uint32_t left;  // steps (or pause ticks) left in the current segment
  const struct segment* list;
  uint8_t len;   // segments in `list`
  uint8_t seg;   // index of the current segment
  bool started;  // queue kicked off
  bool active;   // selected by QRUN and not yet finished
};

struct stepper_slot {
  FastAccelStepper* stepper;
  struct qe_cursor cur;
  uint8_t step_pin;
  uint8_t dir_pin;
  enum saleae_driver driver;
};

// Which pin mode the current CONFIG connected, and the channel stride that goes
// with it. The stride is not derivable from the count alone -- 4 steppers is
// 8 channels with `dir` and 4 with `nodir` -- so `MAP` reports it and the host
// derives the channel map from it rather than assuming a mode.
static uint8_t chan_stride = SALEAE_STRIDE_DIR;

static FastAccelStepperEngine engine;
static bool engine_ready = false;
static struct stepper_slot slots[SALEAE_MAX_STEPPERS];
static uint8_t slot_count = 0;

// Two programs. `shared_program` is what a 3-argument QSEG appends to, and
// every stepper walks it unless it was given a program of its own -- so all the
// scenarios that drive several steppers from one command list keep working
// unchanged. `own_program` is per stepper and exists for SR_15, which needs two
// steppers at *different* periods; the shared program cannot express that.
static struct segment shared_program[QE_MAX_SEG];
static uint8_t shared_len = 0;
static struct segment own_program[SALEAE_MAX_STEPPERS][QE_MAX_SEG];
static uint8_t own_len[SALEAE_MAX_STEPPERS];
static bool own_used[SALEAE_MAX_STEPPERS];

// The program a stepper walks: its own if it has one, else the shared list.
static void program_for(uint8_t idx, const struct segment** list,
                        uint8_t* len) {
  if (own_used[idx]) {
    *list = own_program[idx];
    *len = own_len[idx];
  } else {
    *list = shared_program;
    *len = shared_len;
  }
}

static void clear_programs(void) {
  shared_len = 0;
  for (uint8_t i = 0; i < SALEAE_MAX_STEPPERS; i++) {
    own_len[i] = 0;
    own_used[i] = false;
  }
}

static bool sr00_active = false;
static bool done_pending = false;
// Set once DONE has been reported for the current program, so an idle loop
// does not restart the completion path. Cleared when a new program is armed.
static bool done_announced = false;
static char linebuf[SALEAE_LINE_MAX];
static uint8_t linelen = 0;

static void reply(const char* text) { saleae_hal_serial_write(text); }

// Resolve one driver name. Returns false for an unknown name *and* for a real
// driver this build cannot provide, so `CONFIG 2 rmt,rmt` on an AVR board is
// refused instead of quietly running two timer queues -- a capture that ran
// something else is indistinguishable from a capture of what was asked for.
//
// On an architecture with a single native driver the list is still explicit, it
// simply repeats that one (`timer` for AVR/SAM, `pio` for Pico). Naming it
// costs nothing and keeps every result tagged with the driver that made it.
static bool parse_driver(const char* name, enum saleae_driver* out) {
  (void)out;
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  if (!strcmp(name, "rmt") || !strcmp(name, "rmt_v2")) {
#if defined(SUPPORT_ESP32_RMT)
    *out = SA_RMT;
    return true;
#endif
  }
  if (!strcmp(name, "mcpwm") || !strcmp(name, "mcpwm_pcnt")) {
#if defined(SUPPORT_ESP32_MCPWM_PCNT)
    *out = SA_MCPWM;
    return true;
#endif
  }
  if (!strcmp(name, "i2s") || !strcmp(name, "i2s_direct")) {
#if defined(SUPPORT_ESP32_I2S)
    *out = SA_I2S;
    return true;
#endif
  }
  if (!strcmp(name, "i2s_mux")) {
#if defined(SUPPORT_ESP32_I2S)
    *out = SA_I2S_MUX;
    return true;
#endif
  }
#else
  if (!strcmp(name, "timer")) {
    *out = SA_TIMER;
    return true;
  }
#if defined(ARDUINO_ARCH_RP2040) || defined(PICO_RP2040) || \
    defined(PICO_SDK_RP2350)
  if (!strcmp(name, "pio")) {
    *out = SA_PIO;
    return true;
  }
#endif
#endif
  return false;
}

// The name a connected stepper is reported under. `rmt` rather than `rmt_v2`:
// the RMT generation is a property of the SDK, which is a tag on the run, not
// of the driver the harness asked for.
static const char* driver_name(enum saleae_driver d) {
  switch (d) {
    case SA_RMT:
      return "rmt";
    case SA_MCPWM:
      return "mcpwm_pcnt";
    case SA_I2S:
      return "i2s_direct";
    case SA_I2S_MUX:
      return "i2s_mux";
    case SA_TIMER:
      return "timer";
    case SA_PIO:
      return "pio";
  }
  return "unknown";
}

static void stop_sr00(void) {
  if (sr00_active) {
    saleae_test_stop();
    sr00_active = false;
  }
}

static void stop_all(void) {
  for (uint8_t i = 0; i < slot_count; i++) {
    if (slots[i].stepper) {
      slots[i].stepper->stopMove();
    }
    memset(&slots[i].cur, 0, sizeof(slots[i].cur));
  }
  clear_programs();
  done_pending = false;
  done_announced = false;
}

static bool connect_stepper(uint8_t idx, enum saleae_driver driver,
                            bool nodir) {
  const uint8_t step_pin = kChanPin[idx * chan_stride];
  // A step-only stepper gets no dir pin at all rather than a repeated one, so
  // setDirectionPin() is not called and nothing on that pin can be mistaken for
  // a direction. The `nodir` consequence is that count_up is always driven true
  // (see qe_feed), because there is no pin to toggle for a false.
  const uint8_t dir_pin = nodir ? 0 : kChanPin[idx * chan_stride + 1];

  FastAccelStepper* s;
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  // No DRIVER_DONT_CARE anywhere: the caller named the driver, and a name this
  // build cannot honour must not reach the engine as a request for "any".
  FasDriver fd;
  switch (driver) {
    case SA_RMT:
      fd = DRIVER_RMT;
      break;
    case SA_MCPWM:
      fd = DRIVER_MCPWM_PCNT;
      break;
#if defined(SUPPORT_ESP32_I2S)
    case SA_I2S:
      fd = DRIVER_I2S_DIRECT;
      break;
    case SA_I2S_MUX:
      fd = DRIVER_I2S_MUX;
      break;
#endif
    default:
      return false;
  }
  s = engine.stepperConnectToPin(step_pin, fd);
#else
  (void)driver;
  s = engine.stepperConnectToPin(step_pin);
#endif
  if (!s) {
    return false;
  }
  if (!nodir) {
    s->setDirectionPin(dir_pin);
  }
  memset(&slots[idx].cur, 0, sizeof(slots[idx].cur));
  slots[idx].stepper = s;
  slots[idx].step_pin = step_pin;
  slots[idx].dir_pin = dir_pin;
  slots[idx].driver = driver;
  return true;
}

// Pin mode. `dir` costs two channels per stepper, `nodir` one, so the mode sets
// the channel stride and with it how many steppers the eight channels reach.
//
// `nodir` is a real pin configuration, not a shorthand: step-only steppers get
// no dir pin, the QSEG direction argument is accepted and forced true (there is
// no pin to toggle for a false), and the host learns it from MAP rather than
// assuming. It is how 8 parallel steppers fit on 8 channels.
static bool parse_pin_mode(char* mode_text, bool* nodir, uint8_t* stride) {
  if (!mode_text) {
    *nodir = false;
    *stride = SALEAE_STRIDE_DIR;
    return true;
  }
  if (!strcmp(mode_text, "dir")) {
    *nodir = false;
    *stride = SALEAE_STRIDE_DIR;
    return true;
  }
  if (!strcmp(mode_text, "nodir")) {
    *nodir = true;
    *stride = SALEAE_STRIDE_NODIR;
    return true;
  }
  return false;
}

// Both bounds, in the refusal, because "too many steppers" cannot say which one
// bit: a driver that binds first (MCPWM/PCNT has 6 queues on IDF 5) is a
// different fact from a channel budget that runs out at 4 with `dir`.
//
// Its own function rather than a buffer in handle_config, because on AVR that
// function's frame is on top of the reply buffer and a second one there cost 58
// bytes of SRAM -- measured, not estimated. The frame is transient and never
// recursed into, so the extra level costs nothing that matters.
static void reply_too_many(long count, uint8_t cap, uint8_t stride) {
  char buf[48];
  snprintf(buf, sizeof(buf), "ERR CONFIG n=%ld max=%u/%u/%u/%u\n", count,
           (unsigned)cap, (unsigned)SALEAE_MAX_STEPPERS,
           (unsigned)SALEAE_CHANNELS, (unsigned)stride);
  reply(buf);
}

// MAP
//
// Which analyzer channel carries which stepper. The host needs this to read the
// capture: with `dir` the channels are step0,dir0,step1,dir1,... and with
// `nodir` they are step0,step1,step2,..., so the same count is a different
// channel map. Reporting count + mode + stride makes the map derivable from one
// reply instead of from a constant the host and firmware each keep.
//
// It also reports the GPIO behind each reachable channel, which is what makes
// the map checkable against the wiring rather than merely self-consistent.
// Whether this build can connect `driver` at all. Mirrors parse_driver()'s
// conditions deliberately and exactly: a capability report built from a second
// copy of these #ifs is a second thing to keep in sync, and when it drifts the
// host plans runs against a driver this build will refuse.
static bool driver_supported(enum saleae_driver driver) {
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  switch (driver) {
    case SA_RMT:
#if defined(SUPPORT_ESP32_RMT)
      return true;
#else
      return false;
#endif
    case SA_MCPWM:
#if defined(SUPPORT_ESP32_MCPWM_PCNT)
      return true;
#else
      return false;
#endif
    case SA_I2S:
    case SA_I2S_MUX:
#if defined(SUPPORT_ESP32_I2S)
      return true;
#else
      return false;
#endif
    default:
      return false;
  }
#else
  (void)driver;
  return true;  // timer, and pio where compiled, are the only drivers here
#endif
}

// Set once IMUX has successfully brought the I2S multiplexer up. The engine's
// own _i2s_mux_initialized is private to StepperQueue, but this app is the one
// that calls initI2sMux(), so this app is the one that knows.
static bool mux_ready = false;

static void handle_drivers(void) {
  // Which drivers this build accepts, and whether the multiplexer is up.
  //
  // The host used to keep its own table of what each chip has -- and it was
  // wrong in both directions: `mcpwm_pcnt` claimed 6 queues where the board
  // connects one, `i2s_mux` claimed 32 where the board connects none. A host
  // cannot detect that, because only the firmware has the answer. Presence is
  // therefore reported here, from the same conditions parse_driver() uses.
  //
  // Queue *counts* are deliberately absent. They are in pd_config.h, which the
  // public headers do not pull in, and adding a library accessor for a test
  // harness would put test scaffolding into the product. They do not need to be
  // predicted either: `scale` sweeps to the analyzer's channel budget and the
  // board's refusal *is* the measured bound. See harness.py scale_bound().
  // Sized from this build's own reply, not from the CONFIG buffer it shares a
  // constant with. The ESP32 form is "OK DRIVERS mux=0 rmt=1 rmt_v2=1
  // mcpwm_pcnt=1 i2s_direct=1 i2s_mux=1 mux_init=0" -- 80 bytes -- while a
  // timer build emits "OK DRIVERS mux=0 timer=1 mux_init=0", 40. avr-gcc puts
  // the stack in .data, so borrowing the CONFIG size would spend 56 bytes of a
  // 328P's 203 remaining on a reply that is half that long.
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  char buf[88];
#else
  char buf[48];
#endif
  int len = snprintf(buf, sizeof(buf), "OK DRIVERS mux=%u", mux_ready ? 1 : 0);
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  len += snprintf(buf + len, sizeof(buf) - len,
                  " rmt=%u rmt_v2=%u mcpwm_pcnt=%u i2s_direct=%u i2s_mux=%u",
                  driver_supported(SA_RMT) ? 1 : 0,
                  driver_supported(SA_RMT) ? 1 : 0,
                  driver_supported(SA_MCPWM) ? 1 : 0,
                  driver_supported(SA_I2S) ? 1 : 0,
                  driver_supported(SA_I2S_MUX) ? 1 : 0);
#else
  len += snprintf(buf + len, sizeof(buf) - len, " timer=1");
#if defined(ARDUINO_ARCH_RP2040) || defined(PICO_RP2040) || \
    defined(PICO_SDK_RP2350)
  len += snprintf(buf + len, sizeof(buf) - len, " pio=1");
#endif
#endif
  // A mux that is compiled in but not initialised would otherwise report
  // i2s_mux=1 and then refuse every CONFIG naming it, which reads as a
  // contradiction. The two fields together say "present, not up yet".
  len += snprintf(buf + len, sizeof(buf) - len, " mux_init=%u\n",
                  mux_ready ? 1 : 0);
  reply(buf);
}

// IMUX <data> <bclk> <ws> -- bring up the I2S multiplexer at runtime.
//
// initI2sMux() must be called before any stepperConnectToPin(DRIVER_I2S_MUX), and
// it cannot be called twice. Making it a serial command rather than a build-time
// constant means wiring a multiplexer up is three pins on an existing firmware,
// not a recompile -- which is what makes the mux testable at all on a rig whose
// stepper pins are the analyzer's channels.
static void handle_imux(const char* data, const char* bclk, const char* ws) {
#if defined(SUPPORT_ESP32_I2S)
  if (!data || !bclk || !ws) {
    reply("ERR IMUX needs <data> <bclk> <ws>\n");
    return;
  }
  if (mux_ready) {
    reply("ERR IMUX already up (initI2sMux() cannot run twice)\n");
    return;
  }
  const uint8_t d = (uint8_t)atoi(data);
  const uint8_t b = (uint8_t)atoi(bclk);
  const uint8_t w = (uint8_t)atoi(ws);
  if (!engine.initI2sMux(d, b, w)) {
    reply("ERR IMUX initI2sMux failed (pins busy, or already initialised)\n");
    return;
  }
  mux_ready = true;
  char buf[48];
  snprintf(buf, sizeof(buf), "OK IMUX data=%u bclk=%u ws=%u\n", d, b, w);
  reply(buf);
#else
  (void)data;
  (void)bclk;
  (void)ws;
  reply("ERR IMUX needs an ESP32 I2S build\n");
#endif
}

static void handle_map(void) {
  // "MAP count=8 mode=nodir stride=1 ch=" plus 8 two-digit pins. Sized from the
  // channel budget for the same RAM reason as SALEAE_CFG_REPLY_MAX.
  char buf[SALEAE_CHANNELS * 4 + 64];
  int len = snprintf(buf, sizeof(buf),
                     "MAP count=%u mode=%s stride=%u ch=", (unsigned)slot_count,
                     chan_stride == SALEAE_STRIDE_NODIR ? "nodir" : "dir",
                     (unsigned)chan_stride);
  const uint8_t used = slot_count ? slot_count * chan_stride : 0;
  for (uint8_t c = 0; c < used && c < SALEAE_CHANNELS; c++) {
    len += snprintf(buf + len, sizeof(buf) - len, "%s%u", c ? "," : "",
                    (unsigned)kChanPin[c]);
  }
  snprintf(buf + len, sizeof(buf) - len, "\n");
  reply(buf);
}

// CONFIG <count> <driver>[,<driver>...] [dir|nodir]
//
// One generic grammar replaces the eight named presets (white paper §3.2): a
// preset was only ever a count, a list of drivers and a pin mode, so naming the
// combinations only means a new name for every one.
//
// It refuses rather than clamps, on all five counts, because a silently reduced
// or differently-driven run makes the capture look like a driver problem:
//   - a count the platform cannot provide,
//   - a count the channel budget cannot carry in the requested pin mode,
//   - a driver name this build has no driver for,
//   - a driver list whose length is not the count,
//   - a pin mode this build does not implement.
static void handle_config(char* count_text, char* driver_list,
                          char* mode_text) {
  stop_sr00();
  if (!engine_ready) {
    engine.init();
    engine_ready = true;
  }

  // Pin mode first: the stride it selects decides how many steppers the eight
  // channels can carry, and so the count cap below depends on it.
  bool nodir;
  uint8_t stride;
  if (!parse_pin_mode(mode_text, &nodir, &stride)) {
    reply("ERR CONFIG mode dir|nodir\n");
    return;
  }
  const uint8_t chan_cap = SALEAE_CHANNELS / stride;

  // Resolve the count *before* touching the array. On AVR SALEAE_MAX_STEPPERS
  // is 2, so filling a 4-entry driver list into a 2-entry array would scribble
  // past it -- and clamping afterwards is too late, the writes have already
  // happened.
  const uint8_t cap =
      SALEAE_MAX_STEPPERS < chan_cap ? SALEAE_MAX_STEPPERS : chan_cap;
  enum saleae_driver drivers[SALEAE_MAX_STEPPERS];
  uint8_t n;

  if (!count_text) {
    reply("ERR CONFIG needs <n> <drv>[,<drv>]\n");
    return;
  }

  char* end = NULL;
  long count = strtol(count_text, &end, 10);
  // A trailing character means one of the superseded preset names ("1ch",
  // "mixed") is still arriving. atol would read the leading digits and quietly
  // configure one stepper, so reject the token instead.
  if (end == count_text || *end != '\0' || count < 1) {
    reply("ERR CONFIG needs <n> <drv>[,<drv>]\n");
    return;
  }
  if (count > cap) {
    reply_too_many(count, cap, stride);
    return;
  }
  n = (uint8_t)count;

  if (!driver_list) {
    reply("ERR CONFIG needs <n> <drv>[,<drv>]\n");
    return;
  }

  uint8_t got = 0;
  for (char* tok = strtok(driver_list, ","); tok; tok = strtok(NULL, ",")) {
    enum saleae_driver d;
    if (!parse_driver(tok, &d)) {
      reply("ERR CONFIG no such driver\n");
      return;
    }
    if (got == n) {
      reply("ERR CONFIG needs <n> <drv>[,<drv>]\n");
      return;
    }
    drivers[got++] = d;
  }
  if (got != n) {
    reply("ERR CONFIG needs <n> <drv>[,<drv>]\n");
    return;
  }

  // Each queue can only be allocated once, so a second CONFIG cannot move an
  // already-connected stepper. Report the existing setup instead of silently
  // running with a different pin map than the host thinks.
  if (slot_count > 0) {
    // Report the existing setup rather than silently running with a different
    // pin map than the host believes. It has to include the mode and the
    // drivers, since a second CONFIG naming a different mode is exactly the
    // case where "already" would otherwise hide a disagreement.
    char buf[SALEAE_SHORT_REPLY_MAX];
    snprintf(buf, sizeof(buf), "OK CONFIG n=%u mode=%s already\n",
             (unsigned)slot_count,
             chan_stride == SALEAE_STRIDE_NODIR ? "nodir" : "dir");
    reply(buf);
    return;
  }

  chan_stride = stride;
  slot_count = n;
  for (uint8_t i = 0; i < n; i++) {
    if (!connect_stepper(i, drivers[i], nodir)) {
      slot_count = i;
      char buf[SALEAE_SHORT_REPLY_MAX];
      snprintf(buf, sizeof(buf), "ERR connect step %u n=%u drv=%s nodir=%u\n",
               i, i, driver_name(drivers[i]), (unsigned)nodir);
      reply(buf);
      return;
    }
  }

  // One buffer rather than several, so the reply cannot be truncated halfway,
  // and sized by SALEAE_CFG_REPLY_MAX because on AVR it is stack.
  char buf[SALEAE_CFG_REPLY_MAX];
  // Naming the drivers that were actually connected is what lets a run be
  // checked against what the board really did -- the one thing an implicit
  // driver choice used to make impossible. The host already knows what it asked
  // for, so this is not how a result is tagged.
  int len = snprintf(buf, sizeof(buf),
                     "OK CONFIG n=%u mode=%s stride=%u drivers=", (unsigned)n,
                     nodir ? "nodir" : "dir", (unsigned)stride);
  for (uint8_t i = 0; i < n; i++) {
    len += snprintf(buf + len, sizeof(buf) - len, "%s%s", i ? "," : "",
                    driver_name(slots[i].driver));
  }
  for (uint8_t i = 0; i < n; i++) {
    len += snprintf(buf + len, sizeof(buf) - len, " maxspeed%u=%u", i,
                    slots[i].stepper->getMaxSpeedInTicks());
  }
  snprintf(buf + len, sizeof(buf) - len, "\n");
  reply(buf);
}

// ---------------------------------------------------------------------------
// addQueueEntry() feeder
// ---------------------------------------------------------------------------

// Keep two slots free so a driver can always inject its direction-change drain
// pause (FasNAxis::all_have_room() reserves the same margin).
#define QE_ROOM_RESERVE 2

static bool qe_has_room(FastAccelStepper* s) {
  return ((uint32_t)s->queueEntries() + QE_ROOM_RESERVE) < (uint32_t)QUEUE_LEN;
}

// Prefill this far before a synchronized kick-off, so every participant has
// non-empty queues when synchronizedStart() arms the timers together.
#define QE_PREFILL ((QUEUE_LEN / 2) < 2 ? 2 : (QUEUE_LEN / 2))

// Advance to the next segment, skipping exhausted ones. Returns false when the
// program is done.
static bool qe_next_segment(struct qe_cursor* c) {
  while (c->seg < c->len && c->left == 0) {
    c->seg++;
    if (c->seg < c->len) {
      c->left =
          c->list[c->seg].steps ? c->list[c->seg].steps : c->list[c->seg].ticks;
    }
  }
  return c->seg < c->len;
}

// Emit commands from the plan until the queue is full or a retryable code is
// returned. Sets *err on a terminal addQueueEntry() code.
static void qe_feed(struct qe_cursor* c, FastAccelStepper* s, bool* err,
                    AqeResultCode* last) {
  while (qe_has_room(s) && qe_next_segment(c)) {
    const struct segment* seg = &c->list[c->seg];
    struct stepper_command_s cmd;
    bool is_pause = (seg->steps == 0);

    if (is_pause) {
      cmd.ticks = (c->left > 65535u) ? 65535u : (uint16_t)c->left;
      cmd.steps = 0;
    } else {
      cmd.ticks = seg->ticks;
      cmd.steps = (c->left > 255u) ? 255u : (uint8_t)c->left;
    }
    // In `nodir` there is no direction pin, so a false has nothing to toggle
    // and the queue would refuse the command with ErrorNoDirPinToToggle.
    // Driving it true unconditionally is the only coherent reading: the
    // direction argument still has to parse (so a scenario's program is
    // unchanged between modes), it simply cannot mean anything without the pin.
    // The direction-observing scenarios are `dir`-mode by construction.
    cmd.count_up = (chan_stride == SALEAE_STRIDE_NODIR) ? true : seg->count_up;

    AqeResultCode rc;
    do {
      rc = s->addQueueEntry(&cmd, c->started);
      // A driver that injected a DIR drain pause did NOT take the command, so
      // the very same command goes back in — immediately, with no delay.
    } while (aqeRetryImmediately(rc));

    if (last) {
      *last = rc;
    }

    if (rc == AqeResultCode::OK) {
      c->left -= is_pause ? cmd.ticks : cmd.steps;
      continue;
    }

    if (aqeIsPauseInjected(rc) || aqeRetry(rc)) {
      // QueueFull / DirPinIsBusy / WaitForEnablePinActive / DeviceNotReady /
      // an injected DIR pause: try again on the next pump.
      return;
    }

    // ErrorTicksTooLow / ErrorNoDirPinToToggle / ErrorEmptyQueueToStart.
    if (err) {
      *err = true;
    }
    return;
  }
}

static bool any_active(void) {
  for (uint8_t i = 0; i < slot_count; i++) {
    if (slots[i].cur.active) {
      return true;
    }
  }
  return false;
}

static bool any_running(void) {
  for (uint8_t i = 0; i < slot_count; i++) {
    if (slots[i].stepper && slots[i].stepper->isRunning()) {
      return true;
    }
  }
  return false;
}

static void qe_finish(void) {
  for (uint8_t i = 0; i < slot_count; i++) {
    slots[i].cur.active = false;
  }
  done_pending = true;
}

// Fill every selected queue, then kick off, then keep feeding as it drains.
static void qe_pump(void) {
  static FastAccelStepper* participants[SALEAE_MAX_STEPPERS];

  for (uint8_t i = 0; i < slot_count; i++) {
    struct qe_cursor* c = &slots[i].cur;
    if (!c->active || c->started) {
      continue;
    }
    bool err = false;
    AqeResultCode last = AqeResultCode::OK;
    while (slots[i].stepper->queueEntries() < QE_PREFILL) {
      qe_feed(c, slots[i].stepper, &err, &last);
      if (err) {
        break;
      }
      if (!qe_next_segment(c)) {
        break;  // program exhausted; queue is as full as it will get
      }
    }
    if (err) {
      c->active = false;
      char buf[64];
      snprintf(buf, sizeof(buf), "ERR QE step%u rc=%d\n", i, (int)last);
      reply(buf);
      done_pending = true;
    }
  }

  // Kick off once every participant is prefilled, so synchronizedStart() arms
  // them on one shared timer compare.
  uint8_t n = 0;
  for (uint8_t i = 0; i < slot_count; i++) {
    if (slots[i].cur.active && !slots[i].cur.started) {
      if (slots[i].stepper->queueEntries() >= QE_PREFILL ||
          !qe_next_segment(&slots[i].cur)) {
        participants[n++] = slots[i].stepper;
      }
    }
  }
  if (n > 0) {
    AqeResultCode rc = engine.synchronizedStart(participants, n);
    for (uint8_t i = 0; i < slot_count; i++) {
      if (slots[i].stepper) {
        slots[i].cur.started = true;
      }
    }
    if (rc != AqeResultCode::OK) {
      char buf[48];
      snprintf(buf, sizeof(buf), "ERR QE start rc=%d\n", (int)rc);
      reply(buf);
      done_pending = true;
    }
  }

  // Top up the running queues.
  for (uint8_t i = 0; i < slot_count; i++) {
    struct qe_cursor* c = &slots[i].cur;
    if (!c->active || !c->started) {
      continue;
    }
    bool err = false;
    AqeResultCode last = AqeResultCode::OK;
    qe_feed(c, slots[i].stepper, &err, &last);
    if (err) {
      c->active = false;
      char buf[64];
      snprintf(buf, sizeof(buf), "ERR QE step%u rc=%d\n", i, (int)last);
      reply(buf);
      done_pending = true;
    }
  }
}

static void handle_qinfo(void) {
  // One speed floor per stepper, comma-separated, then the largest of them as
  // `maxall`. The commas are not cosmetic: without them a 3-stepper board
  // replies `maxspeed=808080` and the host reads that as the single number
  // 808080, so every QSEG built from it is refused for exceeding the 16-bit
  // ticks field and the run fails with a message about the tick range rather
  // than about what was wrong. `maxall` is what a shared program is planned
  // against, and it leads the reply so it cannot be the field that gets
  // truncated.
  //
  // Sizing: "QINFO tps=16000000 mincmd=65535 qlen=32 maxall=65535 " is 48
  // characters before any per-stepper field, and 8 floors at 5 digits plus a
  // comma each is 48 more. SALEAE_SHORT_REPLY_MAX is sized for the AVR rung
  // (48) which caps at 2 steppers, so a separate larger buffer is needed for
  // the many-stepper rungs; the two ladder rungs are otherwise identical.
  char buf[SALEAE_QINFO_REPLY_MAX];
  int len = snprintf(
      buf, sizeof(buf),
      "QINFO tps=%lu mincmd=%u qlen=%u maxall=", (unsigned long)TICKS_PER_S,
      (unsigned)MIN_CMD_TICKS, (unsigned)QUEUE_LEN);
  uint32_t floor = 0;
  for (uint8_t i = 0; i < slot_count; i++) {
    uint32_t t = slots[i].stepper ? slots[i].stepper->getMaxSpeedInTicks() : 0;
    if (t > floor) {
      floor = t;
    }
  }
  len += snprintf(buf + len, sizeof(buf) - len, "%lu", (unsigned long)floor);
  for (uint8_t i = 0; i < slot_count; i++) {
    uint32_t t = slots[i].stepper ? slots[i].stepper->getMaxSpeedInTicks() : 0;
    int n = snprintf(buf + len, sizeof(buf) - len, " maxspeed%u=%lu",
                     (unsigned)i, (unsigned long)t);
    if (n < 0 || len + n >= (int)sizeof(buf) - 1) {
      break;
    }
    len += n;
  }
  snprintf(buf + len, sizeof(buf) - len, "\n");
  reply(buf);
}

// QSEG <steps> <ticks> <dir>            append to the shared program
// QSEG <idx> <steps> <ticks> <dir>      append to stepper idx's own program
//
// The two forms differ in argument count, so no existing command can be
// misread as the other. The 3-argument form keeps its meaning exactly: the
// shared program, walked by every stepper that has no program of its own.
static void handle_qseg(char* a1, char* a2, char* a3, char* a4) {
  struct segment* target;
  uint8_t* len;
  const char* steps_s;
  const char* ticks_s;
  const char* dir_s;

  if (a4) {
    long idx = atol(a1);
    if (idx < 0 || idx >= slot_count) {
      reply("ERR QSEG stepper out of range\n");
      return;
    }
    if (!own_used[idx]) {
      own_used[idx] = true;
      own_len[idx] = 0;
    }
    target = own_program[idx];
    len = &own_len[idx];
    steps_s = a2;
    ticks_s = a3;
    dir_s = a4;
  } else {
    target = shared_program;
    len = &shared_len;
    steps_s = a1;
    ticks_s = a2;
    dir_s = a3;
  }

  if (!steps_s || !ticks_s || !dir_s) {
    reply("ERR QSEG needs <steps> <ticks> <dir>\n");
    return;
  }
  if (*len >= QE_MAX_SEG) {
    char buf[48];
    snprintf(buf, sizeof(buf), "ERR QSEG max %u\n", (unsigned)QE_MAX_SEG);
    reply(buf);
    return;
  }

  long steps = atol(steps_s);
  long ticks = atol(ticks_s);
  long dir = atol(dir_s);
  if (steps < 0 || ticks < 1 || ticks > 65535 || (dir != 0 && dir != 1)) {
    reply("ERR QSEG steps>=0 ticks=1..65535 dir=0|1\n");
    return;
  }
  if (steps > 65535) {
    steps = 65535;  // pump splits this into <=255 step commands anyway
  }

  struct segment* seg = &target[(*len)++];
  seg->steps = (uint16_t)steps;
  seg->ticks = (uint16_t)ticks;
  seg->count_up = (dir == 1);

  char buf[48];
  snprintf(buf, sizeof(buf), "OK QSEG %u/%u\n", (unsigned)*len,
           (unsigned)QE_MAX_SEG);
  reply(buf);
}

static void handle_qrun(char* mask_text) {
  stop_sr00();
  if (slot_count == 0) {
    reply("ERR no config\n");
    return;
  }
  long mask = mask_text ? atol(mask_text) : 1;
  if (mask <= 0 || mask > 0xff) {
    reply("ERR QRUN mask=1..255\n");
    return;
  }

  // A selected stepper with no program of its own walks the shared one, so the
  // single-program scenarios are unaffected. "No program" therefore means no
  // selected stepper has anything to run -- not that the shared list is empty.
  uint8_t selected = 0;
  uint8_t runnable = 0;
  for (uint8_t i = 0; i < slot_count; i++) {
    struct qe_cursor* c = &slots[i].cur;
    memset(c, 0, sizeof(*c));
    if (!(mask & (1 << i))) {
      continue;
    }
    program_for(i, &c->list, &c->len);
    if (c->len == 0) {
      continue;
    }
    c->seg = 0;
    c->left = c->list[0].steps ? c->list[0].steps : c->list[0].ticks;
    c->active = true;
    selected++;
    runnable++;
  }
  if (selected == 0) {
    reply("ERR QRUN mask selects no stepper\n");
    return;
  }
  if (runnable == 0) {
    reply("ERR no program\n");
    return;
  }

  done_pending = false;
  done_announced = false;
  qe_pump();
  reply("OK QRUN\n");
}

static void handle_line(char* line) {
  char cmd[16] = {0};
  char arg1[32] = {0};
  // arg2 is the CONFIG driver list, one name per stepper, so it is the only
  // argument that grows with the stepper count: 4 x "i2s_direct" is 43
  // characters and truncating it to 32 would refuse a legal request with a
  // confusing "no such driver" on the last, half-cut name.
  // One byte larger than the sscanf width above: sscanf writes at most
  // `width` characters and then a NUL, so a buffer of exactly `width` overflows
  // by that terminator.
  char arg2[SALEAE_ARG2_MAX + 1] = {0};
  char arg3[32] = {0};
  char arg4[32] = {0};
  // arg2's field width comes from SALEAE_ARG2_MAX rather than a literal, so it
  // tracks the stepper count instead of drifting from it: a width shorter than
  // the buffer truncates the driver list and refuses a legal request, and one
  // longer than the buffer overflows it.
  int n = sscanf(line, "%15s %31s %" SALEAE_STR(SALEAE_ARG2_MAX) "s %31s %31s",
                 cmd, arg1, arg2, arg3, arg4);

  if (n <= 0) {
    return;
  }

  if (!strcmp(cmd, "SR00")) {
    saleae_test_setup();
    sr00_active = true;
    reply("OK SR00\n");
  } else if (!strcmp(cmd, "CONFIG")) {
    if (n < 2) {
      reply("ERR CONFIG needs <count> <driver>[,<driver>...]\n");
      return;
    }
    handle_config(arg1, arg2, n > 2 ? arg3 : NULL);
  } else if (!strcmp(cmd, "MAP")) {
    handle_map();
  } else if (!strcmp(cmd, "DRIVERS")) {
    handle_drivers();
  } else if (!strcmp(cmd, "IMUX")) {
    handle_imux(n > 1 ? arg1 : NULL, n > 2 ? arg2 : NULL, n > 3 ? arg3 : NULL);
  } else if (!strcmp(cmd, "QINFO")) {
    handle_qinfo();
  } else if (!strcmp(cmd, "QCLR")) {
    stop_all();
    stop_sr00();
    reply("OK QCLR\n");
  } else if (!strcmp(cmd, "QSEG")) {
    handle_qseg(n > 1 ? arg1 : NULL, n > 2 ? arg2 : NULL, n > 3 ? arg3 : NULL,
                n > 4 ? arg4 : NULL);
  } else if (!strcmp(cmd, "QRUN")) {
    handle_qrun(n > 1 ? arg1 : NULL);
  } else if (!strcmp(cmd, "POS")) {
    char buf[80];
    int len = snprintf(buf, sizeof(buf), "POS");
    for (uint8_t i = 0; i < slot_count; i++) {
      len += snprintf(
          buf + len, sizeof(buf) - len, " %ld",
          slots[i].stepper ? (long)slots[i].stepper->getCurrentPosition() : 0L);
    }
    snprintf(buf + len, sizeof(buf) - len, "\n");
    reply(buf);
  } else if (!strcmp(cmd, "STOP")) {
    stop_all();
    stop_sr00();
    reply("OK STOP\n");
  } else {
    reply("ERR unknown\n");
  }
}

extern "C" void saleae_app_setup(void) {
  saleae_hal_serial_begin(SALEAE_SERIAL_BAUD);
  reply("READY\n");
  saleae_test_setup();
  // SR_00 is NOT started here. It toggles eight pins at 1 Hz, so leaving it
  // running from boot fills every capture taken before the first CONFIG with
  // 1 Hz square waves -- which buries the very pulses a scenario is trying to
  // measure, and costs a second per cycle to sit through.
  //
  // The host starts it deliberately with the SR00 command when it wants the
  // channel-identification pre-check, and any CONFIG stops it. Detection runs
  // therefore see a quiet pin until the scenario under test begins.
}

extern "C" void saleae_app_loop(void) {
  int c;
  while ((c = saleae_hal_serial_read()) >= 0) {
    if (c == '\n' || c == '\r') {
      if (linelen > 0) {
        linebuf[linelen] = '\0';
        handle_line(linebuf);
        linelen = 0;
      }
    } else if (linelen < SALEAE_LINE_MAX - 1) {
      linebuf[linelen++] = (char)c;
    }
  }

  if (sr00_active) {
    saleae_test_loop();
  } else if (any_active()) {
    qe_pump();
  } else {
    saleae_hal_delay_ms(1);
  }

  // `done_announced` latches completion. Without it `qe_finish()` re-arms
  // done_pending on every idle pass through the loop, so DONE is reprinted
  // forever and floods the host: once a program ends, the queue is empty and
  // not running, which is exactly the condition that re-enters qe_finish().
  if (!done_pending && !done_announced && !sr00_active && !any_active() &&
      !any_running()) {
    qe_finish();
  }

  if (done_pending && !any_running()) {
    char buf[96];
    int len = snprintf(buf, sizeof(buf), "DONE");
    uint8_t n = slot_count ? slot_count : 1;
    for (uint8_t i = 0; i < n; i++) {
      len += snprintf(
          buf + len, sizeof(buf) - len, " %ld",
          slots[i].stepper ? (long)slots[i].stepper->getCurrentPosition() : 0L);
    }
    snprintf(buf + len, sizeof(buf) - len, "\n");
    reply(buf);
    done_pending = false;
    done_announced = true;
  }
}