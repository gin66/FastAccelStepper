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
 *                              rmt | mcpwm_pcnt | i2s_direct | i2s_mux on
 *                              the ESP32 family, timer on AVR/SAM/SAMD, pio
 *                              on Pico; a driver the running build has no
 *                              queues for is refused.
 *   DRIVERS                    which drivers this build accepts, and whether
 *                              the I2S multiplexer is up. Read from the same
 *                              conditions CONFIG parses names with, so a
 *                              capability report cannot disagree with what
 *                              CONFIG will actually do. The host keeps no
 *                              table of its own: a host-side table was wrong
 *                              about this board (6 MCPWM queues where there is
 *                              one, 32 mux queues where there are none) and a
 *                              host cannot tell that it is wrong.
 *   IMUX                      bring the I2S multiplexer up, at runtime.
 *                              initI2sMux() must precede any mux stepper and
 *                              cannot run twice, so it is a serial command
 *                              rather than a build flag: wiring a multiplexer
 *                              up is one word on existing firmware instead of
 *                              a recompile. It takes no arguments -- the bus is
 *                              the last three analyzer CHANNELS and their GPIOs
 *                              come from the channel table, so the bus wiring
 *                              has exactly one home. Refused on a non-I2S
 *                              build.
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
 *   QFILL <mask> [entries]     queue the program with start = false, `entries`
 *                              deep (default: the whole queue), and leave it
 *                              there; QRUN then starts exactly what was filled
 *                              and adds nothing afterwards, so the queue drains
 *                              exactly what this command put in it.
 *                              The reply carries the depth reached, not the one
 *                              asked for.
 *   POS                        reply positions of all steppers
 *   STOP                       stop move / self-test
 *
 * Two stops, and they differ in what happens to what is already queued:
 *   STOP                       stopMove(): decelerates, so queued motion still
 *                              runs out
 *   XSTOP                      forceStopAndNewPosition(): queue emptied, so the
 *                              queued commands never run
 *
 * The library has three stops and they differ in *how* they stop and *what
 * happens to the position*: stopMove() decelerates normally, forceStop() stops
 * abruptly but lets the queue run out (position kept), forceStopAndNewPosition()
 * stops as fast as the hardware allows and empties the queue (position lost, the
 * caller supplies it). They are not three strengths of one operation.
 *
 * STOP and XSTOP are those two extremes, and neither of the remaining ones has a
 * scenario here -- a limit of this harness, not a judgement about them. Both
 * differ from XSTOP in ways this harness cannot observe: stopMove() is a flag
 * the ramp generator reads, and this harness drives addQueueEntry() directly and
 * never runs a ramp; forceStop()'s effect on a harness-filled queue is the
 * admission latch, which refuses *later* addQueueEntry() calls, and QRUN stops
 * feeding once the fill is in, so there are none for it to refuse. Only
 * forceStopAndNewPosition() empties the queue, so only it has a waveform of its
 * own to assert.
 *
 * Characterization scenarios are assembled from segments, e.g.
 *   QCLR | QSEG 255 80 1 | QSEG 0 1600 1 | QSEG 1 80 1 | QRUN 1
 * is "255 steps at max speed, a pause, then a single step" — the case that
 * exposes MCPWM/PCNT counter-limit overrun handling.
 *
 * Strings: every literal here is in flash on AVR, not in SRAM
 * -----------------------------------------------------------
 * This file is mostly text, and on AVR a string literal is an SRAM allocation:
 * the linker script copies .rodata into RAM to initialise it at reset. Before
 * that was accounted for, .data was 1136 B — 96 B of variables and **1040 B of
 * string pool**, 51 % of a 328P's 2048 B, with 80 B left. So every literal goes
 * through SAL_PSTR() and every function that reads one uses the `_P` variant
 * (common/saleae_str.h), and a literal reply goes out through reply_p() rather
 * than reply(). The rule underneath it: a flash string is never handed to
 * anything that reads RAM — which is why driver_name() and pin_mode_name() fill
 * a caller's buffer instead of returning a pointer, since a `%s` argument has
 * to be in RAM. The `const` pin table is SAL_PROGMEM for the same reason.
 *
 * TestAvrRamBudget in scripts/tests/test_saleae.py enforces all of it, because
 * no host-side test can see this and a 328P build only fails once the part is
 * full.
 */

#include "saleae_app.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FastAccelStepper.h"
#include "FastAccelStepperEngine.h"
#include "saleae_hal.h"
#include "saleae_str.h"
#include "saleae_test.h"

#define SALEAE_SERIAL_BAUD 115200

// Analyzer channels and the pin stride, which together decide how many steppers
// fit. A stepper with a direction pin costs two channels (step + dir), a
// step-only one costs a single channel, so `dir` reaches 4 and `nodir` reaches
// 8 on the same eight channels (white paper 3.3/10.1).
#define SALEAE_CHANNELS 8
#define SALEAE_STRIDE_DIR 2
#define SALEAE_STRIDE_NODIR 1

// The I2S multiplexer takes the LAST three analyzer channels: data=D5,
// bclk=D6, ws=D7.
//
// The tail, not the head, is the load-bearing part of the choice. The
// stepper channel map is `stride * i` from channel 0 and it has to stay that
// way: `CHAN_PIN(idx * stride)` is the whole of it, and SR_00's eight pins are
// the same table in the same order. Reserving the head instead would put an
// offset in front of every index and with it a second way for the firmware and
// the host to disagree about which pin a channel is.
//
// It also keeps the surviving channels their names. With the bus on D5..D7 the
// five stepper channels are still D0..D4, so a mux capture is a superset of the
// non-mux one: a capture of an `i2s_mux` run drops the last three channels and
// gains S0..S31, and D0..D4 need no renaming on either side of that.
#define SALEAE_BUS_BASE (SALEAE_CHANNELS - 3)
#define SALEAE_BUS_COUNT 3

// Bounded program: 8 segments is 8 * 6 bytes = 48 bytes of RAM, shared by all
// steppers. Every characterization scenario needs fewer.
#define QE_MAX_SEG 8

// The I2S mux word is 32 bits wide and every frame carries all of it, so a
// multiplexed stepper costs a *slot*, not an analyzer channel: the three bus
// wires are the only ones it occupies. That is the reason a build with I2S
// addresses all 32 slots -- with the mux up, thirty-two of them are measurable
// on a rig whose steppers are the analyzer's channels, and the analyzer's own
// budget stops applying to them. The physical cap is still enforced, per pin
// mode, in handle_config().
//
// The library cannot hand out more steppers than MAX_STEPPER, and each costs
// RAM, so never ask for more. AVR is the tight case (2 on a 328P) and is
// untouched by any of this: it has no I2S.
#if defined(SUPPORT_ESP32_I2S)
#define SALEAE_STEPPER_BOUND 32
#elif defined(MAX_STEPPER)
#define SALEAE_STEPPER_BOUND (MAX_STEPPER < 8 ? MAX_STEPPER : 8)
#else
#define SALEAE_STEPPER_BOUND 8
#endif
#if defined(MAX_STEPPER) && (MAX_STEPPER < SALEAE_STEPPER_BOUND)
#define SALEAE_MAX_STEPPERS MAX_STEPPER
#else
#define SALEAE_MAX_STEPPERS SALEAE_STEPPER_BOUND
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
#elif SALEAE_MAX_STEPPERS <= 8
#define SALEAE_ARG2_MAX 96
#else
// Thirty-two steppers of the longest driver name: 32 * 11 - 1 == 351, rounded
// up. The one rung above the ladder, reached only by a build with the I2S mux,
// where the RAM for a 384-byte line buffer is not the constraint.
#define SALEAE_ARG2_MAX 384
#endif

static_assert(SALEAE_ARG2_MAX >= 12 * SALEAE_MAX_STEPPERS,
              "SALEAE_ARG2_MAX too small for the driver list");

// Enough for "CONFIG " + count + the driver list + " nodir" + slack.
#define SALEAE_LINE_MAX (SALEAE_ARG2_MAX + 32)

// Reply buffers are sized from the stepper count too, for the same RAM reason.
// avr-gcc puts the stack in .data, so an oversized local buffer is not free on
// a 328P: a flat 256-byte CONFIG reply cost 112 bytes of RAM there and pushed
// the build to 93 % of SRAM, for no benefit, since a 328P has two steppers.
//
// The numbers below were chosen when the AVR build had 80 bytes free; it now
// has 1196 (the string pool moved to flash, see the header), so they are
// conservative rather than tight. They are kept: a buffer sized for the stepper
// count is right on every target, and the alternative -- a flat size that has
// to be re-derived whenever a target gains steppers -- is how the 48-byte
// CONFIG bug above happened.
//
// The CONFIG reply is the larger of the two -- "OK CONFIG n=8 mode=nodir
// stride=1 drivers=" plus 8 driver names plus 8 "maxspeedN=<ticks>" fields
// needs about 26 bytes per stepper on top of a fixed 48. The short replies
// carry no driver names, so they get their own size.
// QINFO is the one reply that grows per stepper: a fixed prefix for
// tps/mincmd/qlen/maxall, then " maxspeedN=<ticks>" each. Sized per rung --
// 48 + 18 * SALEAE_MAX_STEPPERS, which is what the worst case needs.
//
// 18, not 16, and the difference is the stepper index: " maxspeed0=65535" is 17
// characters and " maxspeed31=65535" is 18. A per-stepper budget sized for a
// one-digit index is exact for eight steppers and short by four bytes per field
// past ten -- which a 32-stepper i2s_mux board reaches, and the reply then
// truncates mid-number and the host reports "no QINFO reply" with no hint that
// the firmware ran out of buffer.
#define QINFO_REPLY_FOR(n) (48 + 18 * (n))

#if SALEAE_MAX_STEPPERS <= 2
#define SALEAE_CFG_REPLY_MAX 96
#define SALEAE_SHORT_REPLY_MAX 48
#define SALEAE_QINFO_REPLY_MAX QINFO_REPLY_FOR(2)
#elif SALEAE_MAX_STEPPERS <= 4
#define SALEAE_CFG_REPLY_MAX 192
#define SALEAE_SHORT_REPLY_MAX 64
#define SALEAE_QINFO_REPLY_MAX QINFO_REPLY_FOR(4)
#elif SALEAE_MAX_STEPPERS <= 8
#define SALEAE_CFG_REPLY_MAX 288
#define SALEAE_SHORT_REPLY_MAX 80
#define SALEAE_QINFO_REPLY_MAX QINFO_REPLY_FOR(8)
#else
// Thirty-two steppers: the driver-name list is 32 * 12 == 384 and the
// maxspeed fields 32 * 19 == 608, so ~1 kB of reply. Only a build with the I2S
// mux reaches this and only on ESP32, where that is stack and not SRAM.
#define SALEAE_CFG_REPLY_MAX 1152
#define SALEAE_SHORT_REPLY_MAX 80
#define SALEAE_QINFO_REPLY_MAX QINFO_REPLY_FOR(32)
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
//
// One macro, not one table plus a copy: the runtime table below and the
// static_asserts beside it are both built from this initializer, so they cannot
// drift. The runtime table is PROGMEM because a `const` array is an SRAM
// allocation on AVR (saleae_str.h); the constexpr mirror is only ever read by
// the static_asserts, so the compiler emits nothing for it and it costs no
// RAM at all.
#if defined(SALEAE_TARGET_ESP32)
#define SAL_CHAN_PINS {2, 0, 4, 16, 17, 5, 18, 19}
#elif defined(ARDUINO_ARCH_AVR)
// 8 and 12 are the dir pins: pin 9 and 10 are the Timer1 compare outputs
// (OC1A/OC1B) and are already the step pins; 0 and 1 are the serial port; 13 is
// the LED. Anything left that is not a compare pin is fine for a plain output.
#define SAL_CHAN_PINS {stepPinStepperA, 8, stepPinStepperB, 12, 0, 0, 0, 0}
#else
// Neither an ESP32 nor an AVR: a plain contiguous range that avoids the UART
// pins. The ESP32 map above is chosen by SALEAE_TARGET_ESP32 rather than
// ARDUINO_ARCH_ESP32 because it describes the wiring, which is the same for
// every SDK variant -- and GPIO3 in this fallback is UART0's RX, which is what
// a plain ESP-IDF build used to land on here (see saleae_hal.h).
#define SAL_CHAN_PINS {2, 3, 4, 5, 6, 7, 8, 9}
#endif

static const uint8_t kChanPin[SALEAE_CHANNELS] SAL_PROGMEM = SAL_CHAN_PINS;
static constexpr uint8_t kChanPinConst[SALEAE_CHANNELS] = SAL_CHAN_PINS;

// The runtime read. `static_assert` cannot use this (it is not a constant
// expression), which is exactly why kChanPinConst exists.
#define CHAN_PIN(i) (sal_pgm_read_byte(&kChanPin[i]))

// A pin cannot be a step output and a direction output at once, and two
// steppers cannot share a pin. On AVR the step pins are the timer compare pins,
// which move with FAS_TIMER_MODULE, so this is checked rather than assumed -- a
// collision here compiles cleanly and then has two peripherals driving one pin.
//
// `nodir` needs one channel per PHYSICAL stepper, so SALEAE_MAX_STEPPERS must
// fit in the channel budget on a build without the mux: that is the binding
// limit for the 8-stepper case, ahead of any driver queue count. (A driver may
// bind first -- MCPWM/PCNT has 6 queues on IDF 5 -- and CONFIG reports the
// refusal per stepper rather than guessing.)
//
// The bound is deliberately absent for the I2S mux, which is the one case that
// does not fit: a mux stepper costs a slot of the 32-bit word rather than a
// channel, so 32 of them are addressable on five remaining channels. What
// replaces it is the word width itself, asserted below, and the physical half
// of the budget in handle_config().
#if !defined(SUPPORT_ESP32_I2S)
static_assert(SALEAE_MAX_STEPPERS <= SALEAE_CHANNELS,
              "SALEAE_MAX_STEPPERS does not fit one channel per stepper");
static_assert(SALEAE_MAX_STEPPERS <= 32,
              "the QRUN/QFILL bitmask is 32 bits wide");
#else
// One slot per stepper, so no run can ask for more slots than the word carries.
static_assert(SALEAE_STEPPER_BOUND <= 32,
              "a mux stepper needs one bit of the 32-bit _mux_state word");
#endif

// The bus occupies the top three channels, so the physical budget is the rest.
// Asserted rather than assumed because SALEAE_MAX_STEPPERS no longer implies
// it: the channel cap is a separate fact now.
static_assert(SALEAE_BUS_BASE + SALEAE_BUS_COUNT == SALEAE_CHANNELS,
              "the I2S bus must be the LAST three analyzer channels");

#if defined(ARDUINO_ARCH_AVR)
// Only channels 0..3 are reachable (MAX_STEPPER is 2); the rest are inert.
static_assert(kChanPinConst[0] != kChanPinConst[1],
              "step A collides with dir A");
static_assert(kChanPinConst[2] != kChanPinConst[3],
              "step B collides with dir B");
static_assert(kChanPinConst[0] != kChanPinConst[2],
              "both steppers on one compare pin");
#else
static_assert(kChanPinConst[0] != kChanPinConst[1],
              "step A collides with dir A");
static_assert(kChanPinConst[2] != kChanPinConst[3],
              "step B collides with dir B");
static_assert(kChanPinConst[0] != kChanPinConst[2],
              "step A collides with step B");
static_assert(kChanPinConst[4] != kChanPinConst[6],
              "step C collides with step D");
static_assert(kChanPinConst[1] != kChanPinConst[7],
              "dir A collides with dir D");
static_assert(kChanPinConst[5] != kChanPinConst[7],
              "dir C collides with dir D");
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
  // Armed by QFILL and waiting for QRUN: the queue is already full and nothing
  // may be added to it in the meantime, which is the whole point of filling it
  // before the start.
  bool fill_only;
  // Set when QRUN started a QFILLed queue. From then on the feeder adds
  // nothing: the queue drains exactly what QFILL put in it and the run ends.
  //
  // This is what makes the stop scenarios measurable. With a top-up, the queue
  // is back to capacity within one main-loop pass of the start -- qe_feed()
  // loops until qe_has_room() is false -- so the steps that follow a stop are
  // always QUEUE_LEN entries' worth and say nothing about the stop. Measured:
  // 7650 steps after the stop on rmt, i2s_direct and mcpwm_pcnt alike, which
  // is (QUEUE_LEN - QE_ROOM_RESERVE) * 255 and nothing else.
  bool no_topup;
};

struct stepper_slot {
  FastAccelStepper* stepper;
  struct qe_cursor cur;
  uint8_t step_pin;
  uint8_t dir_pin;
  // The bit of the 32-bit _mux_state word this stepper's step signal is, or
  // SAL_NO_MUX_SLOT when the stepper owns a GPIO instead. Not derivable from
  // `step_pin` on the host side -- it lives inside PIN_I2S_FLAG, which the
  // channel map has no idea about -- so MAP reports it.
  uint8_t mux_slot;
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

// How many analyzer channels the steppers have actually claimed, and which bits
// of the mux word they have claimed.
//
// These are separate cursors because a multiplexed stepper costs a slot and no
// channel at all, so `slot_count * chan_stride` -- which was the whole of the
// channel accounting -- stops being either number. Both are reported by MAP, so
// the host reads the map rather than deriving one from the count.
static uint8_t chan_used = 0;
static uint32_t mux_slots_used = 0;

#define SAL_NO_MUX_SLOT 0xFF

// Set once IMUX has successfully brought the I2S multiplexer up. The engine's
// own _i2s_mux_initialized is private to StepperQueue, but this app is the one
// that calls initI2sMux(), so this app is the one that knows. Declared up here
// because the channel budget depends on it, and a "budget" that does not know
// whether the bus is on the wires is a guess.
static bool mux_ready = false;

// Analyzer channels a *physical* stepper may still claim. The bus is off the
// top of the range, so bringing it up costs three channels and nothing else
// moves -- see SALEAE_BUS_BASE.
static uint8_t channels_free(void) {
  const uint8_t total = mux_ready
                            ? (uint8_t)(SALEAE_CHANNELS - SALEAE_BUS_COUNT)
                            : SALEAE_CHANNELS;
  return (uint8_t)(total - chan_used);
}

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
// Where a reply buffer lives. The two platforms want opposite answers, for the
// same underlying reason: on both, a buffer costs RAM either way, but what is
// scarce is different.
//
// Off AVR the command loop runs on an RTOS task with a fixed stack -- 3584 B for
// app_main by default -- and a local array holds its stack slot for the WHOLE
// function, not just where it is written. `handle_config` therefore still had its
// 1152-byte SALEAE_CFG_REPLY_MAX resident while it called rmt_new_tx_channel(),
// and the overflow ran off the top of the stack into the DRAM tlsf pool, where
// it surfaced much later as a corrupted free list inside tlsf_malloc. Measured
// peak for `CONFIG 1 rmt dir` on ESP-IDF 5.5.3: 4272 B of 3584 B before, 2336 B
// after. See extras/doc/implemented/idf55_main_task_stack_overflow.md.
//
// On AVR it is the other way round: the stack is ordinary SRAM above .bss, so a
// stack local and a static cost the same *peak*. But a static is committed for
// the whole life of the program while stack space is reused by every other call
// path, and these buffers total ~640 B on a part with 2048 to begin with.
// Committing them would spend headroom this target does not have, so AVR keeps
// them automatic. `.data` stays at 40 B.
#if defined(__AVR__)
#define SAL_REPLY_BUF
#else
#define SAL_REPLY_BUF static
#endif

// uint16_t, not uint8_t.
//
// A uint8_t length wraps at 256, and the wrap does not drop the tail of a long
// line -- it restarts writing at linebuf[0] and overwrites the *head*. A
// 32-stepper CONFIG is 271 characters, so the driver list overwrote the
// "CONFIG 32 " that introduced it and the command came back as unrecognisable:
// the firmware reported the wrong half of the damage, because sscanf then read
// a token out of the middle of the argument. It went unnoticed for as long as
// no legal line reached 256 characters, which the ARG2_MAX ladder (96, i.e. 128
// with the count) guaranteed.
//
// Two bytes on a 328P buys a line buffer that can hold the longest line the
// protocol can produce, which is the whole contract.
static uint16_t linelen = 0;

// Two ways out, and which one is correct depends on where the text lives.
//
//   reply()   `text` is a RAM buffer this file built with sal_snprintf().
//   reply_p() `text` is a flash literal (SAL_PSTR on AVR).
//
// On AVR the distinction is not cosmetic: a flash string read through the RAM
// path returns whatever SRAM happens to hold at that address, so a reply that
// "works" on ESP32 prints garbage on a 328P. Every literal reply must go
// through reply_p(). See saleae_str.h.
static void reply(const char* text) { saleae_hal_serial_write(text); }

static void reply_p(const char* text) { saleae_hal_serial_write_p(text); }

// Longest driver name is "mcpwm_pcnt"/"i2s_direct" at 10, so 12 leaves a NUL
// and a byte of slack. The names are copied out of flash rather than returned
// as a pointer, because every use of one is a `%s` argument.
#define SAL_DRV_NAME_MAX 12

// Resolve one driver name. Returns false for an unknown name *and* for a real
// driver this build cannot provide, so `CONFIG 2 rmt,rmt` on an AVR board is
// refused instead of quietly running two timer queues -- a capture that ran
// something else is indistinguishable from a capture of what was asked for.
//
// On an architecture with a single native driver the list is still explicit, it
// simply repeats that one (`timer` for AVR/SAM, `pio` for Pico). Naming it
// costs nothing and keeps every result tagged with the driver that made it.
//
// One accepted name per driver, and no aliases. An earlier revision also took
// `rmt_v2`, `mcpwm` and `i2s`, which read as three more drivers and were not:
// `rmt_v2` and `rmt` are the same queue, and which RMT implementation is behind
// it (V1 on IDF4, V2 on IDF5/6) is a property of the SDK the firmware was built
// with, not a choice this protocol can make. A second spelling is one more way
// for a result to be tagged with a driver name that does not exist.
static bool parse_driver(const char* name, enum saleae_driver* out) {
  (void)out;
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  if (!sal_strcmp(name, SAL_PSTR("rmt"))) {
#if (defined(SUPPORT_ESP32_RMT_V1) || defined(SUPPORT_ESP32_RMT_V2))
    *out = SA_RMT;
    return true;
#endif
  }
  if (!sal_strcmp(name, SAL_PSTR("mcpwm_pcnt"))) {
#if defined(SUPPORT_ESP32_MCPWM_PCNT)
    *out = SA_MCPWM;
    return true;
#endif
  }
  if (!sal_strcmp(name, SAL_PSTR("i2s_direct"))) {
#if defined(SUPPORT_ESP32_I2S)
    *out = SA_I2S;
    return true;
#endif
  }
  if (!sal_strcmp(name, SAL_PSTR("i2s_mux"))) {
#if defined(SUPPORT_ESP32_I2S)
    *out = SA_I2S_MUX;
    return true;
#endif
  }
#else
  if (!sal_strcmp(name, SAL_PSTR("timer"))) {
    *out = SA_TIMER;
    return true;
  }
#if defined(ARDUINO_ARCH_RP2040) || defined(PICO_RP2040) || \
    defined(PICO_SDK_RP2350)
  if (!sal_strcmp(name, SAL_PSTR("pio"))) {
    *out = SA_PIO;
    return true;
  }
#endif
#endif
  return false;
}

// The name a connected stepper is reported under: the same single name CONFIG
// accepts. Which RMT generation is behind it (V1 on IDF4, V2 on IDF5/6) is a
// property of the SDK, which is a tag on the run, not part of the driver's name.
//
// It writes into a caller-supplied buffer rather than returning a pointer,
// because both callers feed the result to `%s` and a flash string must be
// copied into RAM before snprintf can read it (saleae_str.h).
static void driver_name(enum saleae_driver d, char* out) {
  switch (d) {
    case SA_RMT:
      sal_to_ram(out, SAL_PSTR("rmt"), SAL_DRV_NAME_MAX);
      return;
    case SA_MCPWM:
      sal_to_ram(out, SAL_PSTR("mcpwm_pcnt"), SAL_DRV_NAME_MAX);
      return;
    case SA_I2S:
      sal_to_ram(out, SAL_PSTR("i2s_direct"), SAL_DRV_NAME_MAX);
      return;
    case SA_I2S_MUX:
      sal_to_ram(out, SAL_PSTR("i2s_mux"), SAL_DRV_NAME_MAX);
      return;
    case SA_TIMER:
      sal_to_ram(out, SAL_PSTR("timer"), SAL_DRV_NAME_MAX);
      return;
    case SA_PIO:
      sal_to_ram(out, SAL_PSTR("pio"), SAL_DRV_NAME_MAX);
      return;
  }
  sal_to_ram(out, SAL_PSTR("unknown"), SAL_DRV_NAME_MAX);
}

// Likewise the pin mode, which CONFIG and MAP both report as `mode=%s`. Two
// names share a 7-byte buffer; there is nothing to be gained by returning a
// pointer to a literal nobody may dereference.
#define SAL_PIN_MODE_MAX 7

static void pin_mode_name(bool nodir, char* out) {
  // Two copies rather than SAL_PSTR(nodir ? "nodir" : "dir"): PSTR takes a
  // literal, because it declares a `static const char[]` and has to size it.
  if (nodir) {
    sal_to_ram(out, SAL_PSTR("nodir"), SAL_PIN_MODE_MAX);
  } else {
    sal_to_ram(out, SAL_PSTR("dir"), SAL_PIN_MODE_MAX);
  }
}

static void stop_sr00(void) {
  if (sr00_active) {
    saleae_test_stop();
    sr00_active = false;
  }
}

// The stops, and the difference between them. The library documents all three
// (FastAccelStepper.h). They differ along two axes -- how the stepper stops, and
// what happens to the position -- so they are not three strengths of one
// operation:
//
//   stopMove()                    decelerates normally. Just a flag the ramp
//                                 generator reads for its *next* command. It
//                                 must NOT truncate motion that is already
//                                 queued; that is its contract.
//   forceStop()                   abrupt, no deceleration, but the queue is
//                                 still processed: what is already queued runs
//                                 out and the position is kept (~20ms).
//   forceStopAndNewPosition()     as fast as the hardware allows AND empties the
//                                 queue, so the position is lost and the caller
//                                 supplies it. No further step will be issued.
//
// Only the third reaches the queue from here, and that is a limit of this
// harness rather than a judgement about the other two. stopMove() sets a flag
// nothing in this harness reads: it drives addQueueEntry() directly and runs no
// ramp for the flag to act on. forceStop() sets the queue admission latch,
// which refuses *later* addQueueEntry() calls -- and QRUN stops feeding once the
// fill is in, so there are none for it to refuse. Hence no scenario for either,
// and hence the harness's own no_topup cursor is what enforces "nothing further
// is added": the harness is the planner, so that guarantee is the harness's to
// keep.
//
// The earlier stop_all() conflated the first two: it called stopMove() *and*
// zeroed the cursor, so it behaved like a partial forceStop while reporting
// itself as a stop. Measured, that hybrid left 7655 of 20000 steps to run on
// i2s_direct and 7608 on rmt -- just under the 8160 a 32-deep queue of
// 255-step commands holds, which is to say it was the harness's own arithmetic
// and not any documented guarantee. Nothing in the library promises that
// number.

static void stop_move_only(void) {
  for (uint8_t i = 0; i < slot_count; i++) {
    if (slots[i].stepper) {
      slots[i].stepper->stopMove();
    }
  }
}

// forceStopAndNewPosition() at the current position: the only one of the three
// that empties the queue.
//
// forceStop() sets the admission latch and leaves everything queued to run out --
// measured: 4080 of 4080 steps after the marker with a filled queue, identical
// on rmt, i2s_direct and mcpwm_pcnt. forceStopAndNewPosition() goes on to
// q->forceStop(), which per driver stops the timer/channel and does
// read_idx = next_write_idx, so the ring is emptied and the queued commands
// never run. That is the difference that puts it on the wire and makes it a
// scenario; forceStop()'s is on the queue's *admission*, which this harness
// stops exercising at the moment of the stop.
//
// The position is passed through unchanged, so POS keeps reporting where the
// stepper really is: the point of this stop is the queue, not the coordinate.
static void abort_queue(void) {
  for (uint8_t i = 0; i < slot_count; i++) {
    if (slots[i].stepper) {
      slots[i].stepper->forceStopAndNewPosition(
          slots[i].stepper->getCurrentPosition());
      memset(&slots[i].cur, 0, sizeof(slots[i].cur));
    }
  }
  clear_programs();
  done_pending = false;
  done_announced = false;
}

static void stop_all(void) {
  for (uint8_t i = 0; i < slot_count; i++) {
    if (slots[i].stepper) {
      slots[i].stepper->stopMove();
      // Rearm the queue admission latch. stopMove() does not touch it, but
      // XSTOP's forceStopAndNewPosition() sets it, and addQueueEntry() then
      // refuses every command -- so a QCLR followed by QSEG/QRUN on the same
      // connection used to queue nothing. QCLR is this harness's "back to
      // idle" verb, so the rearm belongs here rather than in CONFIG: CONFIG
      // also rearms (via _initVars()), which is exactly why every scenario
      // used to pass and the one broken case was never run.
      slots[i].stepper->resumeCommands();
    }
    memset(&slots[i].cur, 0, sizeof(slots[i].cur));
  }
  clear_programs();
  done_pending = false;
  done_announced = false;
}

#if defined(SUPPORT_ESP32_I2S)
// Lowest bit of the 32-bit _mux_state word not yet claimed. A step signal and a
// direction signal compete for the same 32 bits (they are both one bit in the
// same word), so the cursor is shared and a run in `dir` spends two per stepper
// -- which is why mux `dir` tops out at 16 steppers and not 32.
static bool mux_next_slot(uint8_t* slot_out) {
  for (uint8_t s = 0; s < 32; s++) {
    if (!(mux_slots_used & (1UL << s))) {
      mux_slots_used |= (1UL << s);
      *slot_out = s;
      return true;
    }
  }
  return false;
}
#endif  // SUPPORT_ESP32_I2S

static bool connect_stepper(uint8_t idx, enum saleae_driver driver,
                            bool nodir) {
  // A multiplexed stepper spends a bit of the mux word and no analyzer channel;
  // every other driver spends `stride` channels. So the two are claimed from
  // separate budgets and the channel cursor only moves for the second kind.
  uint8_t step_slot = SAL_NO_MUX_SLOT;
  uint8_t dir_slot = SAL_NO_MUX_SLOT;
  uint8_t step_pin;
  uint8_t dir_pin = 0;
#if defined(SUPPORT_ESP32_I2S)
  if (driver == SA_I2S_MUX) {
    // PIN_I2S_FLAG is what tells the library's tryAllocateQueue() this is a mux
    // slot and not a GPIO, and the low five bits are the slot. Without it the
    // request falls through to isValidStepPin() as the plain GPIO this channel
    // happens to map to, which is how every i2s_mux CONFIG was refused with
    // "connect step 0" until the flag was set here.
    if (!mux_next_slot(&step_slot)) {
      return false;
    }
    if (!nodir && !mux_next_slot(&dir_slot)) {
      return false;
    }
    step_pin = (uint8_t)(PIN_I2S_FLAG | step_slot);
    dir_pin = nodir ? 0 : (uint8_t)(PIN_I2S_FLAG | dir_slot);
  } else
#endif
  {
    step_pin = CHAN_PIN(chan_used);
    // A step-only stepper gets no dir pin at all rather than a repeated one, so
    // setDirectionPin() is not called and nothing on that pin can be mistaken
    // for a direction. The `nodir` consequence is that count_up is always
    // driven true (see qe_feed), because there is no pin to toggle for a false.
    dir_pin = nodir ? 0 : CHAN_PIN(chan_used + 1);
  }

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
    // Give the claims back, so a refused stepper does not leave a hole that the
    // next one skips past and MAP then reports as a gap.
    if (step_slot != SAL_NO_MUX_SLOT) {
      mux_slots_used &= ~(1UL << step_slot);
    }
    if (dir_slot != SAL_NO_MUX_SLOT) {
      mux_slots_used &= ~(1UL << dir_slot);
    }
    return false;
  }
  if (!nodir) {
    s->setDirectionPin(dir_pin);
  }
  memset(&slots[idx].cur, 0, sizeof(slots[idx].cur));
  slots[idx].stepper = s;
  slots[idx].step_pin = step_pin;
  slots[idx].dir_pin = dir_pin;
  slots[idx].mux_slot = step_slot;
  slots[idx].driver = driver;
  if (step_slot == SAL_NO_MUX_SLOT) {
    chan_used = (uint8_t)(chan_used + (nodir ? 1 : 2));
  }
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
  if (!sal_strcmp(mode_text, SAL_PSTR("dir"))) {
    *nodir = false;
    *stride = SALEAE_STRIDE_DIR;
    return true;
  }
  if (!sal_strcmp(mode_text, SAL_PSTR("nodir"))) {
    *nodir = true;
    *stride = SALEAE_STRIDE_NODIR;
    return true;
  }
  return false;
}

// Both bounds, in the refusal, because "too many steppers" cannot say which one
// bit: the slot array (SALEAE_MAX_STEPPERS) is a different fact from the
// analyzer's channel budget, and a driver that binds first (MCPWM/PCNT has 6
// queues on IDF 5) is a third.
//
// Its own function rather than a buffer in handle_config, because on AVR that
// function's frame is on top of the reply buffer and a second one there cost 58
// bytes of SRAM -- measured, not estimated. The frame is transient and never
// recursed into, so the extra level costs nothing that matters.
static void reply_too_many(long count, uint8_t stride) {
  SAL_REPLY_BUF char buf[64];
  sal_snprintf(buf, sizeof(buf),
               SAL_PSTR("ERR CONFIG n=%ld max=%u slots=%u chans=%u/%u\n"),
               count, (unsigned)SALEAE_MAX_STEPPERS, (unsigned)channels_free(),
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
//
// Only the `SUPPORT_SELECT_DRIVER_TYPE` reply names drivers, so on a build
// without driver selection (AVR, and the RP2040 timer build) this is unused --
// and an unused static function is a warning, which is not allowed. The guard
// matches the sole call site in handle_drivers().
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
static bool driver_supported(enum saleae_driver driver) {
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  switch (driver) {
    case SA_RMT:
#if (defined(SUPPORT_ESP32_RMT_V1) || defined(SUPPORT_ESP32_RMT_V2))
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
#endif  // SUPPORT_SELECT_DRIVER_TYPE

// The event-marker channel: the analyzer channel index whose pin the firmware
// pulses at the instant it processes STOP, or 0xFF for none.
//
// Without it the host knows only that it *sent* STOP at some wall-clock moment,
// and has to infer the stop instant from pulses ceasing. That inference is what
// made SR_25 unanswerable: on this rig the requested capture is not the
// delivered capture, so "the pulses stopped" and "the capture ended" cannot be
// told apart. Toggling a pin the stepper does not own puts the STOP instant on
// the waveform, and the harness can then measure the only quantity that matters
// for a machine that is actually moving: how many steps and how long *after* it
// was told to stop.
//
// It has to be a channel no stepper is using, so a configuration that fills the
// analyzer (8 steppers in `nodir`) has none and MARK is refused rather than
// silently overwriting a step pin.
#define SALEAE_NO_MARKER 0xFF
static uint8_t marker_channel = SALEAE_NO_MARKER;

// Alternates the marker level, so every marked event is exactly one edge and
// the level then holds until the next one.
//
// A timed high pulse was the first idea and it needs a sub-millisecond delay
// the HAL does not have (and an ESP-IDF busy-wait would block the very loop
// that drains the queue, which is the thing being measured). Alternating needs
// no timing primitive and no width to reason about: the level persists, so the
// edge is unambiguous however long afterwards the host reads it.
static uint8_t marker_level = 0;

static void mark_event(void) {
  if (marker_channel == SALEAE_NO_MARKER) {
    return;
  }
  marker_level ^= 1;
  saleae_hal_write(CHAN_PIN(marker_channel), marker_level);
}

// MARK <ch> -- designate an analyzer channel as the event marker.
//
// Two replies and nothing else. That was once forced on it -- the first version
// formatted its own errors and took the AVR build from 1845 to 1992 of 2048
// bytes, because a string literal is an SRAM allocation there. It is no longer
// forced (the literals are in flash now), but the simplification is kept,
// because MAP's `marker=` field is the diagnosis anyway: a refused MARK leaves
// marker=255. More error text here would be text no one reads.
static void handle_mark(const char* arg) {
  if (!arg) {
    reply_p(SAL_PSTR("ERR MARK\n"));
    return;
  }
  if (arg[0] == 'n') {
    marker_channel = SALEAE_NO_MARKER;
    reply_p(SAL_PSTR("OK MARK\n"));
    return;
  }
  // Only 0..7, one digit. Not atoi: this needs a digit range, not a parser.
  if (arg[1] != '\0' || arg[0] < '0' || arg[0] > ('0' + SALEAE_CHANNELS - 1)) {
    reply_p(SAL_PSTR("ERR MARK\n"));
    return;
  }
  const int ch = arg[0] - '0';
  // A channel a stepper owns cannot be the marker: it would be unreadable
  // against that stepper's own edges. Neither can a bus channel -- the I2S bus
  // is running its own protocol on it and a marker edge there is not an edge at
  // all. That is why the bound is channels_free() rather than `used`: with the
  // mux up, three channels are claimed by nothing this stepper owns.
  if (ch < channels_free()) {
    reply_p(SAL_PSTR("ERR MARK\n"));
    return;
  }
  marker_channel = (uint8_t)ch;
  reply_p(SAL_PSTR("OK MARK\n"));
}

// Set once IMUX has successfully brought the I2S multiplexer up: see the
// declaration beside the channel budget, which is what reads it.

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
  // constant with. The ESP32 form is "OK DRIVERS mux=0 rmt=1 mcpwm_pcnt=1
  // i2s_direct=1 i2s_mux=1 mux_init=0" -- 68 bytes -- while a
  // timer build emits "OK DRIVERS mux=0 timer=1 mux_init=0", 40. avr-gcc puts
  // the stack in .data, so borrowing the CONFIG size would spend 56 bytes of a
  // 328P's 203 remaining on a reply that is half that long.
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  SAL_REPLY_BUF char buf[76];
#else
  SAL_REPLY_BUF char buf[48];
#endif
  int len = sal_snprintf(buf, sizeof(buf), SAL_PSTR("OK DRIVERS mux=%u"),
                         mux_ready ? 1 : 0);
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  len += sal_snprintf(
      buf + len, sizeof(buf) - len,
      SAL_PSTR(" rmt=%u mcpwm_pcnt=%u i2s_direct=%u i2s_mux=%u"),
      driver_supported(SA_RMT) ? 1 : 0,
      driver_supported(SA_MCPWM) ? 1 : 0, driver_supported(SA_I2S) ? 1 : 0,
      driver_supported(SA_I2S_MUX) ? 1 : 0);
#else
  len += sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR(" timer=1"));
#if defined(ARDUINO_ARCH_RP2040) || defined(PICO_RP2040) || \
    defined(PICO_SDK_RP2350)
  len += sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR(" pio=1"));
#endif
#endif
  // A mux that is compiled in but not initialised would otherwise report
  // i2s_mux=1 and then refuse every CONFIG naming it, which reads as a
  // contradiction. The two fields together say "present, not up yet".
  len += sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR(" mux_init=%u\n"),
                      mux_ready ? 1 : 0);
  reply(buf);
}

// IMUX -- bring up the I2S multiplexer at runtime.
//
// initI2sMux() must be called before any stepperConnectToPin(DRIVER_I2S_MUX),
// and it cannot be called twice. Making it a serial command rather than a
// build-time constant means wiring a multiplexer up is one word on an existing
// firmware, not a recompile -- which is what makes the mux testable at all on a
// rig whose stepper pins are the analyzer's channels.
//
// It takes no arguments, which is the point. The bus is the last three analyzer
// CHANNELS and the GPIOs behind them come out of CHAN_PIN(), so there is
// exactly one place that knows the bus wiring and the host cannot ask for a bus
// the channel map does not describe. The GPIO-to-channel table is per board and
// per cable -- the ESP32-DevKitC map is not the ESP32-S3 one -- so naming the
// pins over serial would have meant naming *this* rig's pins from a host that
// has no way to know them, and the failure would be a capture that decodes into
// the wrong 32 slots.
static void handle_imux(void) {
#if defined(SUPPORT_ESP32_I2S)
  if (mux_ready) {
    reply_p(SAL_PSTR("ERR IMUX already up (initI2sMux() cannot run twice)\n"));
    return;
  }
  const uint8_t d = CHAN_PIN(SALEAE_BUS_BASE);
  const uint8_t b = CHAN_PIN(SALEAE_BUS_BASE + 1);
  const uint8_t w = CHAN_PIN(SALEAE_BUS_BASE + 2);
  if (!engine.initI2sMux(d, b, w)) {
    reply_p(SAL_PSTR(
        "ERR IMUX initI2sMux failed (pins busy, or already initialised)"
        "\n"));
    return;
  }
  mux_ready = true;
  // Channels before pins, and both by name: `data=5` on its own is ambiguous on
  // this board, where GPIO 5 is also channel 5. The host needs the channels to
  // build a decoder config and the pins to check it against the wiring.
  SAL_REPLY_BUF char buf[80];
  sal_snprintf(buf, sizeof(buf),
               SAL_PSTR("OK IMUX ch=%u,%u,%u pin=%u,%u,%u D%d=D%d D%d=D%d "
                        "D%d=D%d\n"),
               (unsigned)SALEAE_BUS_BASE, (unsigned)(SALEAE_BUS_BASE + 1),
               (unsigned)(SALEAE_BUS_BASE + 2), (unsigned)d, (unsigned)b,
               (unsigned)w, (unsigned)SALEAE_BUS_BASE, (unsigned)d,
               (unsigned)(SALEAE_BUS_BASE + 1), (unsigned)b,
               (unsigned)(SALEAE_BUS_BASE + 2), (unsigned)w);
  reply(buf);
#else
  reply_p(SAL_PSTR("ERR IMUX needs an ESP32 I2S build\n"));
#endif
}

// Without configurable driver type there is no mux, so `bus` and `slots` do not
// exist and `ch` is the only variable part -- at most two channels on a 328P.
// The full form's buffer (SALEAE_CHANNELS*4 + SALEAE_MAX_STEPPERS*4 + 96) is
// 136 bytes there for a reply of at most 66, so this rung keeps only the three
// scalars the host needs to build a channel map and spends 32 instead. The pin
// list is what it gives up: the host keeps deriving the map from count/mode/
// stride, and only records `pins` for the report.
#if !defined(SUPPORT_SELECT_DRIVER_TYPE)
static void handle_map(void) {
  SAL_REPLY_BUF char buf[32];
  SAL_REPLY_BUF char mode[SAL_PIN_MODE_MAX];
  pin_mode_name(chan_stride == SALEAE_STRIDE_NODIR, mode);
  sal_snprintf(buf, sizeof(buf),
               SAL_PSTR("MAP count=%u mode=%s stride=%u ch=-\n"),
               (unsigned)slot_count, mode, (unsigned)chan_stride);
  reply(buf);
}
#else
static void handle_map(void) {
  // "MAP count=8 mode=nodir stride=1 ch=" plus 8 two-digit pins. Sized from the
  // channel budget for the same RAM reason as SALEAE_CFG_REPLY_MAX.
  //
  // `mode` is the one field that cannot be a literal in the format string: it
  // depends on chan_stride, and a `%s` argument has to be in RAM. Hence the
  // pin_mode_name() copy.
  //
  // `ch` lists the GPIO behind each channel the steppers claimed, and only
  // those: `chan_used`, not slot_count * stride. The two differ as soon as a
  // multiplexed stepper is in the list, and a host that read eight entries for
  // two mux steppers would be reading bus pins as step pins.
  //
  // `bus` and `slots` are the mux half of the map, and they are what a decoder
  // config is built from: which three channels carry the bus, and which bit of
  // the 32-bit word each stepper is. A mux stepper's step signal is not on a
  // channel at all, so without `slots` the host would look for stepper A's step
  // edge on a pin that carries somebody else's, find nothing, and report a
  // driver that emits nothing. `slots` uses `-` for a GPIO stepper so the field
  // lines up with the stepper letters one to one.
  SAL_REPLY_BUF char buf[SALEAE_CHANNELS * 4 + SALEAE_MAX_STEPPERS * 4 + 96];
  SAL_REPLY_BUF char mode[SAL_PIN_MODE_MAX];
  pin_mode_name(chan_stride == SALEAE_STRIDE_NODIR, mode);
  int len = sal_snprintf(buf, sizeof(buf),
                         SAL_PSTR("MAP count=%u mode=%s stride=%u ch="),
                         (unsigned)slot_count, mode, (unsigned)chan_stride);
  for (uint8_t c = 0; c < chan_used && c < SALEAE_CHANNELS; c++) {
    // Two formats rather than a `"%s%u"` with `c ? "," : ""`: that would put
    // the separator in RAM as well as flash, and the whole point of the two
    // lines below is that no string literal ends up in SRAM. It is also the
    // only way to keep the leading separator off channel 0 without a runtime
    // string.
    if (c) {
      len += sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR(",%u"),
                          (unsigned)CHAN_PIN(c));
    } else {
      len += sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR("%u"),
                          (unsigned)CHAN_PIN(c));
    }
  }
#if defined(SUPPORT_ESP32_I2S)
  len += sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR(" bus="));
  if (mux_ready) {
    len +=
        sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR("%u,%u,%u"),
                     (unsigned)SALEAE_BUS_BASE, (unsigned)(SALEAE_BUS_BASE + 1),
                     (unsigned)(SALEAE_BUS_BASE + 2));
  } else {
    len += sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR("-"));
  }
  len += sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR(" slots="));
  for (uint8_t i = 0; i < slot_count; i++) {
    if (i) {
      len += sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR(","));
    }
    if (slots[i].mux_slot == SAL_NO_MUX_SLOT) {
      len += sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR("-"));
    } else {
      len += sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR("%u"),
                          (unsigned)slots[i].mux_slot);
    }
  }
#endif
  len += sal_snprintf(
      buf + len, sizeof(buf) - len, SAL_PSTR(" marker=%u"),
      marker_channel == SALEAE_NO_MARKER ? 0xFFu : (unsigned)marker_channel);
  sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR("\n"));
  reply(buf);
}
#endif  // SUPPORT_SELECT_DRIVER_TYPE

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

  // Pin mode first: the stride it selects decides how many channels each
  // physical stepper costs, and so the channel cap below depends on it.
  bool nodir;
  uint8_t stride;
  if (!parse_pin_mode(mode_text, &nodir, &stride)) {
    reply_p(SAL_PSTR("ERR CONFIG mode dir|nodir\n"));
    return;
  }

  // Resolve the count *before* touching the array. On AVR SALEAE_MAX_STEPPERS
  // is 2, so filling a 4-entry driver list into a 2-entry array would scribble
  // past it -- and clamping afterwards is too late, the writes have already
  // happened.
  enum saleae_driver drivers[SALEAE_MAX_STEPPERS];
  uint8_t n;

  if (!count_text) {
    reply_p(SAL_PSTR("ERR CONFIG needs <n> <drv>[,<driver>]\n"));
    return;
  }

  char* end = NULL;
  long count = strtol(count_text, &end, 10);
  // A trailing character means one of the superseded preset names ("1ch",
  // "mixed") is still arriving. atol would read the leading digits and quietly
  // configure one stepper, so reject the token instead.
  if (end == count_text || *end != '\0' || count < 1) {
    reply_p(SAL_PSTR("ERR CONFIG needs <n> <drv>[,<driver>]\n"));
    return;
  }
  if (count > SALEAE_MAX_STEPPERS) {
    // The array bound, which is the only one a driver list cannot dodge: a
    // 4-entry list filled into a 2-entry array scribbles past it.
    reply_too_many(count, stride);
    return;
  }
  n = (uint8_t)count;

  if (!driver_list) {
    reply_p(SAL_PSTR("ERR CONFIG needs <n> <drv>[,<driver>]\n"));
    return;
  }

  // The delimiter is a two-byte array rather than the literal "," because on
  // AVR a string literal is an SRAM allocation -- and one comma does not earn
  // two bytes of a 328P's RAM. Char constants are immediates, so this costs
  // nothing. strtok() is kept rather than hand-rolled: its treatment of a run
  // of commas is exactly what makes "CONFIG 1 timer," and "CONFIG 1 timer" the
  // same request, and reimplementing that is where a protocol would quietly
  // change.
  const char comma[2] = {',', '\0'};
  uint8_t got = 0;
  for (char* tok = strtok(driver_list, comma); tok; tok = strtok(NULL, comma)) {
    enum saleae_driver d;
    if (!parse_driver(tok, &d)) {
      reply_p(SAL_PSTR("ERR CONFIG no such driver\n"));
      return;
    }
    if (got == n) {
      reply_p(SAL_PSTR("ERR CONFIG needs <n> <drv>[,<driver>]\n"));
      return;
    }
    drivers[got++] = d;
  }
  if (got != n) {
    reply_p(SAL_PSTR("ERR CONFIG needs <n> <drv>[,<driver>]\n"));
    return;
  }

  // The channel budget is checked here, against the drivers actually named,
  // because a multiplexed stepper spends no channel and every other one spends
  // `stride`. Checking it before the list is parsed -- as the count-vs-cap test
  // used to -- could only ask whether `count` channels fit, which is the wrong
  // question the moment one entry is `i2s_mux`, and which would refuse
  // `CONFIG 32 i2s_mux,...` on a rig that has thirty-two free slots.
  //
  // Two budgets, and the refusal names both because "too many steppers" cannot
  // say which one bit: a driver that binds first (MCPWM/PCNT has 6 queues on
  // IDF 5) is a different fact from a channel budget that runs out at 4 with
  // `dir`.
  uint8_t want_phy = 0;
#if defined(SUPPORT_ESP32_I2S)
  uint8_t want_mux = 0;
#endif
  for (uint8_t i = 0; i < n; i++) {
#if defined(SUPPORT_ESP32_I2S)
    if (drivers[i] == SA_I2S_MUX) {
      want_mux++;
      continue;
    }
#endif
    want_phy = (uint8_t)(want_phy + stride);
  }
  const uint8_t chan_cap = mux_ready
                               ? (uint8_t)(SALEAE_CHANNELS - SALEAE_BUS_COUNT)
                               : (uint8_t)SALEAE_CHANNELS;
  if (want_phy > chan_cap) {
    SAL_REPLY_BUF char buf[64];
    sal_snprintf(buf, sizeof(buf),
                 SAL_PSTR("ERR CONFIG n=%u needs %u channels, max=%u\n"),
                 (unsigned)n, (unsigned)want_phy, (unsigned)chan_cap);
    reply(buf);
    return;
  }
#if defined(SUPPORT_ESP32_I2S)
  // One slot per step, plus one more for a mux direction pin, all out of the
  // same 32 bits. This is the bound that makes mux `dir` reach 16 rather than
  // 32, and it is checked here rather than left to a silent refusal from
  // tryAllocateQueue()'s bitmask -- a refusal there names no bound.
  const uint8_t slots_wanted = (uint8_t)(want_mux * (nodir ? 1 : 2));
  if (slots_wanted > 32) {
    SAL_REPLY_BUF char buf[64];
    sal_snprintf(buf, sizeof(buf),
                 SAL_PSTR("ERR CONFIG mux n=%u needs %u slots, max=32\n"),
                 (unsigned)n, (unsigned)slots_wanted);
    reply(buf);
    return;
  }
#endif

  // Each queue can only be allocated once, so a second CONFIG cannot move an
  // already-connected stepper. Report the existing setup instead of silently
  // running with a different pin map than the host thinks.
  if (slot_count > 0) {
    // Report the existing setup rather than silently running with a different
    // pin map than the host believes. It has to include the mode and the
    // drivers, since a second CONFIG naming a different mode is exactly the
    // case where "already" would otherwise hide a disagreement.
    char buf[SALEAE_SHORT_REPLY_MAX];
    char mode[SAL_PIN_MODE_MAX];
    pin_mode_name(chan_stride == SALEAE_STRIDE_NODIR, mode);
    sal_snprintf(buf, sizeof(buf), SAL_PSTR("OK CONFIG n=%u mode=%s already\n"),
                 (unsigned)slot_count, mode);
    reply(buf);
    return;
  }

  chan_stride = stride;
  slot_count = n;
  chan_used = 0;
  mux_slots_used = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (!connect_stepper(i, drivers[i], nodir)) {
      slot_count = i;
      char buf[SALEAE_SHORT_REPLY_MAX];
      char drv[SAL_DRV_NAME_MAX];
      driver_name(drivers[i], drv);
      sal_snprintf(buf, sizeof(buf),
                   SAL_PSTR("ERR connect step %u n=%u drv=%s nodir=%u\n"), i, i,
                   drv, (unsigned)nodir);
      reply(buf);
      return;
    }
  }

  // One buffer rather than several, so the reply cannot be truncated halfway,
  // and sized by SALEAE_CFG_REPLY_MAX because on AVR it is stack.
  SAL_REPLY_BUF char buf[SALEAE_CFG_REPLY_MAX];
  // `mode` and `drv` are RAM copies of flash literals, because both are `%s`
  // arguments and snprintf reads its arguments out of RAM. See saleae_str.h.
  SAL_REPLY_BUF char mode[SAL_PIN_MODE_MAX];
  SAL_REPLY_BUF char drv[SAL_DRV_NAME_MAX];
  pin_mode_name(nodir, mode);
  // Naming the drivers that were actually connected is what lets a run be
  // checked against what the board really did -- the one thing an implicit
  // driver choice used to make impossible. The host already knows what it asked
  // for, so this is not how a result is tagged.
  int len = sal_snprintf(buf, sizeof(buf),
                         SAL_PSTR("OK CONFIG n=%u mode=%s stride=%u drivers="),
                         (unsigned)n, mode, (unsigned)stride);
  for (uint8_t i = 0; i < n; i++) {
    driver_name(slots[i].driver, drv);
    // Leading separator as its own format, so no separator literal is needed --
    // see handle_map() for why. The reply is unchanged either way.
    if (i) {
      len += sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR(",%s"), drv);
    } else {
      len += sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR("%s"), drv);
    }
  }
  for (uint8_t i = 0; i < n; i++) {
    len +=
        sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR(" maxspeed%u=%u"),
                     i, slots[i].stepper->getMaxSpeedInTicks());
  }
  sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR("\n"));
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
// returned. `cap` bounds how deep this call may fill the queue, in entries: 0
// means "as deep as qe_has_room() allows", and a nonzero value stops at that
// depth instead, which is how QFILL fills to a requested depth rather than to
// whatever the queue happens to hold. Sets *err on a terminal addQueueEntry()
// code.
static void qe_feed(struct qe_cursor* c, FastAccelStepper* s, uint8_t cap,
                    bool* err, AqeResultCode* last) {
  while (qe_has_room(s) && (cap == 0 || s->queueEntries() < cap) &&
         qe_next_segment(c)) {
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
    if (!c->active || c->started || c->fill_only) {
      continue;
    }
    bool err = false;
    AqeResultCode last = AqeResultCode::OK;
    while (slots[i].stepper->queueEntries() < QE_PREFILL) {
      qe_feed(c, slots[i].stepper, 0, &err, &last);
      if (err) {
        break;
      }
      if (!qe_next_segment(c)) {
        break;  // program exhausted; queue is as full as it will get
      }
    }
    if (err) {
      c->active = false;
      SAL_REPLY_BUF char buf[64];
      sal_snprintf(buf, sizeof(buf), SAL_PSTR("ERR QE step%u rc=%d\n"), i,
                   (int)last);
      reply(buf);
      done_pending = true;
    }
  }

  // Kick off once every participant is prefilled, so synchronizedStart() arms
  // them on one shared timer compare.
  uint8_t n = 0;
  for (uint8_t i = 0; i < slot_count; i++) {
    if (slots[i].cur.active && !slots[i].cur.started &&
        !slots[i].cur.fill_only) {
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
      sal_snprintf(buf, sizeof(buf), SAL_PSTR("ERR QE start rc=%d\n"), (int)rc);
      reply(buf);
      done_pending = true;
    }
  }

  // Top up the running queues. A queue QFILL started is deliberately left
  // alone: see `no_topup`.
  for (uint8_t i = 0; i < slot_count; i++) {
    struct qe_cursor* c = &slots[i].cur;
    if (!c->active || !c->started || c->fill_only || c->no_topup) {
      continue;
    }
    bool err = false;
    AqeResultCode last = AqeResultCode::OK;
    qe_feed(c, slots[i].stepper, 0, &err, &last);
    if (err) {
      c->active = false;
      SAL_REPLY_BUF char buf[64];
      sal_snprintf(buf, sizeof(buf), SAL_PSTR("ERR QE step%u rc=%d\n"), i,
                   (int)last);
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
  SAL_REPLY_BUF char buf[SALEAE_QINFO_REPLY_MAX];
  int len = sal_snprintf(
      buf, sizeof(buf), SAL_PSTR("QINFO tps=%lu mincmd=%u qlen=%u maxall="),
      (unsigned long)TICKS_PER_S, (unsigned)MIN_CMD_TICKS, (unsigned)QUEUE_LEN);
  uint32_t floor = 0;
  for (uint8_t i = 0; i < slot_count; i++) {
    uint32_t t = slots[i].stepper ? slots[i].stepper->getMaxSpeedInTicks() : 0;
    if (t > floor) {
      floor = t;
    }
  }
  len += sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR("%lu"),
                      (unsigned long)floor);
  for (uint8_t i = 0; i < slot_count; i++) {
    uint32_t t = slots[i].stepper ? slots[i].stepper->getMaxSpeedInTicks() : 0;
    int n =
        sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR(" maxspeed%u=%lu"),
                     (unsigned)i, (unsigned long)t);
    if (n < 0 || len + n >= (int)sizeof(buf) - 1) {
      break;
    }
    len += n;
  }
  sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR("\n"));
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
      reply_p(SAL_PSTR("ERR QSEG stepper out of range\n"));
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
    reply_p(SAL_PSTR("ERR QSEG needs <steps> <ticks> <dir>\n"));
    return;
  }
  if (*len >= QE_MAX_SEG) {
    char buf[48];
    sal_snprintf(buf, sizeof(buf), SAL_PSTR("ERR QSEG max %u\n"),
                 (unsigned)QE_MAX_SEG);
    reply(buf);
    return;
  }

  long steps = atol(steps_s);
  long ticks = atol(ticks_s);
  long dir = atol(dir_s);
  if (steps < 0 || ticks < 1 || ticks > 65535 || (dir != 0 && dir != 1)) {
    reply_p(SAL_PSTR("ERR QSEG steps>=0 ticks=1..65535 dir=0|1\n"));
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
  sal_snprintf(buf, sizeof(buf), SAL_PSTR("OK QSEG %u/%u\n"), (unsigned)*len,
               (unsigned)QE_MAX_SEG);
  reply(buf);
}

// The QRUN/QFILL stepper selector. Decimal as before, and hexadecimal as well
// because the all-32-steppers mask has no useful decimal form: `QRUN
// 0xFFFFFFFF` says "every slot" and `QRUN 4294967295` says nothing a reader can
// check by eye. A leading 0x is the whole signal, so no flag argument is needed
// and the existing decimal calls are untouched.
//
// strtoul, not atol: `atol` returns `long`, which on AVR is 16 bits, so a mask
// of more than 32767 would silently wrap into a *valid-looking* different mask
// -- QRUN 0x10001 selecting stepper B on a board that meant B and the 17th.
static uint32_t parse_mask(const char* text) {
  if (!text) {
    return 1;
  }
  const int base =
      (text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) ? 16 : 10;
  return (uint32_t)strtoul(text, NULL, base);
}

// Arm the cursors `mask` selects at the head of their programs. `fill_only`
// arms them for QFILL (queued, not started) rather than for QRUN.
#define ARM_OK 0
#define ARM_NO_CONFIG 1
#define ARM_BAD_MASK 2
#define ARM_NONE_SELECTED 3
#define ARM_NO_PROGRAM 4

static uint8_t arm_cursors(uint32_t mask, bool fill_only) {
  if (slot_count == 0) {
    return ARM_NO_CONFIG;
  }
  // 32 bits wide, because the I2S mux word is: `QRUN 0xFFFFFFFF` is the whole
  // point of the maximum-speed scenario. A build without the mux still refuses
  // anything above its own stepper count below, so widening the mask cannot
  // silently widen a run.
  if (mask == 0) {
    return ARM_BAD_MASK;
  }

  // A selected stepper with no program of its own walks the shared one, so the
  // single-program scenarios are unaffected. "No program" therefore means no
  // selected stepper has anything to run -- not that the shared list is empty.
  uint8_t selected = 0;
  uint8_t runnable = 0;
  for (uint8_t i = 0; i < slot_count; i++) {
    struct qe_cursor* c = &slots[i].cur;
    if (!fill_only && c->fill_only) {
      // QFILL already put this stepper's commands in the queue, so QRUN only
      // has to start them. Re-arming here would replay the program from its
      // first segment into a queue that already holds it, and every one of
      // those steps would come out twice.
      c->fill_only = false;
      // And nothing is added from here on: the run drains what QFILL put in and
      // stops, so the queue depth at any later instant -- including at a stop
      // -- is the depth the board reported, not a race with the feeder.
      c->no_topup = true;
      if (mask & (1UL << i)) {
        selected++;
        runnable++;
      }
      continue;
    }
    memset(c, 0, sizeof(*c));
    if (!(mask & (1UL << i))) {
      continue;
    }
    program_for(i, &c->list, &c->len);
    if (c->len == 0) {
      continue;
    }
    c->seg = 0;
    c->left = c->list[0].steps ? c->list[0].steps : c->list[0].ticks;
    c->active = true;
    c->fill_only = fill_only;
    selected++;
    runnable++;
  }
  if (selected == 0) {
    return ARM_NONE_SELECTED;
  }
  if (runnable == 0) {
    return ARM_NO_PROGRAM;
  }
  return ARM_OK;
}

// `cmd` is a flash literal (SAL_PSTR at the call site), so it is copied into
// `name` before snprintf sees it: the two errors that name the command are
// formatted, and a `%s` cannot be handed a flash address.
static void reply_arm_error(uint8_t code, const char* cmd) {
  char buf[40];
  char name[8];
  sal_to_ram(name, cmd, sizeof(name));
  switch (code) {
    case ARM_NO_CONFIG:
      reply_p(SAL_PSTR("ERR no config\n"));
      break;
    case ARM_BAD_MASK:
      sal_snprintf(buf, sizeof(buf), SAL_PSTR("ERR %s mask=1..0xFFFFFFFF\n"),
                   name);
      reply(buf);
      break;
    case ARM_NO_PROGRAM:
      reply_p(SAL_PSTR("ERR no program\n"));
      break;
    default:
      sal_snprintf(buf, sizeof(buf), SAL_PSTR("ERR %s selects no stepper\n"),
                   name);
      reply(buf);
  }
}

static void handle_qrun(char* mask_text) {
  stop_sr00();
  uint32_t mask = parse_mask(mask_text);
  uint8_t rc = arm_cursors(mask, false);
  if (rc != ARM_OK) {
    reply_arm_error(rc, SAL_PSTR("QRUN"));
    return;
  }

  done_pending = false;
  done_announced = false;
  qe_pump();
  reply_p(SAL_PSTR("OK QRUN\n"));
}

// QFILL <mask> [entries] -- queue the program with start = false, as deep as
// `entries` (default: the whole queue) and stop there. QRUN then starts exactly
// what was filled.
//
// The alternative is what QRUN alone does: prefill half the queue and top it up
// from the main loop. That leaves the queue depth at any given instant to a
// race between the loop and the drain, so a scenario that stops mid-run
// measures whatever the feeder happened to be ahead by -- which is how SR_29
// came to report a step count after the marker that was a third of the queue
// bound and said nothing about forceStop(). Filling first makes the depth at
// the stop a number the DUT reports rather than one the host infers.
//
// The reply carries the depth actually reached, not the one requested: the
// queue may be shorter than asked for (QUEUE_LEN is 16 on AVR and 32 on ESP32),
// and QE_ROOM_RESERVE keeps entries free for a driver's DIR-drain pause. The
// host asserts against the achieved depth, so a board that cannot hold the
// requested one is characterized rather than failed.
static void handle_qfill(char* mask_text, char* entries_text) {
  stop_sr00();
  uint32_t mask = parse_mask(mask_text);
  long want = entries_text ? atol(entries_text) : QUEUE_LEN;
  if (want < 1 || want > QUEUE_LEN) {
    reply_p(SAL_PSTR("ERR QFILL entries=1..\n"));
    return;
  }
  uint8_t rc = arm_cursors(mask, true);
  if (rc != ARM_OK) {
    reply_arm_error(rc, SAL_PSTR("QFILL"));
    return;
  }

  // Filled here rather than left to the main loop, because the reply has to
  // report the depth and the queue can only have drained since.
  uint8_t depth = (uint8_t)want;
  bool err = false;
  uint8_t bad = 0;
  AqeResultCode last = AqeResultCode::OK;
  for (uint8_t i = 0; i < slot_count; i++) {
    struct qe_cursor* c = &slots[i].cur;
    if (!c->active || !c->fill_only) {
      continue;
    }
    qe_feed(c, slots[i].stepper, (uint8_t)want, &err, &last);
    if (err) {
      c->active = false;
      c->fill_only = false;
      bad = i;
      break;
    }
    uint8_t n = slots[i].stepper->queueEntries();
    if (n < depth) {
      depth = n;
    }
  }
  if (err) {
    char buf[48];
    sal_snprintf(buf, sizeof(buf), SAL_PSTR("ERR QE step%u rc=%d\n"),
                 (unsigned)bad, (int)last);
    reply(buf);
    return;
  }

  char buf[32];
  sal_snprintf(buf, sizeof(buf), SAL_PSTR("OK QFILL q=%u\n"), (unsigned)depth);
  reply(buf);
}

static void handle_line(char* line) {
  SAL_REPLY_BUF char cmd[16];
  SAL_REPLY_BUF char arg1[32];
  // arg2 is the CONFIG driver list, one name per stepper, so it is the only
  // argument that grows with the stepper count: 4 x "i2s_direct" is 43
  // characters and truncating it to 32 would refuse a legal request with a
  // confusing "no such driver" on the last, half-cut name.
  // One byte larger than the width below, because a field stores at most
  // `width` characters and then a NUL.
  SAL_REPLY_BUF char arg2[SALEAE_ARG2_MAX + 1];
  SAL_REPLY_BUF char arg3[32];
  SAL_REPLY_BUF char arg4[32];
  // The widths come from the buffer sizes rather than from literals in a format
  // string, so they cannot drift apart: a width shorter than the buffer
  // truncates the driver list and refuses a legal request, and one longer than
  // the buffer overflows it.
  //
  // sal_tokenize, not sscanf: libc `sscanf` costs 1496 bytes of stack per call
  // and `snprintf` 384, against a 3584-byte main task that also has to reach
  // the driver constructors -- the overflow runs off the top of the stack into
  // the DRAM heap and surfaces much later as a corrupted free list. See
  // saleae_str.h and extras/doc/implemented/idf55_main_task_stack_overflow.md.
  const struct sal_field fields[] = {
      {cmd, sizeof(cmd) - 1},
      {arg1, sizeof(arg1) - 1},
      {arg2, sizeof(arg2) - 1},
      {arg3, sizeof(arg3) - 1},
      {arg4, sizeof(arg4) - 1},
  };
  const int n = sal_tokenize(line, fields, sizeof(fields) / sizeof(fields[0]));

  if (n <= 0) {
    return;
  }

  if (!sal_strcmp(cmd, SAL_PSTR("SR00"))) {
    saleae_test_setup();
    sr00_active = true;
    reply_p(SAL_PSTR("OK SR00\n"));
  } else if (!sal_strcmp(cmd, SAL_PSTR("CONFIG"))) {
    if (n < 2) {
      reply_p(SAL_PSTR("ERR CONFIG needs <count> <driver>[,<driver>...]\n"));
      return;
    }
    handle_config(arg1, arg2, n > 2 ? arg3 : NULL);
  } else if (!sal_strcmp(cmd, SAL_PSTR("MAP"))) {
    handle_map();
  } else if (!sal_strcmp(cmd, SAL_PSTR("MARK"))) {
    handle_mark(n > 1 ? arg1 : NULL);
  } else if (!sal_strcmp(cmd, SAL_PSTR("DRIVERS"))) {
    handle_drivers();
  } else if (!sal_strcmp(cmd, SAL_PSTR("IMUX"))) {
    handle_imux();
  } else if (!sal_strcmp(cmd, SAL_PSTR("QINFO"))) {
    handle_qinfo();
  } else if (!sal_strcmp(cmd, SAL_PSTR("QCLR"))) {
    stop_all();
    stop_sr00();
    reply_p(SAL_PSTR("OK QCLR\n"));
  } else if (!sal_strcmp(cmd, SAL_PSTR("QSEG"))) {
    handle_qseg(n > 1 ? arg1 : NULL, n > 2 ? arg2 : NULL, n > 3 ? arg3 : NULL,
                n > 4 ? arg4 : NULL);
  } else if (!sal_strcmp(cmd, SAL_PSTR("QRUN"))) {
    handle_qrun(n > 1 ? arg1 : NULL);
  } else if (!sal_strcmp(cmd, SAL_PSTR("QFILL"))) {
    handle_qfill(n > 1 ? arg1 : NULL, n > 2 ? arg2 : NULL);
  } else if (!sal_strcmp(cmd, SAL_PSTR("POS"))) {
    // One slot per stepper, and " %ld" is up to 13 bytes with a leading space
    // and a sign -- so 32 of them do not fit the 80 bytes an eight-stepper run
    // needs. Sized from the same stepper count as the other replies.
    SAL_REPLY_BUF char buf[8 * SALEAE_MAX_STEPPERS + 16];
    int len = sal_snprintf(buf, sizeof(buf), SAL_PSTR("POS"));
    for (uint8_t i = 0; i < slot_count; i++) {
      len += sal_snprintf(
          buf + len, sizeof(buf) - len, SAL_PSTR(" %ld"),
          slots[i].stepper ? (long)slots[i].stepper->getCurrentPosition() : 0L);
    }
    sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR("\n"));
    reply(buf);
  } else if (!sal_strcmp(cmd, SAL_PSTR("STOP"))) {
    // stopMove() alone. The feeder is left alone on purpose: the contract is
    // that already-queued motion still runs, so a run that keeps stepping after
    // this is the documented behaviour and is what SR_25 asserts. Marked on the
    // marker channel so that boundary is on the waveform rather than inferred.
    stop_move_only();
    stop_sr00();
    mark_event();
    reply_p(SAL_PSTR("OK STOP stopmove\n"));
  } else if (!sal_strcmp(cmd, SAL_PSTR("XSTOP"))) {
    // forceStopAndNewPosition(): stop adding *and* empty the queue. The only
    // one of the three stops that the queued commands do not survive.
    abort_queue();
    stop_sr00();
    mark_event();
    reply_p(SAL_PSTR("OK XSTOP abortqueue\n"));
  } else {
    // Echo the token back. "ERR unknown" on its own cannot tell a typo from a
    // line that arrived damaged -- a serial protocol that drops bytes reports
    // the *command* as unrecognisable when it was the argument that was lost,
    // and that was exactly the 32-stepper CONFIG driver list, arriving one
    // buffer too long. `cmd` is a RAM copy of the first token, so it costs
    // nothing to print.
    char buf[40];
    sal_snprintf(buf, sizeof(buf), SAL_PSTR("ERR unknown '%s'\n"), cmd);
    reply(buf);
  }
}

extern "C" void saleae_app_setup(void) {
  saleae_hal_serial_begin(SALEAE_SERIAL_BAUD);
  reply_p(SAL_PSTR("READY\n"));
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
    saleae_hal_idle();
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
    SAL_REPLY_BUF char buf[96];
    int len = sal_snprintf(buf, sizeof(buf), SAL_PSTR("DONE"));
    uint8_t n = slot_count ? slot_count : 1;
    for (uint8_t i = 0; i < n; i++) {
      len += sal_snprintf(
          buf + len, sizeof(buf) - len, SAL_PSTR(" %ld"),
          slots[i].stepper ? (long)slots[i].stepper->getCurrentPosition() : 0L);
    }
    sal_snprintf(buf + len, sizeof(buf) - len, SAL_PSTR("\n"));
    reply(buf);
    done_pending = false;
    done_announced = true;
  }
}