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
 *   CONFIG <name> [d0,d1,...]  apply channel config; mixed takes per-stepper
 *                              drivers (rmt/mcpwm/i2s/i2s_mux/auto). 1ch and
 * 2ch exist because on AVR the achievable speed depends on the number of
 * connected steppers. QINFO                      tick rate, MIN_CMD_TICKS,
 * QUEUE_LEN and the per-stepper speed floor QCLR                      drop the
 * program and stop QSEG <steps> <ticks> <dir> append a segment; steps=0 is a
 * pause of <ticks> ticks, dir is 0 or 1 QRUN <mask>                run the
 * program on the steppers selected by the bitmask, synchronized start POS reply
 * positions of all steppers STOP                       stop move / self-test
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
#define SALEAE_LINE_MAX 64

// Bounded program: 8 segments is 8 * 6 bytes = 48 bytes of RAM, shared by all
// steppers. Every characterization scenario needs fewer.
#define QE_MAX_SEG 8

// The library cannot hand out more steppers than MAX_STEPPER, and each costs
// RAM, so never ask for more. AVR is the tight case (2 on a 328P).
#if defined(MAX_STEPPER)
#define SALEAE_MAX_STEPPERS (MAX_STEPPER < 4 ? MAX_STEPPER : 4)
#else
#define SALEAE_MAX_STEPPERS 4
#endif

// Stepper step/dir pins. On AVR the step pin MUST be the one the library maps
// to the timer compare output (stepPinStepperA/B), which depends on
// FAS_TIMER_MODULE, so use the library's own macros there instead of literals.
#if defined(ARDUINO_ARCH_ESP32)
static constexpr uint8_t kStepPins[4] = {2, 4, 17, 18};
static constexpr uint8_t kDirPins[4] = {0, 16, 5, 19};
#elif defined(ARDUINO_ARCH_AVR)
static constexpr uint8_t kStepPins[4] = {stepPinStepperA, stepPinStepperB, 0,
                                         0};
// 8 and 12. Pin 9 and 10 are the Timer1 compare outputs (OC1A/OC1B) and are
// already the step pins; 0 and 1 are the serial port; 13 is the LED. Anything
// left that is not a compare pin is fine for a plain output.
static constexpr uint8_t kDirPins[4] = {8, 12, 0, 0};
#else
static constexpr uint8_t kStepPins[4] = {2, 4, 6, 8};
static constexpr uint8_t kDirPins[4] = {3, 5, 7, 9};
#endif

// A pin cannot be a step output and a direction output at once. On AVR the step
// pins are the timer compare pins, which move with FAS_TIMER_MODULE, so this is
// checked rather than assumed -- a collision here compiles cleanly and then has
// two peripherals driving one pin.
static_assert(kDirPins[0] != kStepPins[0], "dir A collides with step A");
static_assert(kDirPins[1] != kStepPins[1], "dir B collides with step B");
static_assert(kDirPins[1] != kStepPins[0], "dir B collides with step A");
static_assert(kDirPins[0] != kStepPins[1], "dir A collides with step B");
#if defined(ARDUINO_ARCH_AVR)
static_assert(kStepPins[0] != kStepPins[1],
              "both steppers on the same timer compare pin");
#endif

// Portable driver selector (FasDriver only exists on platforms with driver
// selection, so keep our own enum and map it at connect time).
enum saleae_driver { SA_AUTO, SA_RMT, SA_MCPWM, SA_I2S, SA_I2S_MUX };

// One entry of the command program. steps == 0 means "pause for ticks ticks".
struct segment {
  uint16_t ticks;
  uint16_t steps;  // split into <=255 step commands by the pump
  bool count_up;
};

// Per-stepper cursor into the shared program. This is the only per-stepper RAM
// the harness adds: 8 bytes on AVR.
struct qe_cursor {
  uint32_t left;  // steps (or pause ticks) left in the current segment
  uint8_t seg;    // index of the current segment
  bool started;   // queue kicked off
  bool active;    // selected by QRUN and not yet finished
};

struct stepper_slot {
  FastAccelStepper* stepper;
  struct qe_cursor cur;
  uint8_t step_pin;
  uint8_t dir_pin;
  enum saleae_driver driver;
};

static FastAccelStepperEngine engine;
static bool engine_ready = false;
static struct stepper_slot slots[SALEAE_MAX_STEPPERS];
static uint8_t slot_count = 0;

static struct segment program[QE_MAX_SEG];
static uint8_t program_len = 0;

static bool sr00_active = false;
static bool done_pending = false;
// Set once DONE has been reported for the current program, so an idle loop
// does not restart the completion path. Cleared when a new program is armed.
static bool done_announced = false;
static char linebuf[SALEAE_LINE_MAX];
static uint8_t linelen = 0;

static void reply(const char* text) { saleae_hal_serial_write(text); }

static enum saleae_driver parse_driver(const char* name) {
  if (!strcmp(name, "rmt") || !strcmp(name, "rmt_v2")) return SA_RMT;
  if (!strcmp(name, "mcpwm") || !strcmp(name, "mcpwm_pcnt")) return SA_MCPWM;
  if (!strcmp(name, "i2s") || !strcmp(name, "i2s_direct")) return SA_I2S;
  if (!strcmp(name, "i2s_mux")) return SA_I2S_MUX;
  return SA_AUTO;
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
  program_len = 0;
  done_pending = false;
  done_announced = false;
}

static bool connect_stepper(uint8_t idx, uint8_t step_pin, uint8_t dir_pin,
                            enum saleae_driver driver) {
  FastAccelStepper* s;
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  FasDriver fd = DRIVER_DONT_CARE;
  switch (driver) {
    case SA_RMT:
      fd = DRIVER_RMT;
      break;
    case SA_MCPWM:
      fd = DRIVER_MCPWM_PCNT;
      break;
#if defined(DRIVER_I2S_DIRECT)
    case SA_I2S:
      fd = DRIVER_I2S_DIRECT;
      break;
    case SA_I2S_MUX:
      fd = DRIVER_I2S_MUX;
      break;
#endif
    default:
      break;
  }
  s = engine.stepperConnectToPin(step_pin, fd);
#else
  (void)driver;
  s = engine.stepperConnectToPin(step_pin);
#endif
  if (!s) {
    return false;
  }
  s->setDirectionPin(dir_pin);
  memset(&slots[idx].cur, 0, sizeof(slots[idx].cur));
  slots[idx].stepper = s;
  slots[idx].step_pin = step_pin;
  slots[idx].dir_pin = dir_pin;
  slots[idx].driver = driver;
  return true;
}

static void handle_config(char* name, char* driver_list) {
  stop_sr00();
  if (!engine_ready) {
    engine.init();
    engine_ready = true;
  }

  // Resolve the requested count *before* touching the array. On AVR
  // SALEAE_MAX_STEPPERS is 2, so filling a 4-entry config into a 2-entry array
  // would scribble past it -- and clamping afterwards is too late, the writes
  // have already happened.
  char line[64];
  uint8_t n = 0;
  enum saleae_driver drivers[SALEAE_MAX_STEPPERS];
  enum saleae_driver fill = SA_AUTO;
  uint8_t cap = SALEAE_MAX_STEPPERS;

  if (!strcmp(name, "1ch")) {
    n = 1;
  } else if (!strcmp(name, "2ch")) {
    n = 2;
  } else if (!strcmp(name, "4ch_rmt")) {
    n = 4;
    fill = SA_RMT;
  } else if (!strcmp(name, "4ch_mcpwm")) {
    n = 4;
    fill = SA_MCPWM;
  } else if (!strcmp(name, "mixed")) {
    if (!driver_list) {
      reply("ERR mixed needs driver list\n");
      return;
    }
    char* tok = strtok(driver_list, ",");
    while (tok && n < cap) {
      drivers[n++] = parse_driver(tok);
      tok = strtok(NULL, ",");
    }
    if (n == 0) {
      reply("ERR mixed needs driver list\n");
      return;
    }
  } else {
    reply("ERR unknown config\n");
    return;
  }

  // Refuse rather than silently truncate: the host asked for a configuration
  // this platform cannot provide, and quietly running fewer steppers would make
  // the capture look like a driver problem.
  if (n > cap) {
    snprintf(line, sizeof(line), "ERR %s needs %u steppers, max %u\n", name,
             (unsigned)n, (unsigned)cap);
    reply(line);
    return;
  }

  if (fill == SA_AUTO && n > 1) {
    // 1ch/2ch leave every stepper on the automatic driver choice.
    for (uint8_t i = 0; i < n; i++) drivers[i] = SA_AUTO;
  } else {
    for (uint8_t i = 0; i < n; i++) drivers[i] = fill;
  }
  if (n == 1) {
    drivers[0] = SA_AUTO;
  }

  // Each queue can only be allocated once, so a second CONFIG cannot move an
  // already-connected stepper. Report the existing setup instead of silently
  // running with a different pin map than the host thinks.
  if (slot_count > 0) {
    char buf[56];
    snprintf(buf, sizeof(buf), "OK CONFIG %s n=%u already\n", name, slot_count);
    reply(buf);
    return;
  }

  slot_count = n;
  for (uint8_t i = 0; i < n; i++) {
    if (!connect_stepper(i, kStepPins[i], kDirPins[i], drivers[i])) {
      slot_count = i;
      char buf[48];
      snprintf(buf, sizeof(buf), "ERR connect step %u n=%u\n", i, i);
      reply(buf);
      return;
    }
  }

  char buf[72];
  int len = snprintf(buf, sizeof(buf), "OK CONFIG %s n=%u", name, n);
  for (uint8_t i = 0; i < n; i++) {
    len += snprintf(buf + len, sizeof(buf) - len, " maxspeed%d=%u", i,
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
  while (c->seg < program_len && c->left == 0) {
    c->seg++;
    if (c->seg < program_len) {
      c->left =
          program[c->seg].steps ? program[c->seg].steps : program[c->seg].ticks;
    }
  }
  return c->seg < program_len;
}

// Emit commands from the plan until the queue is full or a retryable code is
// returned. Sets *err on a terminal addQueueEntry() code.
static void qe_feed(struct qe_cursor* c, FastAccelStepper* s, bool* err,
                    AqeResultCode* last) {
  while (qe_has_room(s) && qe_next_segment(c)) {
    const struct segment* seg = &program[c->seg];
    struct stepper_command_s cmd;
    bool is_pause = (seg->steps == 0);

    if (is_pause) {
      cmd.ticks = (c->left > 65535u) ? 65535u : (uint16_t)c->left;
      cmd.steps = 0;
    } else {
      cmd.ticks = seg->ticks;
      cmd.steps = (c->left > 255u) ? 255u : (uint8_t)c->left;
    }
    cmd.count_up = seg->count_up;

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
  char buf[128];
  int len = snprintf(
      buf, sizeof(buf),
      "QINFO tps=%lu mincmd=%u qlen=%u maxspeed=", (unsigned long)TICKS_PER_S,
      (unsigned)MIN_CMD_TICKS, (unsigned)QUEUE_LEN);
  for (uint8_t i = 0; i < slot_count; i++) {
    len +=
        snprintf(buf + len, sizeof(buf) - len, "%u",
                 slots[i].stepper ? slots[i].stepper->getMaxSpeedInTicks() : 0);
    if (len >= (int)sizeof(buf) - 8) {
      break;
    }
  }
  snprintf(buf + len, sizeof(buf) - len, "\n");
  reply(buf);
}

static void handle_qseg(char* a_steps, char* a_ticks, char* a_dir) {
  if (!a_steps || !a_ticks || !a_dir) {
    reply("ERR QSEG needs <steps> <ticks> <dir>\n");
    return;
  }
  if (program_len >= QE_MAX_SEG) {
    char buf[48];
    snprintf(buf, sizeof(buf), "ERR QSEG max %u\n", (unsigned)QE_MAX_SEG);
    reply(buf);
    return;
  }

  long steps = atol(a_steps);
  long ticks = atol(a_ticks);
  long dir = atol(a_dir);
  if (steps < 0 || ticks < 1 || ticks > 65535 || (dir != 0 && dir != 1)) {
    reply("ERR QSEG steps>=0 ticks=1..65535 dir=0|1\n");
    return;
  }
  if (steps > 65535) {
    steps = 65535;  // pump splits this into <=255 step commands anyway
  }

  struct segment* s = &program[program_len++];
  s->steps = (uint16_t)steps;
  s->ticks = (uint16_t)ticks;
  s->count_up = (dir == 1);

  char buf[48];
  snprintf(buf, sizeof(buf), "OK QSEG %u/%u\n", (unsigned)program_len,
           (unsigned)QE_MAX_SEG);
  reply(buf);
}

static void handle_qrun(char* mask_text) {
  stop_sr00();
  if (slot_count == 0) {
    reply("ERR no config\n");
    return;
  }
  if (program_len == 0) {
    reply("ERR no program\n");
    return;
  }

  long mask = mask_text ? atol(mask_text) : 1;
  if (mask <= 0 || mask > 0xff) {
    reply("ERR QRUN mask=1..255\n");
    return;
  }

  uint8_t selected = 0;
  for (uint8_t i = 0; i < slot_count; i++) {
    struct qe_cursor* c = &slots[i].cur;
    memset(c, 0, sizeof(*c));
    if (mask & (1 << i)) {
      c->seg = 0;
      c->left = program[0].steps ? program[0].steps : program[0].ticks;
      c->active = true;
      selected++;
    }
  }
  if (selected == 0) {
    reply("ERR QRUN mask selects no stepper\n");
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
  char arg2[32] = {0};
  char arg3[32] = {0};
  int n = sscanf(line, "%15s %31s %31s %31s", cmd, arg1, arg2, arg3);

  if (n <= 0) {
    return;
  }

  if (!strcmp(cmd, "SR00")) {
    saleae_test_setup();
    sr00_active = true;
    reply("OK SR00\n");
  } else if (!strcmp(cmd, "CONFIG")) {
    if (n < 2) {
      reply("ERR CONFIG needs <name>\n");
      return;
    }
    handle_config(arg1, arg2);
  } else if (!strcmp(cmd, "QINFO")) {
    handle_qinfo();
  } else if (!strcmp(cmd, "QCLR")) {
    stop_all();
    stop_sr00();
    reply("OK QCLR\n");
  } else if (!strcmp(cmd, "QSEG")) {
    handle_qseg(n > 1 ? arg1 : NULL, n > 2 ? arg2 : NULL, n > 3 ? arg3 : NULL);
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
  sr00_active = true;
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