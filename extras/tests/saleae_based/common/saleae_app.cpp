/*
 * saleae_app.cpp — Host command channel, channel configs and test scenarios.
 *
 * Newline-terminated text protocol over the serial console:
 *
 *   SR00                       SR_00 connection self-test (8 pins, 1 Hz)
 *   CONFIG <name> [d0,d1,...]  apply channel config; mixed takes per-stepper
 *                              drivers (rmt/mcpwm/i2s/i2s_mux/auto)
 *   SR01 <steps> <speed_us>    move stepper A only
 *   MOVEALL <steps> <speed_us> move every configured stepper
 *   POS                        reply positions of all steppers
 *   STOP                       stop move / self-test
 *
 * Channel configs (white paper §3.2/§3.3): 4ch_rmt, 4ch_mcpwm and mixed
 * (per-stepper driver selection). SR_00 and the steppers share pins, so any
 * stepper command stops the self-test first.
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
#define SALEAE_LINE_MAX 96
#define SALEAE_MAX_STEPPERS 4

// Stepper step/dir pins (white paper §3.3). AVR uses a timer-supported pin.
#if defined(ARDUINO_ARCH_ESP32)
static const uint8_t kStepPins[SALEAE_MAX_STEPPERS] = {2, 4, 17, 18};
static const uint8_t kDirPins[SALEAE_MAX_STEPPERS] = {0, 16, 5, 19};
#elif defined(ARDUINO_ARCH_AVR)
static const uint8_t kStepPins[SALEAE_MAX_STEPPERS] = {9, 10, 11, 12};
static const uint8_t kDirPins[SALEAE_MAX_STEPPERS] = {8, 7, 6, 5};
#else
static const uint8_t kStepPins[SALEAE_MAX_STEPPERS] = {2, 4, 6, 8};
static const uint8_t kDirPins[SALEAE_MAX_STEPPERS] = {3, 5, 7, 9};
#endif

// Portable driver selector (FasDriver only exists on platforms with driver
// selection, so keep our own enum and map it at connect time).
enum saleae_driver { SA_AUTO, SA_RMT, SA_MCPWM, SA_I2S, SA_I2S_MUX };

struct stepper_slot {
  FastAccelStepper *stepper;
  uint8_t step_pin;
  uint8_t dir_pin;
  enum saleae_driver driver;
};

static FastAccelStepperEngine engine;
static bool engine_ready = false;
static stepper_slot slots[SALEAE_MAX_STEPPERS];
static uint8_t slot_count = 0;

static bool sr00_active = false;
static bool done_pending = false;
static char linebuf[SALEAE_LINE_MAX];
static uint8_t linelen = 0;

static void reply(const char *text) { saleae_hal_serial_write(text); }

static enum saleae_driver parse_driver(const char *name) {
  if (!strcmp(name, "rmt") || !strcmp(name, "rmt_v2")) return SA_RMT;
  if (!strcmp(name, "mcpwm") || !strcmp(name, "mcpwm_pcnt")) return SA_MCPWM;
  if (!strcmp(name, "i2s") || !strcmp(name, "i2s_direct")) return SA_I2S;
  if (!strcmp(name, "i2s_mux")) return SA_I2S_MUX;
  return SA_AUTO;
}

static const char *driver_name(enum saleae_driver d) {
  switch (d) {
    case SA_RMT: return "rmt";
    case SA_MCPWM: return "mcpwm";
    case SA_I2S: return "i2s";
    case SA_I2S_MUX: return "i2s_mux";
    default: return "auto";
  }
}

static void stop_sr00(void) {
  if (sr00_active) {
    saleae_test_stop();
    sr00_active = false;
  }
}

static bool connect_stepper(uint8_t idx, uint8_t step_pin, uint8_t dir_pin,
                            enum saleae_driver driver) {
  FastAccelStepper *s;
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  FasDriver fd = DRIVER_DONT_CARE;
  switch (driver) {
    case SA_RMT: fd = DRIVER_RMT; break;
    case SA_MCPWM: fd = DRIVER_MCPWM_PCNT; break;
#if defined(DRIVER_I2S_DIRECT)
    case SA_I2S: fd = DRIVER_I2S_DIRECT; break;
    case SA_I2S_MUX: fd = DRIVER_I2S_MUX; break;
#endif
    default: break;
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
  s->setAcceleration(1000);
  slots[idx].stepper = s;
  slots[idx].step_pin = step_pin;
  slots[idx].dir_pin = dir_pin;
  slots[idx].driver = driver;
  return true;
}

static void handle_config(char *name, char *driver_list) {
  stop_sr00();
  if (!engine_ready) {
    engine.init();
    engine_ready = true;
  }

  uint8_t n = 0;
  enum saleae_driver drivers[SALEAE_MAX_STEPPERS];

  if (!strcmp(name, "4ch_rmt")) {
    n = 4;
    for (uint8_t i = 0; i < n; i++) drivers[i] = SA_RMT;
  } else if (!strcmp(name, "4ch_mcpwm")) {
    n = 4;
    for (uint8_t i = 0; i < n; i++) drivers[i] = SA_MCPWM;
  } else if (!strcmp(name, "mixed")) {
    char *tok = strtok(driver_list, ",");
    while (tok && n < SALEAE_MAX_STEPPERS) {
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

  slot_count = n;
  for (uint8_t i = 0; i < n; i++) {
    if (!connect_stepper(i, kStepPins[i], kDirPins[i], drivers[i])) {
      slot_count = i;
      reply("ERR stepper connect failed\n");
      return;
    }
  }

  char buf[64];
  snprintf(buf, sizeof(buf), "OK CONFIG %s n=%u\n", name, n);
  reply(buf);
}

static void handle_move(long steps, long speed_us, bool all) {
  stop_sr00();
  if (slot_count == 0) {
    reply("ERR no config\n");
    return;
  }
  // Ramp up in ~0.1 s so the move is effectively constant speed (white paper
  // test_01). accel [steps/s^2] = 10 * target rate [steps/s].
  uint32_t accel = (uint32_t)(10ULL * 1000000ULL / (uint32_t)speed_us);
  uint8_t n = all ? slot_count : 1;
  for (uint8_t i = 0; i < n; i++) {
    slots[i].stepper->setSpeedInUs((uint32_t)speed_us);
    slots[i].stepper->setAcceleration(accel);
    slots[i].stepper->move((int32_t)steps);
  }
  done_pending = true;
  reply(all ? "OK MOVEALL\n" : "OK SR01\n");
}

static bool any_running(void) {
  for (uint8_t i = 0; i < slot_count; i++) {
    if (slots[i].stepper && slots[i].stepper->isRunning()) return true;
  }
  return false;
}

static void handle_line(char *line) {
  char cmd[16] = {0};
  char arg1[32] = {0};
  char arg2[64] = {0};
  int n = sscanf(line, "%15s %31s %63s", cmd, arg1, arg2);

  if (n <= 0) return;

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
  } else if (!strcmp(cmd, "SR01") || !strcmp(cmd, "MOVEALL")) {
    char v1[16], v2[16];
    long steps = 0, speed = 0;
    if (sscanf(line, "%15s %15s %15s", cmd, v1, v2) == 3) {
      steps = atol(v1);
      speed = atol(v2);
    }
    if (speed <= 0 || steps == 0) {
      reply("ERR needs <steps> <speed_us>\n");
      return;
    }
    handle_move(steps, speed, !strcmp(cmd, "MOVEALL"));
  } else if (!strcmp(cmd, "POS")) {
    char buf[80];
    int len = snprintf(buf, sizeof(buf), "POS");
    for (uint8_t i = 0; i < slot_count; i++) {
      len += snprintf(buf + len, sizeof(buf) - len, " %ld",
                      slots[i].stepper
                          ? (long)slots[i].stepper->getCurrentPosition()
                          : 0L);
    }
    snprintf(buf + len, sizeof(buf) - len, "\n");
    reply(buf);
  } else if (!strcmp(cmd, "STOP")) {
    for (uint8_t i = 0; i < slot_count; i++) {
      if (slots[i].stepper) slots[i].stepper->stopMove();
    }
    done_pending = false;
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
  } else {
    saleae_hal_delay_ms(1);
  }

  if (done_pending && !any_running()) {
    char buf[96];
    int len = snprintf(buf, sizeof(buf), "DONE");
    uint8_t n = slot_count ? slot_count : 1;
    for (uint8_t i = 0; i < n; i++) {
      len += snprintf(buf + len, sizeof(buf) - len, " %ld",
                      slots[i].stepper
                          ? (long)slots[i].stepper->getCurrentPosition()
                          : 0L);
    }
    snprintf(buf + len, sizeof(buf) - len, "\n");
    reply(buf);
    done_pending = false;
  }
}
