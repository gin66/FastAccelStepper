/*
 * saleae_app.cpp — Host command channel and test scenarios.
 *
 * A tiny newline-terminated text protocol over the serial console:
 *
 *   SR00                     SR_00 connection self-test (8 pins, 1 Hz, dut)
 *   SR01 <steps> <speed_us>  constant-speed move, reply "DONE <pos>" when done
 *   POS                      reply "POS <position>"
 *   STOP                     stop any move / self-test
 *
 * On boot SR_00 runs so a freshly flashed board proves all I/Os are alive.
 * SR_00 and the stepper share pins, so SR01/STOP stop the self-test first.
 */

#include "saleae_app.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FastAccelStepper.h"
#include "saleae_hal.h"
#include "saleae_test.h"

#define SALEAE_STEP_PIN 2  // CH0 / GPIO 2
#define SALEAE_DIR_PIN 0   // CH1 / GPIO 0
#define SALEAE_SERIAL_BAUD 115200
#define SALEAE_LINE_MAX 64

static FastAccelStepperEngine engine;
static FastAccelStepper *stepper = NULL;
static bool sr00_active = false;
static bool done_pending = false;
static char linebuf[SALEAE_LINE_MAX];
static uint8_t linelen = 0;

static void reply(const char *text) { saleae_hal_serial_write(text); }

static void ensure_stepper(void) {
  if (stepper) {
    return;
  }
  engine.init();
  stepper = engine.stepperConnectToPin(SALEAE_STEP_PIN);
  if (stepper) {
    stepper->setDirectionPin(SALEAE_DIR_PIN);
    stepper->setAcceleration(1000);
  }
}

static void stop_sr00(void) {
  if (sr00_active) {
    saleae_test_stop();
    sr00_active = false;
  }
}

static void handle_line(char *line) {
  char cmd[16] = {0};
  long steps = 0;
  long speed_us = 0;
  int n = sscanf(line, "%15s %ld %ld", cmd, &steps, &speed_us);

  if (n <= 0) {
    return;
  }
  if (strcmp(cmd, "SR00") == 0) {
    saleae_test_setup();
    sr00_active = true;
    reply("OK SR00\n");
  } else if (strcmp(cmd, "SR01") == 0) {
    if (n < 3) {
      reply("ERR SR01 needs <steps> <speed_us>\n");
      return;
    }
    stop_sr00();
    ensure_stepper();
    if (!stepper) {
      reply("ERR no stepper\n");
      return;
    }
    stepper->setSpeedInUs((uint32_t)speed_us);
    stepper->move((int32_t)steps);
    done_pending = true;
    char buf[48];
    snprintf(buf, sizeof(buf), "OK SR01 %ld\n", steps);
    reply(buf);
  } else if (strcmp(cmd, "POS") == 0) {
    char buf[48];
    snprintf(buf, sizeof(buf), "POS %ld\n",
             stepper ? (long)stepper->getCurrentPosition() : 0L);
    reply(buf);
  } else if (strcmp(cmd, "STOP") == 0) {
    if (stepper) {
      stepper->stopMove();
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
  // Prove all I/Os on boot; a STOP or any stepper command takes over.
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

  if (done_pending && stepper && !stepper->isRunning()) {
    char buf[48];
    snprintf(buf, sizeof(buf), "DONE %ld\n",
             (long)stepper->getCurrentPosition());
    reply(buf);
    done_pending = false;
  }
}
