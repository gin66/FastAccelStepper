// ============================================================================
// replay-stepper — replay a REAL sampled stroke 1:1 on a bench ESP32
//
// Some miscounts only show up with a specific command stream (e.g. a wave
// drawn by the full dot-bot pipeline) and not with debug-stepper's synthetic
// diamond profile. This sketch replays the exact motion commands the frontend
// generated for one motor, through the same moveTimed streaming path as the
// station-worker, with the same counters as debug-stepper.
//
// Getting a pattern in here:
//   1. Frontend: draw/generate the stroke, then in the browser console run
//        exportMotionCommands()
//      -> downloads motion-commands.json (the full command stream).
//   2. node commands-to-pattern.mjs motion-commands.json left
//      (pick the role whose motor shows the miscount)
//   3. Compile & flash this sketch.
//
// Backend selection defaults to the station-worker behavior:
// stepperConnectToPin() without a driver argument, so the allocator's default
// applies (RMT on IDF5, MCPWM/PCNT first on IDF4 classic ESP32). The 'b'
// serial command switches directly to the OTHER backend (RMT <-> MCPWM); the
// choice is kept in RTC memory and applied after an automatic reboot. The
// actually chosen backend is printed at startup.
//
// Serial usage (115200 baud):
//   r        run 1 pass: pattern forward, then mirrored back (net 0)
//   rN       run N such passes (e.g. r10)
//   o        run the pattern one-way only (ends REPLAY_PATTERN_NET_STEPS away)
//   s / sN   chunked one-way run: stop and compare all counters at standstill
//            every N commands (default 25) to localize WHERE steps get lost
//   p        print counters (library pos, commanded sum, PCNT)
//   z        zero all counters (align MotionStudio first)
//   d        toggle DIR delay 0us <-> 200us
//   b        switch to the other driver backend (RMT <-> MCPWM) + reboot
//   h        help
// ============================================================================

#include "FastAccelStepper.h"
#include "replay-pattern.h"

#include <cstdlib>
#include <cstring>
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_idf_version.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if defined(SUPPORT_ESP32_PULSE_COUNTER) && defined(SUPPORT_SELECT_DRIVER_TYPE)

// --- Serial helpers: ESP-IDF UART driver on the console UART (115200 baud) --
static void serialInit() {
  uart_config_t uart_config;
  memset(&uart_config, 0, sizeof(uart_config));
  uart_config.baud_rate = 115200;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  uart_config.source_clk = UART_SCLK_DEFAULT;
#else
  uart_config.source_clk = UART_SCLK_APB;
#endif
  uart_param_config(UART_NUM_0, &uart_config);
  uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
}

static int serialAvailable() {
  size_t len = 0;
  uart_get_buffered_data_len(UART_NUM_0, &len);
  return (int)len;
}

static int serialRead() {
  uint8_t ch = 0;
  int n = uart_read_bytes(UART_NUM_0, &ch, 1, 0);
  return (n == 1) ? (int)ch : -1;
}

static void printCoreVersions() {
  printf("ESP-IDF: %u.%u.%u (%s)\n", ESP_IDF_VERSION_MAJOR,
         ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH, esp_get_idf_version());
}

// --- Pin configuration: identical to station-worker ---
#define STEP_PIN 19
#define DIR_PIN 18
#define ENABLE_PIN 23

// Direction convention: RIGHT/CENTER stations use true, LEFT uses false
// (station-worker: dirHighCountsUp = (side == RIGHT || side == CENTER)).
// IMPORTANT: when replaying a `left` pattern, set this to false — the
// miscount is polarity-coupled, so the DIR polarity can change which
// reversals trigger it.
#define DIR_HIGH_COUNTS_UP false

#define DIR_CHANGE_DELAY_US 200
static uint16_t dirChangeDelayUs = DIR_CHANGE_DELAY_US;

// Driver backend selection. The backend is fixed at stepperConnectToPin()
// time, so the 'b' serial command stores the next choice in RTC memory
// (survives a software reset) and reboots. There are only two real
// backends, so 'b' is a toggle: it always switches to whichever backend is
// NOT currently active. A cold boot (power cycle) starts in worker-like
// default mode, i.e. the allocator picks (RMT on IDF5, MCPWM on IDF4) —
// that already counts as the backend it resolved to.
#define BACKEND_DEFAULT 0
#define BACKEND_RMT 1
#define BACKEND_MCPWM 2
#define BACKEND_MAGIC 0xB0F1D0A7u

RTC_NOINIT_ATTR uint32_t backendMagic;
RTC_NOINIT_ATTR uint32_t backendChoice;

static uint8_t currentBackendChoice() {
  if (backendMagic == BACKEND_MAGIC && backendChoice <= BACKEND_MCPWM) {
    return (uint8_t)backendChoice;
  }
  return BACKEND_DEFAULT;
}

static const char* backendChoiceName(uint8_t choice) {
  switch (choice) {
    case BACKEND_RMT:
      return "force RMT";
    case BACKEND_MCPWM:
      return "force MCPWM/PCNT";
    default:
      return "worker-like default";
  }
}

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* stepper = NULL;

// The backend 'b' switches to: the one that is not active right now. The
// actually allocated driver decides (covers default mode); if allocation
// failed, fall back to the stored choice so 'b' can still rescue.
static uint8_t nextBackendChoice() {
  if (stepper != NULL) {
    return (strcmp(stepper->driverTypeString(), "RMT") == 0) ? BACKEND_MCPWM
                                                             : BACKEND_RMT;
  }
  return (currentBackendChoice() == BACKEND_RMT) ? BACKEND_MCPWM : BACKEND_RMT;
}

static void switchBackendAndReboot() {
  uint8_t next = nextBackendChoice();
  backendMagic = BACKEND_MAGIC;
  backendChoice = next;
  printf("Backend -> %s, rebooting...\n", backendChoiceName(next));
  vTaskDelay(pdMS_TO_TICKS(100));
  esp_restart();
}

// ---------------------------------------------------------------------------
// Counters (same scheme as debug-stepper)
// ---------------------------------------------------------------------------
static int64_t commandedSum = 0;  // what we asked the library to do
static int64_t pcntBase = 0;      // PCNT accumulator (16-bit hw counter)
static int16_t lastPcntRaw = 0;
static bool pcntRangeWarned = false;

static int64_t readPcntAccum() {
  int16_t raw = stepper->readPulseCounter();
  int16_t delta = raw - lastPcntRaw;
  lastPcntRaw = raw;
  pcntBase += delta;
  // The hw window is +/-16384. We sample every command (max ~a few hundred
  // steps apart), so deltas are exact as long as the raw value stays inside
  // the window between samples. Warn when a replayed stroke gets close.
  if (!pcntRangeWarned && (raw > 15000 || raw < -15000)) {
    pcntRangeWarned = true;
    printf(
        "WARN: PCNT raw near +/-16384 window edge - PCNT value may "
        "wrap; trust the driver counter (MotionStudio) for this run\n");
  }
  return pcntBase;
}

// ---------------------------------------------------------------------------
// Streaming through the same moveTimed path as the worker firmware
// ---------------------------------------------------------------------------
static bool feedCommand(int16_t steps, uint32_t ticks, int32_t& drift,
                        bool start = false) {
  uint32_t duration = ticks + drift;
  uint32_t actual = 0;
  for (;;) {
    MoveTimedResultCode rc =
        stepper->moveTimed(steps, duration, &actual, start);
    switch (rc) {
      case MOVE_TIMED_OK:
      case MOVE_TIMED_EMPTY:
        drift = (int32_t)(duration - actual);
        commandedSum += steps;
        readPcntAccum();  // keep the accumulator inside the hw window
        return true;
      case MOVE_TIMED_BUSY:
      case MoveTimedResultCode::QueueFull:
      case MoveTimedResultCode::DirPinIsBusy:
      case MoveTimedResultCode::DirPin2msPauseAdded:
      case MoveTimedResultCode::DirChangePauseInjected:
      case MoveTimedResultCode::WaitForEnablePinActive:
        // Positive codes = retry; DirChangePauseInjected means the pauses
        // were queued but the command was NOT - resend it (pre-1.3.0).
        esp_rom_delay_us(200);
        break;
      case MoveTimedResultCode::DeviceNotReady:
        esp_rom_delay_us(10);
        break;
      default:
        printf("moveTimed error: %s (steps=%d dur=%lu)\n", toString(rc), steps,
               (unsigned long)duration);
        return false;
    }
  }
}

// Fetch command i of a pass: forward half replays the pattern as sampled,
// mirrored half runs it backwards with negated steps so a pass nets to 0.
static void passCommand(uint32_t i, bool mirrored, int16_t& steps,
                        uint32_t& ticks) {
  if (!mirrored) {
    steps = REPLAY_PATTERN[i].steps;
    ticks = REPLAY_PATTERN[i].ticks;
  } else {
    uint32_t j = (REPLAY_PATTERN_LEN - 1) - i;
    steps = (int16_t)(-REPLAY_PATTERN[j].steps);
    ticks = REPLAY_PATTERN[j].ticks;
  }
}

static void runPasses(uint16_t passes, bool oneWay) {
  int32_t startPos = stepper->getCurrentPosition();
  int64_t startPcnt = readPcntAccum();
  int64_t startCmd = commandedSum;
  pcntRangeWarned = false;

  uint32_t cmdsPerPass = oneWay ? REPLAY_PATTERN_LEN : 2u * REPLAY_PATTERN_LEN;
  printf("Replaying %u pass(es), %lu commands each (%s)...\n", passes,
         (unsigned long)cmdsPerPass,
         oneWay ? "one-way" : "forward + mirrored return");

  int32_t drift = 0;
  bool started = false;
  uint16_t prefilled = 0;

  for (uint16_t c = 0; c < passes; c++) {
    for (uint32_t i = 0; i < cmdsPerPass; i++) {
      int16_t steps;
      uint32_t ticks;
      passCommand(i % REPLAY_PATTERN_LEN, !oneWay && i >= REPLAY_PATTERN_LEN,
                  steps, ticks);

      // Prefill without starting, exactly like the worker's startExecution()
      if (!started) {
        bool enqueued = false;
        while (!enqueued) {
          uint32_t actual = 0;
          MoveTimedResultCode rc =
              stepper->moveTimed(steps, ticks, &actual, false);
          switch (rc) {
            case MOVE_TIMED_OK:
            case MOVE_TIMED_EMPTY:
              drift = (int32_t)(ticks - actual);
              commandedSum += steps;
              prefilled++;
              enqueued = true;
              break;
            case MoveTimedResultCode::DirChangePauseInjected:
            case MoveTimedResultCode::DirPin2msPauseAdded:
              esp_rom_delay_us(200);  // pauses queued, command not: retry
              break;
            default:
              // queue full -> start it and fall through to normal feeding
              stepper->moveTimed(0, 0, NULL, true);
              started = true;
              enqueued = true;
              break;
          }
        }
        if (!started) {
          continue;
        }
      }
      if (!feedCommand(steps, ticks, drift)) return;
    }
  }
  if (!started) {
    stepper->moveTimed(0, 0, NULL, true);  // short run: everything prefilled
    started = true;
  }

  while (stepper->isRunning()) {
    readPcntAccum();
    vTaskDelay(1);
  }
  vTaskDelay(pdMS_TO_TICKS(50));

  int32_t endPos = stepper->getCurrentPosition();
  int64_t endPcnt = readPcntAccum();

  printf("--- Replay complete ---\n");
  printf("Prefilled:            %u commands\n", prefilled);
  printf("Library position:     %ld (delta %ld)\n", (long)endPos,
         (long)(endPos - startPos));
  printf("Commanded sum:        %lld (delta %lld)\n", commandedSum,
         commandedSum - startCmd);
  printf("PCNT (wire counter):  %lld (delta %lld)\n", endPcnt,
         endPcnt - startPcnt);
  printf("PCNT - library:       %lld\n", endPcnt - (int64_t)endPos);
  printf("Read the encoder/pulse count in MotionStudio now.\n");
}

// Chunked one-way run: feed a slice of the pattern, let the queue drain to a
// full stop, and compare all counters at standstill before continuing. At
// standstill library position and PCNT must agree exactly, so any step loss
// is localized to the chunk (command range) where the difference jumped.
// Chunk boundaries add no direction changes, but they do relax the
// queue-full streaming condition — if the loss disappears completely in
// chunked mode, that is a finding in itself (the bug then needs continuous
// streaming, not just the step pattern).
static void runChunked(uint16_t chunkLen) {
  if (chunkLen == 0) chunkLen = 25;
  int32_t startPos = stepper->getCurrentPosition();
  int64_t startPcnt = readPcntAccum();
  int64_t startCmd = commandedSum;
  pcntRangeWarned = false;
  int64_t lastDiff = startPcnt - (int64_t)startPos;

  printf(
      "Chunked one-way replay, %u commands per chunk "
      "(stop + compare between chunks)...\n",
      chunkLen);

  int32_t drift = 0;
  uint32_t i = 0;
  uint16_t chunk = 0;
  while (i < REPLAY_PATTERN_LEN) {
    uint32_t first = i;
    int32_t chunkSteps = 0;
    for (; i < REPLAY_PATTERN_LEN && (i - first) < chunkLen; i++) {
      // start=true: begin executing immediately; feeding from RAM is far
      // faster than execution, so the queue stays ahead within a chunk.
      chunkSteps += REPLAY_PATTERN[i].steps;
      if (!feedCommand(REPLAY_PATTERN[i].steps, REPLAY_PATTERN[i].ticks, drift,
                       true)) {
        return;
      }
    }
    while (stepper->isRunning()) {
      readPcntAccum();
      vTaskDelay(1);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    int32_t pos = stepper->getCurrentPosition();
    int64_t pcnt = readPcntAccum();
    int64_t diff = pcnt - (int64_t)pos;
    printf(
        "chunk %2u  cmds %3lu..%3lu  steps %+5ld  "
        "PCNT-library %+lld (change %+lld)\n",
        chunk, (unsigned long)first, (unsigned long)(i - 1), (long)chunkSteps,
        diff, diff - lastDiff);
    lastDiff = diff;
    chunk++;
  }

  printf("--- Chunked replay complete ---\n");
  printf("Library position:     %ld (delta %ld)\n",
         (long)stepper->getCurrentPosition(),
         (long)(stepper->getCurrentPosition() - startPos));
  printf("Commanded sum:        %lld (delta %lld)\n", commandedSum,
         commandedSum - startCmd);
  int64_t endPcnt = readPcntAccum();
  printf("PCNT (wire counter):  %lld (delta %lld)\n", endPcnt,
         endPcnt - startPcnt);
  printf("PCNT - library:       %lld\n",
         endPcnt - (int64_t)stepper->getCurrentPosition());
}

static void printCounters() {
  printf("Library position:     %ld\n", (long)stepper->getCurrentPosition());
  printf("Commanded sum:        %lld\n", commandedSum);
  printf("PCNT (wire counter):  %lld\n", readPcntAccum());
}

static void printHelp() {
  printf("Platform: ESP-IDF %u.%u.%u\n", ESP_IDF_VERSION_MAJOR,
         ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH);
  printf("  r      run 1 pass (forward + mirrored return, net 0)\n");
  printf("  rN     run N passes (e.g. r10)\n");
  printf("  o      run pattern one-way only (ends offset by net steps)\n");
  printf(
      "  s / sN chunked one-way: stop + compare every N cmds (default 25)\n");
  printf("  p      print counters\n");
  printf("  z      zero counters\n");
  printf("  d      toggle DIR delay 0us <-> 200us\n");
  printf(
      "  b      switch to the other driver backend (RMT <-> MCPWM) + reboot\n");
  printf("  h      this help\n");
}

void setup() {
  serialInit();
  vTaskDelay(pdMS_TO_TICKS(200));

  printCoreVersions();
  engine.init();
  uint8_t choice = currentBackendChoice();
  printf("Backend choice: %s ('b' switches to the other backend)\n",
         backendChoiceName(choice));
  switch (choice) {
    case BACKEND_RMT:
      stepper = engine.stepperConnectToPin(STEP_PIN, DRIVER_RMT);
      break;
#if defined(SUPPORT_ESP32_MCPWM_PCNT)
    case BACKEND_MCPWM:
      stepper = engine.stepperConnectToPin(STEP_PIN, DRIVER_MCPWM_PCNT);
      break;
#endif
    default:
      stepper = engine.stepperConnectToPin(STEP_PIN);  // worker-like default
      break;
  }
  if (!stepper) {
    printf(
        "ERROR: stepperConnectToPin failed (backend unavailable on "
        "this platform?) - 'b' switches to the next backend\n");
  }
  while (!stepper) {
    // Keep the serial command loop alive so 'b' can rescue a bad choice.
    while (serialAvailable()) {
      char ch = serialRead();
      if (ch == 'b') {
        switchBackendAndReboot();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  stepper->setDirectionPin(DIR_PIN, DIR_HIGH_COUNTS_UP, dirChangeDelayUs);
  stepper->setEnablePin(ENABLE_PIN);
  stepper->setAutoEnable(false);
  stepper->enableOutputs();
  vTaskDelay(pdMS_TO_TICKS(100));

  if (!stepper->attachToPulseCounter(7)) {
    printf("WARN: attachToPulseCounter failed - PCNT readings unavailable\n");
  }
  stepper->clearPulseCounter();
  lastPcntRaw = 0;

  printf("Driver backend: %s\n", stepper->driverTypeString());
  printf("Pattern: %d commands, net %d steps\n", REPLAY_PATTERN_LEN,
         REPLAY_PATTERN_NET_STEPS);
  printHelp();
  printf("Ready.\n");
}

void loop() {
  static char buf[16];
  static uint8_t len = 0;

  while (serialAvailable()) {
    char ch = serialRead();
    if (ch == '\n' || ch == '\r') {
      if (len == 0) continue;
      buf[len] = 0;
      len = 0;
      switch (buf[0]) {
        case 'r': {
          long n = atol(buf + 1);
          runPasses(n > 0 ? (uint16_t)n : 1, false);
          break;
        }
        case 'o':
          runPasses(1, true);
          break;
        case 's': {
          long n = atol(buf + 1);
          runChunked(n > 0 ? (uint16_t)n : 25);
          break;
        }
        case 'p':
          printCounters();
          break;
        case 'z':
          stepper->setCurrentPosition(0);
          stepper->clearPulseCounter();
          lastPcntRaw = 0;
          pcntBase = 0;
          commandedSum = 0;
          printf("counters zeroed\n");
          break;
        case 'd':
          dirChangeDelayUs = dirChangeDelayUs ? 0 : DIR_CHANGE_DELAY_US;
          stepper->setDirectionPin(DIR_PIN, DIR_HIGH_COUNTS_UP,
                                   dirChangeDelayUs);
          printf("DIR delay = %u us\n", dirChangeDelayUs);
          break;
        case 'b':
          switchBackendAndReboot();
          break;
        case 'h':
          printHelp();
          break;
        default:
          printf("unknown command '%c' - h for help\n", buf[0]);
          break;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = ch;
    }
  }
}

#else

// No ESP32 pulse counter / selectable driver (e.g. esp-idf 6): the direction
// change miscount cannot be observed, so keep the application a no-op.
void setup() {}

void loop() {}

#endif  // SUPPORT_ESP32_PULSE_COUNTER && SUPPORT_SELECT_DRIVER_TYPE

// IDF 4.x/5.0: deinit() returns ESP_ERR_INVALID_STATE while idle tasks are
// still subscribed (CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_*). IDF 5.1+
// unsubscribes idle tasks itself.
static void disableTaskWdt() {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 1, 0)
  (void)esp_task_wdt_delete(xTaskGetIdleTaskHandle());
#if !defined(CONFIG_FREERTOS_UNICORE)
  (void)esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(1));
#endif
#endif
  (void)esp_task_wdt_deinit();
}

extern "C" void app_main() {
  disableTaskWdt();
  setup();
  while (true) {
    loop();
  }
}
