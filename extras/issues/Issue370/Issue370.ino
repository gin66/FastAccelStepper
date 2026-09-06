// ============================================================================
// debug-stepper — Minimal reproduction of the FastAccelStepper RMT
// direction-change miscount
//
// What it does: streams motion commands through the SAME code path as the
// real worker (moveTimed continuous queue, prefill + start, drift
// compensation) — but with the simplest possible profile:
//
//   Per cycle:
//     A: fast steps UP    (cruise, ~3.9 kHz — same as the diamond corner)
//     B: reversal to DOWN at full rate      <-- the DANGEROUS reversal
//     C: fast steps DOWN, decelerating into single-step (slow-path) commands
//     D: slow re-entry UP (reversal happens at slow-path rate — safe)
//
// Every cycle contains exactly ONE dangerous reversal, always the same
// polarity (UP -> DOWN). Expected result on a CL42T with encoder:
//
//   driver count  =  library count  -  2 * cycles
//
// because at each dangerous reversal the DIR pin flips at the start of the
// last fast UP pulse (RMT one-symbol prefetch), so the driver counts that
// pulse as DOWN. The library bookkeeping and the on-chip PCNT (sampling DIR
// at the ESP-side rising edge) both stay consistent with the commands.
//
// Serial usage (115200 baud):
//   r        run 1 cycle
//   r50      run 50 cycles
//   p        print counters (library pos, commanded sum, PCNT)
//   z        zero all counters (do this after aligning MotionStudio)
//   f / s    faster / slower base rate (finds the speed threshold)
//   h        help
// ============================================================================

#include "FastAccelStepper.h"

// The whole reproduction below relies on the ESP32 pulse counter (PCNT) and on
// the selectable RMT driver backend. On platforms without pulse counter support
// and/or without selectable driver types, fall back to an empty setup()/loop()
// so the sketch still compiles (it is a no-op there).
#if defined(SUPPORT_ESP32_PULSE_COUNTER) && defined(SUPPORT_SELECT_DRIVER_TYPE)

// --- Pin configuration: identical to station-worker ---
#define STEP_PIN 19
#define DIR_PIN 18
#define ENABLE_PIN 23

// Direction convention: RIGHT/CENTER stations use true, LEFT uses false.
// (station-worker.ino: dirHighCountsUp = (side == RIGHT || side == CENTER))
#define DIR_HIGH_COUNTS_UP true

// Same DIR setup delay as the main firmware (clamped to 200us minimum anyway)
#define DIR_CHANGE_DELAY_US 200

// --- Motion profile (defaults reproduce the diamond corner exactly) ---
// Base command duration: 105960 ticks @16MHz = 6.6225 ms (1mm @ 151 mm/s)
static uint32_t baseTicks = 105960;
// Fast phase: steps per command (26 @ 105960 -> ~4075 ticks/step ~= 3.93 kHz)
#define FAST_STEPS 26
// Number of fast commands per direction per cycle (amplitude = FAST_STEPS *
// FAST_CMDS steps ~= 520 steps ~= 19.5 mm belt at 26667 steps/m)
#define FAST_CMDS 20

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* stepper = NULL;

// ---------------------------------------------------------------------------
// Command pattern
// ---------------------------------------------------------------------------
struct Cmd {
  int16_t steps;
  uint32_t ticks;
};

#define MAX_PATTERN 128
static Cmd pattern[MAX_PATTERN];
static uint16_t patternLen = 0;

static void addCmd(int16_t steps, uint32_t ticks) {
  if (patternLen < MAX_PATTERN) {
    pattern[patternLen].steps = steps;
    pattern[patternLen].ticks = ticks;
    patternLen++;
  }
}

// Build one cycle. Net steps per cycle must be exactly 0 so the motor stays
// in place across a run; a residual (if any) is appended as slow single
// steps, which reverse through the safe slow path.
static void buildPattern() {
  patternLen = 0;
  int32_t sum = 0;

  // A: fast UP at cruise — the run-up into the dangerous reversal
  for (int i = 0; i < FAST_CMDS; i++) { addCmd(+FAST_STEPS, baseTicks); sum += FAST_STEPS; }

  // B: DANGEROUS reversal — exactly the diamond corner (cmd#245 -26 -> cmd#246 +2/+7,
  // mirrored): direction flips between a full-rate command and slow opposite steps.
  addCmd(-2, baseTicks); sum -= 2;
  addCmd(-7, baseTicks); sum -= 7;

  // C: fast DOWN, then decelerate into slow-path commands (1 step per command
  // at baseTicks -> ~6.6 ms/step -> rate > 65535 ticks/step -> the library
  // pads with pause entries; the following reversal is therefore safe).
  for (int i = 0; i < FAST_CMDS - 2; i++) { addCmd(-FAST_STEPS, baseTicks); sum -= FAST_STEPS; }
  const int16_t decel[] = { -17, -10, -7, -5, -2, -1, -1 };
  for (int16_t s : decel) { addCmd(s, baseTicks); sum += s; }

  // D: SAFE reversal — first UP command follows a slow-path DOWN command
  const int16_t accel[] = { +1, +2, +5, +10, +17 };
  for (int16_t s : accel) { addCmd(s, baseTicks); sum += s; }

  // Balance to net zero with slow single steps (safe path)
  while (sum > 0) { addCmd(-1, baseTicks); sum -= 1; }
  while (sum < 0) { addCmd(+1, baseTicks); sum += 1; }
}

// ---------------------------------------------------------------------------
// Streaming through the same moveTimed path as the worker firmware
// ---------------------------------------------------------------------------
static int64_t commandedSum = 0;   // what we asked the library to do
static int64_t pcntBase = 0;       // PCNT accumulator (16-bit hw counter)
static int16_t lastPcntRaw = 0;

static int64_t readPcntAccum() {
  int16_t raw = stepper->readPulseCounter();
  // accumulate across the +/-16384 hardware range; we stay far away from the
  // limits within one run (max excursion ~520), so plain delta tracking works
  int16_t delta = raw - lastPcntRaw;
  lastPcntRaw = raw;
  pcntBase += delta;
  return pcntBase;
}

static bool feedCommand(int16_t steps, uint32_t ticks, int32_t& drift) {
  uint32_t duration = ticks + drift;
  uint32_t actual = 0;
  for (;;) {
    MoveTimedResultCode rc = stepper->moveTimed(steps, duration, &actual, false);
    switch (rc) {
      case MOVE_TIMED_OK:
      case MOVE_TIMED_EMPTY:
        drift = (int32_t)(duration - actual);
        commandedSum += steps;
        return true;
      case MOVE_TIMED_BUSY:
      case MoveTimedResultCode::QueueFull:
      case MoveTimedResultCode::DirPinIsBusy:
      case MoveTimedResultCode::WaitForEnablePinActive:
        delayMicroseconds(200);  // queue full / dir pending: retry
        break;
      case MoveTimedResultCode::DeviceNotReady:
        delayMicroseconds(10);
        break;
      default:
        Serial.printf("moveTimed error: %s (steps=%d dur=%lu)\n",
                      toString(rc), steps, (unsigned long)duration);
        return false;
    }
  }
}

static void runCycles(uint16_t cycles) {
  buildPattern();
  int16_t step_cnt = 0;
  for (uint16_t i = 0; i < patternLen; i++) {
     step_cnt += pattern[i].steps;
     Serial.printf("%2d: steps=%d ticks=%d => %d steps\n",i, pattern[i].steps, pattern[i].ticks, step_cnt);
  }

  int32_t startPos = stepper->getCurrentPosition();
  int64_t startPcnt = readPcntAccum();
  int64_t startCmd = commandedSum;

  Serial.printf("Running %u cycle(s), %u commands each, baseTicks=%lu "
                "(fast rate ~%.0f steps/s)...\n",
                cycles, patternLen, (unsigned long)baseTicks,
                16000000.0 * FAST_STEPS / baseTicks);

  int32_t drift = 0;
  bool started = false;
  uint16_t prefilled = 0;
  bool pcnt_ok = false;

  for (uint16_t c = 0; c < cycles; c++) {
    for (uint16_t i = 0; i < patternLen; i++) {
      // Prefill without starting, exactly like the worker's startExecution()
      if (!started) {
        uint32_t actual = 0;
        MoveTimedResultCode rc =
            stepper->moveTimed(pattern[i].steps, pattern[i].ticks, &actual, false);
        if (rc == MOVE_TIMED_OK || rc == MOVE_TIMED_EMPTY) {
          drift = (int32_t)(pattern[i].ticks - actual);
          commandedSum += pattern[i].steps;
          prefilled++;
          continue;
        }
        Serial.printf("return: %s\n", toString(rc));
        // queue full -> start it and fall through to normal feeding
        stepper->moveTimed(0, 0, NULL, true);
        started = true;
      }
      if (stepper->readPulseCounter() != 0) pcnt_ok = true;
      if (!feedCommand(pattern[i].steps, pattern[i].ticks, drift)) return;
    }
  }
  if (!started) {
    stepper->moveTimed(0, 0, NULL, true);  // short run: everything prefilled
    started = true;
  }

  // Wait for physical completion
  while (stepper->isRunning()) delay(2);
  delay(50);

  int32_t endPos = stepper->getCurrentPosition();
  int64_t endPcnt = readPcntAccum();

  Serial.println("--- Run complete ---");
  Serial.printf("Prefilled:            %u commands\n", prefilled);
  Serial.printf("Library position:     %ld (delta %ld)\n", (long)endPos, (long)(endPos - startPos));
  Serial.printf("Commanded sum:        %lld (delta %lld)\n", commandedSum, commandedSum - startCmd);
  Serial.printf("PCNT (wire counter):  %lld (delta %lld)\n", endPcnt, endPcnt - startPcnt);
  Serial.printf("PCNT - library:       %lld\n", endPcnt - (int64_t)endPos);
  if (!pcnt_ok) {
     Serial.println("pcnt is not working");
  }
  Serial.printf("Expected driver drift after this run: %d counts "
                "(one -2 per cycle at the fast reversal)\n", -2 * (int)cycles);
  Serial.println("Read the encoder/pulse count in MotionStudio now.");
}

static void printCounters() {
  Serial.printf("Library position: %ld  Commanded sum: %lld  PCNT: %lld\n",
                (long)stepper->getCurrentPosition(), commandedSum, readPcntAccum());
}

static void printHelp() {
  Serial.println("debug-stepper — minimal RMT direction-change repro");
  Serial.println("  r      run 1 cycle          rN   run N cycles (e.g. r50)");
  Serial.println("  p      print counters       z    zero counters");
  Serial.println("  f / s  faster / slower base rate (x0.8 / x1.25)");
  Serial.println("  h      this help");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN, DRIVER_RMT);  // default backend: RMT
  if (!stepper) {
    Serial.println("ERROR: stepperConnectToPin failed");
    while (1) delay(1000);
  }

  stepper->setDirectionPin(DIR_PIN, DIR_HIGH_COUNTS_UP, DIR_CHANGE_DELAY_US);
  stepper->setEnablePin(ENABLE_PIN);
  stepper->setAutoEnable(false);
  stepper->enableOutputs();
  delay(100);

  if (!stepper->attachToPulseCounter(7)) {
    Serial.println("WARN: attachToPulseCounter failed — PCNT readings unavailable");
  }
  stepper->clearPulseCounter();
  lastPcntRaw = 0;

  Serial.printf("Driver backend: %s\n", stepper->driverTypeString());
  printHelp();
  Serial.println("Ready.");
}

void loop() {
  static char buf[16];
  static uint8_t len = 0;

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len == 0) continue;
      buf[len] = 0;
      len = 0;
      switch (buf[0]) {
        case 'r': {
          long n = atol(buf + 1);
          runCycles(n > 0 ? (uint16_t)n : 1);
          break;
        }
        case 'p': printCounters(); break;
        case 'z':
          stepper->setCurrentPosition(0);
          stepper->clearPulseCounter();
          lastPcntRaw = 0;
          pcntBase = 0;
          commandedSum = 0;
          Serial.println("Counters zeroed. Align MotionStudio reading now.");
          break;
        case 'f':
          baseTicks = (uint32_t)(baseTicks * 0.8);
          Serial.printf("baseTicks=%lu (fast rate ~%.0f steps/s)\n",
                        (unsigned long)baseTicks, 16000000.0 * FAST_STEPS / baseTicks);
          break;
        case 's':
          baseTicks = (uint32_t)(baseTicks * 1.25);
          Serial.printf("baseTicks=%lu (fast rate ~%.0f steps/s)\n",
                        (unsigned long)baseTicks, 16000000.0 * FAST_STEPS / baseTicks);
          break;
        case 'h': printHelp(); break;
        default: Serial.println("? (h for help)"); break;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = ch;
    }
  }
}

#else

// No ESP32 pulse counter / selectable driver: nothing to reproduce, so keep the
// sketch compilable with an empty application.
void setup() {
}

void loop() {
}

#endif      // SUPPORT_ESP32_PULSE_COUNTER && SUPPORT_SELECT_DRIVER_TYPE
