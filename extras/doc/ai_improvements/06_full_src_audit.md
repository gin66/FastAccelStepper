# Full `src/` Architecture Audit

Generated: 2026-03-18
Updated: 2026-03-18 (applied fixes marked ~~strikethrough~~)

Comprehensive audit of all source files under `src/`, identifying bugs, dead code,
leftover code, code smells, inconsistencies, and technical debt.

---

## Bugs (Highest Priority)

| # | File | Line(s) | Description | Status |
|---|------|---------|-------------|--------|
| 1 | `pd_esp32/pd_config.h` | 30 | `#pragma "Last supported..."` should be `#error` -- silently ignored by compilers | open |
| 2 | `pd_esp32/pd_config_idf4.h:31`, `pd_config_idf5.h:35` | -- | `HAVE_ESP32S3_PULSE_COUNTER` set for **ESP32S2** (copy-paste bug) | open |
| ~~3~~ | ~~`FastAccelStepper.cpp`~~ | ~~885~~ | ~~`d *= steps` uses loop variable instead of `d *= cmd.steps`~~ | **fixed** |
| 4 | `FastAccelStepper_idf5_esp32_pcnt.cpp` | 81-100 | 5 error paths leak `punit`/`pcnt_chan` before returning `false` | open |
| ~~5~~ | ~~`FastAccelStepperEngine.cpp`~~ | ~~17, 150~~ | ~~File-scope `fas_stepper[]` shadowed by local `static`~~ | **fixed** |
| 6 | `FastAccelStepperEngine.cpp` | 123-124 | No bounds check on `_stepper_cnt` increment -- buffer overflow | open |
| 7 | `StepperISR_idf4_esp32c3_rmt.cpp:61`, `StepperISR_idf4_esp32s3_rmt.cpp:62` | -- | Hardcoded `61/62` in `stop_rmt()` is wrong for C3/S3 where `PART_SIZE=22` (should be `43/44`) | open |
| ~~8~~ | ~~`StepperISR_esp32xx_rmt.cpp`~~ | ~~116-118~~ | ~~Dead redundant condition: inner `if (steps > PART_SIZE)` always true~~ | **fixed** (changed to `steps < 2*PART_SIZE`) |
| ~~9~~ | ~~`sam_queue.cpp`~~ | ~~64-65~~ | ~~`timeElapsed = micros() - ...` uses a second `micros()` call instead of saved `t`~~ | **fixed** |
| 10 | `sam_queue.cpp` | 140-145 | `static` locals in ISR initialized once from mutable pointer -- stale data on remap | open |
| 11 | `pico_queue.cpp` | 192-196 | Inverted break condition: breaks when FIFO has data, should break when empty | open |
| 12 | `esp32_queue.cpp` | 154 | `queues_allocated` never incremented in `SUPPORT_SELECT_DRIVER_TYPE + SUPPORT_DYNAMIC_ALLOCATION` path -- `MAX_STEPPER` check is dead code | open |

---

## Dead Code / Leftover Code

| # | File | Line(s) | Description |
|---|------|---------|-------------|
| 14 | `sam_queue.cpp` | 4-5 | `const bool enabled = false;` and `uint32_t junk = 0;` -- never used |
| 15 | `sam_queue.cpp` | 45-51 | `strobePin()` -- defined but never called |
| 16 | `sam_queue.cpp` | 23-33, 246-249 | `KEEP_SCORE` debugging block -- permanently disabled (`KEEP_SCORE` never defined) |
| 17 | `pd_test/test_queue.h` | 20-24, 29-33 | ESP32-specific `SUPPORT_ESP32_RMT` blocks -- can never activate in test platform |
| 18 | `AVRStepperPins.h` | 58-76 | `FAS_TIMER_MODULE == 3` and `== 5` branches -- values never assigned |
| 19 | `result_codes.h` | 27-29, 74-76, 116-118, 161-163 | `aqeIsOk()`, `moveIsOk()`, `moveTimedIsOk()`, `delayIsValid()` -- never called |
| 20 | `result_codes.h` | 78-91, 124-153, 165-176 | 3 `toString()` overloads -- never called |
| 21 | `Log2RepresentationConst.h` | 9, 25, 29, 33, 37, 49, 71 | 7 `LOG2_CONST_*` constants -- unused in `src/` |
| 22 | `Log2Representation.h` | 28 | `log2_rsquare(x)` macro -- never used |
| 23 | `RampControl.h` | 16-18 | `advanceTargetPositionWithinInterruptDisabledScope()` -- never called |
| 24 | `RampCalculator.cpp` | 31-180 | `calculate_ticks_v1`..`v8` behind `#ifdef TEST_TIMING` -- dead benchmark functions |
| 25 | `pd_config_idf5.h` / `pd_config_idf6.h` | ~20 lines | Commented-out `SUPPORT_ESP32_MCPWM_PCNT`, `NEED_MCPWM_HEADERS`, etc. |
| 26 | `pd_config_idf5.h` / `pd_config_idf6.h` | 139-154 | `NEED_MCPWM_HEADERS` / `NEED_PCNT_HEADERS` include blocks -- dead (never compiled) |
| 27 | `StepperISR_esp32xx_rmt.cpp` | 117 | `steps_to_do = PART_SIZE` -- dead assignment, always overwritten by line 119 |

Note: item #13 (accidental LLM prompt in `pd_pico/pico_pio.cpp:1-10`) was present at audit
time but may have been removed separately.

---

## Code Duplication (DRY Violations)

| # | Files | Lines | Description |
|---|------|-------|-------------|
| 28 | 3x IDF4 RMT files + idf5 | ~600 lines total | `stop_rmt()`, `tx_intr_handler()`, `init_rmt()`, `forceStop_rmt()`, `isReadyForCommands_rmt()`, `_getPerformedPulses_rmt()`, `startQueue_rmt()` -- nearly identical across 4 files |
| 29 | `pd_config_idf5.h` vs `pd_config_idf6.h` | entire files | 95%+ duplicated content |
| 30 | `FastAccelStepperEngine.cpp` | 21-35 vs 37-52 | `init()` function body duplicated |
| 31 | `FastAccelStepperEngine.cpp` | 108-165 | `stepperConnectToPin()` x4 variants with shared logic |
| 32 | `FastAccelStepper_idf4_esp32_pcnt.cpp` vs `idf5` | 67-74 vs 111-122 | GPIO matrix setup nearly identical |
| 33 | `FastAccelStepper.cpp` + `FastAccelStepperEngine.cpp` | 5-8 / 8-11 | `printf`/`puts` redefinition macro duplicated |
| 34 | `RampControl.cpp` | 446-453 vs 489-497 | Identical pause-ticks-left calculation (only difference: `(uint32_t)` cast) |
| 35 | `RampControl.cpp` | 141-149 vs 241-249 | Identical "ramp complete" early-return blocks |
| 36 | `test_pc.h` vs `Log2Representation.cpp` | -- | `PROGMEM` / `pgm_read_byte_near` polyfill duplicated |
| 37 | `avr_queue.h`, `pico_queue.h`, `sam_queue.h` | -- | `SET_ENABLE_PIN_STATE` macro verbatim duplicated |
| 38 | `sam_queue.cpp` | 310-335 | 6 identical `attachInterrupt()` calls in switch cases |

---

## Inconsistencies

### Preprocessor Style

| # | File | Details |
|---|------|---------|
| 39 | `esp32_queue.h` (lines 21, 35, 53, 68, 71 vs 76, 229) | `#ifdef SUPPORT_ESP32_RMT` mixed with `#if defined(SUPPORT_ESP32_RMT)` interchangeably |

### Platform Configuration

| # | File | Details |
|---|------|---------|
| 40 | `pd_config_idf4.h` vs `idf5/6.h` | `PART_SIZE` formula: IDF4 uses `((RMT_SIZE-1)/4)<<1`=30; IDF5/6 use `RMT_SIZE>>1`=32 |
| 41 | `StepperISR_idf5_esp32_rmt.cpp:182` vs all IDF4 | `lastChunkContainsSteps` init: IDF5=`false`; all IDF4=`true` |
| 42 | `pd_avr/pd_config.h:15` vs all others | `MIN_CMD_TICKS` divisor: AVR `/25000` (5x shorter); all others `/5000` |
| 43 | IDF4 RMT files vs C3/S3 vs IDF5 | TRACE debug output: ESP32=`Serial`; C3/S3=`USBSerial`; IDF5=`printf` |

### Naming Conventions

| # | File | Details |
|---|------|---------|
| 44 | `fas_queue`/`fas_arch` vs `fas_ramp`/`log2` | Header guard prefix: `FAS_*` vs no prefix (`RAMP_CONTROL_H`, `LOG2REPRESENTATION_H`) |
| 45 | `fas_queue/` vs `fas_ramp/` | File naming: `snake_case.cpp` vs `PascalCase.cpp` |
| 46 | `queue_add_entry.cpp:14` vs elsewhere | `NULL` vs `nullptr` inconsistency |

### Logic Patterns

| # | File | Details |
|---|------|---------|
| 47 | `FastAccelStepper.cpp:554, 562` | Bitwise `&` used instead of logical `&&` for `bool` return values |
| 48 | `StepperISR_idf4_esp32_rmt.cpp:160` vs C3/S3/IDF5 | `connect()` vs `connect_rmt()` called in `init_rmt()` |
| 49 | `StepperISR_idf4_esp32_rmt.cpp:131-132` vs `idf5:85-86` | Pin setup order: `digitalWrite` then `pinMode` vs `pinMode` then `digitalWrite` |
| 50 | `pd_config.h:19-21` | `SUPPORT_DYNAMIC_ALLOCATION` missing for IDF6 |
| 51 | `esp32_queue.h:217-233` | `AFTER_DIR_CHANGE_DELAY_TICKS` missing in non-I2S path |

---

## Typos

All fixed.

| # | File | Line | Fix |
|---|------|------|-----|
| ~~52~~ | ~~`FastAccelStepper.h`~~ | ~~271~~ | ~~"changeing" -> "changing"~~ | **fixed** |
| ~~53~~ | ~~`FastAccelStepper.h`~~ | ~~501~~ | ~~"obsolote" -> "obsolete"~~ | **fixed** |
| ~~54~~ | ~~`FastAccelStepper_idf5_esp32_pcnt.cpp`~~ | ~~8~~ | ~~"not save" -> "not safe"~~ | **fixed** |
| ~~55~~ | ~~`pd_avr/pd_config.h`~~ | ~~36, 44, 53, 60, 64~~ | ~~"derivate" -> "derivative" (5 occurrences)~~ | **fixed** |
| ~~56~~ | ~~`sam_queue.cpp`~~ | ~~7, 119~~ | ~~"Periphal" -> "Peripheral" (declaration + definition)~~ | **fixed** |
| ~~57~~ | ~~`pico_pio.h`~~ | ~~1, 2, 19~~ | ~~`PD_PICO_PICO_PIO_H` -> `PD_PICO_PIO_H`~~ | **fixed** |

---

## TODOs / Technical Debt

| # | File | Line | Description |
|---|------|------|-------------|
| 58 | `queue_get_position.cpp` | 42 | `break; // TODO: ERROR` -- known bad position transitions silently ignored |
| 59 | `esp32_queue.cpp` | 461 | `break; // TODO: ERROR` -- same pattern in `getCurrentPosition()` |
| 60 | `pd_sam/pd_config.h` | 27 | `// TO BE CHECKED` -- `SUPPORT_QUEUE_ENTRY_END_POS_U16` unverified on SAM |
| 61 | `FastAccelStepper.h` | 609 | "Will be renamed in future release" -- `getPeriodInUsAfterCommandsCompleted()` |
| 62 | `FastAccelStepper_idf5_esp32_pcnt.cpp` | 9-19 | Private struct redeclaration to access `unit_id`/`channel_id` -- fragile across ESP-IDF versions |

---

## Commented-Out Code (should be removed)

Approximately **80+ lines** of commented-out code across the codebase:

- `StepperISR_idf4_esp32_rmt.cpp:27-28, 148-149, 211-212, 226, 234, 254-255, 257, 271, 273`
- `StepperISR_idf4_esp32c3_rmt.cpp:47-48, 180-181, 210-212, 288, 336-338`
- `StepperISR_idf4_esp32s3_rmt.cpp:48-49, 181-182, 211-213, 289, 337-339`
- `StepperISR_idf5_esp32_rmt.cpp:109-110, 129, 149, 177-180`
- `StepperISR_idf4_esp32_mcpwm_pcnt.cpp:259, 473-474, 548-553`
- `StepperISR_esp32xx_rmt.cpp:68-69`
- `FastAccelStepper.cpp:309-310, 321-322, 335-336, 880-882`
- `FastAccelStepper_idf4_esp32_pcnt.cpp:65-66`
- `pico_pio.cpp:75, 79-83, 196-199`
- `avr_queue.cpp:18-21` (behind `#ifdef DISABLE`)
- `queue_add_entry.cpp:7-10, 29`
- `RampControl.cpp:174, 415-417, 463`
- `RampGenerator.cpp:246`
- `RampCalculator.cpp:73-76`
- `pd_pico/pico_queue.cpp:72-77` (duplicate retry call)
- `pd_config_idf5.h:135, 147-148, 163`
- `pd_config_idf6.h:135, 147-148, 163`
- `i2s_fill.cpp:74-79, 87-90, 114-118`

---

## Design / Encapsulation Issues

| # | File | Line | Description | Status |
|---|------|------|-------------|--------|
| ~~63~~ | ~~`FastAccelStepperEngine.h`~~ | ~~145~~ | ~~`_delay_ms` is `public` but named as private (leading underscore)~~ | **fixed** |
| 64 | `FastAccelStepper.h` | 49 | `#include "fas_ramp/RampGenerator.h"` in public header -- could be forward-declared | open |
| 65 | `stepper_queue.h` | 6 | `#include "FastAccelStepper.h"` creates heavy dependency from queue layer to API | open |
| 66 | `FastAccelStepper.cpp` | 739-891 | `moveTimed()` -- ~153 lines, should be split into helpers | open |
| 67 | `FastAccelStepper.cpp` | 67-207 | `addQueueEntry()` -- ~140 lines, should be split into helpers | open |
| 68 | `FastAccelStepper_idf5_esp32_pcnt.cpp` | 21-127 | `attachToPulseCounter()` -- ~106 lines | open |

---

## Summary

| Category | Total | Open | Fixed |
|----------|-------|------|-------|
| **Bugs** | 12 | 8 | 4 |
| **Dead / leftover code** | 14 | 14 | 0 |
| **Code duplication** | 11 | 11 | 0 |
| **Inconsistencies** | 13 | 13 | 0 |
| **Typos** | 6 | 0 | 6 |
| **TODOs / tech debt** | 5 | 5 | 0 |
| **Design / encapsulation** | 6 | 5 | 1 |

### Remaining Recommended Fix Priority

1. **Bug #7** -- Hardcoded `61/62` in C3/S3 `stop_rmt()` causes incorrect pause duration
2. **Bug #2** -- `HAVE_ESP32S3_PULSE_COUNTER` on ESP32S2 may use wrong registers
3. **Bug #6** -- `_stepper_cnt` overflow risk
4. **Bug #1** -- `#pragma` should be `#error`
5. **Bug #4** -- Memory leaks in idf5 pcnt error paths
6. **Bugs #10, #11, #12** -- Remaining open bugs
