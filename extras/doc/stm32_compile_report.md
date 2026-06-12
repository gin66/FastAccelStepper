# STM32 Compile Test Results

PlatformIO 6.1.19, `toolchain-gccarmnoneeabi 12.3.1`, `framework-arduinoststm32 2.12.0`.

## STM32F1 (Cortex-M3, 72MHz)

| Board | Chip | Result |
|-------|------|--------|
| `blackpill_f103c8` | STM32F103C8 | ✅ PASS |
| `bluepill_f103c8` | STM32F103C8 | ✅ PASS |
| `bluepill_f103c8_128k` | STM32F103CB | ✅ PASS |
| `genericSTM32F103C8` | STM32F103C8 | ✅ PASS |

## STM32F4 (Cortex-M4, 168MHz)

| Board | Chip | Result |
|-------|------|--------|
| `black_f407ve` | STM32F407VE | ✅ PASS |
| `black_f407vg` | STM32F407VG | ✅ PASS |
| `genericSTM32F407VET6` | STM32F407VE | ✅ PASS |
| `disco_f407vg` | STM32F407VG | ✅ PASS |

## STM32F411 (Cortex-M4, 100MHz)

| Board | Chip | Result |
|-------|------|--------|
| `blackpill_f411ce` | STM32F411CE | ✅ PASS |
| `genericSTM32F411CE` | STM32F411CE | ✅ PASS |
| `nucleo_f411re` | STM32F411RE | ✅ PASS |
| `disco_f411ve` | STM32F411VE | ❌ FAIL (no Arduino framework support, mbed only) |

## STM32F401 (Cortex-M4, 84MHz)

| Board | Chip | Result |
|-------|------|--------|
| `blackpill_f401ce` | STM32F401CE | ✅ PASS |
| `blackpill_f401cc` | STM32F401CC | ✅ PASS |
| `genericSTM32F401CE` | STM32F401CE | ✅ PASS |
| `nucleo_f401re` | STM32F401RE | ✅ PASS |

## Summary

| Category | Count |
|----------|-------|
| ✅ Boards PASS | **15/16** |
| ❌ Boards FAIL (code) | **0** |
| ❌ Boards FAIL (platform limit) | **1** (`disco_f411ve` — no Arduino) |
| **PASS rate (Arduino boards)** | **15/15 = 100%** |