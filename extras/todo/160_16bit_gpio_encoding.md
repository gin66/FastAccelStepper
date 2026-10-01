# 160 16-bit GPIO pin encoding (generic, 8-bit fallback for small MCUs)

## Problem

The current pin encoding uses **uint8_t** with a 6-bit pin number + 2 flag
bits:

```
Bits 0-5  → pin number (0–63)
Bit 6     → PIN_I2S_FLAG (ESP32 I2S mux slot)
Bit 7     → PIN_EXTERNAL_FLAG (use callback for pin control)
Value 255 → PIN_UNDEFINED (no pin)
```

This limits pin numbers to 63, which is insufficient on modern MCUs
(ESP32-S3: 48 GPIO, ESP32-C6: 46, RP2040: 30, STM32: 80+).

## Goal

Replace the 8-bit pin encoding with a **generic 16-bit encoding** while
keeping the 8-bit scheme for small MCUs (AVR, etc.) that only support
~64 pins.

## Proposed 16-bit encoding

```
Bits 0-14  → pin number (0–16383)
Bit 15     → PIN_EXTERNAL_FLAG (use callback)
Bit 14     → PIN_I2S_FLAG (ESP32 I2S mux slot)
Bits 13-8  → reserved for future flags (e.g. PIN_SET_ONLY, PIN_OPEN_DRAIN)
Value 0xFFFF → PIN_UNDEFINED (no pin)
```

The `PIN_EXTERNAL_FLAG` and `PIN_I2S_FLAG` are now **independent flags**
in the upper half of the 16-bit value, no longer encoding the pin number
itself.

### Small-MCU 8-bit encoding (retained)

For platforms with `MAX_PIN_NUMBER <= 63` (AVR, SAM D21):

```
Bits 0-5  → pin number (0–63)
Bit 6     → PIN_I2S_FLAG (may be unused on these platforms)
Bit 7     → PIN_EXTERNAL_FLAG
Value 255 → PIN_UNDEFINED
```

This is **not changed** — small MCUs keep their compact representation.

## Flag bits (16-bit only)

| Bit  | Flag             | Meaning |
|------|------------------|---------|
| 14   | `PIN_I2S_FLAG`   | ESP32 I2S mux slot (0–31) |
| 15   | `PIN_EXTERNAL_FLAG` | Use callback for pin control |
| 13   | `PIN_SET_ONLY`   | Use atomic GPIO set instead of toggle (issue #316) |
| 12   | `PIN_OPEN_DRAIN` | Configure as open-drain (future) |
| 11-8 | reserved         | Future flags |

## Implementation plan

### Phase 1 — Core type change

1. **Define pin type**: `typedef uint16_t pin_t;` in `fas_arch/common.h`
   (or `uint8_t` when compiled for small MCUs via `#if defined(SUPPORT_8BIT_PIN)`).
2. **Update constants**: `PIN_UNDEFINED = 0xFFFF` (16-bit),
   `PIN_EXTERNAL_FLAG = (1 << 15)`, `PIN_I2S_FLAG = (1 << 14)`.
3. **Update all `uint8_t` pin parameters** to `pin_t` in:
   - `FastAccelStepperEngine::stepperConnectToPin(pin_t)`
   - `FastAccelStepper::init(..., pin_t step_pin)`
   - `FastAccelStepper::setDirPin(pin_t, ...)`
   - `FastAccelStepper::setEnablePin(pin_t, ...)`
   - `FastAccelStepper::getStepPin()` → `pin_t`
   - `FastAccelStepper::getDirPin()` → `pin_t`
   - `FastAccelStepper::usesAutoEnablePin(pin_t)` → `pin_t`

### Phase 2 — Queue structures

4. Update `_step_pin` and `dirPin` fields in every platform's
   `StepperQueue` from `uint8_t` to `pin_t`.
5. Update `tryAllocateQueue()` signatures across all platforms.
6. Update flag-checking logic (`& PIN_EXTERNAL_FLAG`, `& PIN_I2S_FLAG`)
   to use the new bit positions.

### Phase 3 — Platform-specific updates

7. **ESP32**: Update RMT/MCPWM/PCNT/I2S init calls to use 16-bit pin
   numbers. I2S mux slot extraction changes from `pin & 0x1F` to
   `pin & 0x3FFF` (or a dedicated `getI2sSlot(pin_t)` helper).
8. **AVR**: Keep 8-bit encoding. Add a compile-time assertion that
   `MAX_PIN_NUMBER < 64`. No API changes needed for AVR callers.
9. **Pico**: Update `isValidStepPin()` to check against RP2040 GPIO
   count (32 or 30).
10. **SAM**: Update `isValidStepPin()` to check against SAM GPIO count.
11. **Teensy**: Update `isValidStepPin()` to check against Teensy GPIO
    count.

### Phase 4 — Validation and tests

12. Add compile-time `static_assert` per platform:
    - Small MCUs: `sizeof(pin_t) == 1`
    - Others: `sizeof(pin_t) == 2`
13. Update all PC-based tests (`test_01`–`test_17`) to use `pin_t`.
14. SimAVAR tests: verify AVR still uses 8-bit pin_t.

## Acceptance criteria

- All platforms compile without warnings.
- AVR memory usage unchanged (8-bit pin_t).
- ESP32/Pico/SAM/Teensy support pin numbers up to their GPIO count.
- External pin callbacks still work (bit 15).
- I2S mux slots still work (bit 14).
- All existing tests pass.

## Status

_idea — not started_