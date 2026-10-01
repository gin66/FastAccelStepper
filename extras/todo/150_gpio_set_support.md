# 150 Support GPIO set instead of toggle (issue #316)

## Issue

GitHub issue #316: Some hardware platforms support a **atomic GPIO set**
operation (write-only to specific bits without read-modify-toggle) which
is faster and safer than the current toggle-based approach.

## Current behaviour

The pulse drivers use a **toggle** operation to step pins:
```cpp
GPIO_OUT_XOR_MASK = 1 << PIN;  // toggle
```

This is a read-modify-write at the register level, which is:
- **Not atomic** on some platforms (race window between read and write).
- **Slower** than a pure set operation.
- **Potentially unsafe** in multi-core or multi-master scenarios.

## Desired behaviour

Detect at compile time whether the platform supports atomic GPIO set:

```cpp
#if defined(SUPPORT_GPIO_SET)
    GPIO_SET_MASK = 1 << PIN;     // atomic set (no toggle)
#else
    GPIO_OUT_XOR_MASK = 1 << PIN; // toggle (existing behaviour)
#endif
```

## Platform analysis

| Platform | Set support | Notes |
|----------|-------------|-------|
| ESP32    | ?           | MCPWM handles this; RMT may benefit |
| AVR      | Partial     | Some chips have port set registers |
| Pico     | Yes         | GPIO set registers available |
| SAM      | Yes         | PIO/SMC set registers |
| Teensy   | ?           | Kinetis has set registers |

## Implementation plan

1. **Audit** each `pd_*/pd_stepper_*.cpp` for toggle vs. set usage.
2. **Add** `SUPPORT_GPIO_SET` config flag per platform in
   `pd_*/pd_config.h`.
3. **Implement** set-based stepping where supported.
4. **Benchmark** — measure speed improvement on supported platforms.
5. **Test** — all existing tests must pass; new PC-based test for
   set-vs-toggle correctness.

## Status

_idea — not started_
