# FastAccelStepper Refactoring Status

## Preface

These documents describe ongoing refactoring efforts. The primary reference is [ROADMAP.md](ROADMAP.md), which tracks the per-architecture StepperQueue migration.

## Implementation Status

### Completed

#### Per-Architecture StepperQueue (ROADMAP.md)
**Status: Steps 1-5 Complete, Step 6 Partial**

The codebase has been restructured with `StepperQueueBase` in `fas_queue/base.h` and per-architecture queue implementations.

The `pd_` prefix in directory names stands for "pulse driver" (though "platform driver" is also suitable).

```
src/
  fas_queue/
    base.h              ← StepperQueueBase (common fields)
    stepper_queue.h     ← Architecture dispatcher
  pd_avr/
    avr_queue.h/cpp     ← AVR implementation
  pd_test/
    test_queue.h        ← PC-based test stub
  pd_sam/
    sam_queue.h/cpp     ← SAM Due implementation
  pd_pico/
    pico_queue.h/cpp    ← RP2040/RP2350 implementation
  pd_esp32/
    esp32_queue.h/cpp   ← ESP32 queue (7 driver files remain)
```

**Remaining work:**
- ESP32 driver unification (7 driver files still separate)

### Not Pursued

The following approaches were evaluated but rejected due to embedded constraints:

1. **Virtual functions/runtime polymorphism** — Overhead unsuitable for ISR code
2. **Strategy pattern for queues** — Dynamic allocation violates embedded constraints
3. **API surface reduction** — Would break backward compatibility
4. **Motion math replacement** — Existing log2-based math is optimized for 8/16-bit MCUs

### Still Relevant

#### Configuration System Overhaul (03_configuration_system.md)
The macro-based configuration system could be improved with `constexpr`, but this is low priority.

#### ESP32 Driver Unification (02_esp32_driver_unification.md)
The 7 ESP32 driver files could potentially be consolidated, but this requires careful testing across IDF versions.

## SAM Platform: Unverified

The SAM (Arduino Due) platform has **6 open issues** identified in [05_sam_platform_audit.md](05_sam_platform_audit.md), including 4 critical bugs. This platform cannot be tested in CI and has not been verified on hardware. Any changes to SAM code must be cross-checked against that document.

## Document Index

| Document | Purpose |
|----------|---------|
| [ROADMAP.md](ROADMAP.md) | Per-architecture queue migration plan and status |
| [driver_architecture.md](driver_architecture.md) | StepperISR driver abstraction and interface analysis |
| [02_esp32_driver_unification.md](02_esp32_driver_unification.md) | ESP32 driver consolidation proposal |
| [03_configuration_system.md](03_configuration_system.md) | Configuration system modernization proposal |
| [04_esp32_i2s_driver.md](04_esp32_i2s_driver.md) | ESP32 I2S output driver (single-pin and serial demux modes) |
| [05_sam_platform_audit.md](05_sam_platform_audit.md) | SAM platform audit against SAM3X8E datasheet (6 open issues) |
| [06_full_src_audit.md](06_full_src_audit.md) | Full `src/` audit: 12 bugs, 15 dead code items, 11 duplication patterns, 13 inconsistencies |

## Contributing

When working on refactoring:
1. Follow existing code style (see AGENTS.md)
2. Maintain backward compatibility
3. Run PC-based tests: `make -C extras/tests/pc_based`
4. Test on target platforms
