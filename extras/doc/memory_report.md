# FastAccelStepper Memory Footprint

Generated: 2026-03-07 20:33:15

Library size measured using the difference method:
- Baseline: Empty Arduino sketch (setup/loop only)
- Test: UsageExample with basic stepper initialization

| Board | RAM (bytes) | Flash (bytes) |
|-------|-------------|---------------|
| ATmega2560 | 370 | 9630 |
| ATmega328P | 249 | 8796 |
| ATmega32U4 | 370 | 9282 |
| ESP32 | 5304 | 30768 |
| ESP32-C3 | 952 | 29190 |
| ESP32-S2 | 1664 | 28892 |
| ESP32-S3 | 3464 | 38236 |
| RP2040 | 2348 | 8564 |
| RP2350 | 3424 | 8260 |
| SAM3X8E | 2380 | 21968 |

**Notes:**
- RAM = .data + .bss (static allocation)
- Flash = .text + .data (code + initialized data)
- Actual usage may vary based on enabled features
- All builds use release optimization (-Os)
- Values include peripheral drivers pulled in by the library:
  - ESP32: RMT, MCPWM, PCNT drivers (~15-20KB of the flash)
  - SAM: Timer/Counter drivers
- For detailed per-symbol analysis, use PlatformIO's "Project Inspect" feature
