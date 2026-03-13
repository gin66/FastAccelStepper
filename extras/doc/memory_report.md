# FastAccelStepper Memory Footprint

Generated: 2026-03-13 21:46:58

Library size measured using the difference method:
- Baseline: Empty Arduino sketch (setup/loop only)
- Test: UsageExample with basic stepper initialization

| Board | RAM (bytes) | Flash (bytes) |
|-------|-------------|---------------|
| ATmega2560 | 373 | 9634 |
| ATmega328P | 251 | 8824 |
| ATmega32U4 | 373 | 9286 |
| ESP32 | 5360 | 30744 |
| ESP32-C3 | 968 | 29432 |
| ESP32-S2 | 1664 | 28820 |
| ESP32-S3 | 3504 | 38268 |
| RP2040 | 2380 | 8628 |
| RP2350 | 3472 | 8308 |
| SAM3X8E | 2432 | 22040 |

**Notes:**
- RAM = .data + .bss (static allocation)
- Flash = .text + .data (code + initialized data)
- Actual usage may vary based on enabled features
- All builds use release optimization (-Os)
- Values include peripheral drivers pulled in by the library:
  - ESP32: RMT, MCPWM, PCNT drivers (~15-20KB of the flash)
  - SAM: Timer/Counter drivers
- For detailed per-symbol analysis, use PlatformIO's "Project Inspect" feature
