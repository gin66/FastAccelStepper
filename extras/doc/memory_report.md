# FastAccelStepper Memory Footprint

Generated: 2026-03-18 11:25:02

Library size measured using the difference method:
- Baseline: Empty Arduino sketch (setup/loop only)
- Test: UsageExample with basic stepper initialization

| Board | RAM (bytes) | Flash (bytes) |
|-------|-------------|---------------|
| ATmega2560 | 373 | 9690 |
| ATmega328P | 251 | 8882 |
| ATmega32U4 | 373 | 9382 |
| ESP32 | 5360 | 30744 |
| ESP32-C3 | 968 | 29454 |
| ESP32-S2 | 1664 | 28812 |
| ESP32-S3 | 3504 | 38260 |
| RP2040 | 2380 | 8688 |
| RP2350 | 3472 | 8368 |
| SAM3X8E | 2432 | 22064 |

**Notes:**
- RAM = .data + .bss (static allocation)
- Flash = .text + .data (code + initialized data)
- Actual usage may vary based on enabled features
- All builds use release optimization (-Os)
- Values include peripheral drivers pulled in by the library:
  - ESP32: RMT, MCPWM, PCNT drivers (~15-20KB of the flash)
  - SAM: Timer/Counter drivers
- For detailed per-symbol analysis, use PlatformIO's "Project Inspect" feature
