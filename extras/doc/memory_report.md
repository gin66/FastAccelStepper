# FastAccelStepper Memory Footprint

Generated: 2026-03-13 21:18:52

Library size measured using the difference method:
- Baseline: Empty Arduino sketch (setup/loop only)
- Test: UsageExample with basic stepper initialization

| Board | RAM (bytes) | Flash (bytes) |
|-------|-------------|---------------|
| ATmega2560 | 373 | 9690 |
| ATmega328P | 251 | 8876 |
| ATmega32U4 | 373 | 9342 |
| ESP32 | 5360 | 30896 |
| ESP32-C3 | 968 | 29570 |
| ESP32-S2 | 1664 | 28920 |
| ESP32-S3 | 3504 | 38368 |
| RP2040 | 2380 | 8676 |
| RP2350 | 3472 | 8340 |
| SAM3X8E | 2432 | 22080 |

**Notes:**
- RAM = .data + .bss (static allocation)
- Flash = .text + .data (code + initialized data)
- Actual usage may vary based on enabled features
- All builds use release optimization (-Os)
- Values include peripheral drivers pulled in by the library:
  - ESP32: RMT, MCPWM, PCNT drivers (~15-20KB of the flash)
  - SAM: Timer/Counter drivers
- For detailed per-symbol analysis, use PlatformIO's "Project Inspect" feature
