# FastAccelStepper Memory Footprint

Generated: 2026-03-19 16:12:40

Library size measured using the difference method:
- Baseline: Empty Arduino sketch (setup/loop only)
- Test: UsageExample with basic stepper initialization

| Board | RAM (bytes) | Flash (bytes) |
|-------|-------------|---------------|
| ATmega2560 | 370 | 9552 |
| ATmega328P | 249 | 8806 |
| ATmega32U4 | 370 | 9246 |
| ESP32 | 5304 | 30388 |
| ESP32-C3 | 968 | 29388 |
| ESP32-S2 | 1664 | 28580 |
| ESP32-S3 | 3472 | 37916 |
| RP2040 | 2380 | 8576 |
| RP2350 | 3472 | 8272 |
| SAM3X8E | 2432 | 21976 |

**Notes:**
- RAM = .data + .bss (static allocation)
- Flash = .text + .data (code + initialized data)
- Actual usage may vary based on enabled features
- All builds use release optimization (-Os)
- Values include peripheral drivers pulled in by the library:
  - ESP32: RMT, MCPWM, PCNT drivers (~15-20KB of the flash)
  - SAM: Timer/Counter drivers
- For detailed per-symbol analysis, use PlatformIO's "Project Inspect" feature
