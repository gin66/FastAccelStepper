# FastAccelStepper Memory Footprint

Generated: 2026-03-06 20:02:45

Library size measured using the difference method:
- Baseline: Empty Arduino sketch (setup/loop only)
- Test: UsageExample with basic stepper initialization

| Board | RAM (bytes) | Flash (bytes) |
|-------|-------------|---------------|
| ATmega2560 | 361 | 9590 |
| ATmega328P | 243 | 8756 |
| ATmega32U4 | 361 | 9026 |
| ESP32 | 5304 | 30184 |
| ESP32-C3 | 952 | 29504 |
| ESP32-S2 | 1664 | 28368 |
| ESP32-S3 | 3464 | 37776 |
| RP2040 | 2348 | 8548 |
| RP2350 | 3424 | 8260 |
| SAM3X8E | 2380 | 21756 |

**Notes:**
- RAM = .data + .bss (static allocation)
- Flash = .text + .data (code + initialized data)
- Actual usage may vary based on enabled features
- All builds use release optimization (-Os)
- Values include peripheral drivers pulled in by the library:
  - ESP32: RMT, MCPWM, PCNT drivers (~15-20KB of the flash)
  - SAM: Timer/Counter drivers
- For detailed per-symbol analysis, use PlatformIO's "Project Inspect" feature
