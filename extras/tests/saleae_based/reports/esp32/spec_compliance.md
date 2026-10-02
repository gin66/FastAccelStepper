# Spec compliance

Measured against what the library promises: the commanded period and the step count. Pulse width is not listed as a compliance item because the driver sets it and the library does not promise a value.

| test | metric | expected | measured | verdict |
|---|---|---|---|---|
| SR_01 | A inter-step period | 40 us (640 ticks) | 39.97 us | ✓ |
| SR_01 | A step count | 8 | 8 | ✓ |
| SR_02 | A inter-step period | 40 us (640 ticks) | 39.97 us | ✓ |
| SR_02 | A step count | 255 | 255 | ✓ |
| SR_03 | A inter-step period | 200 us (3200 ticks) | 199.83 us | ✓ |
| SR_03 | A step count | 8 | 8 | ✓ |
| SR_04 | A inter-step period | 4095.94 us (65535 ticks) | 4092.49 us | ✓ |
| SR_04 | A step count | 4 | 4 | ✓ |
| SR_05 | A inter-step period | 40 us (640 ticks) | 39.96 us | ✓ |
| SR_05 | A step count | 16 | 16 | ✓ |
| SR_06 | A inter-step period | 100 us (1600 ticks) | 99.92 us | ✓ |
| SR_06 | A step count | 4 | 4 | ✓ |
| SR_07 | A inter-step period | 40 us (640 ticks) | 39.97 us | ✓ |
| SR_07 | A step count | 2000 | 2000 | ✓ |
| SR_08 | A inter-step period | 40 us (640 ticks) | 39.97 us | ✓ |
| SR_08 | A step count | 4000 | 4000 | ✓ |
| SR_09 | A inter-step period | 40 us (640 ticks) | 128.78 us | ✗ |
| SR_09 | A step count | 10 | 10 | ✓ |
| SR_10 | A inter-step period | 40 us (640 ticks) | 78.4 us | ✗ |
| SR_10 | A step count | 40 | 40 | ✓ |
| SR_11 | A inter-step period | 40 us (640 ticks) | 78.4 us | ✗ |
| SR_11 | A step count | 40 | 40 | ✓ |
| SR_12 | A inter-step period | 40 us (640 ticks) | 143.33 us | ✗ |
| SR_12 | A step count | 30 | 30 | ✓ |
| SR_13 | A step count | 8 | 0 | ✗ |
| SR_14 | A inter-step period | 40 us (640 ticks) | 39.97 us | ✓ |
| SR_14 | A step count | 2000 | 2000 | ✓ |
| SR_14 | B inter-step period | 40 us (640 ticks) | 39.97 us | ✓ |
| SR_14 | B step count | 2000 | 2000 | ✓ |
| SR_15 | A inter-step period | 40 us (640 ticks) | 39.97 us | ✓ |
| SR_15 | A step count | 200 | 200 | ✓ |
| SR_15 | B inter-step period | 40 us (640 ticks) | 79.93 us | ✗ |
| SR_15 | B step count | 200 | 200 | ✓ |
| SR_16 | A inter-step period | 40 us (640 ticks) | 39.97 us | ✓ |
| SR_16 | A step count | 64 | 64 | ✓ |
| SR_16 | B inter-step period | 40 us (640 ticks) | 39.97 us | ✓ |
| SR_16 | B step count | 64 | 64 | ✓ |
| SR_17 | A inter-step period | 40 us (640 ticks) | 39.97 us | ✓ |
| SR_17 | A step count | 200 | 200 | ✓ |
| SR_17 | B inter-step period | 40 us (640 ticks) | 39.97 us | ✓ |
| SR_17 | B step count | 200 | 200 | ✓ |
| SR_18 | A inter-step period | 40 us (640 ticks) | 41.53 us | ✗ |
| SR_18 | A step count | 256 | 256 | ✓ |
| SR_19 | A inter-step period | 40 us (640 ticks) | 39.97 us | ✓ |
| SR_19 | A step count | 201 | 201 | ✓ |
| SR_20 | A inter-step period | 40 us (640 ticks) | 40.75 us | ✓ |
| SR_20 | A step count | 510 | 510 | ✓ |
| SR_21 | A inter-step period | 40 us (640 ticks) | 39.97 us | ✓ |
| SR_21 | A step count | 200 | 200 | ✓ |
| SR_23 | A inter-step period | 40 us (640 ticks) | 39.97 us | ✓ |
| SR_23 | A step count | 64 | 64 | ✓ |
| SR_25 | A inter-step period | 40 us (640 ticks) | 39.97 us | ✓ |
| SR_25 | A step count | 20000 | 11475 | ✗ |
| SR_26 | A inter-step period | 4095.94 us (65535 ticks) | 8184.96 us | ✗ |
| SR_26 | A step count | 2 | 2 | ✓ |
| SR_27 | A step count | 1 | 1 | ✓ |
