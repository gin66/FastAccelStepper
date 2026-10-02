# esp32_rmt_v2_1ch

17 tests, 17 passed.

- **Architecture:** esp32
- **Driver:** rmt_v2

| test | verdict | steps | period us (min–max) | pulse high us (min–max) | tag | note |
|---|---|---|---|---|---|---|
| SR_01 | PASS | 8/8 | 39.9583–40 | 15.5833–15.625 | esp32_rmt_v2_1ch | period 39.9583–40 us |
| SR_02 | PASS | 255/255 | 39.9167–40 | 15.5417–15.625 | esp32_rmt_v2_1ch | period 39.9167–40 us |
| SR_03 | PASS | 8/8 | 199.7917–199.875 | 15.5833–15.625 | esp32_rmt_v2_1ch | period 199.7917–199.875 us |
| SR_04 | PASS | 4/4 | 4092.4583–4092.5 | 15.5833–15.625 | esp32_rmt_v2_1ch | period 4092.4583–4092.5 us |
| SR_05 | PASS | 16/16 | 39.9167–40 | 15.5833–15.625 | esp32_rmt_v2_1ch | period 39.9167–40 us |
| SR_06 | PASS | 4/4 | 99.9167–99.9167 | 15.625–15.625 | esp32_rmt_v2_1ch | period 99.9167–99.9167 us |
| SR_07 | PASS | 2000/2000 | 39.9167–40 | 15.5417–15.625 | esp32_rmt_v2_1ch | period 39.9167–40 us |
| SR_08 | PASS | 4000/4000 | 39.9167–40 | 15.5417–15.625 | esp32_rmt_v2_1ch | period 39.9167–40 us |
| SR_09 | PASS | 10/10 | 39.9583–839.2917 | 15.5833–15.625 | esp32_rmt_v2_1ch | pause 800 us → gap 839.2917 us (expected 840) |
| SR_10 | PASS | 40/40 | 39.9167–1538.6667 | 15.5417–15.625 | esp32_rmt_v2_1ch | period 39.9167–1538.6667 us |
| SR_11 | PASS | 40/40 | 39.9167–1538.7083 | 15.5833–15.625 | esp32_rmt_v2_1ch | period 39.9167–1538.7083 us |
| SR_12 | PASS | 30/30 | 39.9583–1538.7083 | 15.5833–15.625 | esp32_rmt_v2_1ch | period 39.9583–1538.7083 us |
| SR_13 | PASS | 0/8 | — | — | esp32_rmt_v2_1ch |  |
| SR_21 | PASS | 200/200 | 39.9167–40 | 15.5417–15.625 | esp32_rmt_v2_1ch | period 39.9167–40 us |
| SR_25 | PASS | 11475/20000 | 39.9167–40 | 15.5417–15.625 | esp32_rmt_v2_1ch | stopped at 11475 of 20000, partial pulses 0 |
| SR_26 | PASS | 2/2 | 8184.9583–8184.9583 | 15.625–15.625 | esp32_rmt_v2_1ch | pause 4095.9375 us → gap 8184.9583 us (expected 8191.875) |
| SR_27 | PASS | 1/1 | — | 15.625–15.625 | esp32_rmt_v2_1ch |  |
