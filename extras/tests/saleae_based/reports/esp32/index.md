# Saleae characterization report

- **Tests:** 25 run, 25 passed, 0 failed
- **Pass rate:** 100.0%
- **Latest result:** 2026-10-02T10:01:27Z

## Pass rate by tag

| tag | passed | run | rate |
|---|---|---|---|
| esp32_auto_1ch | 17 | 17 | 100.0% |
| esp32_auto_2ch | 3 | 3 | 100.0% |
| esp32_i2s_i2s | 1 | 1 | 100.0% |
| esp32_mcpwm_pcnt_mcpwm | 3 | 3 | 100.0% |
| esp32_rmt_mcpwm_pcnt_mixed_rmt_mcpwm | 1 | 1 | 100.0% |

## Results

| test | verdict | steps | period us (min–max) | pulse high us (min–max) | tag | note |
|---|---|---|---|---|---|---|
| SR_01 | PASS | 8/8 | 39.9583–40 | 15.5833–15.625 | esp32_auto_1ch | period 39.9583–40 us |
| SR_02 | PASS | 255/255 | 39.9167–40 | 15.5417–15.625 | esp32_auto_1ch | period 39.9167–40 us |
| SR_03 | PASS | 8/8 | 199.7917–199.875 | 15.5417–15.625 | esp32_auto_1ch | period 199.7917–199.875 us |
| SR_04 | PASS | 4/4 | 4092.4583–4092.5 | 15.5833–15.625 | esp32_auto_1ch | period 4092.4583–4092.5 us |
| SR_05 | PASS | 16/16 | 39.9583–40 | 15.5833–15.625 | esp32_auto_1ch | period 39.9583–40 us |
| SR_06 | PASS | 4/4 | 99.875–99.9167 | 15.625–15.625 | esp32_auto_1ch | period 99.875–99.9167 us |
| SR_07 | PASS | 2000/2000 | 39.9167–40 | 15.5417–15.625 | esp32_auto_1ch | period 39.9167–40 us |
| SR_08 | PASS | 4000/4000 | 39.9167–40 | 15.5417–15.625 | esp32_auto_1ch | period 39.9167–40 us |
| SR_09 | PASS | 10/10 | 39.9583–839.2917 | 15.5833–15.625 | esp32_auto_1ch | pause 800 us → gap 839.2917 us (expected 840) |
| SR_10 | PASS | 40/40 | 39.9167–1538.6667 | 15.5833–15.625 | esp32_auto_1ch | period 39.9167–1538.6667 us |
| SR_11 | PASS | 40/40 | 39.9167–1538.6667 | 15.5833–15.625 | esp32_auto_1ch | period 39.9167–1538.6667 us |
| SR_12 | PASS | 30/30 | 39.9583–1538.7083 | 15.5833–15.625 | esp32_auto_1ch | period 39.9583–1538.7083 us |
| SR_13 | PASS | 0/8 | — | — | esp32_auto_1ch |  |
| SR_14 | PASS | 2000/2000 | 39.9167–40 | 15.5417–15.625 | esp32_auto_2ch | skew 29.5417 us |
| SR_15 | PASS | 200/200 | 39.9167–40 | 15.5417–15.625 | esp32_auto_2ch | skew 27.1667 us |
| SR_16 | PASS | 64/64 | 39.9167–40 | 15.5833–15.625 | esp32_auto_2ch | period 39.9167–40 us |
| SR_17 | PASS | 200/200 | 39.9167–40 | 15.5833–15.625 | esp32_rmt_mcpwm_pcnt_mixed_rmt_mcpwm | skew 29.5417 us |
| SR_18 | PASS | 256/256 | 39.9167–439.625 | 15.5417–15.625 | esp32_mcpwm_pcnt_mcpwm | period 39.9167–439.625 us |
| SR_19 | PASS | 201/201 | 39.9167–40 | 15.5417–15.625 | esp32_mcpwm_pcnt_mcpwm | period 39.9167–40 us |
| SR_20 | PASS | 510/510 | 39.9167–439.6667 | 15.5417–15.625 | esp32_mcpwm_pcnt_mcpwm | period 39.9167–439.6667 us |
| SR_21 | PASS | 200/200 | 39.9167–40 | 15.5417–15.625 | esp32_auto_1ch | period 39.9167–40 us |
| SR_23 | PASS | 64/64 | 39.9167–40 | 15.5833–15.625 | esp32_i2s_i2s | period 39.9167–40 us |
| SR_25 | PASS | 11475/20000 | 39.9167–40 | 15.5417–15.625 | esp32_auto_1ch | stopped at 11475 of 20000, partial pulses 0 |
| SR_26 | PASS | 2/2 | 8184.9583–8184.9583 | 15.625–15.625 | esp32_auto_1ch | pause 4095.9375 us → gap 8184.9583 us (expected 8191.875) |
| SR_27 | PASS | 1/1 | — | 15.625–15.625 | esp32_auto_1ch |  |

## Cross-configuration comparison

More than one configuration is present, so the same test may have been measured under more than one. Rows only appear where a test was actually run in both.

_No test was run under more than one configuration in this results set, so there is nothing to compare side by side. Point `--baseline` at a results directory from another board or driver to get a comparison._

## Reading the numbers

Periods and pulse widths are reported as a **distribution** (min–max, spread, median), not a single average. A driver that holds a fixed pulse width shows min == max and that is itself the finding; one short pulse in ten thousand moves only the minimum. Pulse width is recorded rather than judged: the driver sets it, so its value is a property of the silicon and becomes the baseline a regression is measured against. Step counts and periods are what the library promises, and those are asserted.
