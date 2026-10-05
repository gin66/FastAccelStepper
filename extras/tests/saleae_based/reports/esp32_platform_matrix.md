# Saleae harness — ESP32 platform-release matrix

- **Generated:** 2026-10-05 12:57
- **Board:** ESP32-DevKitC, Saleae Logic 8ch (`fx2lafw:conn=8.88`), serial `/dev/cu.usbserial-0001`
- **Firmware rows:** 6 — one build+flash each
- **Matrix definition:** `scripts/harness.py` (`RELEASE_MATRIX`, `release_runs()`)
- **Raw results:** `results/` (git-ignored), captures in `capture/`; per-run logs under `/tmp/saleae_matrix`

Every row is a firmware flashed **once**; all driver and combination runs below were measured against that one flash. Drivers come from asking the board what it accepts, so a column that is absent is a driver this SDK has no queues for.

## Firmware matrix

| framework | version | ESP-IDF | PlatformIO env | drivers the board accepts | catalogue | scale sweeps | sync | measured |
|---|---|---|---|---|---|---|---|---|
| arduino | 4.4.0 | 4.4.7 | `esp32_V4_4_0` | rmt, mcpwm_pcnt | catalogue | 2 | sync | 2026-10-05 12:22:54 |
| arduino | 5.3.0 | 4.4.7 | `esp32_V5_3_0` | rmt, mcpwm_pcnt | catalogue | 2 | sync | 2026-10-05 12:26:39 |
| arduino | 6.13.0 | 4.4.7 | `esp32_V6_13_0` | rmt, mcpwm_pcnt | catalogue | 2 | sync | 2026-10-05 12:30:12 |
| idf | 5.3.0 | 4.4.3 | `esp32_idf_V5_3_0` | rmt, mcpwm_pcnt | catalogue | 2 | sync | 2026-10-05 12:33:47 |
| idf | 6.13.0 | 5.5.3 | `esp32_idf_V6_13_0` | rmt, mcpwm_pcnt, i2s_direct, i2s_mux | catalogue | 4 | sync | 2026-10-05 12:37:50 |
| idf | 7.1.2 | 6.1.0 | `esp32_idf_V7_1_2` | rmt, mcpwm_pcnt, i2s_direct, i2s_mux | catalogue | 4 | sync | 2026-10-05 12:47:36 |

> The rows were not measured in one sitting: a row whose upload failed is re-run on its own, and the *measured* column is when each row's flash happened. Rows that share a timestamp were measured against the board back to back.

## Scenario catalogue (SR_00 … SR_30)

Unparameterized: every scenario runs its own fixed program and is judged by its own evaluator.

| test | arduino-4.4.0 | arduino-5.3.0 | arduino-6.13.0 | idf-5.3.0 | idf-6.13.0 | idf-7.1.2 |
|---|---|---|---|---|---|---|
| SR_00 | pass | pass | pass | pass | pass | pass |
| SR_01 | pass [vcd](capture/sr_01_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_01_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_01_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_01_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_01_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_01_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_02 | pass [vcd](capture/sr_02_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_02_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_02_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_02_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_02_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_02_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_03 | pass [vcd](capture/sr_03_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_03_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_03_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_03_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_03_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_03_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_04 | pass [vcd](capture/sr_04_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_04_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_04_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_04_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_04_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_04_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_05 | pass [vcd](capture/sr_05_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_05_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_05_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_05_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_05_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_05_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_06 | pass [vcd](capture/sr_06_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_06_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_06_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_06_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_06_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_06_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_07 | pass [vcd](capture/sr_07_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_07_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_07_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_07_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_07_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_07_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_08 | pass [vcd](capture/sr_08_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_08_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_08_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_08_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_08_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_08_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_09 | pass [vcd](capture/sr_09_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_09_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_09_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_09_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_09_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_09_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_10 | pass [vcd](capture/sr_10_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_10_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_10_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_10_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_10_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_10_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_11 | pass [vcd](capture/sr_11_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_11_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_11_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_11_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_11_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_11_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_12 | pass [vcd](capture/sr_12_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_12_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_12_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_12_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_12_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_12_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_13 | pass [vcd](capture/sr_13_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_13_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_13_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_13_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_13_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_13_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_14 | pass [vcd](capture/sr_14_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_14_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_14_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_14_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_14_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_14_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_15 | pass [vcd](capture/sr_15_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_15_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_15_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_15_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_15_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_15_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_16 | pass [vcd](capture/sr_16_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_16_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_16_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_16_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_16_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_16_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_17 | pass [vcd](capture/sr_17_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_17_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_17_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_17_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_17_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_17_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_18 | pass [vcd](capture/sr_18_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_18_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_18_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_18_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_18_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_18_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_19 | pass [vcd](capture/sr_19_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_19_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_19_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_19_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_19_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_19_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_20 | pass [vcd](capture/sr_20_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_20_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_20_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_20_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_20_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_20_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_21 | pass [vcd](capture/sr_21_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_21_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_21_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_21_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_21_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_21_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_23 | n/a (no such driver) | n/a (no such driver) | n/a (no such driver) | n/a (no such driver) | pass [vcd](capture/sr_23_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_23_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_25 | pass [vcd](capture/sr_25_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_25_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_25_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_25_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_25_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_25_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_26 | pass [vcd](capture/sr_26_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_26_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_26_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_26_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_26_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_26_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_27 | pass [vcd](capture/sr_27_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_27_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_27_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_27_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_27_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_27_esp32_idf7_1_2_rmt1_dir.vcd) |
| SR_30 | pass [vcd](capture/sr_30_esp32_arduino4_4_0_rmt1_dir.vcd) | pass [vcd](capture/sr_30_esp32_arduino5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_30_esp32_arduino6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_30_esp32_idf5_3_0_rmt1_dir.vcd) | pass [vcd](capture/sr_30_esp32_idf6_13_0_rmt1_dir.vcd) | pass [vcd](capture/sr_30_esp32_idf7_1_2_rmt1_dir.vcd) |

`pass` / `FAIL` / `incomplete` / `refused (bound)` / `n/a` / `skip` are the recorded verdicts; a capture link opens the VCD the verdict came from. `skip` is either *not implemented* (SR_22, SR_24, SR_28, SR_29) or *SR_00 failed*, which is the harness refusing to measure on dead channels. **`FAIL` and `incomplete` are the only cells here that are findings** — see Findings.

`n/a` means this build has no queues for the driver the scenario CONFIGS (`ERR CONFIG no such driver`), so the scenario was never applicable to this row and its `failed` verdict is not a defect. `refused (bound)` is the board declining a CONFIG at a limit, which is what `scale` and `sync` exist to find.

## Findings

Defects only: a panic, a crash, a step count or a period that is wrong, and measurements that could not be made at all. Refusals are **not** listed here — a `scale` sweep is *asked* where a driver's limit is and a refusal is its answer, so those live in the sweep tables below where they read as a bound. The one exception is a scenario that names a driver the build does not have; that is a capability answer, not a failure, and it is counted at the bottom of this section rather than tabulated.

| matrix row | what | measurements | note |
|---|---|---|---|
| idf-6.13.0 | incomplete | 1 | incomplete capture: S2 missing (stepper B) |
| idf-7.1.2 | incomplete | 1 | incomplete capture: S2 missing (stepper B) |

Expanded below, one line per measurement.

## Every finding, one line each

| matrix row | test | class | note |
|---|---|---|---|
| idf-6.13.0 | sync i2s_mux+i2s_mux | incomplete | incomplete capture: S2 missing (stepper B) |
| idf-7.1.2 | sync i2s_mux+i2s_mux | incomplete | incomplete capture: S2 missing (stepper B) |

Not findings, and not listed above: **24** refusal(s), which are the measured limits in the sweep tables below, and **32** `no such driver` answer(s), which are scenarios this build has no queues for. A full accounting of every result is in `results/` and in `results/tag_index.json`.

## Driver scale sweeps (how many steppers in parallel)

`nodir`, one shared program, each stepper's own step count and period asserted. The board decides where the sweep stops: a CONFIG refusal is the measured bound.

**arduino-4.4.0 / scale:rmt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_rmt_nodir_n1_esp32_arduino4_4_0_rmt_scalenodir_rmtnodirn1.vcd) | 64/64 | 10.00 |  |
| 2 | pass [vcd](capture/scale_rmt_nodir_n2_esp32_arduino4_4_0_rmt_scalenodir_rmtnodirn2.vcd) | 64/64 | 10.00 |  |
| 3 | pass [vcd](capture/scale_rmt_nodir_n3_esp32_arduino4_4_0_rmt_scalenodir_rmtnodirn3.vcd) | 64/64 | 10.00 |  |
| 4 | pass [vcd](capture/scale_rmt_nodir_n4_esp32_arduino4_4_0_rmt_scalenodir_rmtnodirn4.vcd) | 64/64 | 10.00 |  |
| 5 | pass [vcd](capture/scale_rmt_nodir_n5_esp32_arduino4_4_0_rmt_scalenodir_rmtnodirn5.vcd) | 64/64 | 10.00 |  |
| 6 | pass [vcd](capture/scale_rmt_nodir_n6_esp32_arduino4_4_0_rmt_scalenodir_rmtnodirn6.vcd) | 64/64 | 10.00 |  |
| 7 | pass [vcd](capture/scale_rmt_nodir_n7_esp32_arduino4_4_0_rmt_scalenodir_rmtnodirn7.vcd) | 64/64 | 10.00 |  |
| 8 | pass [vcd](capture/scale_rmt_nodir_n8_esp32_arduino4_4_0_rmt_scalenodir_rmtnodirn8.vcd) | 64/64 | 10.00 |  |

**arduino-4.4.0 / scale:mcpwm_pcnt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n1_esp32_arduino4_4_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn1.vcd) | 64/64 | 10.00 |  |
| 2 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n2_esp32_arduino4_4_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn2.vcd) | 64/64 | 10.00 |  |
| 3 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n3_esp32_arduino4_4_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn3.vcd) | 64/64 | 10.00 |  |
| 4 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n4_esp32_arduino4_4_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn4.vcd) | 64/64 | 10.00 |  |
| 5 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n5_esp32_arduino4_4_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn5.vcd) | 64/64 | 10.00 |  |
| 6 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n6_esp32_arduino4_4_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn6.vcd) | 64/64 | 10.00 |  |
| 7 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |

**arduino-5.3.0 / scale:rmt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_rmt_nodir_n1_esp32_arduino5_3_0_rmt_scalenodir_rmtnodirn1.vcd) | 64/64 | 10.00 |  |
| 2 | pass [vcd](capture/scale_rmt_nodir_n2_esp32_arduino5_3_0_rmt_scalenodir_rmtnodirn2.vcd) | 64/64 | 10.00 |  |
| 3 | pass [vcd](capture/scale_rmt_nodir_n3_esp32_arduino5_3_0_rmt_scalenodir_rmtnodirn3.vcd) | 64/64 | 10.00 |  |
| 4 | pass [vcd](capture/scale_rmt_nodir_n4_esp32_arduino5_3_0_rmt_scalenodir_rmtnodirn4.vcd) | 64/64 | 10.00 |  |
| 5 | pass [vcd](capture/scale_rmt_nodir_n5_esp32_arduino5_3_0_rmt_scalenodir_rmtnodirn5.vcd) | 64/64 | 10.00 |  |
| 6 | pass [vcd](capture/scale_rmt_nodir_n6_esp32_arduino5_3_0_rmt_scalenodir_rmtnodirn6.vcd) | 64/64 | 10.00 |  |
| 7 | pass [vcd](capture/scale_rmt_nodir_n7_esp32_arduino5_3_0_rmt_scalenodir_rmtnodirn7.vcd) | 64/64 | 10.00 |  |
| 8 | pass [vcd](capture/scale_rmt_nodir_n8_esp32_arduino5_3_0_rmt_scalenodir_rmtnodirn8.vcd) | 64/64 | 10.00 |  |

**arduino-5.3.0 / scale:mcpwm_pcnt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n1_esp32_arduino5_3_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn1.vcd) | 64/64 | 10.00 |  |
| 2 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n2_esp32_arduino5_3_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn2.vcd) | 64/64 | 10.00 |  |
| 3 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n3_esp32_arduino5_3_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn3.vcd) | 64/64 | 10.00 |  |
| 4 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n4_esp32_arduino5_3_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn4.vcd) | 64/64 | 10.00 |  |
| 5 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n5_esp32_arduino5_3_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn5.vcd) | 64/64 | 10.00 |  |
| 6 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n6_esp32_arduino5_3_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn6.vcd) | 64/64 | 10.00 |  |
| 7 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |

**arduino-6.13.0 / scale:rmt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_rmt_nodir_n1_esp32_arduino6_13_0_rmt_scalenodir_rmtnodirn1.vcd) | 64/64 | 10.00 |  |
| 2 | pass [vcd](capture/scale_rmt_nodir_n2_esp32_arduino6_13_0_rmt_scalenodir_rmtnodirn2.vcd) | 64/64 | 10.00 |  |
| 3 | pass [vcd](capture/scale_rmt_nodir_n3_esp32_arduino6_13_0_rmt_scalenodir_rmtnodirn3.vcd) | 64/64 | 10.00 |  |
| 4 | pass [vcd](capture/scale_rmt_nodir_n4_esp32_arduino6_13_0_rmt_scalenodir_rmtnodirn4.vcd) | 64/64 | 10.00 |  |
| 5 | pass [vcd](capture/scale_rmt_nodir_n5_esp32_arduino6_13_0_rmt_scalenodir_rmtnodirn5.vcd) | 64/64 | 10.00 |  |
| 6 | pass [vcd](capture/scale_rmt_nodir_n6_esp32_arduino6_13_0_rmt_scalenodir_rmtnodirn6.vcd) | 64/64 | 10.00 |  |
| 7 | pass [vcd](capture/scale_rmt_nodir_n7_esp32_arduino6_13_0_rmt_scalenodir_rmtnodirn7.vcd) | 64/64 | 10.00 |  |
| 8 | pass [vcd](capture/scale_rmt_nodir_n8_esp32_arduino6_13_0_rmt_scalenodir_rmtnodirn8.vcd) | 64/64 | 10.00 |  |

**arduino-6.13.0 / scale:mcpwm_pcnt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n1_esp32_arduino6_13_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn1.vcd) | 64/64 | 10.00 |  |
| 2 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n2_esp32_arduino6_13_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn2.vcd) | 64/64 | 10.00 |  |
| 3 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n3_esp32_arduino6_13_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn3.vcd) | 64/64 | 10.00 |  |
| 4 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n4_esp32_arduino6_13_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn4.vcd) | 64/64 | 10.00 |  |
| 5 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n5_esp32_arduino6_13_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn5.vcd) | 64/64 | 10.00 |  |
| 6 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n6_esp32_arduino6_13_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn6.vcd) | 64/64 | 10.00 |  |
| 7 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |

**idf-5.3.0 / scale:rmt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_rmt_nodir_n1_esp32_idf5_3_0_rmt_scalenodir_rmtnodirn1.vcd) | 64/64 | 10.00 |  |
| 2 | pass [vcd](capture/scale_rmt_nodir_n2_esp32_idf5_3_0_rmt_scalenodir_rmtnodirn2.vcd) | 64/64 | 10.00 |  |
| 3 | pass [vcd](capture/scale_rmt_nodir_n3_esp32_idf5_3_0_rmt_scalenodir_rmtnodirn3.vcd) | 64/64 | 10.00 |  |
| 4 | pass [vcd](capture/scale_rmt_nodir_n4_esp32_idf5_3_0_rmt_scalenodir_rmtnodirn4.vcd) | 64/64 | 10.00 |  |
| 5 | pass [vcd](capture/scale_rmt_nodir_n5_esp32_idf5_3_0_rmt_scalenodir_rmtnodirn5.vcd) | 64/64 | 10.00 |  |
| 6 | pass [vcd](capture/scale_rmt_nodir_n6_esp32_idf5_3_0_rmt_scalenodir_rmtnodirn6.vcd) | 64/64 | 10.00 |  |
| 7 | pass [vcd](capture/scale_rmt_nodir_n7_esp32_idf5_3_0_rmt_scalenodir_rmtnodirn7.vcd) | 64/64 | 10.00 |  |
| 8 | pass [vcd](capture/scale_rmt_nodir_n8_esp32_idf5_3_0_rmt_scalenodir_rmtnodirn8.vcd) | 64/64 | 10.00 |  |

**idf-5.3.0 / scale:mcpwm_pcnt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n1_esp32_idf5_3_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn1.vcd) | 64/64 | 10.00 |  |
| 2 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n2_esp32_idf5_3_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn2.vcd) | 64/64 | 10.00 |  |
| 3 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n3_esp32_idf5_3_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn3.vcd) | 64/64 | 10.00 |  |
| 4 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n4_esp32_idf5_3_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn4.vcd) | 64/64 | 10.00 |  |
| 5 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n5_esp32_idf5_3_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn5.vcd) | 64/64 | 10.00 |  |
| 6 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n6_esp32_idf5_3_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn6.vcd) | 64/64 | 10.00 |  |
| 7 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |

**idf-6.13.0 / scale:rmt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_rmt_nodir_n1_esp32_idf6_13_0_rmt_scalenodir_rmtnodirn1.vcd) | 64/64 | 10.00 |  |
| 2 | pass [vcd](capture/scale_rmt_nodir_n2_esp32_idf6_13_0_rmt_scalenodir_rmtnodirn2.vcd) | 64/64 | 10.00 |  |
| 3 | pass [vcd](capture/scale_rmt_nodir_n3_esp32_idf6_13_0_rmt_scalenodir_rmtnodirn3.vcd) | 64/64 | 10.00 |  |
| 4 | pass [vcd](capture/scale_rmt_nodir_n4_esp32_idf6_13_0_rmt_scalenodir_rmtnodirn4.vcd) | 64/64 | 10.00 |  |
| 5 | pass [vcd](capture/scale_rmt_nodir_n5_esp32_idf6_13_0_rmt_scalenodir_rmtnodirn5.vcd) | 64/64 | 10.00 |  |
| 6 | pass [vcd](capture/scale_rmt_nodir_n6_esp32_idf6_13_0_rmt_scalenodir_rmtnodirn6.vcd) | 64/64 | 10.00 |  |
| 7 | pass [vcd](capture/scale_rmt_nodir_n7_esp32_idf6_13_0_rmt_scalenodir_rmtnodirn7.vcd) | 64/64 | 10.00 |  |
| 8 | pass [vcd](capture/scale_rmt_nodir_n8_esp32_idf6_13_0_rmt_scalenodir_rmtnodirn8.vcd) | 64/64 | 10.00 |  |

**idf-6.13.0 / scale:mcpwm_pcnt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n1_esp32_idf6_13_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn1.vcd) | 64/64 | 10.00 |  |
| 2 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n2_esp32_idf6_13_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn2.vcd) | 64/64 | 10.00 |  |
| 3 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n3_esp32_idf6_13_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn3.vcd) | 64/64 | 10.00 |  |
| 4 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n4_esp32_idf6_13_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn4.vcd) | 64/64 | 10.00 |  |
| 5 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n5_esp32_idf6_13_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn5.vcd) | 64/64 | 10.00 |  |
| 6 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n6_esp32_idf6_13_0_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn6.vcd) | 64/64 | 10.00 |  |
| 7 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |

**idf-6.13.0 / scale:i2s_direct**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_i2s_direct_nodir_n1_esp32_idf6_13_0_i2s_direct_scalenodir_i2s_directnodirn1.vcd) | 64/64 | 10.00 |  |
| 2 | pass [vcd](capture/scale_i2s_direct_nodir_n2_esp32_idf6_13_0_i2s_direct_scalenodir_i2s_directnodirn2.vcd) | 64/64 | 10.00 |  |
| 3 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 4 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 5 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 6 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 7 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |

**idf-6.13.0 / scale:i2s_mux**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_i2s_mux_nodir_n1_esp32_idf6_13_0_i2s_mux_scalenodir_i2s_muxnodirn1.vcd) | 64/64 | 24.93 |  |
| 2 | pass [vcd](capture/scale_i2s_mux_nodir_n2_esp32_idf6_13_0_i2s_mux_scalenodir_i2s_muxnodirn2.vcd) | 64/64 | 24.93 |  |
| 3 | pass [vcd](capture/scale_i2s_mux_nodir_n3_esp32_idf6_13_0_i2s_mux_scalenodir_i2s_muxnodirn3.vcd) | 64/64 | 24.93 |  |
| 4 | pass [vcd](capture/scale_i2s_mux_nodir_n4_esp32_idf6_13_0_i2s_mux_scalenodir_i2s_muxnodirn4.vcd) | 64/64 | 24.93 |  |
| 5 | pass [vcd](capture/scale_i2s_mux_nodir_n5_esp32_idf6_13_0_i2s_mux_scalenodir_i2s_muxnodirn5.vcd) | 64/64 | 24.93 |  |
| 6 | pass [vcd](capture/scale_i2s_mux_nodir_n6_esp32_idf6_13_0_i2s_mux_scalenodir_i2s_muxnodirn6.vcd) | 64/64 | 24.93 |  |
| 7 | pass [vcd](capture/scale_i2s_mux_nodir_n7_esp32_idf6_13_0_i2s_mux_scalenodir_i2s_muxnodirn7.vcd) | 64/64 | 24.93 |  |
| 8 | pass [vcd](capture/scale_i2s_mux_nodir_n8_esp32_idf6_13_0_i2s_mux_scalenodir_i2s_muxnodirn8.vcd) | 64/64 | 24.93 |  |

**idf-7.1.2 / scale:rmt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_rmt_nodir_n1_esp32_idf7_1_2_rmt_scalenodir_rmtnodirn1.vcd) | 64/64 | 10.00 |  |
| 2 | pass [vcd](capture/scale_rmt_nodir_n2_esp32_idf7_1_2_rmt_scalenodir_rmtnodirn2.vcd) | 64/64 | 10.00 |  |
| 3 | pass [vcd](capture/scale_rmt_nodir_n3_esp32_idf7_1_2_rmt_scalenodir_rmtnodirn3.vcd) | 64/64 | 10.00 |  |
| 4 | pass [vcd](capture/scale_rmt_nodir_n4_esp32_idf7_1_2_rmt_scalenodir_rmtnodirn4.vcd) | 64/64 | 10.00 |  |
| 5 | pass [vcd](capture/scale_rmt_nodir_n5_esp32_idf7_1_2_rmt_scalenodir_rmtnodirn5.vcd) | 64/64 | 10.00 |  |
| 6 | pass [vcd](capture/scale_rmt_nodir_n6_esp32_idf7_1_2_rmt_scalenodir_rmtnodirn6.vcd) | 64/64 | 10.00 |  |
| 7 | pass [vcd](capture/scale_rmt_nodir_n7_esp32_idf7_1_2_rmt_scalenodir_rmtnodirn7.vcd) | 64/64 | 10.00 |  |
| 8 | pass [vcd](capture/scale_rmt_nodir_n8_esp32_idf7_1_2_rmt_scalenodir_rmtnodirn8.vcd) | 64/64 | 10.00 |  |

**idf-7.1.2 / scale:mcpwm_pcnt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n1_esp32_idf7_1_2_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn1.vcd) | 64/64 | 10.00 |  |
| 2 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n2_esp32_idf7_1_2_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn2.vcd) | 64/64 | 10.00 |  |
| 3 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n3_esp32_idf7_1_2_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn3.vcd) | 64/64 | 10.00 |  |
| 4 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n4_esp32_idf7_1_2_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn4.vcd) | 64/64 | 10.00 |  |
| 5 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n5_esp32_idf7_1_2_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn5.vcd) | 64/64 | 10.00 |  |
| 6 | pass [vcd](capture/scale_mcpwm_pcnt_nodir_n6_esp32_idf7_1_2_mcpwm_pcnt_scalenodir_mcpwm_pcntnodirn6.vcd) | 64/64 | 10.00 |  |
| 7 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |

**idf-7.1.2 / scale:i2s_direct**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_i2s_direct_nodir_n1_esp32_idf7_1_2_i2s_direct_scalenodir_i2s_directnodirn1.vcd) | 64/64 | 10.00 |  |
| 2 | pass [vcd](capture/scale_i2s_direct_nodir_n2_esp32_idf7_1_2_i2s_direct_scalenodir_i2s_directnodirn2.vcd) | 64/64 | 10.00 |  |
| 3 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 4 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 5 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 6 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 7 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |

**idf-7.1.2 / scale:i2s_mux**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass [vcd](capture/scale_i2s_mux_nodir_n1_esp32_idf7_1_2_i2s_mux_scalenodir_i2s_muxnodirn1.vcd) | 64/64 | 24.93 |  |
| 2 | pass [vcd](capture/scale_i2s_mux_nodir_n2_esp32_idf7_1_2_i2s_mux_scalenodir_i2s_muxnodirn2.vcd) | 64/64 | 24.93 |  |
| 3 | pass [vcd](capture/scale_i2s_mux_nodir_n3_esp32_idf7_1_2_i2s_mux_scalenodir_i2s_muxnodirn3.vcd) | 64/64 | 24.93 |  |
| 4 | pass [vcd](capture/scale_i2s_mux_nodir_n4_esp32_idf7_1_2_i2s_mux_scalenodir_i2s_muxnodirn4.vcd) | 64/64 | 24.93 |  |
| 5 | pass [vcd](capture/scale_i2s_mux_nodir_n5_esp32_idf7_1_2_i2s_mux_scalenodir_i2s_muxnodirn5.vcd) | 64/64 | 24.93 |  |
| 6 | pass [vcd](capture/scale_i2s_mux_nodir_n6_esp32_idf7_1_2_i2s_mux_scalenodir_i2s_muxnodirn6.vcd) | 64/64 | 24.93 |  |
| 7 | pass [vcd](capture/scale_i2s_mux_nodir_n7_esp32_idf7_1_2_i2s_mux_scalenodir_i2s_muxnodirn7.vcd) | 64/64 | 24.93 |  |
| 8 | pass [vcd](capture/scale_i2s_mux_nodir_n8_esp32_idf7_1_2_i2s_mux_scalenodir_i2s_muxnodirn8.vcd) | 64/64 | 24.93 |  |

## Driver combinations (synchronized start)

Every driver-list combination this board could connect, two steppers each, each at its own period. First-step skew is **reported, not gated** (eval_sync): how closely two steppers begin is a property of the pulse driver and of the interrupt latency at that instant, not a correctness property of the queue — so read the ratio in step periods, which is the only comparable form of it. What *is* asserted per stepper is its own commanded step count and period.

**arduino-4.4.0 / sync**

| drivers | verdict | first-step skew us | in step periods | note |
|---|---|---|---|---|
| i2s_direct+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_direct+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_mux+i2s_mux | n/a (no such driver) | – | – | DONE 0 |
| mcpwm_pcnt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+mcpwm_pcnt | pass [vcd](capture/sync_mcpwm_pcnt+mcpwm_pcnt_dir_n2_esp32_arduino4_4_0_rmt_syncdir_mcpwm_pcntdirn2.vcd) | 3.0 | 0.30 |  |
| rmt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+mcpwm_pcnt | pass [vcd](capture/sync_rmt+mcpwm_pcnt_dir_n2_esp32_arduino4_4_0_rmt_syncdir_rmt+mcpwm_pcntdirn2.vcd) | 15.5 | 1.55 |  |
| rmt+rmt | pass [vcd](capture/sync_rmt+rmt_dir_n2_esp32_arduino4_4_0_rmt_syncdir_rmtdirn2.vcd) | 21.25 | 2.12 |  |

**arduino-5.3.0 / sync**

| drivers | verdict | first-step skew us | in step periods | note |
|---|---|---|---|---|
| i2s_direct+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_direct+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_mux+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+mcpwm_pcnt | pass [vcd](capture/sync_mcpwm_pcnt+mcpwm_pcnt_dir_n2_esp32_arduino5_3_0_rmt_syncdir_mcpwm_pcntdirn2.vcd) | 3.25 | 0.33 |  |
| rmt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+mcpwm_pcnt | pass [vcd](capture/sync_rmt+mcpwm_pcnt_dir_n2_esp32_arduino5_3_0_rmt_syncdir_rmt+mcpwm_pcntdirn2.vcd) | 31.25 | 3.12 |  |
| rmt+rmt | pass [vcd](capture/sync_rmt+rmt_dir_n2_esp32_arduino5_3_0_rmt_syncdir_rmtdirn2.vcd) | 36.75 | 3.67 |  |

**arduino-6.13.0 / sync**

| drivers | verdict | first-step skew us | in step periods | note |
|---|---|---|---|---|
| i2s_direct+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_direct+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_mux+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+mcpwm_pcnt | pass [vcd](capture/sync_mcpwm_pcnt+mcpwm_pcnt_dir_n2_esp32_arduino6_13_0_rmt_syncdir_mcpwm_pcntdirn2.vcd) | 3.25 | 0.33 |  |
| rmt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+mcpwm_pcnt | pass [vcd](capture/sync_rmt+mcpwm_pcnt_dir_n2_esp32_arduino6_13_0_rmt_syncdir_rmt+mcpwm_pcntdirn2.vcd) | 19.25 | 1.93 |  |
| rmt+rmt | pass [vcd](capture/sync_rmt+rmt_dir_n2_esp32_arduino6_13_0_rmt_syncdir_rmtdirn2.vcd) | 17.25 | 1.73 |  |

**idf-5.3.0 / sync**

| drivers | verdict | first-step skew us | in step periods | note |
|---|---|---|---|---|
| i2s_direct+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_direct+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_mux+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+mcpwm_pcnt | pass [vcd](capture/sync_mcpwm_pcnt+mcpwm_pcnt_dir_n2_esp32_idf5_3_0_rmt_syncdir_mcpwm_pcntdirn2.vcd) | 4.5 | 0.45 |  |
| rmt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+mcpwm_pcnt | pass [vcd](capture/sync_rmt+mcpwm_pcnt_dir_n2_esp32_idf5_3_0_rmt_syncdir_rmt+mcpwm_pcntdirn2.vcd) | 44.5 | 4.45 |  |
| rmt+rmt | pass [vcd](capture/sync_rmt+rmt_dir_n2_esp32_idf5_3_0_rmt_syncdir_rmtdirn2.vcd) | 27.5 | 2.75 |  |

**idf-6.13.0 / sync**

| drivers | verdict | first-step skew us | in step periods | note |
|---|---|---|---|---|
| i2s_direct+i2s_direct | pass [vcd](capture/sync_i2s_direct+i2s_direct_dir_n2_esp32_idf6_13_0_rmt_syncdir_i2s_directdirn2.vcd) | 38.8333 | 3.89 |  |
| i2s_direct+i2s_mux | pass [vcd](capture/sync_i2s_direct+i2s_mux_dir_n2_esp32_idf6_13_0_rmt_syncdir_i2s_direct+i2s_muxdirn2.vcd) | 232.5 | 9.31 |  |
| i2s_mux+i2s_mux | **incomplete** [vcd](capture/sync_i2s_mux+i2s_mux_dir_n2_esp32_idf6_13_0_rmt_syncdir_i2s_muxdirn2.vcd) | – | – | incomplete capture: S2 missing (stepper B) |
| mcpwm_pcnt+i2s_direct | pass [vcd](capture/sync_mcpwm_pcnt+i2s_direct_dir_n2_esp32_idf6_13_0_rmt_syncdir_mcpwm_pcnt+i2s_directdirn2.vcd) | 683.1667 | 68.37 |  |
| mcpwm_pcnt+i2s_mux | pass [vcd](capture/sync_mcpwm_pcnt+i2s_mux_dir_n2_esp32_idf6_13_0_rmt_syncdir_mcpwm_pcnt+i2s_muxdirn2.vcd) | 781.4167 | 31.28 |  |
| mcpwm_pcnt+mcpwm_pcnt | pass [vcd](capture/sync_mcpwm_pcnt+mcpwm_pcnt_dir_n2_esp32_idf6_13_0_rmt_syncdir_mcpwm_pcntdirn2.vcd) | 13.3333 | 1.33 |  |
| rmt+i2s_direct | pass [vcd](capture/sync_rmt+i2s_direct_dir_n2_esp32_idf6_13_0_rmt_syncdir_rmt+i2s_directdirn2.vcd) | 697.0417 | 69.76 |  |
| rmt+i2s_mux | pass [vcd](capture/sync_rmt+i2s_mux_dir_n2_esp32_idf6_13_0_rmt_syncdir_rmt+i2s_muxdirn2.vcd) | 1051.1667 | 42.08 |  |
| rmt+mcpwm_pcnt | pass [vcd](capture/sync_rmt+mcpwm_pcnt_dir_n2_esp32_idf6_13_0_rmt_syncdir_rmt+mcpwm_pcntdirn2.vcd) | 73.2917 | 7.33 |  |
| rmt+rmt | pass [vcd](capture/sync_rmt+rmt_dir_n2_esp32_idf6_13_0_rmt_syncdir_rmtdirn2.vcd) | 67.625 | 6.77 |  |

**idf-7.1.2 / sync**

| drivers | verdict | first-step skew us | in step periods | note |
|---|---|---|---|---|
| i2s_direct+i2s_direct | pass [vcd](capture/sync_i2s_direct+i2s_direct_dir_n2_esp32_idf7_1_2_rmt_syncdir_i2s_directdirn2.vcd) | 412.5 | 41.28 |  |
| i2s_direct+i2s_mux | pass [vcd](capture/sync_i2s_direct+i2s_mux_dir_n2_esp32_idf7_1_2_rmt_syncdir_i2s_direct+i2s_muxdirn2.vcd) | 227.8333 | 9.12 |  |
| i2s_mux+i2s_mux | **incomplete** [vcd](capture/sync_i2s_mux+i2s_mux_dir_n2_esp32_idf7_1_2_rmt_syncdir_i2s_muxdirn2.vcd) | – | – | incomplete capture: S2 missing (stepper B) |
| mcpwm_pcnt+i2s_direct | pass [vcd](capture/sync_mcpwm_pcnt+i2s_direct_dir_n2_esp32_idf7_1_2_rmt_syncdir_mcpwm_pcnt+i2s_directdirn2.vcd) | 846.0833 | 84.68 |  |
| mcpwm_pcnt+i2s_mux | pass [vcd](capture/sync_mcpwm_pcnt+i2s_mux_dir_n2_esp32_idf7_1_2_rmt_syncdir_mcpwm_pcnt+i2s_muxdirn2.vcd) | 1051.8333 | 42.11 |  |
| mcpwm_pcnt+mcpwm_pcnt | pass [vcd](capture/sync_mcpwm_pcnt+mcpwm_pcnt_dir_n2_esp32_idf7_1_2_rmt_syncdir_mcpwm_pcntdirn2.vcd) | 13.3333 | 1.33 |  |
| rmt+i2s_direct | pass [vcd](capture/sync_rmt+i2s_direct_dir_n2_esp32_idf7_1_2_rmt_syncdir_rmt+i2s_directdirn2.vcd) | 1093.4583 | 109.43 |  |
| rmt+i2s_mux | pass [vcd](capture/sync_rmt+i2s_mux_dir_n2_esp32_idf7_1_2_rmt_syncdir_rmt+i2s_muxdirn2.vcd) | 948.375 | 37.97 |  |
| rmt+mcpwm_pcnt | pass [vcd](capture/sync_rmt+mcpwm_pcnt_dir_n2_esp32_idf7_1_2_rmt_syncdir_rmt+mcpwm_pcntdirn2.vcd) | 80.5 | 8.06 |  |
| rmt+rmt | pass [vcd](capture/sync_rmt+rmt_dir_n2_esp32_idf7_1_2_rmt_syncdir_rmtdirn2.vcd) | 66.0833 | 6.61 |  |

## Reading this

- A `refused` is a measurement: the firmware refused the CONFIG, which is how a driver reaches its own queue count.
- `error` is the host or the capture failing, not a step that came out wrong; the run log says which.
- Periods come from the analyzer, so they carry the sample period as their resolution; the tolerance each evaluator uses is in its own result JSON.
- `frameworks/versions` are PlatformIO platform versions; the ESP-IDF column is what runs underneath. Every Arduino row is IDF 4.4.7 because Arduino core is built on IDF 4.4.7 on every espressif32 release (see `extras/doc/platformio-espressif-versions.md`).
- A named scenario that CONFIGS a driver the build has no queues for is recorded **failed** with `ERR CONFIG no such driver`. That is the firmware reporting a capability, not a measurement going wrong, and the *drivers the board accepts* column of the firmware matrix is where it is read as the capability it is. It is not suppressed here because the scenario did not run.
