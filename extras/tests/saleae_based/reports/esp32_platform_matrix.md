# Saleae harness — ESP32 platform-release matrix

- **Generated:** 2026-10-06 12:55  _(rebuilt from the recorded results; nothing was measured in this invocation — `--report-only`)_
- **Board:** ESP32-DevKitC, serial `/dev/cu.usbserial-0001`
- **Analyzer:** not recorded (this report was rebuilt from an index written before the analyzer was identified per row)
- **Firmware rows:** 6 — one build+flash each
- **Matrix definition:** `scripts/harness.py` (`RELEASE_MATRIX`, `release_runs()`)
- **Where the waveforms are:** captures and result records are **local and git-ignored** — `capture/<tag>.sr` plus the `.vcd` sigrok derives beside it, and `results/<tag>.json`, per-run logs under `/tmp/saleae_matrix` — so nothing in this file links to them and a fresh checkout has none of them. A measurement is named by its tag key, which is what those filenames are built from.

Every row is a firmware flashed **once**; all driver and combination runs below were measured against that one flash. Drivers come from asking the board what it accepts, so a column that is absent is a driver this SDK has no queues for.

> **This file was rebuilt from the recorded results, not measured.** The verdicts below are re-evaluations of captures already on disk, which is what makes it possible to refresh the report when the board is busy or absent — and it is why the *measured* column, not the *generated* one above, is the date to read: it is when each row's firmware was flashed.

## Firmware matrix

| framework | version | ESP-IDF | PlatformIO env | drivers the board accepts | catalogue | scale sweeps | sync | measured |
|---|---|---|---|---|---|---|---|---|
| arduino | 4.4.0 | 4.4.7 | `esp32_V4_4_0` | rmt, mcpwm_pcnt | catalogue | 2 | sync | 2026-10-06 09:58:26 |
| arduino | 5.3.0 | 4.4.7 | `esp32_V5_3_0` | rmt, mcpwm_pcnt | catalogue | 2 | sync | 2026-10-06 10:01:52 |
| arduino | 6.13.0 | 4.4.7 | `esp32_V6_13_0` | rmt, mcpwm_pcnt | catalogue | 2 | sync | 2026-10-05 23:34:20 |
| idf | 5.3.0 | 4.4.3 | `esp32_idf_V5_3_0` | rmt, mcpwm_pcnt | catalogue | 2 | sync | 2026-10-05 23:37:52 |
| idf | 6.13.0 | 5.5.3 | `esp32_idf_V6_13_0` | rmt, mcpwm_pcnt, i2s_direct, i2s_mux | catalogue | 4 | sync | 2026-10-05 23:42:17 |
| idf | 7.1.2 | 6.1.0 | `esp32_idf_V7_1_2` | rmt, mcpwm_pcnt, i2s_direct, i2s_mux | catalogue | 4 | sync | 2026-10-06 10:33:41 |

> The rows were not measured in one sitting: a row whose upload failed is re-run on its own, and the *measured* column is when each row's flash happened. Rows that share a timestamp were measured against the board back to back.

## Scenario catalogue (SR_00 … SR_31)

Unparameterized: every scenario runs its own fixed program and is judged by its own evaluator.

| test | arduino-4.4.0 | arduino-5.3.0 | arduino-6.13.0 | idf-5.3.0 | idf-6.13.0 | idf-7.1.2 |
|---|---|---|---|---|---|---|
| SR_00 | pass | pass | pass | pass | pass | pass |
| SR_01 | pass | pass | pass | pass | pass | pass |
| SR_02 | pass | pass | pass | pass | pass | pass |
| SR_03 | pass | pass | pass | pass | pass | pass |
| SR_04 | pass | pass | pass | pass | pass | pass |
| SR_05 | pass | pass | pass | pass | pass | pass |
| SR_06 | pass | pass | pass | pass | pass | pass |
| SR_07 | pass | pass | pass | pass | pass | pass |
| SR_08 | pass | pass | pass | pass | pass | pass |
| SR_09 | pass | pass | pass | pass | pass | pass |
| SR_10 | pass | pass | pass | pass | pass | pass |
| SR_11 | pass | pass | pass | pass | pass | pass |
| SR_12 | pass | pass | pass | pass | pass | pass |
| SR_13 | pass | pass | pass | pass | pass | pass |
| SR_14 | pass | pass | pass | pass | pass | pass |
| SR_15 | pass | pass | pass | pass | pass | pass |
| SR_16 | pass | pass | pass | pass | pass | pass |
| SR_17 | pass | pass | pass | pass | pass | pass |
| SR_18 | pass | pass | pass | pass | pass | pass |
| SR_19 | pass | pass | pass | pass | pass | pass |
| SR_20 | pass | pass | pass | pass | pass | pass |
| SR_21 | pass | pass | pass | pass | pass | pass |
| SR_23 | n/a (no such driver) | n/a (no such driver) | n/a (no such driver) | n/a (no such driver) | pass | pass |
| SR_25 | pass | pass | pass | pass | pass | pass |
| SR_26 | pass | pass | pass | pass | pass | pass |
| SR_27 | pass | pass | pass | pass | pass | pass |
| SR_30 | pass | pass | pass | pass | pass | pass |
| SR_31 | pass | pass | pass | pass | pass | pass |

`pass` / `FAIL` / `incomplete` / `refused (bound)` / `n/a` / `skip` are the recorded verdicts, and there is no link on them: the waveform a verdict came from is a capture in `capture/`, which is git-ignored (a mux `dir` capture is a 100 MB VCD), so a link here would be dead in every checkout that has not just run the matrix. `skip` is either *not implemented* (SR_22, SR_24, SR_28, SR_29) or *SR_00 failed*, which is the harness refusing to measure on dead channels. **`FAIL` and `incomplete` are the only cells here that are findings** — see Findings.

`n/a` means this build has no queues for the driver the scenario CONFIGS (`ERR CONFIG no such driver`), so the scenario was never applicable to this row and its `failed` verdict is not a defect. `refused (bound)` is the board declining a CONFIG at a limit, which is what `scale` and `sync` exist to find.

## Findings

Defects only: a panic, a crash, a step count or a period that is wrong, and measurements that could not be made at all. Refusals are **not** listed here — a `scale` sweep is *asked* where a driver's limit is and a refusal is its answer, so those live in the sweep tables below where they read as a bound. The one exception is a scenario that names a driver the build does not have; that is a capability answer, not a failure, and it is counted at the bottom of this section rather than tabulated.

| matrix row | what | measurements | note |
|---|---|---|---|
| idf-6.13.0 | defect | 1 | failed: A steps 65/64 (1 extra) |

Expanded below, one line per measurement.

## Every finding, one line each

| matrix row | test | class | note |
|---|---|---|---|
| idf-6.13.0 | sync mcpwm_pcnt+i2s_direct | defect | failed: A steps 65/64 (1 extra) |

Not findings, and not listed above: **24** refusal(s), which are the measured limits in the sweep tables below, and **32** `no such driver` answer(s), which are scenarios this build has no queues for. A full accounting of every result, including the ones this file does not tabulate, is in the local `results/` directory (git-ignored), indexed by `results/tag_index.json`.

## Driver scale sweeps (how many steppers in parallel)

`nodir`, one shared program, each stepper's own step count and period asserted. The board decides where the sweep stops: a CONFIG refusal is the measured bound.

**arduino-4.4.0 / scale:rmt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 10.00 |  |
| 2 | pass | 64/64 | 10.00 |  |
| 3 | pass | 64/64 | 10.00 |  |
| 4 | pass | 64/64 | 10.00 |  |
| 5 | pass | 64/64 | 10.00 |  |
| 6 | pass | 64/64 | 10.00 |  |
| 7 | pass | 64/64 | 10.00 |  |
| 8 | pass | 64/64 | 10.00 |  |

**arduino-4.4.0 / scale:mcpwm_pcnt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 10.00 |  |
| 2 | pass | 64/64 | 10.00 |  |
| 3 | pass | 64/64 | 10.00 |  |
| 4 | pass | 64/64 | 10.00 |  |
| 5 | pass | 64/64 | 10.00 |  |
| 6 | pass | 64/64 | 10.00 |  |
| 7 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |

**arduino-5.3.0 / scale:rmt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 10.00 |  |
| 2 | pass | 64/64 | 10.00 |  |
| 3 | pass | 64/64 | 10.00 |  |
| 4 | pass | 64/64 | 10.00 |  |
| 5 | pass | 64/64 | 10.00 |  |
| 6 | pass | 64/64 | 10.00 |  |
| 7 | pass | 64/64 | 10.00 |  |
| 8 | pass | 64/64 | 10.00 |  |

**arduino-5.3.0 / scale:mcpwm_pcnt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 10.00 |  |
| 2 | pass | 64/64 | 10.00 |  |
| 3 | pass | 64/64 | 10.00 |  |
| 4 | pass | 64/64 | 10.00 |  |
| 5 | pass | 64/64 | 10.00 |  |
| 6 | pass | 64/64 | 10.00 |  |
| 7 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |

**arduino-6.13.0 / scale:rmt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 10.00 |  |
| 2 | pass | 64/64 | 10.00 |  |
| 3 | pass | 64/64 | 10.00 |  |
| 4 | pass | 64/64 | 10.00 |  |
| 5 | pass | 64/64 | 10.00 |  |
| 6 | pass | 64/64 | 10.00 |  |
| 7 | pass | 64/64 | 10.00 |  |
| 8 | pass | 64/64 | 10.00 |  |

**arduino-6.13.0 / scale:mcpwm_pcnt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 10.00 |  |
| 2 | pass | 64/64 | 10.00 |  |
| 3 | pass | 64/64 | 10.00 |  |
| 4 | pass | 64/64 | 10.00 |  |
| 5 | pass | 64/64 | 10.00 |  |
| 6 | pass | 64/64 | 10.00 |  |
| 7 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |

**idf-5.3.0 / scale:rmt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 10.00 |  |
| 2 | pass | 64/64 | 10.00 |  |
| 3 | pass | 64/64 | 10.00 |  |
| 4 | pass | 64/64 | 10.00 |  |
| 5 | pass | 64/64 | 10.00 |  |
| 6 | pass | 64/64 | 10.00 |  |
| 7 | pass | 64/64 | 10.00 |  |
| 8 | pass | 64/64 | 10.00 |  |

**idf-5.3.0 / scale:mcpwm_pcnt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 10.00 |  |
| 2 | pass | 64/64 | 10.00 |  |
| 3 | pass | 64/64 | 10.00 |  |
| 4 | pass | 64/64 | 10.00 |  |
| 5 | pass | 64/64 | 10.00 |  |
| 6 | pass | 64/64 | 10.00 |  |
| 7 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |

**idf-6.13.0 / scale:rmt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 10.00 |  |
| 2 | pass | 64/64 | 10.00 |  |
| 3 | pass | 64/64 | 10.00 |  |
| 4 | pass | 64/64 | 10.00 |  |
| 5 | pass | 64/64 | 10.00 |  |
| 6 | pass | 64/64 | 10.00 |  |
| 7 | pass | 64/64 | 10.00 |  |
| 8 | pass | 64/64 | 10.00 |  |

**idf-6.13.0 / scale:mcpwm_pcnt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 10.00 |  |
| 2 | pass | 64/64 | 10.00 |  |
| 3 | pass | 64/64 | 10.00 |  |
| 4 | pass | 64/64 | 10.00 |  |
| 5 | pass | 64/64 | 10.00 |  |
| 6 | pass | 64/64 | 10.00 |  |
| 7 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |

**idf-6.13.0 / scale:i2s_direct**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 10.00 |  |
| 2 | pass | 64/64 | 10.00 |  |
| 3 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 4 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 5 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 6 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 7 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |

**idf-6.13.0 / scale:i2s_mux**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 24.93 |  |
| 2 | pass | 64/64 | 24.93 |  |
| 3 | pass | 64/64 | 24.93 |  |
| 4 | pass | 64/64 | 24.93 |  |
| 5 | pass | 64/64 | 24.93 |  |
| 6 | pass | 64/64 | 24.93 |  |
| 7 | pass | 64/64 | 24.93 |  |
| 8 | pass | 64/64 | 24.93 |  |

**idf-7.1.2 / scale:rmt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 10.00 |  |
| 2 | pass | 64/64 | 10.00 |  |
| 3 | pass | 64/64 | 10.00 |  |
| 4 | pass | 64/64 | 10.00 |  |
| 5 | pass | 64/64 | 10.00 |  |
| 6 | pass | 64/64 | 10.00 |  |
| 7 | pass | 64/64 | 10.00 |  |
| 8 | pass | 64/64 | 10.00 |  |

**idf-7.1.2 / scale:mcpwm_pcnt**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 10.00 |  |
| 2 | pass | 64/64 | 10.00 |  |
| 3 | pass | 64/64 | 10.00 |  |
| 4 | pass | 64/64 | 10.00 |  |
| 5 | pass | 64/64 | 10.00 |  |
| 6 | pass | 64/64 | 10.00 |  |
| 7 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1 |

**idf-7.1.2 / scale:i2s_direct**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 10.00 |  |
| 2 | pass | 64/64 | 10.00 |  |
| 3 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 4 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 5 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 6 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 7 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |
| 8 | refused (bound) | – | – | ERR connect step 2 n=2 drv=i2s_direct nodir=1 |

**idf-7.1.2 / scale:i2s_mux**

| n | verdict | steps each | period us | note |
|---|---|---|---|---|
| 1 | pass | 64/64 | 24.93 |  |
| 2 | pass | 64/64 | 24.93 |  |
| 3 | pass | 64/64 | 24.93 |  |
| 4 | pass | 64/64 | 24.93 |  |
| 5 | pass | 64/64 | 24.93 |  |
| 6 | pass | 64/64 | 24.93 |  |
| 7 | pass | 64/64 | 24.93 |  |
| 8 | pass | 64/64 | 24.93 |  |

## Driver combinations (synchronized start)

Every driver-list combination this board could connect, two steppers each, each at its own period. First-step skew is **reported, not gated** (eval_sync): how closely two steppers begin is a property of the pulse driver and of the interrupt latency at that instant, not a correctness property of the queue — so read the ratio in step periods, which is the only comparable form of it. What *is* asserted per stepper is its own commanded step count and period — counted over the *commanded move*, not over the capture: a capture starts before the test is triggered over serial and outlives it, so it holds the host's own round trip and the board's idle afterwards. A pulse outside the move is not evidence about a driver; it is still recorded, as `window.steps_outside` and each pulse's offset, and `report.py` prints the count beside the step count.

**arduino-4.4.0 / sync**

| drivers | verdict | first-step skew us | in step periods | note |
|---|---|---|---|---|
| i2s_direct+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_direct+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_mux+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+mcpwm_pcnt | pass | 3.25 | 0.33 |  |
| rmt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+mcpwm_pcnt | pass | 18.0 | 1.80 |  |
| rmt+rmt | pass | 17.25 | 1.73 |  |

**arduino-5.3.0 / sync**

| drivers | verdict | first-step skew us | in step periods | note |
|---|---|---|---|---|
| i2s_direct+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_direct+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_mux+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+mcpwm_pcnt | pass | 3.25 | 0.33 |  |
| rmt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+mcpwm_pcnt | pass | 23.5 | 2.35 |  |
| rmt+rmt | pass | 17.25 | 1.73 |  |

**arduino-6.13.0 / sync**

| drivers | verdict | first-step skew us | in step periods | note |
|---|---|---|---|---|
| i2s_direct+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_direct+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_mux+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+mcpwm_pcnt | pass | 3.25 | 0.33 |  |
| rmt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+mcpwm_pcnt | pass | 35.25 | 3.52 |  |
| rmt+rmt | pass | 17.25 | 1.73 |  |

**idf-5.3.0 / sync**

| drivers | verdict | first-step skew us | in step periods | note |
|---|---|---|---|---|
| i2s_direct+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_direct+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| i2s_mux+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| mcpwm_pcnt+mcpwm_pcnt | pass | 4.5 | 0.45 |  |
| rmt+i2s_direct | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+i2s_mux | n/a (no such driver) | – | – | ERR CONFIG no such driver |
| rmt+mcpwm_pcnt | pass | 28.5 | 2.85 |  |
| rmt+rmt | pass | 27.25 | 2.73 |  |

**idf-6.13.0 / sync**

| drivers | verdict | first-step skew us | in step periods | note |
|---|---|---|---|---|
| i2s_direct+i2s_direct | pass | 36.875 | 3.69 |  |
| i2s_direct+i2s_mux | pass | 6.625 | 0.27 |  |
| i2s_mux+i2s_mux | pass | 0.0 | 0.00 |  |
| mcpwm_pcnt+i2s_direct | **FAIL** | 739.25 | 73.98 | failed: A steps 65/64 (1 extra) |
| mcpwm_pcnt+i2s_mux | pass | 1102.2083 | 44.13 |  |
| mcpwm_pcnt+mcpwm_pcnt | pass | 13.3333 | 1.33 |  |
| rmt+i2s_direct | pass | 800.5 | 80.12 |  |
| rmt+i2s_mux | pass | 1000.0833 | 40.04 |  |
| rmt+mcpwm_pcnt | pass | 60.7917 | 6.08 |  |
| rmt+rmt | pass | 65.9583 | 6.60 |  |

**idf-7.1.2 / sync**

| drivers | verdict | first-step skew us | in step periods | note |
|---|---|---|---|---|
| i2s_direct+i2s_direct | pass | 75.5 | 7.56 |  |
| i2s_direct+i2s_mux | pass | 222.4167 | 8.90 |  |
| i2s_mux+i2s_mux | pass | 0.0 | 0.00 |  |
| mcpwm_pcnt+i2s_direct | pass | 914.7917 | 91.55 |  |
| mcpwm_pcnt+i2s_mux | pass | 880.625 | 35.25 |  |
| mcpwm_pcnt+mcpwm_pcnt | pass | 13.3333 | 1.33 |  |
| rmt+i2s_direct | pass | 703.2917 | 70.39 |  |
| rmt+i2s_mux | pass | 771.875 | 30.90 |  |
| rmt+mcpwm_pcnt | pass | 76.6667 | 7.67 |  |
| rmt+rmt | pass | 62.125 | 6.22 |  |

## Reading this

- A `refused` is a measurement: the firmware refused the CONFIG, which is how a driver reaches its own queue count.
- `error` is the host or the capture failing, not a step that came out wrong; the run log says which.
- Periods come from the analyzer, so they carry the sample period as their resolution; the tolerance each evaluator uses is in its own result JSON.
- `frameworks/versions` are PlatformIO platform versions; the ESP-IDF column is what runs underneath. Every Arduino row is IDF 4.4.7 because Arduino core is built on IDF 4.4.7 on every espressif32 release (see `extras/doc/platformio-espressif-versions.md`).
- A named scenario that CONFIGS a driver the build has no queues for is recorded **failed** with `ERR CONFIG no such driver`. That is the firmware reporting a capability, not a measurement going wrong, and the *drivers the board accepts* column of the firmware matrix is where it is read as the capability it is. It is not suppressed here because the scenario did not run.
