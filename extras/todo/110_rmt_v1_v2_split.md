# Split the ESP32 RMT sources into V1 and V2

Priority: **110** — low hanging fruit. Naming only. The IDF5/6
translator already works.

Status: not started.

## State

`SUPPORT_ESP32_RMT` is set for both the IDF4 half filler and the
IDF5/6 translator. `SUPPORT_ESP32_RMT_V2` then selects the translator,
and the half filler is excluded with `!SUPPORT_ESP32_RMT_V2`.

Replace that with `SUPPORT_RMT_V1` for the IDF4 path
(`StepperISR_esp32xx_rmt.cpp`, `rmt_fill_buffer()` /
`rmt_apply_command()`) and `SUPPORT_RMT_V2` for the IDF5/6 path
(`StepperISR_idf5_esp32_rmt.cpp`,
`StepperISR_idf5_esp32_rmt_encode.cpp`). Rename the files to match.
No change to the encoding.
