# Split the ESP32 RMT sources into V1 and V2

Priority: **110** — low hanging fruit. Naming only. The IDF5/6
translator already works.

Status: **done**. The umbrella `SUPPORT_ESP32_RMT` is gone; the two
mutually exclusive RMT paths each have their own flag and their own
file names. No change to the encoding.

## What the two paths are

They are alternatives, never both: a build is either IDF4 (V1, the half
filler that writes RMT symbols itself) or IDF5/6 (V2, the encoder-API
translator). Nothing in the library or the harness can select between
them at runtime, which is why they are two names rather than one name
with a value — and why the harness reports one RMT driver and tags the
*SDK version* instead.

| | V1 | V2 |
|---|---|---|
| flag | `SUPPORT_ESP32_RMT_V1` | `SUPPORT_ESP32_RMT_V2` |
| defined in | `pd_config_idf4.h` | `pd_config_idf5.h`, `pd_config_idf6.h` |
| fill | `StepperISR_rmt_v1.cpp` (`rmt_fill_buffer()` / `rmt_apply_command()`) | `StepperISR_rmt_v2_encode.cpp` (`rmt_encode_fill()`) |
| queue methods | `StepperISR_rmt_v1_esp32{,_c3,_s3}.cpp` | `StepperISR_rmt_v2.cpp` |

## What changed

- `pd_config_idf4.h` defines `SUPPORT_ESP32_RMT_V1`;
  `pd_config_idf5.h` / `idf6.h` define only `SUPPORT_ESP32_RMT_V2`.
- Shared code tests `SUPPORT_ESP32_RMT_V1 || SUPPORT_ESP32_RMT_V2`
  where it used to test the umbrella, and the single flag where it used
  to test umbrella-and-not-V2. No new `ESP_IDF_VERSION` test: the two
  RMT source files and the RMT branches in `esp32_queue.cpp` no longer
  test the IDF version at all.
- The per-chip V1 files keep their `HAVE_ESP32*_RMT` chip condition —
  `SUPPORT_ESP32_RMT_V1` alone is true on every RMT chip, so dropping it
  would compile the C3 queue methods into an ESP32 build.
- Files renamed so the name says which path it is:
  `StepperISR_esp32xx_rmt.cpp` → `StepperISR_rmt_v1.cpp`,
  `StepperISR_idf5_esp32_rmt{,_encode}.cpp` → `StepperISR_rmt_v2{,_encode}.cpp`,
  `StepperISR_idf4_esp32{,c3,s3}_rmt.cpp` → `StepperISR_rmt_v1_esp32{,c3,s3}.cpp`.
  The chip suffix stays on the per-chip files because it selects a chip,
  not a version; `xx` and `idf5` are gone because the `_v1` / `_v2`
  suffix already says it.
- `SUPPORT_ESP32_RMT_SYNC` and `SUPPORT_ESP32_RMT_TICK_LOST` are
  unchanged: they describe a hardware capability, not a path.
- PC tests follow: `test_17/18/24` define `SUPPORT_ESP32_RMT_V1` and
  include `StepperISR_rmt_v1.cpp`, `test_30` defines
  `SUPPORT_ESP32_RMT_V2`. `test_queue.h` declares `stop_rmt()` (as the
  real class does) and those three tests define it, which is only
  reachable now that `rmt_apply_command()` is no longer hidden behind an
  IDF version test that the PC build never satisfied.

Verified by compiling `saleae_avr`, `esp32_V6_13_0` (IDF 4.4.7,
V1), `esp32_idf_V5_3_0` (IDF 4.4.3, V1) and `esp32_idf_V6_13_0`
(IDF 5.5.3, V2), and by `make -C extras/tests/pc_based` plus
test_17/18/24/30.

See `extras/doc/driver_architecture.md`, "SUPPORT_ macros".