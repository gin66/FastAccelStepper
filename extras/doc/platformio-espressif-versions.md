# PlatformIO ESP32 Platform — Release / ESP-IDF / Arduino Cross-Reference

Version mapping extracted from https://github.com/platformio/platform-espressif32/releases.

## Version Table

| PlatformIO | ESP-IDF | Arduino Core | Arduino's underlying IDF | Key Notes |
|---|---|---|---|---|
| **7.1.x** | — | v2.0.17 | v4.4.7 | Bug fixes only |
| **7.1.0** | **v6.1.0** | v2.0.17 | v4.4.7 | First IDF 6.1 support |
| **7.0.1** | **v6.0.1** | v2.0.17 | v4.4.7 | |
| **7.0.0** | **v6.0.0** | v2.0.17 | v4.4.7 | Major IDF 6.0, toolchain 15.2.0 |
| **6.13.0** | **v5.5.3** | v2.0.17 | v4.4.7 | Toolchain 14.2.0+20251107 |
| **6.12.0** | **v5.5.0** | v2.0.17 | v4.4.7 | CMake 3.30, Secure Features |
| **6.11.0** | **v5.4.1** | — | — | ROM ELF package added |
| **6.10.0** | **v5.4.0** | v2.0.17 | v4.4.7 | Toolchain 14.2.0 |
| **6.9.0** | **v5.3.1** | v2.0.17 | v4.4.7 | |
| **6.8.1** | — | — | — | Debug print fix |
| **6.8.0** | **v5.3.0** | **v2.0.17** | — | Arduino 2.0.17 support |
| **6.7.0** | **v5.2.1** | **v2.0.16** | v4.4.7 | Toolchain 13.2.0 |
| **6.6.0** | **v5.2.1** | **v2.0.14** | — | LTO via GCC wrapped ar/ranlib |
| **6.5.0** | **v5.1.2** | **v2.0.14** | — | |
| **6.4.0** | **v5.1.1** | **v2.0.11** | — | ESP32-C6 initial support |
| **5.4.0** | **v4.4.5** | **v2.0.6** | — | |
| **6.3.2** | **v5.0.2** | **v2.0.9** | — | Strict Python deps |
| **6.3.1** | **v5.0.2** | **v2.0.9** | — | urllib3 fix |
| **6.3.0** | **v5.0.2** | **v2.0.9** | — | |
| **6.2.0** | **v5.0.1** | **v2.0.8** | — | esptoolpy 4.5.1 |
| **6.1.0** | **v5.0.1** | **v2.0.7** | — | |
| **6.0.1** | **v5.0.0** | **v2.0.6** | — | Python deps handling |
| **6.0.0** | **v5.0.0** | **v2.0.6** | — | **Major IDF 5.0**, GCC 11.2.0 |
| **5.3.0** | **v4.4.3** | **v2.0.6** | — | Toolchains 8.4.0r2-patch5 |
| **5.2.0** | **v4.4.2** | **v2.0.5** | — | |
| **5.1.1** | **v4.4.1** | **v2.0.4** | — | |
| **5.1.0** | **v4.4.1** | **v2.0.4** | — | Prebuilt bootloader merge |
| **5.0.0** | **v4.4.1** | **v2.0.3** | — | ESP-based debug probes |
| **4.4.0** | — | — | — | PIO Core 6.0 compat |
| **4.3.0** | — | **v2.0.3** | — | |
| **4.2.0** | — | **v2.0.2** | — | CMSIS-DAP debug |
| **4.1.0** | — | **v2.0.1** | — | Toolchain 8.4.0-patch3 |
| **4.0.0** | — | **v2.0.0** | — | Dynamic toolchain, Simba/Pumbaa deprecated |
| **3.5.0** | **v4.3.2** | — | — | |
| **3.4.0** | **v4.3.1** | — | — | |
| **3.3.2** | — | — | — | |
| **3.3.1** | — | — | — | |
| **3.3.0** | **v4.3** | — | — | **ESP32-C3** support |
| **3.2.1** | **v4.2.1** | — | — | |
| **3.2.0** | — | **v1.0.6** | — | |
| **3.1.1** | — | **v1.0.5** | — | |
| **3.1.0** | — | **v1.0.5** | — | |
| **3.0.0** | **v4.2** | — | — | **ESP32-S2** support |
| **2.1.0** | — | — | — | |
| **2.0.0** | **v4.1** | — | — | ESP32-S2 Beta |
| **1.12.4** | — | — | — | |
| **1.12.3** | — | — | — | |
| **1.12.2** | **v4.0.1** | — | — | |

## Key Observations

1. **Arduino always runs on ESP-IDF v4.4.7** regardless of which ESP-IDF version
   the platform uses directly. When you select `framework = arduino`, you get
   Arduino core on top of IDF v4.4.7, even if the platform's native ESP-IDF is
   v6.x.

2. **IDF version jumps** (major platform releases):
   - **6.0.0** → ESP-IDF 5.0 (major API break)
   - **7.0.0** → ESP-IDF 6.0 (major API break)
   - **7.1.0** → ESP-IDF 6.1

3. **Arduino core versions** progressed from v1.0.x → v2.0.x, with v2.0.17
   being the latest as of 7.1.x.

4. **Toolchain progression**: GCC toolchains went from 8.4.0 → 13.2.0 → 14.2.0 →
   15.2.0 across the releases.

5. **ESP32 chip support expansion**:
   - Original ESP32 → ESP32-S2 (3.0.0) → ESP32-C3 (3.3.0) → ESP32-C6
     (6.4.0) → ESP32-H2 (6.12.0+)
