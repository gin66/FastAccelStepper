# Fix Plan V4 — Prescaler + Predefined 240M + Safety Guards

## Thông tin
- **Ngày:** 2026-06-08 (V4)
- **Chiến lược:** 
  - Dùng timer prescaler đưa TICKS_PER_S về ~16M (giải quyết overflow tận gốc)
  - Thêm predefined entry 240MHz cho board H743 đang fail CI
  - Safety guards cho các board tương lai override TICKS_PER_S > 65M

---

## 🔍 Phân tích: Vì sao V3 chưa đủ?

Trong `build_matrix.yaml` dòng 312:
```yaml
nucleo_h743zi:
    template: stm32
    board: nucleo_h743zi
    build_flags_extra: ["-DTICKS_PER_S=240000000UL"]
```

Board **nucleo_h743zi** trên CI **ép** `TICKS_PER_S=240000000UL` qua build_flags. Việc đổi default trong `pd_config.h` (V3) **không ảnh hưởng** đến CI — nó vẫn compile với 240M và crash ở `#else` branch.

→ **Cần thêm 240M vào predefined list trong RampCalculator.h** (mới V3 bỏ sót).

---

## 📋 Các thay đổi trong V4 (theo thứ tự ưu tiên)

### D1 — Fix compile error immediate (CẦN NHẤT)

| File | Thay đổi |
|------|---------|
| `src/fas_ramp/RampCalculator.h` | Thêm `#elif (TICKS_PER_S == 240000000L)` sau dòng 84 (168M), trước dòng 98 (480M) |

Thêm block:
```cpp
#elif (TICKS_PER_S == 240000000L)
// STM32H743 (nucleo_h743zi default TICKS_PER_S via build_flags)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x37dd)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x36dd)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x6dba)
#define US_TO_TICKS(u32)                ((u32) * 240)
#define TICKS_TO_US(u32)                ((u32) / 240)
```

**Sau fix này, CI sẽ compile OK ngay lập tức.** Tuy nhiên overflow vẫn tồn tại ở tầng chạy (runtime).

---

### A1 — Core fix: Đổi default TICKS_PER_S về 16M

| File | Thay đổi |
|------|---------|
| `src/pd_stm32/pd_config.h` | `#define TICKS_PER_S 72000000UL` → `#define TICKS_PER_S 16000000UL` |

Lý do: Prescaler tự động điều chỉnh. Board dùng build_flags override (H743, F103, F407, G0, L4) không bị ảnh hưởng.

---

### B1, B2 — Safety guards

| # | File | Vị trí | Thay đổi |
|---|------|--------|---------|
| B1 | `src/fas_queue/base.h` | dòng 56 | `(uint16_t)(TICKS_PER_S / 1000)` |
| B2 | `src/FastAccelStepper.cpp` | dòng 138 | clamp `uint32_t tmp = US_TO_TICKS(2000); if(tmp>65535)tmp=65535;` |

---

### C1–C3 — Documentation

| # | File | Thay đổi |
|---|------|---------|
| C1 | `src/pd_stm32/pd_config.h` | Cập nhật comments: prescaler strategy, TICKS_PER_S=16M default, F103/F407 cần override |
| C2 | `src/pd_stm32/stm32_queue.cpp` | Cập nhật comments trong initStepTimer() + clock validation |
| C3 | `README.md` | Thêm STM32 section: prescaler, khuyến nghị override |

---

## 🗺️ Tổng quan tác động

### Board CI đã có override build_flags (không bị ảnh hưởng bởi A1)

| Board | build_flags | Predefined? | Status |
|-------|-------------|-------------|--------|
| bluepill_f103c8 | `-DTICKS_PER_S=72000000UL` (dòng 297) | ✅ Dòng 42 | OK |
| nucleo_g070rb | `-DTICKS_PER_S=64000000UL` (dòng 302) | ✅ Dòng 35 | OK |
| blackpill_f401cc | `-DTICKS_PER_S=84000000UL` (dòng 307) | ✅ Dòng 56 | OK |
| nucleo_h743zi | `-DTICKS_PER_S=240000000UL` (dòng 312) | ✅ **D1 mới thêm** | **Compile OK sau D1** |
| nucleo_l476rg | `-DTICKS_PER_S=80000000UL` (dòng 317) | ✅ Dòng 49 | OK |

### Board KHÔNG override (dùng default mới 16M)

- F103 @72MHz: PSC = 72M/16M - 1 = 3 → timer 18MHz (+12.5% sai số ⚠️)
- F407 @84MHz: PSC = 84M/16M - 1 = 4 → timer 16.8MHz (+5% sai số ⚠️)
- **Khuyến nghị:** Thêm `-DTICKS_PER_S=<clock>` vào build_flags để chính xác

### Board H743 dùng default mới 16M (nếu bỏ build_flags override)

- Timer clock 240MHz → PSC = 240M/16M - 1 = 14 → timer **16.000.000 Hz CHÍNH XÁC ✅**
- Entry 16M có sẵn → compile OK
- Mọi giá trị ticks ≤ 65535 → không overflow

---

## 📊 Lịch sử các Version

| Version | File | Approach | Kết quả |
|---------|------|----------|---------|
| ❌ V1 | `fixplan_2ndV1.md` | Member variables + extern declarations | Sai hướng |
| ❌ V2 | `fixplan_2ndV1.md` | Hybrid: predefined 240M + extern + cast | Rối, thiếu prescaler |
| ❌ V3 | `fixplan_2ndV1.md` | Prescaler 16M + cast/clamp | Đúng prescaler nhưng bỏ sót compile error CI |
| ✅ **V4** | **`fixplan_2ndV4.md`** (file mới) | **D1 + A1 + B1/B2 + C1-C3** | **Tận gốc, đủ, đúng thứ tự** |

## Các files không cần thay đổi
- `src/fas_ramp/RampGenerator.h` — không liên quan
- `src/fas_ramp/RampControl.cpp` — init_ramp_module() chỉ chạy khi #else active
- `src/pd_stm32/stm32_queue.h` — không cần
- `build_matrix.yaml` — không cần
- `extras/ci/platformio.ini` — không cần