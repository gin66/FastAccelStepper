# Fix Plan V5 — Prescaler + Predefined 240M (đúng hằng số) + Safety Guards

## Thông tin
- **Ngày:** 2026-06-08 (V5)
- **Chiến lược:**
  - Bỏ `build_flags_extra` của H743 để dùng default TICKS_PER_S=16M → prescaler auto → timer 16M chính xác
  - Thêm 240M vào predefined list (dự phòng cho ai override)
  - Safety guards + documentation

---

## 📋 Các thay đổi

### 1 — build_matrix.yaml: Bỏ override của H743

**File:** `extras/ci/build_matrix.yaml`

**Dòng 309-312 hiện tại:**
```yaml
  nucleo_h743zi:
    template: stm32
    board: nucleo_h743zi
    build_flags_extra: ["-DTICKS_PER_S=240000000UL"]
```

**Sửa thành:**
```yaml
  nucleo_h743zi:
    template: stm32
    board: nucleo_h743zi
```

Kết quả: H743 dùng TICKS_PER_S mặc định 16M → prescaler = 240M/16M - 1 = 14 → timer 16.000.000 Hz chính xác ✅

---

### 2 — pd_config.h: Đổi default TICKS_PER_S

**File:** `src/pd_stm32/pd_config.h`

| Cũ | Mới |
|----|-----|
| `#define TICKS_PER_S 72000000UL` | `#define TICKS_PER_S 16000000UL` |

**Tác động:** Prescaler tự động adjust. Board có build_flags override không bị ảnh hưởng.

---

### 3 — RampCalculator.h: Thêm 240M (dự phòng)

**File:** `src/fas_ramp/RampCalculator.h`

Thêm block mới sau dòng 84 (sau 168M) và trước dòng 98 (480M):

```cpp
#elif (TICKS_PER_S == 240000000L)
// STM32H743 (fallback nếu ai đó override TICKS_PER_S=240M)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x37AD)  // log2(240M)×512 = 14253
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x36AD)  // 0x37AD - 0x0100
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x6D5B)  // 2×log2(240M) - 1 → 27995
#define US_TO_TICKS(u32)                ((u32) * 240)
#define TICKS_TO_US(u32)                ((u32) / 240)
```

**Kiểm tra chéo hằng số:**
- `0x37AD` = 14253 = log2(240.000.000) × 512 = 27.8385 × 512 = 14253 ✅
- `0x36AD` = 0x37AD - 0x100 (log2(sqrt2) × 512 = 256) ✅
- `0x6D5B` = 27995 = (2 × 27.8385 - 1) × 512 = 54.677 × 512 = 27995 ✅

---

### 4 — base.h: Safety guard

**File:** `src/fas_queue/base.h` dòng 56

```cpp
// Cũ
max_speed_in_ticks = TICKS_PER_S / 1000;

// Mới
max_speed_in_ticks = (uint16_t)(TICKS_PER_S / 1000);
```

---

### 5 — FastAccelStepper.cpp: Safety guard

**File:** `src/FastAccelStepper.cpp` dòng 138

```cpp
// Cũ
struct stepper_command_s pause_cmd = {
    .ticks = US_TO_TICKS((uint16_t)2000),

// Mới
uint32_t _tmp_pause = US_TO_TICKS(2000);
if (_tmp_pause > 65535) _tmp_pause = 65535;
struct stepper_command_s pause_cmd = {
    .ticks = (uint16_t)_tmp_pause,
```

---

### 6 — Documentation

**README.md:** Thêm STM32 section — prescaler + khuyến nghị override cho F103/F407
**pd_config.h:** Cập nhật comments — giải thích 16M default, prescaler strategy
**stm32_queue.cpp:** Cập nhật comments — clock validation với TICKS_PER_S mới

---

## 🗺️ Tổng quan tác động

### Board CI

| Board | TICKS_PER_S | Predefined | Tràn? | Status |
|-------|-------------|------------|-------|--------|
| nucleo_h743zi | 16M (default) | ✅ Dòng 9 | ✅ 16.000 < 65.535 | **OK** |
| bluepill_f103c8 | 72M (override) | ✅ Dòng 42 | ❌ chỉ safety guard | Compile OK |
| nucleo_g070rb | 64M (override) | ✅ Dòng 35 | ✅ 64.000 < 65.535 | OK |
| blackpill_f401cc | 84M (override) | ✅ Dòng 56 | ❌ chỉ safety guard | Compile OK |
| nucleo_l476rg | 80M (override) | ✅ Dòng 49 | ❌ chỉ safety guard | Compile OK |

### Board không override (dùng default 16M)

- F103/F407: Sai số tốc độ +12.5%/+5% — khuyến nghị `-DTICKS_PER_S=<clock>`
- H743: Chính xác ✅

---

## 📊 Lịch sử các Version

| Version | File | Approach | Kết quả |
|---------|------|----------|---------|
| ❌ V1 | `fixplan_2ndV1.md` | Member variables + extern | Sai hướng |
| ❌ V2 | `fixplan_2ndV1.md` | Hybrid: 240M + extern + cast | Rối |
| ❌ V3 | `fixplan_2ndV1.md` | Prescaler 16M + cast/clamp | Bỏ sót compile error CI |
| ❌ V4 | `fixplan_2ndV4.md` | D1 (sai hằng số) + A1 + B1/B2 | Hằng số log2 sai |
| ✅ **V5** | **`fixplan_2ndV5.md`** | **Bỏ build_flags H743 + prescaler + D1 đúng** | **Tận gốc, sạch** |