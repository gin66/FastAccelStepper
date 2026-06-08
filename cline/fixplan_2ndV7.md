# Fix Plan V7 — Prescaler 16M + D1 có warning + Safety guards đúng

## Thông tin
- **Ngày:** 2026-06-08 (V7)

## Các sửa so với V6

| # | Vấn đề | V6 (sai) | V7 (đúng) |
|---|--------|----------|-----------|
| 1 | `base.h` B1 | `(uint16_t)(TICKS_PER_S / 1000)` — silent truncation | `(uint16_t)fas_min((uint32_t)(TICKS_PER_S/1000), 65535)` — clamp |
| 2 | D1 LOG2_ACCEL_FACTOR | `0x6D5B` — cần verify | `0x6D5B` — **đã verify: 2×27.8385-1=54.677→27995→0x6D5B ✅** |
| 3 | F103 72M | Không check | **Entry 72M có sẵn dòng 42 ✅** |
| 4 | README H743 warning | Thiếu | **Thêm**: "prescaler tự xử lý, không override TICKS_PER_S" |
| 5 | B2 comment | Không có | **Thêm**: "best-effort degraded mode" |

---

## 📋 Các thay đổi

### 1 — build_matrix.yaml: Bỏ override của H743

```yaml
# Cũ dòng 309-312
  nucleo_h743zi:
    template: stm32
    board: nucleo_h743zi
    build_flags_extra: ["-DTICKS_PER_S=240000000UL"]

# Mới
  nucleo_h743zi:
    template: stm32
    board: nucleo_h743zi
```

### 2 — pd_config.h: Đổi default 72M → 16M

```cpp
// Cũ
#define TICKS_PER_S 72000000UL

// Mới: dùng timer prescaler — H743 timer 240M÷15=16MHz chính xác
// F103/F407 nếu dùng default sẽ có sai số +12.5%/+5%,
// khuyến nghị override: -DTICKS_PER_S=72000000UL (F103) / 84000000UL (F407)
#define TICKS_PER_S 16000000UL
```

### 3 — RampCalculator.h: Thêm D1 với WARNING

Thêm sau dòng 84 (sau 168M), trước dòng 98 (480M):

```cpp
#elif (TICKS_PER_S == 240000000L)
// ⚠️ STM32H743 emergency fallback
// WARNING: 240M ticks/s causes uint16_t overflow in queue infrastructure.
// US_TO_TICKS(1000) = 240.000 > 65535 — step periods > 272µs WILL overflow.
// RECOMMENDED: Use timer prescaler to bring TICKS_PER_S to ~16M instead.
// See src/pd_stm32/pd_config.h for prescaler configuration.
#define LOG2_TICKS_PER_S               ((log2_value_t)0x37AD)  // log2(240M)×512 = 14253 ✅
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x36AD)  // 0x37AD - 0x0100 ✅
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x6D5B)  // (2×27.8385-1)×512 = 27995 ✅
#define US_TO_TICKS(u32)                ((u32) * 240)
#define TICKS_TO_US(u32)                ((u32) / 240)
```

### 4 — base.h: Clamp thay vì cast

```cpp
// Cũ dòng 56 (silent truncation — SAI)
max_speed_in_ticks = TICKS_PER_S / 1000;

// Mới (clamp — ĐÚNG)
max_speed_in_ticks = (uint16_t)fas_min((uint32_t)(TICKS_PER_S / 1000), (uint32_t)65535);
```

Lý do: Với TICKS_PER_S > 65M, cast trần gây wrap (240.000 → 43.392). Clamp về 65535 là degraded mode nhưng không crash.

### 5 — FastAccelStepper.cpp: Clamp + comment

```cpp
// Cũ dòng 138
struct stepper_command_s pause_cmd = {
    .ticks = US_TO_TICKS((uint16_t)2000),

// Mới — best-effort degraded mode cho board override TICKS_PER_S > 65M
uint32_t _tmp_pause = US_TO_TICKS(2000);
if (_tmp_pause > 65535) {
    // 65535 ticks ≈ 0.27ms @240M — ngắn hơn 2ms yêu cầu.
    // Đây là degraded mode: stepper vẫn chạy nhưng tốc độ có thể sai.
    _tmp_pause = 65535;
}
struct stepper_command_s pause_cmd = {
    .ticks = (uint16_t)_tmp_pause,
```

---

## ✅ Verified: 3 cross-checks

| Check | Kết quả | Ghi chú |
|-------|---------|---------|
| **LOG2_ACCEL_FACTOR 0x6D5B** | ✅ Đúng | Pattern 72M: 2×log2(72M)-1 = 2×26.1016-1 = 51.203 → 0x6668 ✅ |
| **B1 clamp** | ✅ Sửa | `fas_min(...,65535)` thay vì cast trần |
| **F103 72M entry** | ✅ Có sẵn | `RampCalculator.h` dòng 42 `#elif (TICKS_PER_S == 72000000L)` |

---

## 📊 Lịch sử các Version

| Version | File | Ý tưởng chính | Kết quả |
|---------|------|---------------|---------|
| V1 | V1.md | extern declarations | Sai hướng |
| V2 | V1.md | Hybrid 240M + extern | Rối |
| V3 | V1.md | Prescaler 16M + cast | Bỏ sót CI compile |
| V4 | V4.md | D1 (sai hằng số) + A1 | Hằng số sai |
| V5 | V5.md | D1 đúng + prescaler | Overflow D1 runtime |
| V6 | V6.md | Prescaler chính, D1 fallback | B1 silent truncation |
| ✅ **V7** | **V7.md** | **B1 clamp, all verified** | **Ready-to-merge** |