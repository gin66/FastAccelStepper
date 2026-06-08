# Fix Plan V6 — Prescaler 16M là giải pháp DUY NHẤT, D1 chỉ là emergency fallback

## Thông tin
- **Ngày:** 2026-06-08 (V6)
- **Vấn đề V5:** D1 (240M entry) dùng `US_TO_TICKS(u32) = (u32)*240` → overflow runtime: `US_TO_TICKS(1000) = 240.000 > 65535` ❌
- **Kết luận:** D1 compile được nhưng runtime VẪN overflow. Prescaler là **giải pháp đúng duy nhất**.

---

## Phân tích: Vì sao D1 tự nó đã sai

D1 được thêm để H743 compile được khi `TICKS_PER_S=240000000UL`. Nhưng:

| Biểu thức | Kết quả @240M | uint16_t? |
|-----------|---------------|-----------|
| `US_TO_TICKS(1000)` | 240.000 | ❌ Tràn |
| `US_TO_TICKS(2000)` | 480.000 | ❌ Tràn |
| `TICKS_PER_S / 1000` (max_speed) | 240.000 | ❌ Tràn |
| `TICKS_PER_S / 5000` (MIN_CMD_TICKS) | 48.000 | ✅ OK |

**D1 chỉ làm CI xanh — nhưng runtime vẫn chết.** Thà compile error còn hơn false security.

## Giải pháp V6: Prescaler là core, D1 là emergency có cảnh báo

### Bước 1 — build_matrix.yaml: Bỏ build_flags_extra của H743

```yaml
# Cũ
  nucleo_h743zi:
    template: stm32
    board: nucleo_h743zi
    build_flags_extra: ["-DTICKS_PER_S=240000000UL"]

# Mới
  nucleo_h743zi:
    template: stm32
    board: nucleo_h743zi
```

→ H743 dùng TICKS_PER_S = 16M (default mới)

### Bước 2 — pd_config.h: Đổi default 72M → 16M

```cpp
// Cũ
#define TICKS_PER_S 72000000UL

// Mới
#define TICKS_PER_S 16000000UL
```

**Tác dụng:**
- H743 (không override): prescaler = 240M/16M - 1 = 14 → timer **16.000.000 Hz ✅**
- Mọi giá trị ticks ≤ 65535 → **không overflow** 🎯

### Bước 3 — RampCalculator.h: Thêm D1 với WARNING

```cpp
#elif (TICKS_PER_S == 240000000L)
// ⚠️ WARNING: 240M ticks/s causes uint16_t overflow in queue.
// US_TO_TICKS(1000) = 240.000 > 65535 — step periods > ~272µs WILL overflow.
// This entry is for EMERGENCY USE ONLY (e.g. forcing TICKS_PER_S via build_flags).
// RECOMMENDED: Use timer prescaler to bring TICKS_PER_S to ~16M instead.
// See src/pd_stm32/pd_config.h and stm32_queue.cpp for prescaler configuration.
#define LOG2_TICKS_PER_S               ((log2_value_t)0x37AD)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x36AD)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x6D5B)
#define US_TO_TICKS(u32)                ((u32) * 240)
#define TICKS_TO_US(u32)                ((u32) / 240)
```

### Bước 4+5 — Safety guards (giữ nguyên)

| File | Thay đổi |
|------|---------|
| `src/fas_queue/base.h` dòng 56 | `(uint16_t)(TICKS_PER_S / 1000)` |
| `src/FastAccelStepper.cpp` dòng 138 | clamp `if(tmp>65535)tmp=65535` |

### Bước 6 — Documentation

- `README.md`: STM32 prescaler + khuyến nghị override cho F103/F407
- `pd_config.h`: Giải thích 16M strategy, F103/F407 cần `-DTICKS_PER_S=72000000UL`
- `stm32_queue.cpp`: Cập nhật clock validation comments

---

## 🗺️ Tổng quan: Prescaler vs D1

| Scenario | TICKS_PER_S | Nguồn | 1ms = ? | Overflow? | Dùng |
|----------|-------------|-------|---------|-----------|------|
| **H743 CI (mới)** | **16M** | default pd_config.h | 16.000 | ✅ **Không** | **Prescaler** |
| H743 người dùng bỏ qua | 240M (build_flags) | tự ép | 240.000 | ❌ Có | D1 + safety |
| F103 (override) | 72M | build_flags | 72.000 | ❌ Có | Safety guard |
| F103 (default mới) | 16M | pd_config.h | 16.000 | ✅ Không | Prescaler (±12.5% speed) |

---

## 📊 Lịch sử các Version

| Version | File | Ý tưởng chính | Kết quả |
|---------|------|---------------|---------|
| V1 | V1.md | extern declarations | Sai hướng |
| V2 | V1.md | Hybrid 240M + extern | Rối |
| V3 | V1.md | Prescaler 16M + cast | Bỏ sót CI compile |
| V4 | V4.md | D1 (sai hằng số) + A1 | Hằng số sai |
| V5 | V5.md | D1 đúng + prescaler | D1 vẫn overflow runtime |
| ✅ **V6** | **V6.md** | **Prescaler là chính, D1 chỉ fallback có warning** | **Tận gốc, an toàn** |