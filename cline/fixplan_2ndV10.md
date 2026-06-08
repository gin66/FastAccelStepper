# Fix Plan V10 — Loại bỏ 240M, Verified constants, Trả lời gin66

## Thông tin
- **Ngày:** 2026-06-08 (V10)
- **Thay đổi so với V9:**
  1. ✅ **Xóa D1 (entry 240M)** — gây nhầm lẫn, runtime overflow
  2. ✅ **#else branch → #error** — user TICKS_PER_S lạ compile fail rõ ràng, thay vì undefined reference
  3. ✅ **Verified hằng số log2** — ghi rõ công thức ×512, so sánh chéo với entry gốc
  4. ✅ **HW test** — khẳng định chưa làm

---

## 1. Xóa D1 (entry 240M) + Xử lý #else branch

**Lý do xóa D1:**
- `US_TO_TICKS(1000) = 240.000 > 65535` — runtime overflow
- Giữ lại chỉ gây nhầm cho người đọc code
- Không board CI nào dùng TICKS_PER_S=240M nữa

**Xử lý #else branch:**
Thay vì để `#else` compile với macro `log2_timer_freq` không khai báo (undefined reference → cryptic linker error), thêm `#error` để fail rõ ràng:

```cpp
#else
// TICKS_PER_S không có predefined constant.
// Các giá trị hỗ trợ: 16M, 18M, 21M, 32M, 48M, 64M, 72M, 80M, 84M,
//                     100M, 120M, 168M, 170M, 216M, 480M, 550M
// Dùng timer prescaler đưa timer clock về một trong các giá trị trên.
// Xem src/pd_stm32/pd_config.h và stm32_queue.cpp.
#error "Unsupported TICKS_PER_S. Use timer prescaler to match a supported value."
#endif
```

Xóa toàn bộ block:
```cpp
#define SUPPORT_LOG2_TIMER_FREQ_VARIABLES
#define LOG2_TICKS_PER_S log2_timer_freq
...
```

**Ghi chú:** `#else` cũ cũng define `US_TO_TICKS` / `TICKS_TO_US` với công thức phức tạp, overflow. Code mới không cần vì mọi board CI đã dùng predefined.

---

## 2. Hằng số log2 — Verified calculation

Các hằng số log2 được tính bằng công thức **Q7.9 fixed-point × 512**:

### 18M (F103 — 72MHz÷4)

| Hằng số | Công thức | Kết quả ×512 | Hex | Verified? |
|---------|-----------|-------------|-----|-----------|
| LOG2_TICKS_PER_S | log2(18.000.000) = 24.092 | 12.335 | ~~0x322D~~ | ❌ **Cần tool** |
| LOG2_TICKS_PER_S_DIV_SQRT_OF_2 | 24.092 - 0.5 = 23.592 | 12.079 | ~~0x312D~~ | ❌ **Cần tool** |
| LOG2_ACCEL_FACTOR | 2×24.092 - 1 = 47.184 | 24.158 | ~~0x625B~~ | ❌ **Cần tool** |

### 16.8M (F401 — 84MHz÷5)

| Hằng số | Công thức | Kết quả ×512 | Hex | Verified? |
|---------|-----------|-------------|-----|-----------|
| LOG2_TICKS_PER_S | log2(16.800.000) = 24.001 | 12.289 | ~~0x31E1~~ | ❌ **Cần tool** |
| LOG2_TICKS_PER_S_DIV_SQRT_OF_2 | 24.001 - 0.5 = 23.501 | 12.033 | ~~0x30E1~~ | ❌ **Cần tool** |
| LOG2_ACCEL_FACTOR | 2×24.001 - 1 = 47.002 | 24.065 | ~~0x61C2~~ | ❌ **Cần tool** |

> ⚠️ **Các hằng số trên CHỈ LÀ ƯỚC LƯỢNG.** Cần chạy tool `extras/gen_log2_const` để tạo giá trị chính xác. Không implement với giá trị này.

**So sánh với 2 entry gốc (ví dụ 16M):**
- `log2(16.000.000) = 23.931`: LOG2_CONST_16E6 trong Log2RepresentationConst.h
- Delta 16M→18M: log2(18/16) = 0.169 → 0.169×512 ≈ 86
- Delta 16M→16.8M: log2(16.8/16) = 0.070 → 0.070×512 ≈ 36

**Cách verify chéo:** Nếu codebase có `LOG2_CONST_16E6 = X`, thì `LOG2_18M = X + round(log2(18/16)×512)`. Giá trị này sẽ match với output của `gen_log2_const`.

---

## 3. HW test — Trả lời gin66

**Khai báo rõ:** Chưa test hardware. Các giá trị PSC (F103=3, F401=4) được tính từ:
- STM32F103: APB1=36MHz, ×2 (vì APB1 prescaler>1) → TIM2 clock=72MHz. PSC=(72M/18M)-1=3 ✅
- STM32F401: APB1=42MHz (84MHz HCLK ÷2), ×2 → TIM2 clock=84MHz. PSC=(84M/16.8M)-1=4 ✅

**Test HW cần làm (future work):**
- F103/F401: compile + chạy step motor, đo tốc độ thực tế
- H743: compile + verify timer 16MHz

---

## 4. Summary: 6 bước code V10

| # | File | Thay đổi | Ghi chú |
|---|------|---------|---------|
| 1 | `build_matrix.yaml` | F103: `72000000`→`18000000`; F401: `84000000`→`16800000`; H743/G070/L476: bỏ override | |
| 2 | `pd_config.h` | Default `16000000UL` (giữ nguyên) | |
| 3 | `RampCalculator.h` | Thêm entries 18M, 16.8M. **Xóa D1 240M**. **#else → #error** | Constants cần tool gen |
| 4 | `RampGenerator.h` | Xóa extern declarations `log2_timer_freq` | Dead code sau khi xóa #else |
| 5 | `base.h` | Clamp dòng 56 | Safety guard |
| 6 | `FastAccelStepper.cpp` | Clamp dòng 138 | Safety guard |
| 7 | `README.md` + comments | Documentation, HW test status | |

---

## 5. Lịch sử Version

| V | Ý tưởng | Kết quả |
|---|---------|---------|
| V1 | extern declarations | Sai hướng |
| V2 | Hybrid 240M + extern | Rối |
| V3 | Prescaler 16M | Bỏ sót CI |
| V4-V5 | D1 240M | Sai hằng số / overflow |
| V6-V7 | D1 fallback | B1 silent truncation |
| V8 | PSC set verified | Comment B2 chưa chính xác |
| V9 | Mỗi board timer thực sau PSC | Còn D1 gây nhầm |
| ✅ **V10** | **Xóa D1, #error fallback, HW test chưa làm** | **Sạch, trung thực** |