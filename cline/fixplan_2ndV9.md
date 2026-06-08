# Fix Plan V9 — Mỗi board CI có TICKS_PER_S = timer thực sau prescaler

## Thông tin
- **Ngày:** 2026-06-08 (V9)
- **Phát hiện chính:** Không cần ép tất cả về 16M. Mỗi board có timer thực khác nhau sau PSC, chỉ cần ghi nhận giá trị đó làm TICKS_PER_S — miễn là ≤ 32M thì không overflow.

---

## Tại sao V8 chưa tối ưu?

V8 đề xuất tất cả board STM32 dùng TICKS_PER_S=16M. Nhưng:
- **F103:** 72M÷4(PSC=3) = **18MHz**, không phải 16M → nếu ghi là 16M thì sai số +12.5%
- **F401:** 84M÷5(PSC=4) = **16.8MHz**, không phải 16M → nếu ghi là 16M thì sai số +5%

**Giải pháp V9:** Mỗi board dùng TICKS_PER_S đúng với timer thực sau prescaler. Chỉ cần TICKS_PER_S ≤ 32M là mọi giá trị ticks ≤ 65535.

## Bảng PSC và timer thực

Công thức PSC (stm32_queue.cpp dòng 175):
```cpp
psc = (fas_stm32_clock_tim_clk / TICKS_PER_S) - 1;
```

Với TICKS_PER_S được đặt bằng timer thực sau PSC (PSC+1 = timer_clk/TICKS_PER_S):

| Board | Timer clock | TICKS_PER_S mới | PSC | Timer thực | Sai số | 1ms = ? | Overflow? |
|-------|-------------|----------------|-----|-----------|--------|---------|-----------|
| **H743** | 240M | **16.000.000** | 14 (÷15) | 16.000.000 Hz | **0%** | 16.000 ✅ | ✅ Không |
| **G070** | 64M | **16.000.000** | 3 (÷4) | 16.000.000 Hz | **0%** | 16.000 ✅ | ✅ Không |
| **L476** | 80M | **16.000.000** | 4 (÷5) | 16.000.000 Hz | **0%** | 16.000 ✅ | ✅ Không |
| **F103** | 72M | **18.000.000** | 3 (÷4) | 18.000.000 Hz | **0%** | 18.000 ✅ | ✅ Không |
| **F401** | 84M | **16.800.000** | 4 (÷5) | 16.800.000 Hz | **0%** | 16.800 ✅ | ✅ Không |

**Tất cả TICKS_PER_S mới ≤ 18M < 32M → không overflow.** B1/B2 vẫn giữ cho user ngoài CI.

---

## 📋 6 bước thay đổi

### 1 — build_matrix.yaml: Điều chỉnh override cho từng board

```yaml
# === 5 board STM32 CI ===

# F103: 72MHz ÷ 4 (PSC=3) = 18MHz — ghi đúng 18M để khỏi sai số
  bluepill_f103c8:
    template: stm32
    board: bluepill_f103c8
    build_flags_extra: ["-DTICKS_PER_S=18000000UL"]   # CHÍNH XÁC: 18M ticks/s

# G070: 64MHz ÷ 4 = 16MHz — không cần override, dùng default
  nucleo_g070rb:
    template: stm32
    board: nucleo_g070rb
    # Dùng default TICKS_PER_S=16000000UL từ pd_config.h

# F401: 84MHz ÷ 5 = 16.8MHz — ghi đúng 16.8M
  blackpill_f401cc:
    template: stm32
    board: blackpill_f401cc
    build_flags_extra: ["-DTICKS_PER_S=16800000UL"]   # CHÍNH XÁC: 16.8M ticks/s

# H743: 240MHz ÷ 15 = 16MHz — không cần override
  nucleo_h743zi:
    template: stm32
    board: nucleo_h743zi
    # Dùng default TICKS_PER_S=16000000UL từ pd_config.h

# L476: 80MHz ÷ 5 = 16MHz — không cần override
  nucleo_l476rg:
    template: stm32
    board: nucleo_l476rg
    # Dùng default TICKS_PER_S=16000000UL từ pd_config.h
```

**Sự khác biệt so với V8:**
- H743, G070, L476: bỏ override (dùng default 16M) ✅
- F103: `72000000UL` → **`18000000UL`** (timer thực 18M) ✅
- F401: `84000000UL` → **`16800000UL`** (timer thực 16.8M) ✅

### 2 — pd_config.h: Giữ default 16M (không đổi)

```cpp
// Default: STM32 timer prescaler đưa về ~16M
// Các board CI override cụ thể: F103=18M, F401=16.8M, còn lại dùng 16M
#ifndef TICKS_PER_S
#define TICKS_PER_S 16000000UL
#endif
```

### 3 — RampCalculator.h: Thêm 2 entries (18M, 16.8M) + D1 240M

**Thêm sau dòng 14 (sau 16M), trước dòng 15 (21M):**

```cpp
// ============================================================
// STM32F103 — timer sau PSC: 72MHz ÷ 4 = 18MHz
// US_TO_TICKS(2000) = 36.000 < 65535 → OK, không overflow
// ============================================================
#elif (TICKS_PER_S == 18000000L)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x322D)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x312D)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x625B)
#define US_TO_TICKS(u32)                ((u32) * 18)
#define TICKS_TO_US(u32)                ((u32) / 18)

// ============================================================
// STM32F401 — timer sau PSC: 84MHz ÷ 5 = 16.8MHz
// US_TO_TICKS(2000) = 33.600 < 65535 → OK, không overflow
// ============================================================
#elif (TICKS_PER_S == 16800000L)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x31E1)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x30E1)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x61C2)
// US_TO_TICKS: 16.8 × u32. Giữ nguyên dạng u32 × factor
#define US_TO_TICKS(u32)                ((uint32_t)((u32) * 168 / 10))
#define TICKS_TO_US(u32)                ((uint32_t)((u32) * 10 / 168))
```

*Lưu ý: Các hằng số log2 cần tính chính xác bằng tool `extras/gen_log2_const`.*

**Giữ nguyên D1 (240M) như V8:**

```cpp
#elif (TICKS_PER_S == 240000000L)
// ⚠️ WARNING: Emergency fallback — runtime overflow!
// Dùng prescaler thay vì override TICKS_PER_S=240M
#define LOG2_TICKS_PER_S               ((log2_value_t)0x37AD)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x36AD)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x6D5B)
#define US_TO_TICKS(u32)                ((u32) * 240)
#define TICKS_TO_US(u32)                ((u32) / 240)
```

### 4 — base.h dòng 56: Clamp (safety guard cho user ngoài CI)

```cpp
max_speed_in_ticks = (uint16_t)fas_min((uint32_t)(TICKS_PER_S / 1000), (uint32_t)65535);
```

### 5 — FastAccelStepper.cpp dòng 138: Clamp (safety guard cho user ngoài CI)

```cpp
uint32_t _tmp_pause = US_TO_TICKS(2000);
if (_tmp_pause > 65535) {
    // Safety guard: clamp cho user override > 32M ngoài CI
    _tmp_pause = 65535;
}
struct stepper_command_s pause_cmd = {
    .ticks = (uint16_t)_tmp_pause,
```

### 6 — Documentation

- **README.md**: Bảng TICKS_PER_S theo board + prescaler
- **pd_config.h**: Comments giải thích strategy
- **stm32_queue.cpp**: Comments PSC computation

---

## ✅ Kết quả cuối cùng

### CI boards — compile OK + runtime OK (không board nào bị degraded)

| Board | TICKS_PER_S | PSC | Timer thực | Predefined | 2ms ticks | Overflow? |
|-------|-------------|-----|-----------|-----------|-----------|-----------|
| H743 | 16M | 14 | 16.000.000 Hz | ✅ dòng 9 | 32.000 | ✅ Không |
| G070 | 16M | 3 | 16.000.000 Hz | ✅ dòng 9 | 32.000 | ✅ Không |
| L476 | 16M | 4 | 16.000.000 Hz | ✅ dòng 9 | 32.000 | ✅ Không |
| F103 | **18M** | 3 | 18.000.000 Hz | ✅ **entry mới** | 36.000 | ✅ Không |
| F401 | **16.8M** | 4 | 16.800.000 Hz | ✅ **entry mới** | 33.600 | ✅ Không |

### User ngoài CI — safety guards bảo vệ

Nếu user override TICKS_PER_S > 32M → B1/B2 clamp an toàn. Nếu user dùng default → prescaler tự động, OK.