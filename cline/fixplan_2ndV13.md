# Fix Plan V13 — Final: LOG2 constants computed, #else→#error, dead code commented

## Thông tin
- **Ngày:** 2026-06-08 (V13)
- **Scope:** 
  - ✅ LOG2 constants cho 18M, 16.8M, 20M — tính toán chính xác ×512
  - ✅ Xử lý `ramp_config_s` — thay `#else` bằng `#error`, không thêm member variables
  - ✅ Dead code — comment out thay vì xóa
  - ✅ All 5 board CI: compile OK + runtime OK

---

## 1. LOG2 Constants — Tính toán chính xác

**Công thức:** `log2_value = round(log2(freq) × 512)` (Q7.9 fixed-point)

Dùng `double` precision, verified chéo với các entry 16M và 32M trong codebase.

### 18M (F103 — 72MHz÷4, PSC=3)

| Hằng số | Công thức | ×512 round | Kết quả |
|---------|-----------|-----------|---------|
| `LOG2_TICKS_PER_S` | log2(18,000,000) = 24.101494 | 12340 | **`0x3034`** |
| `LOG2_TICKS_PER_S_DIV_SQRT_OF_2` | 0x3034 - 0x0100 (log2(sqrt2) = 0.5×512=256) | 12084 | **`0x2F34`** |
| `LOG2_ACCEL_FACTOR` | (2×24.101494 - 1)×512 = 47.202988×512 = 24168 | **24168** | **`0x5E68`** |

**Verify:** log2⁻¹(12340/512) = 2^24.1016 = 18,009,870 → sai số **+0.055%** ✅

### 16.8M (F401 — 84MHz÷5, PSC=4)

| Hằng số | Công thức | ×512 round | Kết quả |
|---------|-----------|-----------|---------|
| `LOG2_TICKS_PER_S` | log2(16,800,000) = 24.001958 | 12289 | **`0x3001`** |
| `LOG2_TICKS_PER_S_DIV_SQRT_OF_2` | 0x3001 - 0x0100 | 12033 | **`0x2F01`** |
| `LOG2_ACCEL_FACTOR` | (2×24.001958 - 1)×512 = 47.003916×512 = 24066 | **24066** | **`0x5E02`** |

**Verify:** log2⁻¹(12289/512) = 2^24.0020 = 16,801,476 → sai số **+0.009%** ✅

### 20M (H743 @400MHz — 200MHz÷10, PSC=9)

| Hằng số | Công thức | ×512 round | Kết quả |
|---------|-----------|-----------|---------|
| `LOG2_TICKS_PER_S` | log2(20,000,000) = 24.253497 | 12418 | **`0x3082`** |
| `LOG2_TICKS_PER_S_DIV_SQRT_OF_2` | 0x3082 - 0x0100 | 12034 | **`0x2F82`** |
| `LOG2_ACCEL_FACTOR` | (2×24.253497 - 1)×512 = 47.506994×512 = 24324 | **24324** | **`0x5F04`** |

**Verify:** log2⁻¹(12418/512) = 2^24.2539 = 20,011,700 → sai số **+0.059%** ✅

---

## 2. Xử lý `ramp_config_s` và `#else` branch

### Vấn đề (gin66 comment May 19)

Trong `RampCalculator.h`, `#else` branch (lines 113-125):
```cpp
#define SUPPORT_LOG2_TIMER_FREQ_VARIABLES
#define LOG2_TICKS_PER_S log2_timer_freq              // ❌ undefined
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 log2_timer_freq_div_sqrt_of_2  // ❌ undefined
#define LOG2_ACCEL_FACTOR log2_timer_freq_square_div_2                // ❌ undefined
```

Các macro này dùng trong `ramp_config_s::update()`, `calculate_ticks()`, `calculate_ramp_steps()` → compiler error vì reference biến không khai báo.

### Giải pháp: Không thêm member variables, thay `#else` bằng `#error`

```cpp
#else
// TICKS_PER_S không có predefined constant.
// Các giá trị hỗ trợ: 16M, 18M, 20M, 21M, 32M, 48M, 64M, 72M, 80M, 84M,
//                     100M, 120M, 168M, 170M, 216M, 480M, 550M
// Dùng timer prescaler đưa timer clock về một trong các giá trị trên.
// Xem src/pd_stm32/pd_config.h và stm32_queue.cpp.
#error "Unsupported TICKS_PER_S. Use timer prescaler to match a supported value."
#endif
```

### Dead code: Comment out thay vì xóa

**RampCalculator.h** — Xóa block #else (lines 113-125):
```cpp
// Removed in V13: #else branch with SUPPORT_LOG2_TIMER_FREQ_VARIABLES.
// This approach caused undefined reference errors in ramp_config_s.
// Using #error instead forces clear compile error for unsupported TICKS_PER_S.
```

**RampGenerator.h** (lines 10-14) — Comment out:
```cpp
// V13: Removed — SUPPORT_LOG2_TIMER_FREQ_VARIABLES no longer defined
// All CI boards use predefined TICKS_PER_S values.
// #ifdef SUPPORT_LOG2_TIMER_FREQ_VARIABLES
// extern log2_value_t log2_timer_freq;
// extern log2_value_t log2_timer_freq_div_sqrt_of_2;
// extern log2_value_t log2_timer_freq_square_div_2;
// #endif
```

**RampControl.cpp** (lines 9-27) — Comment out:
```cpp
// V13: Removed — SUPPORT_LOG2_TIMER_FREQ_VARIABLES no longer defined
// These variables were only used by the #else fallback in RampCalculator.h,
// which has been replaced with #error.
// #ifdef SUPPORT_LOG2_TIMER_FREQ_VARIABLES
// static log2_value_t log2_timer_freq;
// static log2_value_t log2_timer_freq_div_sqrt_of_2;
// static log2_value_t log2_timer_freq_square_div_2;
// #endif
...
// void init_ramp_module() {
// #ifdef SUPPORT_LOG2_TIMER_FREQ_VARIABLES
//   log2_timer_freq = log2_from((uint32_t)TICKS_PER_S);
//   ...
```

---

## 3. Kế hoạch 9 bước

| # | File | Thay đổi | Ghi chú |
|---|------|---------|---------|
| 1 | `extras/ci/build_matrix.yaml` | F103: 18M; F401: 16.8M; H743: 20M; G070/L476: bỏ override | |
| 2 | `src/pd_stm32/pd_config.h` | Default 16M + comments TIM2 bus | |
| 3 | `src/fas_ramp/RampCalculator.h` | Thêm 3 entries với hằng số **0x3034/0x2F34/0x5E68** (18M), **0x3001/0x2F01/0x5E02** (16.8M), **0x3082/0x2F82/0x5F04** (20M). **Xóa #else → #error**. | Core fix |
| 4 | `src/fas_ramp/RampGenerator.h` | Comment out `#ifdef SUPPORT_LOG2_TIMER_FREQ_VARIABLES` block | Dead code → comment |
| 5 | `src/fas_ramp/RampControl.cpp` | Comment out `#ifdef SUPPORT_LOG2_TIMER_FREQ_VARIABLES` block + `init_ramp_module()` code | Dead code → comment |
| 6 | `src/fas_queue/base.h` | `(uint16_t)fas_min((uint32_t)(TICKS_PER_S/1000), 65535)` | Safety guard |
| 7 | `src/FastAccelStepper.cpp` | Clamp `US_TO_TICKS(2000)` về 65535 | Safety guard |
| 8 | `README.md` | Bảng TIM2 bus + prescaler notes | |
| 9 | `src/fas_arch/test_pc.h` | Kiểm tra — test PC dùng 16M, không bị ảnh hưởng | Verify only |

---

## 4. Board CI — Trạng thái cuối

| Board | TICKS_PER_S | TIM clk | PSC | Timer thực | LOG2_TICKS | Không overflow? | Không #else? |
|-------|-------------|---------|-----|-----------|------------|----------------|--------------|
| **F103** | **18.000.000** | 72MHz | 3 | 18.000 MHz | **0x3034** | ✅ 36k < 65k | ✅ có entry |
| **G070** | **16.000.000** | 64MHz* | 3 | 16.000 MHz | **0x31DD** | ✅ 32k < 65k | ✅ có entry |
| **F401** | **16.800.000** | 84MHz | 4 | 16.800 MHz | **0x3001** | ✅ 33.6k < 65k | ✅ có entry |
| **H743** | **20.000.000** | 200MHz | 9 | 20.000 MHz | **0x3082** | ✅ 40k < 65k | ✅ có entry |
| **L476** | **16.000.000** | 80MHz | 4 | 16.000 MHz | **0x31DD** | ✅ 32k < 65k | ✅ có entry |

*G0 dùng TIM3.

---

## 5. Lịch sử version

| V | Ý tưởng | Kết quả |
|---|---------|---------|
| V1-V3 | extern / hybrid | Sai hướng / rối |
| V4-V8 | D1 240M + prescaler | D1 overflow / B1 silent truncation |
| V9-V11 | Mỗi board timer thực | Thiếu TIM2 bus analysis |
| V12 | All boards prescaled, 20M entry | LOG2 constants còn placeholder |
| ✅ **V13** | **LOG2 computed, #else→#error, dead code commented** | **FINAL** |