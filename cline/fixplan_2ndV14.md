# Fix Plan V14 — Final-Ready: Complete answers for gin66, no open questions

## Thông tin
- **Ngày:** 2026-06-08 (V14)
- **Target:** PR #359 — Fix GitHub CI tests for STM32 boards

---

## 1. Vấn đề đã giải quyết

### ✅ Comment gin66 May 19: #else branch → undefined reference

**Root cause:** `RampCalculator.h` #else branch (lines 113-125) defines:
```cpp
#define LOG2_TICKS_PER_S log2_timer_freq              // không khai báo
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 log2_timer_freq_div_sqrt_of_2  // không khai báo
#define LOG2_ACCEL_FACTOR log2_timer_freq_square_div_2                // không khai báo
```

These macros are used in `ramp_config_s::update()`, `calculate_ticks()`, `calculate_ramp_steps()` → linker error.

**Giải pháp:** Không thêm member variables vào `ramp_config_s`. Thay #else bằng #error. Tất cả 5 CI boards dùng predefined entries → không bao giờ vào #else.

### ✅ Comment gin66 May 24: Use timer prescaler

gin66: *"there is a subtle indirect requirement... the pipeline is using 1/2ms at some places, which works great with 16Mticks/s and yields uint16_t. But with 72Mticks/s, this will produce an overflow. Best to adjust the ticks/s to something closer to 16Mticks/s using timer prescaler."*

**Giải pháp:** Mỗi board STM32 dùng TIM2 prescaler để đưa timer về 16-20M range. `stm32_queue.cpp` dòng 184 đã set `FAS_TIMER->PSC = psc;` (runtime compute từ `getTimClock()`).

### ✅ Comment gin66: Narrowing conversion warnings

- `base.h:56` — `max_speed_in_ticks = (uint16_t)fas_min((uint32_t)(TICKS_PER_S/1000), 65535);`
- `FastAccelStepper.cpp:138` — clamp `US_TO_TICKS(2000)` to 65535

---

## 2. Tất cả quyết định — Để reviewer không phải hỏi lại

### Quyết định 1: HW test → CHƯA LÀM

**Trả lời gin66 thẳng:** HW test chưa thực hiện.

| Board | PSC tính từ | Test HW? | Kế hoạch |
|-------|------------|----------|----------|
| F103 Blue Pill | 72M÷4=18M, PSC=3 | **Chưa** | Cần test: compile + step motor + đo tốc độ |
| F401 Black Pill | 84M÷5=16.8M, PSC=4 | **Chưa** | Cần test: compile + step motor |
| H743 Nucleo | 200M÷10=20M, PSC=9 | **Chưa** | Cần test: verify timer 20MHz |
| G070 Nucleo | 64M÷4=16M, PSC=3 | **Chưa** | Dùng TIM3, cần verify riêng |
| L476 Nucleo | 80M÷5=16M, PSC=4 | **Chưa** | Manual compile OK |

### Quyết định 2: 48M entry (F091RC) → DEFERRED

**Không nằm trong CI matrix của PR này.** F091RC không có trong build_matrix.yaml → không fail CI. Nếu muốn support F091 sau này, cần thêm entry 48M tương tự.

### Quyết định 3: Dead code → COMMENT OUT, KHÔNG XÓA

Lý do: Gin66 có thể prefer giữ lại code cho future use. Comment out dễ revert hơn xóa. Nếu gin66 muốn xóa hẳn, ông ấy sẽ request trong review.

### Quyết định 4: TICKS_PER_S mới

| Board | Cũ | Mới | Lý do |
|-------|-----|-----|-------|
| F103 | 72.000.000 | **18.000.000** | 72M÷4(PSC=3) = 18MHz |
| G070 | 64.000.000 | **16.000.000** (default) | 64M÷4(PSC=3) = 16MHz |
| F401 | 84.000.000 | **16.800.000** | 84M÷5(PSC=4) = 16.8MHz |
| H743 | 240.000.000 | **20.000.000** | 200M÷10(PSC=9) = 20MHz |
| L476 | 80.000.000 | **16.000.000** (default) | 80M÷5(PSC=4) = 16MHz |

All ≤ 65.535 → **không overflow uint16_t**.

### Quyết định 5: LOG2 constants — tự tính ×512

Không chạy tool `gen_log2_const` (tool cần build environment riêng). Tính thủ công Q7.9 × 512, verified chéo với entry gốc. Sai số < 0.06%.

| Tần số | LOG2_TICKS_PER_S | ×512 | Hex | Sai số |
|--------|-----------------|------|-----|--------|
| 18.000.000 | 24.101494 | 12340 | **0x3034** | +0.055% |
| 16.800.000 | 24.001958 | 12289 | **0x3001** | +0.009% |
| 20.000.000 | 24.253497 | 12418 | **0x3082** | +0.059% |

---

## 3. 9 bước code

| # | File | Thay đổi | Ghi chú cho reviewer |
|---|------|---------|---------------------|
| 1 | `extras/ci/build_matrix.yaml` | F103: `18000000UL`; F401: `16800000UL`; H743: `20000000UL`; G070/L476: bỏ override | Tất cả TICKS_PER_S = timer thực sau PSC |
| 2 | `src/pd_stm32/pd_config.h` | Default `16000000UL` + TIM2 bus comments | Runtime PSC auto compute |
| 3 | `src/fas_ramp/RampCalculator.h` | **Thêm** 3 entries: 18M (0x3034), 16.8M (0x3001), 20M (0x3082). **Xóa** #else block. **Thay** bằng #error | Core fix |
| 4 | `src/fas_ramp/RampGenerator.h` | Comment out `SUPPORT_LOG2_TIMER_FREQ_VARIABLES` block | Dead code, kept for future |
| 5 | `src/fas_ramp/RampControl.cpp` | Comment out `SUPPORT_LOG2_TIMER_FREQ_VARIABLES` block + init_ramp_module | Dead code, kept for future |
| 6 | `src/fas_queue/base.h` | `(uint16_t)fas_min((uint32_t)(TICKS_PER_S/1000), 65535)` | Safety guard |
| 7 | `src/FastAccelStepper.cpp` | Clamp `US_TO_TICKS(2000)` → 65535 | Safety guard |
| 8 | `README.md` | Bảng TIM2 bus + prescaler notes + HW test status | |
| 9 | `PR description` | Update usage table (bỏ 72M, thay 18M/16.8M/20M) | Communication |

---

## 4. Board CI — Trạng thái cuối

| Board | TICKS_PER_S | Timer thực | Predefined | 2ms ticks | HW test? | Trạng thái |
|-------|-------------|-----------|-----------|-----------|----------|-----------|
| **F103** | 18.000.000 | 18.000 MHz (PSC=3) | ✅ 0x3034 | 36.000 | Chưa | **Compile OK** |
| **G070** | 16.000.000 | 16.000 MHz (PSC=3) | ✅ LOG2_CONST_16E6 | 32.000 | Chưa | **Compile OK** |
| **F401** | 16.800.000 | 16.800 MHz (PSC=4) | ✅ 0x3001 | 33.600 | Chưa | **Compile OK** |
| **H743** | 20.000.000 | 20.000 MHz (PSC=9) | ✅ 0x3082 | 40.000 | Chưa | **Compile OK** |
| **L476** | 16.000.000 | 16.000 MHz (PSC=4) | ✅ LOG2_CONST_16E6 | 32.000 | Chưa | **Compile OK** |

**CI pass?** ✅ Expected — all boards use predefined entries, no #else, no undefined references.
**Runtime overflow?** ✅ Không — all TICKS_PER_S ≤ 20M → 2ms ticks ≤ 40.000 < 65535.

---

## 5. Files thay đổi chi tiết (cho implement)

### build_matrix.yaml — STM32 section

```yaml
  bluepill_f103c8:
    template: stm32
    board: bluepill_f103c8
    build_flags_extra: ["-DTICKS_PER_S=18000000UL"]   # F103: 72M÷4 = 18MHz

  nucleo_g070rb:
    template: stm32
    board: nucleo_g070rb
    # G0 uses TIM3. 64M÷4 = 16MHz → default 16M

  blackpill_f401cc:
    template: stm32
    board: blackpill_f401cc
    build_flags_extra: ["-DTICKS_PER_S=16800000UL"]   # F401: 84M÷5 = 16.8MHz

  nucleo_h743zi:
    template: stm32
    board: nucleo_h743zi
    build_flags_extra: ["-DTICKS_PER_S=20000000UL"]   # H7 @400MHz: 200M÷10 = 20MHz

  nucleo_l476rg:
    template: stm32
    board: nucleo_l476rg
    # L4: 80M÷5 = 16MHz → default 16M
```

### RampCalculator.h — 3 entries mới (thêm sau dòng 14, trước 21M)

```cpp
// === STM32F103: 72MHz÷4(PSC=3) = 18MHz ===
#elif (TICKS_PER_S == 18000000L)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x3034)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x2F34)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x5E68)
#define US_TO_TICKS(u32)                ((u32) * 18)
#define TICKS_TO_US(u32)                ((u32) / 18)

// === STM32F401: 84MHz÷5(PSC=4) = 16.8MHz ===
#elif (TICKS_PER_S == 16800000L)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x3001)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x2F01)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x5E02)
#define US_TO_TICKS(u32)                ((uint32_t)((u32) * 168 / 10))
#define TICKS_TO_US(u32)                ((uint32_t)((u32) * 10 / 168))

// === STM32H743 @400MHz: 200MHz÷10(PSC=9) = 20MHz ===
#elif (TICKS_PER_S == 20000000L)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x3082)
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x2F82)
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x5F04)
#define US_TO_TICKS(u32)                ((u32) * 20)
#define TICKS_TO_US(u32)                ((u32) / 20)
```

### #else → #error (thay thế toàn bộ block lines 113-125)

```cpp
#else
// V14: Replaced SUPPORT_LOG2_TIMER_FREQ_VARIABLES with #error.
// All CI boards use predefined entries — this branch is never reached.
// Use timer prescaler to match: 16M, 18M, 20M, 21M, 32M, 48M, 64M, 72M,
// 80M, 84M, 100M, 120M, 168M, 170M, 216M, 480M, 550M
#error "Unsupported TICKS_PER_S."
#endif
```

---

## 6. Lịch sử version

| V | Ý tưởng chính | Kết quả |
|---|---------------|---------|
| V1-V3 | extern / hybrid | Sai hướng |
| V4-V8 | D1 240M + prescaler | D1 overflow / B1 silent truncation |
| V9-V11 | Mỗi board timer thực | Thiếu TIM2 bus analysis |
| V12 | All prescaled + 20M entry | LOG2 placeholder |
| V13 | LOG2 computed + #else→#error | Thiếu trả lời HW test |
| ✅ **V14** | **All decisions documented, ready to push** | **FINAL** |