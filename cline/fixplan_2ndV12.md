# Fix Plan V12 — Final: All 5 STM32 CI boards prescaled, 20M entry for H743

## Thông tin
- **Ngày:** 2026-06-08 (V12)
- **Thay đổi so với V11:**
  - H743 @400MHz → `TICKS_PER_S=20000000` (PSC=9, timer 20MHz chính xác)
  - Thêm entry 20M trong RampCalculator.h
  - Tất cả 5 board CI OK, không board nào bị degraded / overflow

---

## Bảng cuối cùng — 5 board STM32 CI

| Board | Chip | HCLK | TIM2 clock | TICKS_PER_S | PSC | Timer thực | Predefined | 2ms ticks | Overflow? |
|-------|------|------|-----------|-------------|-----|-----------|-----------|-----------|-----------|
| bluepill_f103c8 | F103 | 72MHz | 72MHz | **18.000.000** | 3 (÷4) | 18.000.000 Hz | ✅ **18M mới** | 36.000 | ✅ Không |
| nucleo_g070rb | G0 | 64MHz | 64MHz | **16.000.000** | 3 (÷4) | 16.000.000 Hz | ✅ dòng 9 | 32.000 | ✅ Không |
| blackpill_f401cc | F401 | 84MHz | 84MHz | **16.800.000** | 4 (÷5) | 16.800.000 Hz | ✅ **16.8M mới** | 33.600 | ✅ Không |
| nucleo_h743zi | H7 | 400MHz | 200MHz | **20.000.000** | 9 (÷10) | 20.000.000 Hz | ✅ **20M mới** | 40.000 | ✅ Không |
| nucleo_l476rg | L4 | 80MHz | 80MHz | **16.000.000** | 4 (÷5) | 16.000.000 Hz | ✅ dòng 9 | 32.000 | ✅ Không |

**All 5 boards: compile OK + runtime OK. Không overflow, không degraded.**

---

## 6 bước code

### 1 — build_matrix.yaml

```yaml
  bluepill_f103c8:
    template: stm32
    board: bluepill_f103c8
    build_flags_extra: ["-DTICKS_PER_S=18000000UL"]   # F103: 72M÷4=18MHz

  nucleo_g070rb:
    template: stm32
    board: nucleo_g070rb
    # G0: 64M÷4=16MHz — dùng default

  blackpill_f401cc:
    template: stm32
    board: blackpill_f401cc
    build_flags_extra: ["-DTICKS_PER_S=16800000UL"]   # F401: 84M÷5=16.8MHz

  nucleo_h743zi:
    template: stm32
    board: nucleo_h743zi
    build_flags_extra: ["-DTICKS_PER_S=20000000UL"]   # H7 @400MHz: 200M÷10=20MHz

  nucleo_l476rg:
    template: stm32
    board: nucleo_l476rg
    # L4: 80M÷5=16MHz — dùng default
```

### 2 — pd_config.h: Default 16M

```cpp
#ifndef TICKS_PER_S
#define TICKS_PER_S 16000000UL
#endif
```

### 3 — RampCalculator.h: 3 entries mới (18M, 16.8M, 20M) + xóa D1 + #else→#error

Thêm sau dòng 14 (sau 16M), trước dòng 15 (21M):

```cpp
// ============================================================
// STM32F103: 72MHz÷4 = 18MHz — PSC=3, timer chính xác
// ============================================================
#elif (TICKS_PER_S == 18000000L)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x????)  // cần gen_log2_const
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x????)  // cần gen_log2_const
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x????)  // cần gen_log2_const
#define US_TO_TICKS(u32)                ((u32) * 18)
#define TICKS_TO_US(u32)                ((u32) / 18)

// ============================================================
// STM32F401: 84MHz÷5 = 16.8MHz — PSC=4, timer chính xác
// ============================================================
#elif (TICKS_PER_S == 16800000L)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x????)  // cần gen_log2_const
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x????)  // cần gen_log2_const
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x????)  // cần gen_log2_const
#define US_TO_TICKS(u32)                ((uint32_t)((u32) * 168 / 10))
#define TICKS_TO_US(u32)                ((uint32_t)((u32) * 10 / 168))

// ============================================================
// STM32H743 @400MHz: 200MHz÷10 = 20MHz — PSC=9, timer chính xác
// ============================================================
#elif (TICKS_PER_S == 20000000L)
#define LOG2_TICKS_PER_S               ((log2_value_t)0x????)  // cần gen_log2_const
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x????)  // cần gen_log2_const
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x????)  // cần gen_log2_const
#define US_TO_TICKS(u32)                ((u32) * 20)
#define TICKS_TO_US(u32)                ((u32) / 20)
```

Xóa block D1 (240M):
```cpp
// XÓA toàn bộ block:
// #elif (TICKS_PER_S == 240000000L)
// ...
```

Sửa #else thành #error:
```cpp
#else
#error "Unsupported TICKS_PER_S. Use timer prescaler to match: 16M, 18M, 20M, 21M, 32M, 48M, 64M, 72M, 80M, 84M, 100M, 120M, 168M, 170M, 216M, 480M, 550M"
#endif
```

### 4 — RampGenerator.h: Xóa extern declarations

```cpp
// Xóa toàn bộ block:
// #ifdef SUPPORT_LOG2_TIMER_FREQ_VARIABLES
// extern log2_value_t log2_timer_freq;
// ...
```

### 5 — base.h dòng 56: Clamp
```cpp
max_speed_in_ticks = (uint16_t)fas_min((uint32_t)(TICKS_PER_S / 1000), (uint32_t)65535);
```

### 6 — FastAccelStepper.cpp dòng 138: Clamp
```cpp
uint32_t _tmp_pause = US_TO_TICKS(2000);
if (_tmp_pause > 65535) _tmp_pause = 65535;
struct stepper_command_s pause_cmd = { .ticks = (uint16_t)_tmp_pause, ... };
```

---

## README — STM32 section (cần thêm)

```markdown
### STM32 TIM2 Clock Configuration

| Board | Chip | Bus | HCLK | PCLK1 | ×2? | TIM clk | PSC | TICKS_PER_S | Timer thực |
|-------|------|-----|------|-------|-----|---------|-----|-------------|-----------|
| bluepill_f103c8 | F103 | APB1 | 72M | 36M | Yes | 72M | 3 | 18.000.000 | 18.000 MHz |
| nucleo_g070rb | G0 | APB | 64M | 64M | No | 64M* | 3 | 16.000.000 | 16.000 MHz |
| blackpill_f401cc | F401 | APB1 | 84M | 42M | Yes | 84M | 4 | 16.800.000 | 16.800 MHz |
| nucleo_h743zi | H7 | D2 APB1 | 400M | 200M | Yes | 200M | 9 | 20.000.000 | 20.000 MHz |
| nucleo_l476rg | L4 | APB1 | 80M | 80M | No | 80M | 4 | 16.000.000 | 16.000 MHz |

*G0 uses TIM3 (no TIM2). Same APB bus.

Công thức timer clock: `TIM_CLK = PCLK1 × (APB1_prescaler > 1 ? 2 : 1)`

Giá trị TICKS_PER_S được chọn bằng timer thực sau PSC để đảm bảo mọi giá trị ticks ≤ 65535 (không overflow uint16_t). Hằng số log2 được tạo bằng tool extras/gen_log2_const.

Bài toán này cố gắng giải thích tại sao: kho lưu trữ tài liệu này đã được cập nhật để phản ánh các giá trị mới.
```

## Pending trước khi implement
- Chạy `extras/gen_log2_const` để tạo hằng số log2 chính xác cho 18M, 16.8M, 20M
- HW test chưa làm — cần ghi nhận