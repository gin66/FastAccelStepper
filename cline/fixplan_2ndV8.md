# Fix Plan V8 — Ver: PSC được SET + Comment chính xác

## Thông tin
- **Ngày:** 2026-06-08 (V8)
- **Sửa so với V7:**
  1. ✅ **Xác nhận**: `FAS_TIMER->PSC = psc;` — **PSC ĐÃ được set** (stm32_queue.cpp dòng 184)
  2. ✅ **Sửa comment B2**: scope là **mọi TICKS_PER_S > 32M**, không chỉ 240M
  3. ✅ **Thêm README warning**: danh sách board bị degraded khi override

---

## 🔑 Xác nhận quan trọng: PSC đã được SET

Trong `src/pd_stm32/stm32_queue.cpp` dòng 170-200:

```cpp
// Dòng 175 — Tính PSC
psc = (fas_stm32_clock_tim_clk / TICKS_PER_S) - 1;

// Dòng 184 — SET PSC (✔ đã có)
FAS_TIMER->PSC = psc;
```

Với H743 dùng default TICKS_PER_S=16M:
- `fas_stm32_clock_tim_clk` = 240MHz (detect tự động)
- `psc = 240M/16M - 1 = 14`
- Timer counter = 240M/(14+1) = **16.000.000 Hz chính xác** ✅

---

## 📋 6 bước thay đổi

### 1 — build_matrix.yaml: Bỏ override H743
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

### 2 — pd_config.h: Default 72M → 16M
```cpp
#define TICKS_PER_S 16000000UL
// Timer prescaler tự động đưa timer về ~16M.
// H743: 240M÷15=16M chính xác ✅
// F103/F407 dùng default: sai số +12.5%/+5% — khuyến nghị override
```

### 3 — RampCalculator.h: D1 + WARNING
```cpp
#elif (TICKS_PER_S == 240000000L)
// ⚠️ WARNING: US_TO_TICKS(1000)=240000>65535 → overflow runtime!
// Emergency fallback. Không override TICKS_PER_S=240M — dùng prescaler.
#define LOG2_TICKS_PER_S               ((log2_value_t)0x37AD)  // ✅ verified
#define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 ((log2_value_t)0x36AD)  // ✅ verified
#define LOG2_ACCEL_FACTOR               ((log2_value_t)0x6D5B)  // ✅ verified
#define US_TO_TICKS(u32)                ((u32) * 240)
#define TICKS_TO_US(u32)                ((u32) / 240)
```

### 4 — base.h dòng 56: Clamp
```cpp
max_speed_in_ticks = (uint16_t)fas_min((uint32_t)(TICKS_PER_S / 1000), (uint32_t)65535);
```

### 5 — FastAccelStepper.cpp dòng 138: Clamp + comment chính xác
```cpp
uint32_t _tmp_pause = US_TO_TICKS(2000);
if (_tmp_pause > 65535) {
    // Degraded mode: Mọi TICKS_PER_S > 32M đều gây overflow.
    // 72M→144k, 84M→168k, 240M→480k — tất cả > 65535.
    // Stepper chạy được nhưng pause ngắn hơn thiết kế.
    // Khuyến nghị: dùng prescaler hoặc TICKS_PER_S ≤ 32M.
    _tmp_pause = 65535;
}
struct stepper_command_s pause_cmd = {
    .ticks = (uint16_t)_tmp_pause,
```

### 6 — README + comments
- **README.md**: STM32 prescaler section + bảng board bị degraded
- **stm32_queue.cpp**: comments initStepTimer() — PSC set OK
- **pd_config.h**: giải thích 16M default, F103/F407 cần override

---

## 🗺️ Board CI — Trạng thái đầy đủ

| Board | TICKS_PER_S | Prescaler? | Predefined | US_TO_TICKS(2000) | Degraded? |
|-------|-------------|-----------|------------|-------------------|-----------|
| **nucleo_h743zi** (mới) | **16M** default | ✅ PSC=14 → 16M | 16M ✅ dòng 9 | 32.000 ✅ | **Không** |
| ~~nucleo_h743zi (cũ)~~ | ~~240M override~~ | ~~PSC=0~~ | ~~#else branch crash~~ | ❌ | ~~CI fail~~ |
| bluepill_f103c8 | 72M override | build_flags | 72M ✅ dòng 42 | **144.000 > 65k ❌** | **B2 degraded** |
| blackpill_f401cc | 84M override | build_flags | 84M ✅ dòng 56 | **168.000 > 65k ❌** | **B2 degraded** |
| nucleo_l476rg | 80M override | build_flags | 80M ✅ dòng 49 | **160.000 > 65k ❌** | **B2 degraded** |
| nucleo_g070rb | 64M override | build_flags | 64M ✅ dòng 35 | 128.000 > 65k ❌ | **B2 degraded** |

---

## 📊 Lịch sử các Version

| Version | Ý tưởng chính | Kết quả |
|---------|---------------|---------|
| V1 | extern declarations | Sai hướng |
| V2 | Hybrid 240M + extern | Rối |
| V3 | Prescaler 16M | Bỏ sót CI |
| V4 | D1 (sai hằng số) | Hằng số sai |
| V5 | D1 đúng + prescaler | D1 overflow |
| V6 | Prescaler chính, D1 fallback | B1 silent truncation |
| V7 | B1 clamp, verify 0x6D5B | Comment B2 chưa chính xác |
| ✅ **V8** | **Xác nhận PSC set, comment B2 chính xác** | **Final ready** |