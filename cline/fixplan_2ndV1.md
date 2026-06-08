# Fix Plan V3 - Prescaler Approach + Safety Guards

## Thông tin
- **Ngày:** 2026-06-08 (cập nhật V3)
- **Scope:**
  - Core fix: Đưa TICKS_PER_S về 16MHz bằng timer prescaler
  - Safety guard: Cast/clamp cho các board tương lai
  - Documentation: Cập nhật README + comments
- **PR tham chiếu:** #359

---

## 🟢 Core Insight của gin66: Timer Prescaler

> "The pipeline is using 1/2ms at some places, which works great with 16Mticks/s and yields uint16_t. But with 72Mticks/s, this will produce an overflow."

Toàn bộ queue infrastructure (base.h, queue_entry, RMT encoder) được thiết kế với giả định `uint16_t` đủ chứa mọi giá trị ticks. Ở 72MHz, 1ms = 72.000 ticks > 65535 → tràn.

**Giải pháp:** Timer prescaler đưa TICKS_PER_S về ~16M, giống cách Pico 80MHz và các board ESP32 đã làm.

Trong `stm32_queue.cpp`, PSC đã được tính tự động:
```cpp
uint32_t psc = (fas_stm32_clock_tim_clk / TICKS_PER_S) - 1;
```
→ Chỉ cần đổi `TICKS_PER_S` trong `pd_config.h` là đủ.

---

## 📋 Các thay đổi trong V3

### A. Core Fix (1 file)

| # | File | Thay đổi | Ghi chú |
|---|------|---------|---------|
| A1 | `src/pd_stm32/pd_config.h` | `TICKS_PER_S` default: `72000000UL` → **`16000000UL`** | Prescaler auto-adjust |

**Tác động lên các board CI:**
- **nucleo_h743zi** (240MHz, build_flags không có TICKS_PER_S):
  Timer clock = 240MHz → PSC = 240M/16M - 1 = 14 → timer **16.000.000 Hz ✅**
  Entry 16M đã có sẵn trong RampCalculator.h → **compile OK**
- **Các board có override TICKS_PER_S** trong build_flags → không bị ảnh hưởng:
  - `bluepill_f103c8`: `-DTICKS_PER_S=72000000UL` ✅
  - `nucleo_g070rb`: `-DTICKS_PER_S=64000000UL` ✅
  - `blackpill_f401cc`: `-DTICKS_PER_S=84000000UL` ✅
  - `nucleo_l476rg`: `-DTICKS_PER_S=80000000UL` ✅

**Tác động lên board không override (F103, F407 dùng default 16M):**
- F103 @72MHz: PSC = 72M/16M - 1 = 3 (÷4) → timer 18MHz → **+12.5% sai số tốc độ** ⚠️
- F407 @84MHz: PSC = 84M/16M - 1 = 4 (÷5) → timer 16.8MHz → **+5% sai số tốc độ** ⚠️
- **Khuyến nghị:** Dùng `build_flags: -DTICKS_PER_S=<clock>` để override cho kết quả chính xác

### B. Safety Guards (2 files)

| # | File | Vị trí | Thay đổi | Loại |
|---|------|--------|---------|------|
| B1 | `src/fas_queue/base.h` | dòng 56 | `(uint16_t)(TICKS_PER_S / 1000)` | Cast |
| B2 | `src/FastAccelStepper.cpp` | dòng 138 | clamp `if(tmp>65535)tmp=65535; (uint16_t)tmp` | Clamp |

### C. Chỉ thay đổi Comments trong toàn bộ repo

| # | File | Thay đổi |
|---|------|---------|
| C1 | `src/pd_stm32/pd_config.h` | Cập nhật comments: giải thích prescaler strategy, TICKS_PER_S=16M default, F103/F407 cần override |
| C2 | `README.md` | Thêm note về STM32: prescaler + khuyến nghị override cho F103/F407 |
| C3 | `src/pd_stm32/stm32_queue.cpp` | Cập nhật comments trong `initStepTimer()` và phần clock validation |

---

## ✅ Files KHÔNG cần thay đổi

| File | Lý do |
|------|-------|
| `src/fas_ramp/RampCalculator.h` | Entry 16M đã có sẵn ở dòng 9-14. Không cần thêm 240M |
| `src/fas_ramp/RampGenerator.h` | Không cần extern declarations (sai hướng — bỏ) |
| `src/fas_ramp/RampControl.cpp` | `init_ramp_module()` vẫn chạy khi `#else` branch active, nhưng với 16M thì `#else` không bao giờ chạy |

---

## 📊 So sánh các Version

| Version | Approach | Số file sửa | Phức tạp | Độ tin cậy |
|---------|----------|-------------|----------|------------|
| ❌ V1 | Member variables + extern declarations | 3 | Trung bình | Compile OK, overflow vẫn tồn tại |
| ❌ V2 | Hybrid: predefined 240M + extern + cast | 3 | Cao | Compile OK, logic rối |
| ✅ **V3** | **Prescaler 16M + safety guards** | **5 (3 file code + 2 file doc)** | **Thấp** | **Tận gốc, clean** |