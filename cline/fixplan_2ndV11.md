# Fix Plan V11 — Timer bus per-chip analysis + H743 frequency verification

## Thông tin
- **Ngày:** 2026-06-08 (V11)
- **Thay đổi so với V10:**
  1. ✅ Kiểm tra TIM2 bus cho từng dòng chip STM32
  2. ⚠️ H743: cần xác nhận tần số thực tế (400MHz vs 480MHz)
  3. ✅ Bảng chi tiết timer clock, APB bus, prescaler
  4. ✅ Notes cho README

---

## TIM2 Bus Analysis — Từng dòng chip

### Công thức tổng quát

```
TIM2 counter clock = PCLK1 × (APB1_prescaler == 1 ? 1 : 2)
```

Nguồn: STM32 Reference Manual — "When the APB prescaler is 1, the timer clock is PCLKx. When it's >1, the timer clock is 2×PCLKx."

### F103 (bluepill_f103c8) — APB1 bus

| Property | Value |
|----------|-------|
| TIM2 bus | **APB1** |
| Datasheet | RM0008 Rev 21, §6.3.3 |
| HCLK | 72MHz |
| APB1 prescaler | ÷2 (PPRE1=4, field≥4) |
| APB1 clock (PCLK1) | 36MHz |
| ×2? | **Yes** (prescaler >1) |
| **TIM2 clock** | **72MHz** |
| PSC (cho 18M) | 72M/18M - 1 = **3** (÷4) |
| Timer thực | 18.000.000 Hz | ✅ Chính xác |

**Kiểm tra:** RM0008: *"TIM2 is on APB1. When APB1 prescaler >1, TIM2 clock = 2×PCLK1."* → 2×36MHz = 72MHz. ✅

### G070 (nucleo_g070rb) — APB bus (single bus)

| Property | Value |
|----------|-------|
| TIM3 bus (**không TIM2!**) | **APB** (single bus, dùng TIM3 thay TIM2) |
| Datasheet | RM0454 Rev 3, §4.2.1 |
| HCLK | 64MHz |
| APB prescaler | ÷1 (PPRE=0, field<4) |
| APB clock | 64MHz |
| ×2? | **No** (prescaler=1) |
| **TIM3 clock** | **64MHz** |
| PSC (cho 16M) | 64M/16M - 1 = **3** (÷4) |
| Timer thực | 16.000.000 Hz | ✅ Chính xác |

**Lưu ý:** G0/C0 **không có TIM2**, dùng **TIM3**. TIM3 also on APB bus.

### F401 (blackpill_f401cc) — APB1 bus

| Property | Value |
|----------|-------|
| TIM2 bus | **APB1** |
| Datasheet | RM0368 Rev 3, §6.3.2 |
| HCLK | 84MHz |
| APB1 prescaler | ÷2 (PPRE1=4) |
| APB1 clock (PCLK1) | 42MHz |
| ×2? | **Yes** (prescaler >1) |
| **TIM2 clock** | **84MHz** |
| PSC (cho 16.8M) | 84M/16.8M - 1 = **4** (÷5) |
| Timer thực | 16.800.000 Hz | ✅ Chính xác |

### L476 (nucleo_l476rg) — APB1 bus

| Property | Value |
|----------|-------|
| TIM2 bus | **APB1** |
| Datasheet | RM0351 Rev 9, §4.3.2 |
| HCLK | 80MHz |
| APB1 prescaler | ÷1 (PPRE1=0) |
| APB1 clock (PCLK1) | 80MHz |
| ×2? | **No** (prescaler=1) |
| **TIM2 clock** | **80MHz** |
| PSC (cho 16M) | 80M/16M - 1 = **4** (÷5) |
| Timer thực | 16.000.000 Hz | ✅ Chính xác |

### ⚠️ H743 (nucleo_h743zi) — D2 domain APB1

| Property | Value @480MHz | Value @400MHz |
|----------|---------------|---------------|
| TIM2 bus | **D2 domain APB1** | D2 domain APB1 |
| Datasheet | RM0433 Rev 7, §6.3.3 | same |
| HCLK | 480MHz | 400MHz |
| D2PPRE1 | ÷4 (dppre1≥4) | ÷2 (dppre1≥4) |
| PCLK1 | 120MHz | 200MHz |
| ×2? | **Yes** | **Yes** |
| **TIM2 clock** | **240MHz** | **200MHz** |
| PSC (cho 16M) | 240M/16M - 1 = **14** (÷15) | 200M/16M - 1 = **11** (÷12) |
| Timer thực | **16.000.000 Hz ✅** | **16.666.667 Hz ❌** |
| US_TO_TICKS(1000) | 16.000 ✅ | 16.667 ✅ |
| US_TO_TICKS(2000) | 32.000 ✅ | 33.334 ✅ |

**Kết luận:** 
- @480MHz: **chính xác 16MHz** sau PSC=14 ✅
- @400MHz: **sai số +4.17%** (16.67MHz thay vì 16M) ⚠️

### Cần kiểm tra: Tần số mặc định của nucleo_h743zi trên PlatformIO

PlatformIO ststm32 board `nucleo_h743zi`:
- `board_build.f_cpu` mặc định trong platform: thường là **400MHz** (HAL ini)
- `board_build.f_cpu` trong template: **không set** → dùng mặc định của platform

**Action:** Cần thêm `board_build_f_cpu: 480000000L` vào build_matrix.yaml nếu muốn chạy 480MHz.

**Hoặc:** Dùng TICKS_PER_S override chính xác:
- @400MHz: `TICKS_PER_S=16666667` → PSC=11 → timer 16.666.667 Hz → thêm entry 16.67M
- Nhưng đây không phải số đẹp (16.666.667), có thể gây nhầm

**Khuyến nghị:** 
1. Nếu board mặc định 400MHz: **giữ override H743** với `TICKS_PER_S=200000000UL` → PSC=9 → timer **20MHz** → cần thêm entry 20M
2. Nếu ép lên 480MHz: **bỏ override** → PSC=14 → timer **16MHz** → entry 16M có sẵn

---

## Bảng tổng hợp cho README

```markdown
### STM32 TIM2 Clock Configuration

| Board | Chip | HCLK | APB1 prescaler | PCLK1 | ×2? | TIM2 clock | PSC for 16M | Actual timer |
|-------|------|------|---------------|-------|-----|-----------|-------------|--------------|
| bluepill_f103c8 | F103 | 72MHz | ÷2 | 36MHz | Yes | **72MHz** | 3 (÷4) | **18.000 MHz** |
| nucleo_g070rb | G0 | 64MHz | ÷1 | 64MHz | No | 64MHz* | 3 (÷4) | **16.000 MHz** |
| blackpill_f401cc | F401 | 84MHz | ÷2 | 42MHz | Yes | **84MHz** | 4 (÷5) | **16.800 MHz** |
| nucleo_h743zi | H7 | 480MHz | ÷4 | 120MHz | Yes | **240MHz** | 14 (÷15) | **16.000 MHz** |
| nucleo_l476rg | L4 | 80MHz | ÷1 | 80MHz | No | **80MHz** | 4 (÷5) | **16.000 MHz** |

*G0 uses TIM3 instead of TIM2 (same APB bus)

### Cách TIM2 clock được tính

Trong `stm32_queue.cpp::getTimClock()`:
1. Gọi `HAL_RCC_GetPCLK1Freq()` → PCLK1 thực tế
2. Nếu APB1 prescaler > 1 (field ≥ 4): nhân đôi (×2)
3. Kết quả = TIM2 counter clock

**Không dùng F_CPU** — runtime detection, đảm bảo chính xác với mọi config.

### Khuyến nghị tần số

Mỗi board dùng TICKS_PER_S bằng **timer thực sau PSC** — đảm bảo:
- US_TO_TICKS(2000) ≤ 65535 → không overflow
- Timer đếm chính xác, không sai số
- Prescaler integer (PSC = timer_clk/TICKS_PER_S - 1 chia hết)

| Board | TICKS_PER_S khuyến nghị | Source | PSC | Timer thực |
|-------|------------------------|--------|-----|-----------|
| F103 | **18.000.000** | override build_flags | 3 | 18.000.000 Hz |
| G070 | **16.000.000** | default pd_config.h | 3 | 16.000.000 Hz |
| F401 | **16.800.000** | override build_flags | 4 | 16.800.000 Hz |
| H743 @480MHz | **16.000.000** | default pd_config.h | 14 | 16.000.000 Hz |
| H743 @400MHz | **20.000.000** or override | cần quyết định | 9 | 20.000.000 Hz |
| L476 | **16.000.000** | default pd_config.h | 4 | 16.000.000 Hz |
```

---

## Cập nhật kế hoạch V11

| # | File | Thay đổi | Ghi chú |
|---|------|---------|---------|
| 1 | `build_matrix.yaml` | F103: 18M; F401: 16.8M; G070/L476: bỏ override; **H743: còn pending do freq** | |
| 2 | `pd_config.h` | Default 16M + comments | |
| 3 | `RampCalculator.h` | Thêm entries 18M, 16.8M; **xóa D1 240M**; **#else→#error**; **có thể thêm 20M cho H743** | Cần tool gen |
| 4 | `RampGenerator.h` | Xóa extern declarations | |
| 5 | `base.h` | Clamp dòng 56 | |
| 6 | `FastAccelStepper.cpp` | Clamp dòng 138 | |
| 7 | **`README.md`** | **Bảng TIM2 bus + H743 frequency note** | |
| 8 | **`pd_config.h`** | **Cập nhật comments: timer bus, PSC formula** | |

### Pending: H743 frequency

Cần xác nhận:
- `platformio run -e nucleo_h743zi` dùng F_CPU nào?
- Nếu 400MHz → `TICKS_PER_S=20000000` → PSC=9 → timer 20MHz → cần entry 20M
- Nếu 480MHz → `TICKS_PER_S=16000000` → PSC=14 → timer 16MHz → entry 16M có sẵn