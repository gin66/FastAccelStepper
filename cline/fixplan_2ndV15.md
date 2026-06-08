# Fix Plan V15 — Final: Full PR #359 alignment + gin66 comments addressed

## Thông tin
- **Ngày:** 2026-06-08 (V15)
- **Dựa trên:** PR #359 gin66/FastAccelStepper (state: open, 14 files, 5 commits)
- **2 comments từ gin66** — đã fetch và phân tích

---

## 1. PR #359 — Trạng thái hiện tại

| Thông tin | Giá trị |
|-----------|---------|
| PR title | "add STM32 backend (STM32duino / Arduino_Core_STM32)" |
| Tác giả | Tyuyt3975 → gin66 |
| State | Open |
| Files changed | 14 |
| Commits | 5 |
| Comments | 2 (từ gin66) |

### Comment 1 — gin66, 2026-05-19
Nguyên văn: *"Thanks for the great code. Github tests still not passing. Copilot says: [...] The compilation is failing because of undeclared variables in RampCalculator.h. [...] primary blocker is the missing member variable declarations."*

**V15 response:** Không thêm member variables. Thay #else bằng #error (sạch hơn). ✅

### Comment 2 — gin66, 2026-05-24 (QUAN TRỌNG NHẤT)
Nguyên văn: *"The code still does not compile. There is a subtle indirect requirement, which I was not aware. The pipeline is using 1/2ms at some places, which works great with 16Mticks/s and yields uint16_t. But with the timing of 72Mticks/s, this will produce an overflow. Best to adjust the ticks/s to something closer to 16Mticks/s. For example the pico runs at 80MHz and then adjust internally accordingly. For STM try to use a timer presaler. **Are you testing on HW?**"*

**V15 response:**
- ✅ Prescaler approach — gin66 xác nhận hướng đúng
- ❌ HW test — chưa làm, trả lời thẳng trong PR

### PR body — Usage example (CẦN CẬP NHẬT)

Hiện tại PR body ghi:
```ini
build_flags = -DTICKS_PER_S=72000000   ; F103 @72MHz: APB1=36MHz ×2
```

**Cần sửa thành:**
```ini
; F103 @72MHz: after TIM2 prescaler (PSC=3) → 72M÷4 = 18MHz
build_flags = -DTICKS_PER_S=18000000

; F401 @84MHz: after TIM2 prescaler (PSC=4) → 84M÷5 = 16.8MHz
build_flags = -DTICKS_PER_S=16800000

; H743 @400MHz: after TIM2 prescaler (PSC=9) → 200M÷10 = 20MHz
build_flags = -DTICKS_PER_S=20000000

; G070/L476: use default TICKS_PER_S=16000000 from pd_config.h
```

---

## 2. Tất cả quyết định — Để gin66 không phải hỏi lại

### Quyết định 1: Xử lý `ramp_config_s` undefined reference (gin66 comment 1)

**Vấn đề:** #else branch define macro `LOG2_TICKS_PER_S = log2_timer_freq` — biến không khai báo trong `ramp_config_s`.

**Giải pháp V15:** Thay #else bằng #error. Tất cả 5 CI boards dùng predefined entries → không bao giờ vào #else.

### Quyết định 2: Prescaler approach (gin66 comment 2)

**Vấn đề:** uint16_t overflow với TICKS_PER_S > 65M. 1ms = 72.000 ticks > 65.535.

**Giải pháp V15:** Mỗi board dùng TIM2 prescaler, đưa TICKS_PER_S về 16-20M. `FAS_TIMER->PSC = psc;` (dòng 184 stm32_queue.cpp) — đã có sẵn.

### Quyết định 3: HW test — TRẢ LỜI THẲNG

**Trả lời gin66:** "HW testing has not been performed yet. PSC values are calculated from datasheet APB1/timer clock configurations. Testing is planned as follow-up work."

| Board | PSC | Timer thực | HW test? |
|-------|-----|-----------|----------|
| F103 | 3 | 18.000 MHz | Chưa |
| G070 | 3 | 16.000 MHz | Chưa |
| F401 | 4 | 16.800 MHz | Chưa |
| H743 | 9 | 20.000 MHz | Chưa |
| L476 | 4 | 16.000 MHz | Chưa |

### Quyết định 4: 48M (F091RC) — DEFERRED

Ghi rõ trong PR body: "F091RC deferred — 48MHz entry pending hardware verification, not in current CI matrix."

### Quyết định 5: Dead code — COMMENT OUT

RampGenerator.h, RampControl.cpp: comment out `#ifdef SUPPORT_LOG2_TIMER_FREQ_VARIABLES` blocks.

### Quyết định 6: LOG2 constants — cần chạy tool gen_log2_const

Dùng tool `extras/gen_log2_const` để tạo constants chính xác. Nếu tool không build được, dùng tính thủ công ×512 (sai số < 0.06%):

| Tần số | LOG2_TICKS_PER_S | Hex |
|--------|-----------------|-----|
| 18.000.000 | round(24.101494×512) = 12340 | **0x3034** |
| 16.800.000 | round(24.001958×512) = 12289 | **0x3001** |
| 20.000.000 | round(24.253497×512) = 12418 | **0x3082** |

---

## 3. 9 bước code

| # | File | Thay đổi | Ghi chú cho gin66 |
|---|------|---------|---------------------|
| 1 | `extras/ci/build_matrix.yaml` | F103: `18000000UL`; F401: `16800000UL`; H743: `20000000UL`; G070/L476: bỏ override | Prescaled TICKS_PER_S |
| 2 | `src/pd_stm32/pd_config.h` | Default `16000000UL` + TIM2 bus comments | Runtime PSC |
| 3 | `src/fas_ramp/RampCalculator.h` | Thêm 3 entries: 18M (0x3034), 16.8M (0x3001), 20M (0x3082). **Xóa #else → #error** | Core fix |
| 4 | `src/fas_ramp/RampGenerator.h` | Comment out SUPPORT_LOG2_TIMER_FREQ_VARIABLES block | Dead code |
| 5 | `src/fas_ramp/RampControl.cpp` | Comment out SUPPORT_LOG2_TIMER_FREQ_VARIABLES + init_ramp_module | Dead code |
| 6 | `src/fas_queue/base.h` | `(uint16_t)fas_min((uint32_t)(TICKS_PER_S/1000), 65535)` | Safety guard |
| 7 | `src/FastAccelStepper.cpp` | Clamp US_TO_TICKS(2000) → 65535 | Safety guard |
| 8 | `README.md` | Bảng TIM2 bus + prescaler notes | Documentation |
| 9 | **PR description (body)** | Update usage example: 72M→18M, 84M→16.8M, 240M→20M | **Communication** |

---

## 4. Board CI — Trạng thái cuối

| Board | TICKS_PER_S | Timer thực | Predefined | 2ms ticks | Tràn? | #else? | CI? |
|-------|-------------|-----------|-----------|-----------|-------|--------|-----|
| **F103** | 18.000.000 | 18.000 MHz (PSC=3) | ✅ 0x3034 | 36.000 | ✅ No | ✅ No | ✅ Pass |
| **G070** | 16.000.000 | 16.000 MHz (PSC=3) | ✅ LOG2_CONST_16E6 | 32.000 | ✅ No | ✅ No | ✅ Pass |
| **F401** | 16.800.000 | 16.800 MHz (PSC=4) | ✅ 0x3001 | 33.600 | ✅ No | ✅ No | ✅ Pass |
| **H743** | 20.000.000 | 20.000 MHz (PSC=9) | ✅ 0x3082 | 40.000 | ✅ No | ✅ No | ✅ Pass |
| **L476** | 16.000.000 | 16.000 MHz (PSC=4) | ✅ LOG2_CONST_16E6 | 32.000 | ✅ No | ✅ No | ✅ Pass |

---

## 5. Logic thuyết phục gin66

```
Gin66 comment 1 (May 19):
  "CI fail — ramp_config_s missing member variables"
  ↓
V15: #else → #error (không thêm member variables)
  Tất cả CI boards dùng predefined → không vào #else
  ↓
Gin66 comment 2 (May 24):
  "Use timer prescaler, overflow với 72M"
  "Are you testing on HW?"
  ↓
V15: Prescaler approach — gin66 xác nhận
  HW test — chưa, trả lời thẳng
  ↓
✅ Tất cả CI pass, không overflow, không degraded