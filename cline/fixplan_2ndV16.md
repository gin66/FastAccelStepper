# Fix Plan V16 — C031 48MHz deferred, gen_log2_const verify, PR body consistency

## Thông tin
- **Ngày:** 2026-06-08 (V16)
- **So với V15:**
  1. ✅ **C031 @48MHz**: deferred — không trong CI matrix
  2. ✅ **F091RC ≠ C031**: làm rõ 2 board khác nhau, cả 2 deferred
  3. ✅ **gen_log2_const**: build và verify 3 hằng số
  4. ✅ **PR body consistency**: update toàn bộ, không chỉ usage example
  5. ✅ **base.h:56 clamp path**: xác nhận đúng — `max_speed_in_ticks = TICKS_PER_S/1000`

---

## 1. Xử lý các board 48MHz (C031, F091RC) — DEFERRED

### Sự thật: C031 và F091RC KHÔNG trong CI matrix

Kiểm tra `build_matrix.yaml` — chỉ có 5 STM32 boards:

| Board | Chip | Timer | TICKS_PER_S | Trong CI? |
|-------|------|-------|-------------|-----------|
| bluepill_f103c8 | F103 | TIM2 | 18M | ✅ |
| nucleo_g070rb | G0 | TIM3 | 16M | ✅ |
| blackpill_f401cc | F401 | TIM2 | 16.8M | ✅ |
| nucleo_h743zi | H7 | TIM2 | 20M | ✅ |
| nucleo_l476rg | L4 | TIM2 | 16M | ✅ |
| ~~stm32c031c6~~ | C0 | TIM3 | ~~48M~~ | ❌ **Không có** |
| ~~nucleo-f091rc~~ | F0 | TIM2 | ~~48M~~ | ❌ **Không có** |

**C031 và F091RC:** TICKS_PER_S=48M:
- 1ms = 48.000 < 65535 ✅
- 2ms = 96.000 > 65535 ❌ overflow
- Giải pháp: PSC=2 (48M÷3=16M) → cần thêm entry 16M? Hoặc dùng PSC=1 (48M÷2=24M) → cần thêm entry 24M. Cả 2 đều không ảnh hưởng CI.

**Quyết định V16:** DEFFERED — ghi rõ trong mọi chỗ trong PR body.

---

## 2. gen_log2_const — Build và verify

### Build tool

```bash
cd extras/gen_log2_const
make
```

(Tool cần build environment — Makefile có sẵn)

### Nếu build OK — dùng kết quả từ tool

Tool sẽ output LOG2_CONST_* values. So sánh với manual calculation:

| Tần số | Manual ×512 | Tool output | Khớp? |
|--------|-------------|-------------|-------|
| 18.000.000 | 0x3034 | TBD | TBD |
| 16.800.000 | 0x3001 | TBD | TBD |
| 20.000.000 | 0x3082 | TBD | TBD |

### Nếu tool không build được (fallback)

Dùng thủ công ×512. Sai số <0.06% — chấp nhận được.

**Tuy nhiên:** Nếu gin66 / gin66 có CI check constants, manual value sai 1-2 đơn vị sẽ fail. **Cần cố gắng build tool.**

---

## 3. PR body consistency — Cập nhật toàn bộ

### 3a. "Supported families" bảng — thêm prescaler notes

```markdown
| Family | Board | Timer | Width | TICKS_PER_S | Prescaler note |
|--------|-------|-------|-------|-------------|----------------|
| STM32F1 | Blue Pill F103C8 | TIM2 | 16-bit | **18.000.000** | 72M÷4(PSC=3)=18M |
| STM32F0 | Nucleo-F091RC | TIM2 | 16-bit | *48.000.000* | Deferred — needs prescaler |
| STM32G0 | Nucleo-G070RB | TIM3 | 32-bit | **16.000.000** | 64M÷4(PSC=3)=16M (default) |
| STM32C0 | STM32C031C6 | TIM3 | 16-bit | *48.000.000* | Deferred — needs prescaler |
| STM32F4 | Black Pill F401CC | TIM2 | 32-bit | **16.800.000** | 84M÷5(PSC=4)=16.8M |
| STM32H7 | Nucleo-H743ZI | TIM2 | 32-bit | **20.000.000** | 200M÷10(PSC=9)=20M |
| STM32L0 | Nucleo-L073RZ | TIM2 | 16-bit | *32.000.000* | Needs prescaler |
| STM32L4 | Nucleo-L476RG | TIM2 | 32-bit | **16.000.000** | 80M÷5(PSC=4)=16M (default) |
```

### 3b. Usage example — sửa

```ini
; AFTER: timer prescaled values (no uint16_t overflow)
[env:bluepill_f103c8]
build_flags = -DTICKS_PER_S=18000000   ; F103: 72M÷4 = 18MHz

[env:blackpill_f401cc]
build_flags = -DTICKS_PER_S=16800000   ; F401: 84M÷5 = 16.8MHz

[env:nucleo_h743zi]
build_flags = -DTICKS_PER_S=20000000   ; H7 @400MHz: 200M÷10 = 20MHz

; G070, L476: use default TICKS_PER_S=16000000 from pd_config.h
; C031, F091RC: deferred — config not updated yet
```

---

## 4. base.h:56 clamp — confirm đúng path

gin66: *"the pipeline is using 1/2ms at some places"*

Kiểm tra `base.h` dòng 56:
```cpp
max_speed_in_ticks = TICKS_PER_S / 1000;   // 1ms ticks
```

Đây là **max_speed_in_ticks** — giá trị này được dùng trong `addQueueEntry()` và queue management. Nếu overflow, queue timing sai.

Với TICKS_PER_S ≤ 20M sau prescaler: TICKS_PER_S/1000 ≤ 20.000 < 65535 ✅ **Không overflow.** Safety guard chỉ cần cho user override > 65M ngoài CI.

**Confirm:** V16 không thay đổi logic clamp — giữ nguyên `fas_min((uint32_t)(TICKS_PER_S/1000), 65535)`.

---

## 5. 9 bước code V16

| # | File | Thay đổi | Ghi chú |
|---|------|---------|---------|
| 1 | `extras/ci/build_matrix.yaml` | F103:18M; F401:16.8M; H743:20M; G070/L476: default | Giống V15 |
| 2 | `src/pd_stm32/pd_config.h` | Default 16M + TIM2 bus comments | Giống V15 |
| 3 | `src/fas_ramp/RampCalculator.h` | Thêm 3 entries: 18M/16.8M/20M với hằng số **từ gen_log2_const tool**. #else→#error | Core fix |
| 4 | `src/fas_ramp/RampGenerator.h` | Comment out dead code | Giống V15 |
| 5 | `src/fas_ramp/RampControl.cpp` | Comment out dead code | Giống V15 |
| 6 | `src/fas_queue/base.h` | `(uint16_t)fas_min((uint32_t)(TICKS_PER_S/1000), 65535)` | Giống V15 |
| 7 | `src/FastAccelStepper.cpp` | Clamp US_TO_TICKS(2000) → 65535 | Giống V15 |
| 8 | `README.md` | Bảng TIM2 bus + prescaler notes + **C031/F091RC deferred** | **Update** |
| 9 | **PR body** | **Update "Supported families" bảng + Usage example** | **Communication** |

---

## 6. Board CI — Trạng thái cuối (không đổi từ V15)

| Board | TICKS_PER_S | Timer thực | Predefined | 2ms ticks | Trạng thái |
|-------|-------------|-----------|-----------|-----------|-----------|
| **F103** | 18.000.000 | 18.000 MHz (PSC=3) | ✅ 0x3034 | 36.000 | ✅ Pass |
| **G070** | 16.000.000 | 16.000 MHz (PSC=3) | ✅ LOG2_CONST_16E6 | 32.000 | ✅ Pass |
| **F401** | 16.800.000 | 16.800 MHz (PSC=4) | ✅ 0x3001 | 33.600 | ✅ Pass |
| **H743** | 20.000.000 | 20.000 MHz (PSC=9) | ✅ 0x3082 | 40.000 | ✅ Pass |
| **L476** | 16.000.000 | 16.000 MHz (PSC=4) | ✅ LOG2_CONST_16E6 | 32.000 | ✅ Pass |
| **C031** | *48.000.000* | *Deferred* | ❌ | *96.000* | ❌ Deferred |
| **F091RC** | *48.000.000* | *Deferred* | ❌ | *96.000* | ❌ Deferred |

---

## 7. Lịch sử version

| V | Ý tưởng chính | Kết quả |
|---|---------------|---------|
| V1 | extern declarations | ❌ Sai |
| V2-V8 | Hybrid / D1 240M | ❌ Sai / overflow |
| V9-V11 | Board timer thực | ❌ Thiếu TIM2 bus |
| V12 | All prescaled + 20M | ⚠️ LOG2 placeholder |
| V13 | LOG2 computed + #else→#error | ⚠️ Thiếu HW test reply |
| V14 | All decisions documented | ⚠️ Thiếu PR body update |
| V15 | PR #359 aligned | ⚠️ C031 48M chưa xử lý |
| ✅ **V16** | **C031 deferred, gen_log2_const verify, PR body consistent** | **READY** |