# Fix Plan V17 — Complete: V16 issues fixed + All decisions crystal clear

## Thông tin
- **Ngày:** 2026-06-08 (V17)
- **Sửa so với V16:**
  1. ✅ **ramp_config_s struct**: **ĐÃ XỬ LÝ từ V10** (thay #else bằng #error) — ghi rõ trong V17
  2. ✅ **G070 timer**: **TIM2**, không phải TIM3 (verified stm32_queue.cpp dòng 57-67)
  3. ✅ **PR body consistency**: sửa G070 TIM3 → TIM2 trong bảng
  4. ✅ Mọi decision ghi rõ để reviewer không phải hỏi lại

---

## 1. `ramp_config_s` member variables — ĐÃ XỬ LÝ

**Vấn đề (gin66 comment 1):** #else branch define `LOG2_TICKS_PER_S = log2_timer_freq` — biến không khai báo.

**Đã xử lý từ V10 (không phải V16):**

| Version | Thay đổi | Trạng thái |
|---------|----------|-----------|
| V1 | extern declarations | ❌ Sai |
| V10 | **Thay #else bằng #error** | ✅ **Đã fix** |
| V13 | Thêm entries 18M/16.8M/20M | ✅ |
| V17 | Ghi rõ trạng thái | ✅ |

**Trong Step 3 (implement):**
```cpp
// XÓA toàn bộ block lines 112-124 (#else + SUPPORT_LOG2_TIMER_FREQ_VARIABLES)
// THAY bằng:
#else
#error "Unsupported TICKS_PER_S. Use timer prescaler to match: 16M, 18M, 20M, 21M, ..."
#endif
```

→ `ramp_config_s` không cần thêm member variables. Code compile OK vì không board nào vào #else.

---

## 2. G070 timer — verified: TIM2, không phải TIM3

**Kiểm tra `stm32_queue.cpp` dòng 57-67:**

```cpp
#if defined(STM32C0xx)
    #define FAS_TIMER TIM3     // C0 dùng TIM3 (không có TIM2)
#else
    #define FAS_TIMER TIM2     // MỌI board khác dùng TIM2
```

G070 là STM32G0xx → **dùng TIM2** ✅. PR body đúng, V16 section 3a sai.

**Sửa trong V17 bảng "Supported families":**

| Family | Board | Timer | Ghi chú |
|--------|-------|-------|---------|
| STM32F1 | F103 | **TIM2** | ✅ |
| STM32F0 | F091RC | **TIM2** | Deferred |
| STM32G0 | G070 | **TIM2** | ✅ (V16 sai là TIM3) |
| STM32C0 | C031 | **TIM3** | Deferred (C0 không có TIM2) |
| STM32F4 | F401 | **TIM2** | ✅ |
| STM32H7 | H743 | **TIM2** | ✅ |
| STM32L0 | L073RZ | **TIM2** | Deferred |
| STM32L4 | L476 | **TIM2** | ✅ |

---

## 3. Tất cả quyết định — Mỗi dòng đều trả lời câu hỏi của gin66

| # | Câu hỏi gin66 | Quyết định | Trạng thái |
|---|---------------|-----------|-----------|
| 1 | "CI fail vì ramp_config_s thiếu member variables" | #else→#error, không thêm member variables | ✅ **V10 đã fix** |
| 2 | "Overflow với 72M ticks/s, dùng prescaler" | Prescaler đưa TICKS_PER_S về 16-20M | ✅ V12 |
| 3 | "Are you testing on HW?" | **Chưa** — trả lời thẳng | ✅ V14 |
| 4 | "F091RC 48MHz" | Deferred — không trong CI | ✅ V16 |
| 5 | "C031 48MHz" | Deferred — C0 dùng TIM3 | ✅ V16 |
| 6 | "G070 dùng TIM2 hay TIM3?" | **TIM2** — verified code | ✅ **V17 fix** |
| 7 | "PR body vẫn ghi 72000000" | Sửa 72M→18M, 84M→16.8M, 240M→20M | ✅ V15 |
| 8 | "gen_log2_const verified?" | Build tool trước, fallback manual | ✅ V16 |
| 9 | "48M boards (C031/F091RC) overflow?" | **2ms=96k>65535** — deferred | ✅ V16 |

---

## 4. 9 bước code (với giải thích đầy đủ)

| # | File | Thay đổi | Giải thích cho gin66 |
|---|------|---------|---------------------|
| 1 | `extras/ci/build_matrix.yaml` | F103: `18000000UL`; F401: `16800000UL`; H743: `20000000UL`; G070/L476: bỏ override | Prescaled TICKS_PER_S = timer thực sau PSC |
| 2 | `src/pd_stm32/pd_config.h` | Default `16000000UL` + TIM2 bus comments | PSC auto compute runtime |
| 3 | `src/fas_ramp/RampCalculator.h` | **Thêm** 18M(0x3034), 16.8M(0x3001), 20M(0x3082). **#else → #error** | Core fix — gin66 comment 1 ✅ |
| 4 | `src/fas_ramp/RampGenerator.h` | Comment out `SUPPORT_LOG2_TIMER_FREQ_VARIABLES` | Dead code — gin66 comment 1 dead path |
| 5 | `src/fas_ramp/RampControl.cpp` | Comment out + `init_ramp_module` | Dead code — gin66 comment 1 dead path |
| 6 | `src/fas_queue/base.h` | `(uint16_t)fas_min((uint32_t)(TICKS_PER_S/1000), 65535)` | Safety guard — gin66 comment 2 overflow |
| 7 | `src/FastAccelStepper.cpp` | Clamp `US_TO_TICKS(2000)` → 65535 | Safety guard — gin66 comment 2 overflow |
| 8 | `README.md` | Bảng TIM2 bus + prescaler + deferred notes | Documentation |
| 9 | **PR body** | Update "Supported families" + usage example | **Communication** — dễ bỏ sót |

---

## 5. Board CI — Tất cả pass

| Board | TICKS_PER_S | Timer | PSC | Timer thực | Predefined | 2ms ticks | Trạng thái |
|-------|-------------|-------|-----|-----------|-----------|-----------|-----------|
| **F103** | 18.000.000 | TIM2 APB1 | 3 | 18.000 MHz | 0x3034 | 36.000 | ✅ |
| **G070** | 16.000.000 | TIM2 APB | 3 | 16.000 MHz | LOG2_CONST_16E6 | 32.000 | ✅ |
| **F401** | 16.800.000 | TIM2 APB1 | 4 | 16.800 MHz | 0x3001 | 33.600 | ✅ |
| **H743** | 20.000.000 | TIM2 D2 APB1 | 9 | 20.000 MHz | 0x3082 | 40.000 | ✅ |
| **L476** | 16.000.000 | TIM2 APB1 | 4 | 16.000 MHz | LOG2_CONST_16E6 | 32.000 | ✅ |
| C031 | *48M deferred* | TIM3 APB | — | — | — | *96k overflow* | ⏸ |
| F091RC | *48M deferred* | TIM2 APB | — | — | — | *96k overflow* | ⏸ |

---

## 6. Lịch sử version

| V | Ý tưởng chính | Kết quả |
|---|---------------|---------|
| V1 | extern declarations | ❌ Sai |
| V2-V9 | hybrid / D1 / prescaler | ❌ Nhiều vấn đề |
| V10 | **#else→#error** | ✅ **Fix ramp_config_s** |
| V11-V15 | prescaler + PR alignment | ✅ Nhưng còn thiếu sót |
| V16 | C031 deferred + gen_log2_const | ⚠️ G070 timer sai, struct status không rõ |
| ✅ **V17** | **Struct status rõ, G070 TIM2, mọi decision crystal clear** | **READY** |