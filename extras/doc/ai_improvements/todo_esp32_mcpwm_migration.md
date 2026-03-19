# ESP32 MCPWM/PCNT Migration to ESP-IDF v5.3+

## Status: SINGLE STEPPER WORKING ON HARDWARE

## Context

The MCPWM/PCNT stepper driver (the most accurate pulse generation mode) was disabled
for ESP-IDF 5.x because the legacy direct-register APIs were completely redesigned.
Only RMT and I2S drivers are available on IDF 5.3+, losing the MCPWM/PCNT advantages
(dedicated hardware, no DMA contention, higher precision).

## Approach

**Hybrid**: New driver API for initialization + ISR registration, direct register
access in ISR for maximum speed. ESP32 only (no S3 initially).

The `soc/mcpwm_struct.h` and `soc/pcnt_struct.h` headers are still available in
IDF 5.x, so the ISR register access code from IDF4 can be reused almost verbatim.

## Critical Findings from Phase 0 (standalone test on ESP32, IDF 5.3.2)

Test project: `extras/tests/mcpwm_pcnt_test/` (deleted after integration)
Platform: `esp32_pioarduino_V53_03_11` (Arduino framework, IDF 5.3.2)

### 1. UP_DOWN mode: only DTEP and UTEZ events are reachable

The new API validates timer event actions. In `MCPWM_TIMER_COUNT_MODE_UP_DOWN`,
the direction bit toggles *before* the event fires, so:
- `UTEP` (up, peak) and `DTEZ` (down, zero) are **INVALID** — API rejects them
- `DTEP` (down, peak) and `UTEZ` (up, zero) are valid

This matches the IDF4 code which only used `gen_dtep = 1` (low at peak).

### 2. Generator actions must use direct register writes

The new API `mcpwm_generator_set_action_on_compare_event()` / `..._on_timer_event()`
requires `update_gen_action_on_tez = 1` (shadow mode). For the hybrid approach, direct
register writes to `gen_utea`/`gen_dtep` work and bypass the shadow mechanism:
```c
mcpwm->operators[timer_in_group].generator[0].val = 0;       // clear all actions
mcpwm->operators[timer_in_group].generator[0].gen_utea = 2;  // HIGH on compare (up)
mcpwm->operators[timer_in_group].generator[0].gen_dtep = 1;  // LOW at peak (down)
mcpwm->operators[timer_in_group].gen_force.val = 0;         // no force level
```

### 3. `gpio_matrix_out()` removed in Arduino ESP32 core for IDF 5.3

The function `gpio_matrix_out()` is no longer declared. Must use direct register access:
```c
GPIO.func_out_sel_cfg[pin].func_sel = PWM0_OUT0A_IDX;
GPIO.func_out_sel_cfg[pin].oen_sel = 0;
```

### 4. `gpio_matrix_in()` also removed

Same issue. For PCNT input routing, `pcnt_new_channel(edge_gpio_num=X)` handles
GPIO input internally. For manual routing, use `gpio_iomux_in()`.

### 5. `pcnt_new_channel(edge_gpio_num=X)` clobbers GPIO output routing

When `pcnt_new_channel()` is called with `edge_gpio_num = step_pin`, it internally
calls `gpio_set_direction()` which resets `func_out_sel` to `0x100` (cancel output).
**Fix**: re-route the MCPWM output AFTER PCNT channel creation:
```c
// 1. mcpwm_new_generator(.gen_gpio_num = pin)  — sets func_out_sel = PWM0_OUT0A_IDX
// 2. pcnt_new_channel(.edge_gpio_num = pin)    — clobbers func_out_sel to 0x100
// 3. Re-route AFTER pcnt setup:
GPIO.func_out_sel_cfg[pin].func_sel = PWM0_OUT0A_IDX;
GPIO.func_out_sel_cfg[pin].oen_sel = 0;
```

### 6. PCNT `low_limit` must be < 0

`pcnt_new_unit()` rejects `low_limit = 0` with `ESP_ERR_INVALID_ARG`. Must use
e.g. `low_limit = -1` or `-32768`.

### 7. PCNT unit_id extraction via private struct

The private `pcnt_unit_t` struct hack works in IDF 5.3.2:
```c
struct pcnt_unit_internal {
    void* group;
    portMUX_TYPE spinlock;
    int unit_id;
};
int unit_id = ((struct pcnt_unit_internal*)pcnt_unit)->unit_id;
```

### 8. Direct register `cnt_h_lim` update works

`PCNT.conf_unit[unit_id].conf2.cnt_h_lim = new_value` works at runtime for
dynamically changing the pulse count threshold.

### 9. `mcpwm_timer_config_t.flags` has no `allow_pd` field in IDF 5.3

Omit it.

### 10. MCPWM interrupt bits

No `MCPWM_OP(timer)` macro exists in IDF 5.3. Individual defines:
`MCPWM_OP0_TEA_INT_CLR = BIT(15)`, `MCPWM_OP1_TEA_INT_CLR = BIT(16)`, etc.
Formula: `(1 << (timer + 15))`.

### 11. Correct callback signatures for IDF 5.3

- MCPWM comparator: `bool (*mcpwm_compare_event_cb_t)(mcpwm_cmpr_handle_t,
  const mcpwm_compare_event_data_t*, void*)` — data struct has `compare_ticks`
  and `direction` fields

### 12. GPIO32/33 are RTC pins

`gpio_get_level()` doesn't work on them. GPIO14 is a strapping pin that can't use
`gpio_matrix_out` (compile error). Use GPIO27 or similar for testing.

### 13. PWM0 signal names in IDF 5.3

`PWM0_OUT0A_IDX=32`, `PWM0_OUT1A_IDX=34`, `PWM0_OUT2A_IDX=36`
Formula: `PWM0_OUT0A_IDX + timer_in_group * 2`

### 14. IDF5 PCNT watch point at 0 does NOT fire on hardware counter reset

**This was the main integration bug.** When `thr_h_lim_en` is set and the counter
reaches the high limit, the hardware resets the counter to 0. The IDF5 watch point
callback registered via `pcnt_unit_add_watch_point(unit, 0)` never fires because
the counter was *reset* to 0, not *counted* to 0.

**Fix**: Bypass the IDF5 PCNT callback mechanism entirely. Use direct ISR via
`esp_intr_alloc(ETS_PCNT_INTR_SOURCE, ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM,
pcnt_isr_handler, NULL, NULL)` and check `PCNT.int_st.val` bits 0-5 (one per unit).
This is the same approach as IDF4's `pcnt_isr_service_install()`.

### 15. `connect()` must be called at end of `init_mcpwm_pcnt()`

The `connect()` method routes the MCPWM output to the step pin AND routes PCNT
input via `gpio_iomux_in()`. Without this call, pulses are generated but never
counted (position stays at 0). The IDF4 code called `connect()` at the end of
`init_mcpwm_pcnt()` — the IDF5 port must do the same.

## Completed Phases

## Remaining TODO

### Hardware tests

- [ ] Multi-step ramp + pause commands (dedicated test)
- [ ] 6 steppers simultaneously via MCPWM
- [ ] MCPWM+RMT coexistence (mixed driver allocation)

### Hardening

- [ ] Compile-time assert or runtime range check for private `pcnt_unit_t` struct
- [ ] Verify `gpio_iomux_in()` works on non-IOMUX pins (currently only tested GPIO14)
- [ ] Test MCPWM group 1 (timers 3-5) — currently only group 0 tested
- [ ] Run existing hw test suite (`extras/tests/esp32_hw_based/`)

## Files

### Created
- `src/pd_esp32/StepperISR_idf5_esp32_mcpwm_pcnt.cpp` — IDF5 MCPWM/PCNT implementation
- `extras/doc/ai_improvements/todo_esp32_mcpwm_migration.md` — This file

### Edited
- `src/pd_esp32/pd_config_idf5.h` — Enabled MCPWM/PCNT, added headers
- `CMakeLists.txt` — Added `esp_driver_mcpwm` to REQUIRES
- `src/pd_esp32/esp32_queue.cpp` — MCPWM_PCNT allocation + DONT_CARE fallback order
- `src/FastAccelStepper_idf5_esp32_pcnt.cpp` — Removed `gpio_matrix_in()` calls

## Key API mapping (IDF4 → IDF5)

### Initialization (new API)

| IDF 4.x | IDF 5.x |
|---------|---------|
| `periph_module_enable(PERIPH_PWM0_MODULE)` | automatic via `mcpwm_new_timer()` |
| `mcpwm_gpio_init(unit, signal, pin)` | `mcpwm_new_generator(.gen_gpio_num = pin)` |
| `mcpwm_isr_register()` | `mcpwm_comparator_register_event_callbacks()` |
| `pcnt_unit_config(&cfg)` | `pcnt_new_unit()` + `pcnt_new_channel()` |
| `pcnt_isr_service_install()` | `esp_intr_alloc(ETS_PCNT_INTR_SOURCE, ...)` (direct) |
| `gpio_matrix_out(pin, sig, inv, oen_inv)` | Direct reg: `GPIO.func_out_sel_cfg[pin].func_sel = sig` |
| `gpio_matrix_in(pin, sig, inv)` | `gpio_iomux_in(pin, signal)` |

### ISR critical path (direct register access, unchanged from IDF4)

| Operation | Register (verified in IDF 5.3.2) |
|-----------|----------------------------------|
| Set timer period | `mcpwm->timer[t].timer_cfg0.timer_period = ticks` |
| Set generator action | `mcpwm->operators[t].generator[0].gen_utea = 1 or 2` |
| Set PCNT high limit | `PCNT.conf_unit[u].conf2.cnt_h_lim = steps` |
| Read PCNT count | `PCNT.cnt_unit[u].cnt_val` |
| MCPWM int enable/disable | `mcpwm->int_ena.val` direct bit manipulation |
| PCNT interrupt status | `PCNT.int_st.val` (one bit per unit) |
| PCNT interrupt clear | `PCNT.int_clr.val = mask` |
| PCNT counter clear | `REG_SET_BIT/CLR_BIT(PCNT_CTRL_REG, ...)` |

### New API for start/stop/query (task context, not ISR)

| Operation | IDF 5.x API |
|-----------|-------------|
| Start timer | `mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP)` |
| Stop timer (in ISR) | `mcpwm->timer[t].timer_cfg1.timer_start = 0` (direct reg) |
| Clear PCNT | `pcnt_unit_clear_count(unit)` |
| Read PCNT | `pcnt_unit_get_count(unit, &val)` |

## Risks

| Risk | Mitigation |
|------|-----------|
| Private `pcnt_unit_t` struct layout changed | Runtime range check on unit_id |
| `gpio_matrix_out()` unavailable in IDF native (non-Arduino) | Use direct register access (works on both) |
| `pcnt_new_channel()` clobbers GPIO routing | Re-route output AFTER PCNT channel creation (verified) |
| ISR latency from new API callback dispatch | MCPWM uses IDF5 callback (only for pause); PCNT uses direct ISR |
| `gpio_iomux_in()` may not work on all pins | Needs testing on non-IOMUX pins |
| MCPWM group 1 untested | Same code path via `mapping->mcpwm` pointer; needs hardware test |
