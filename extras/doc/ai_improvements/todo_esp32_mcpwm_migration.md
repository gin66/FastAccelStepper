# ESP32 MCPWM/PCNT Migration to ESP-IDF v5.3+

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

Test project: `extras/tests/mcpwm_pcnt_test/`
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
MCPWM0.operators[0].generator[0].val = 0;       // clear all actions
MCPWM0.operators[0].generator[0].gen_utea = 2;  // HIGH on compare (up)
MCPWM0.operators[0].generator[0].gen_dtep = 1;  // LOW at peak (down)
MCPWM0.operators[0].gen_force.val = 0;         // no force level
```

### 3. `gpio_matrix_out()` removed in Arduino ESP32 core for IDF 5.3

The function `gpio_matrix_out()` is no longer declared. Must use direct register access:
```c
GPIO.func_out_sel_cfg[pin].func_sel = PWM0_OUT0A_IDX;
GPIO.func_out_sel_cfg[pin].oen_sel = 0;
```

### 4. `pcnt_new_channel(edge_gpio_num=X)` clobbers GPIO output routing

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

### 5. PCNT `low_limit` must be < 0

`pcnt_new_unit()` rejects `low_limit = 0` with `ESP_ERR_INVALID_ARG`. Must use
e.g. `low_limit = -1` or `-32768`.

### 6. PCNT unit_id extraction via private struct

The private `pcnt_unit_t` struct hack works in IDF 5.3.2:
```c
struct pcnt_unit_internal {
    void* group;
    portMUX_TYPE spinlock;
    int unit_id;
};
int unit_id = ((struct pcnt_unit_internal*)pcnt_unit)->unit_id;
```

### 7. Direct register `cnt_h_lim` update works

`PCNT.conf_unit[unit_id].conf2.cnt_h_lim = new_value` works at runtime for
dynamically changing the pulse count threshold.

### 8. `pcnt_unit_config_t` has no `allow_pd` flag

The IDF 5.3 `mcpwm_timer_config_t.flags` does not include `allow_pd` (unlike later
IDF versions). Omit it.

## TODO

### Phase 0: Standalone MCPWM+PCNT test application

- [x] **Create standalone test project** `extras/tests/mcpwm_pcnt_test/`
- [x] **Test: MCPWM generates pulses on a GPIO pin** — works, 40kHz confirmed on LA
- [x] **Test: PCNT counts rising edges from the same GPIO** — works (14493 in 2s)
- [x] **Test: PCNT watch point fires at expected count** — works (fired=1, value=100)
- [x] **Test: PCNT high-limit change via direct register** — works
- [x] **Flash to connected ESP32, verify all tests pass** — 3/3 PASS

### Phase 1: Configuration changes

- [ ] Edit `pd_config_idf5.h`: Enable `SUPPORT_ESP32_MCPWM_PCNT` for `CONFIG_IDF_TARGET_ESP32`
- [ ] Edit `pd_config_idf5.h`: Set `QUEUES_MCPWM_PCNT 6`
- [ ] Edit `pd_config_idf5.h`: Uncomment `#define NEED_MCPWM_HEADERS`
- [ ] Edit `pd_config_idf5.h`: Add new MCPWM API headers under `NEED_MCPWM_HEADERS`:
  - `driver/mcpwm_timer.h`, `driver/mcpwm_oper.h`, `driver/mcpwm_cmpr.h`, `driver/mcpwm_gen.h`
  - `soc/mcpwm_struct.h`, `soc/mcpwm_reg.h` (for ISR direct register access)
- [ ] Edit `pd_config_idf5.h`: Uncomment `soc/pcnt_reg.h`, `soc/pcnt_struct.h` under `NEED_PCNT_HEADERS`
- [ ] Edit `CMakeLists.txt`: Add `esp_driver_mcpwm` to REQUIRES
- [ ] Verify `SUPPORT_SELECT_DRIVER_TYPE` auto-enables in `pd_config.h` when MCPWM+RMT queues > 0

### Phase 2: New implementation file

- [ ] Create `StepperISR_idf5_esp32_mcpwm_pcnt.cpp` guarded by
  `#if defined(SUPPORT_ESP32_MCPWM_PCNT) && (ESP_IDF_VERSION_MAJOR >= 5)`
- [ ] Define `mapping_s` struct with new API handles:
  - `mcpwm_timer_handle_t timer`, `mcpwm_oper_handle_t oper`,
    `mcpwm_cmpr_handle_t cmpr`, `mcpwm_gen_handle_t gen`
  - `pcnt_unit_handle_t pcnt_unit`
  - `uint8_t pcnt_unit_id` (for direct register access in ISR)
  - `mcpwm_dev_t* mcpwm` (register base pointer from group_id)
- [ ] Define `channel2mapping[]` static table (6 entries for ESP32, populated at runtime)

### Phase 3: Initialization

- [ ] Implement `init_mcpwm_pcnt()` — MCPWM object chain creation:
  - `mcpwm_new_timer()` with 16 MHz resolution, `MCPWM_TIMER_COUNT_MODE_UP_DOWN`
  - `mcpwm_new_operator()` + `mcpwm_operator_connect_timer()`
  - `mcpwm_new_comparator()` + `mcpwm_comparator_set_compare_value(cmpr, 1)`
  - `mcpwm_new_generator()` with `.gen_gpio_num = step_pin`
  - Direct register writes for generator actions (gen_utea, gen_dtep, gen_force)
  - Clear dt_cfg and carrier_cfg
- [ ] Implement `init_mcpwm_pcnt()` — PCNT setup:
  - `pcnt_new_unit()` + `pcnt_new_channel()` with edge_gpio_num = step_pin
  - `pcnt_channel_set_edge_action(INCREASE on rise, HOLD on fall)`
  - Private struct hack for `unit_id` extraction
  - Re-route GPIO output AFTER pcnt_new_channel (see Finding #4)
- [ ] Implement `init_mcpwm_pcnt()` — Callback registration (BEFORE enable):
  - `pcnt_unit_register_event_callbacks()` for step completion
  - `mcpwm_comparator_register_event_callbacks()` for pause completion
- [ ] Implement `init_mcpwm_pcnt()` — Enable and start

### Phase 4: ISR implementation (direct register access)

- [ ] Implement `prepare_for_next_command()`:
  - `PCNT.conf_unit[unit_id].conf2.cnt_h_lim = next_steps` (direct register, verified)
- [ ] Implement `apply_command()`:
  - Set timer period via direct reg (`timer_cfg0.timer_period = ticks`)
  - `gen_utea = 1` for pause, `gen_utea = 2` for step (direct register, verified)
  - Late-pulse compensation: read `cnt_val` twice, adjust `cnt_h_lim`
  - MCPWM compare interrupt enable/disable via `int_ena` direct register
- [ ] Implement `what_is_next()`: same logic as IDF4
- [ ] Implement `pcnt_watch_cb()`: dispatch to `what_is_next()` for step completion
- [ ] Implement `mcpwm_compare_cb()`: dispatch to `what_is_next()` for pause completion

### Phase 5: Start/Stop/Query + Queue allocation

- [ ] Implement start/stop/query functions
- [ ] Edit `esp32_queue.cpp`: Add MCPWM_PCNT case in `tryAllocateQueue()`
- [ ] Edit `esp32_queue.cpp`: Update `DONT_CARE` fallback

### Phase 6: Build and hardware tests

- [ ] Build test: compile for `esp32` target with IDF 5.3+
- [ ] Flash to connected ESP32, run basic single-stepper pulse test
- [ ] Multi-step ramp + pause commands
- [ ] 6 steppers simultaneously via MCPWM
- [ ] MCPWM+RMT coexistence (mixed driver allocation)

## Key API mapping (IDF4 → IDF5)

### Initialization (new API)

| IDF 4.x | IDF 5.x |
|---------|---------|
| `periph_module_enable(PERIPH_PWM0_MODULE)` | automatic via `mcpwm_new_timer()` |
| `mcpwm_gpio_init(unit, signal, pin)` | `mcpwm_new_generator(.gen_gpio_num = pin)` |
| `mcpwm_isr_register()` | `mcpwm_comparator_register_event_callbacks()` |
| `pcnt_unit_config(&cfg)` | `pcnt_new_unit()` + `pcnt_new_channel()` |
| `pcnt_isr_service_install()` | `pcnt_unit_register_event_callbacks()` |
| `gpio_matrix_out(pin, sig, inv, oen_inv)` | Direct reg: `GPIO.func_out_sel_cfg[pin].func_sel = sig` |

### ISR critical path (direct register access, unchanged)

| Operation | Register (verified in IDF 5.3.2) |
|-----------|----------------------------------|
| Set timer period | `mcpwm->timer[t].timer_cfg0.timer_period = ticks` |
| Set generator action | `mcpwm->operators[t].generator[0].gen_utea = 1 or 2` |
| Set PCNT high limit | `PCNT.conf_unit[u].conf2.cnt_h_lim = steps` |
| Read PCNT count | `PCNT.cnt_unit[u].cnt_val` |
| MCPWM int enable/disable | `mcpwm->int_ena.val` direct bit manipulation |

### New API for start/stop/query (task context, not ISR)

| Operation | IDF 5.x API |
|-----------|-------------|
| Start timer | `mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP)` |
| Stop timer | `mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY)` |
| Clear PCNT | `pcnt_unit_clear_count(unit)` |
| Read PCNT | `pcnt_unit_get_count(unit, &val)` |

## Risks

| Risk | Mitigation |
|------|-----------|
| Private `pcnt_unit_t` struct layout changed | Compile-time assert + runtime range check |
| `gpio_matrix_out()` unavailable in IDF native (non-Arduino) | Use direct register access (works on both) |
| `pcnt_new_channel()` clobbers GPIO routing | Re-route output AFTER PCNT channel creation (verified) |
| ISR latency from new API callback dispatch | Hybrid: only registration is new API, ISR uses direct registers |
