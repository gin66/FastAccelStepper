# ESP-IDF 5.x MCPWM & PCNT API Migration Research

**Date**: 2026-03-19
**Purpose**: Planning the port of FastAccelStepper from legacy direct-register MCPWM/PCNT access to the new ESP-IDF 5.3+ driver API

---

## 1. Background: What Changed and When

### The Breaking Change (ESP-IDF 5.0)

The MCPWM and PCNT drivers were completely redesigned starting in **ESP-IDF 5.0**. The legacy direct-register style APIs were deprecated (not removed, but emit build warnings). The key shift:

- **Legacy API (IDF 4.x)**: User directly accessed hardware registers via `MCPWM0`, `MCPWM1`, `PCNT` structs, and manually managed ISRs via `mcpwm_isr_register()`, `pcnt_isr_register()`.
- **New API (IDF 5.0+)**: Object-oriented handle-based API. Driver manages ISR registration, power management, register locking. The old `soc/mcpwm_struct.h` and `soc/pcnt_struct.h` are still available but not recommended.

### FastAccelStepper's Current State

- **IDF 4.x** (`pd_config_idf4.h`): Uses MCPWM+PCNT with direct register access (`#include <soc/mcpwm_struct.h>`, `<soc/pcnt_struct.h>`). Up to 6 stepper queues via MCPWM/PCNT on ESP32, 4 on ESP32-S3.
- **IDF 5.x** (`pd_config_idf5.h`): MCPWM/PCNT **completely disabled** (`QUEUES_MCPWM_PCNT 0`). Only RMT is used. The PCNT headers switch to `driver/pulse_cnt.h` (new API) but register headers are commented out.
- **IDF 6.x** (`pd_config_idf6.h`): Same as IDF 5.x - MCPWM/PCNT disabled, PCNT headers also commented out.

The goal is to re-enable MCPWM/PCNT support using the new driver API.

---

## 2. New MCPWM API Architecture

### 2.1 Component Structure

The new MCPWM driver is in `esp_driver_mcpwm` component. Include path: `driver/mcpwm_prelude.h` (includes all submodules).

### 2.2 Object Hierarchy

```
MCPWM Group 0                    MCPWM Group 1
  ├── mcpwm_timer_handle_t        ├── mcpwm_timer_handle_t  (up to 3 timers/group on ESP32)
  ├── mcpwm_oper_handle_t         ├── mcpwm_oper_handle_t   (up to 3 operators/group on ESP32)
  │     ├── mcpwm_cmpr_handle_t   │     ├── mcpwm_cmpr_handle_t  (up to 2 comparators/operator)
  │     │     └── mcpwm_gen_handle_t  (up to 2 generators/operator, each → 1 GPIO)
  │     └── mcpwm_gen_handle_t
  ├── mcpwm_fault_handle_t        ├── mcpwm_fault_handle_t  (up to 3 fault inputs/group)
  └── mcpwm_sync_handle_t         └── mcpwm_sync_handle_t   (up to 3 sync inputs/group)
```

### 2.3 Key Type Definitions (from `hal/mcpwm_types.h`)

```c
typedef struct mcpwm_timer_t *mcpwm_timer_handle_t;
typedef struct mcpwm_oper_t *mcpwm_oper_handle_t;
typedef struct mcpwm_cmpr_t *mcpwm_cmpr_handle_t;
typedef struct mcpwm_gen_t *mcpwm_gen_handle_t;
typedef struct mcpwm_fault_t *mcpwm_fault_handle_t;
typedef struct mcpwm_sync_t *mcpwm_sync_handle_t;
```

### 2.4 Timer Count Modes

```c
typedef enum {
    MCPWM_TIMER_COUNT_MODE_PAUSE,   // Timer paused
    MCPWM_TIMER_COUNT_MODE_UP,      // Counting up only
    MCPWM_TIMER_COUNT_MODE_DOWN,    // Counting down only
    MCPWM_TIMER_COUNT_MODE_UP_DOWN, // Counting up AND down (mode 3 / triangular wave)
} mcpwm_timer_count_mode_t;
```

### 2.5 Generator Actions

```c
typedef enum {
    MCPWM_GEN_ACTION_KEEP,   // Keep current level
    MCPWM_GEN_ACTION_LOW,    // Force to low level
    MCPWM_GEN_ACTION_HIGH,   // Force to high level
    MCPWM_GEN_ACTION_TOGGLE, // Toggle level
} mcpwm_generator_action_t;
```

### 2.6 Timer Events

```c
typedef enum {
    MCPWM_TIMER_EVENT_EMPTY,   // Timer counts to zero
    MCPWM_TIMER_EVENT_FULL,    // Timer counts to peak
    MCPWM_TIMER_EVENT_INVALID,
} mcpwm_timer_event_t;
```

### 2.7 Timer Start/Stop Commands

```c
typedef enum {
    MCPWM_TIMER_STOP_EMPTY,       // Stop when next count reaches zero
    MCPWM_TIMER_STOP_FULL,        // Stop when next count reaches peak
    MCPWM_TIMER_START_NO_STOP,    // Start and keep running
    MCPWM_TIMER_START_STOP_EMPTY, // Start and stop when reaches zero
    MCPWM_TIMER_START_STOP_FULL,  // Start and stop when reaches peak
} mcpwm_timer_start_stop_cmd_t;
```

---

## 3. MCPWM API Usage Pattern

### 3.1 Creating and Configuring a Timer

```c
mcpwm_timer_config_t timer_config = {
    .group_id = 0,                                    // MCPWM group 0
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,           // Usually 160 MHz PLL
    .resolution_hz = 16 * 1000 * 1000,                // 16 MHz for FastAccelStepper
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,           // Up-counting for pulse generation
    .period_ticks = 1000,                              // Period in ticks
    .intr_priority = 0,                                // Default priority
    .flags = {
        .update_period_on_empty = 0,                  // Immediate update (or TEZ)
        .update_period_on_sync = 0,
        .allow_pd = 0,                                 // Don't power down
    },
};

mcpwm_timer_handle_t timer = NULL;
ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
```

### 3.2 Creating an Operator and Connecting to Timer

```c
mcpwm_operator_config_t operator_config = {
    .group_id = 0,  // Must be same group as timer
    .intr_priority = 0,
    .flags = {
        .update_gen_action_on_tez = 1,  // Update generator actions on timer empty
        .update_gen_action_on_tep = 1,  // Update generator actions on timer peak
        .update_gen_action_on_sync = 0,
        .update_dead_time_on_tez = 0,
        .update_dead_time_on_tep = 0,
        .update_dead_time_on_sync = 0,
    },
};

mcpwm_oper_handle_t oper = NULL;
ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

// Connect operator to timer
ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));
```

### 3.3 Creating a Comparator

```c
mcpwm_comparator_config_t comparator_config = {
    .intr_priority = 0,
    .flags = {
        .update_cmp_on_tez = 0,   // Update compare value immediately
        .update_cmp_on_tep = 0,
        .update_cmp_on_sync = 0,
    },
};

mcpwm_cmpr_handle_t comparator = NULL;
ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

// Set compare value at runtime
ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 500));
```

### 3.4 Creating a Generator (GPIO Output)

```c
mcpwm_generator_config_t generator_config = {
    .gen_gpio_num = GPIO_NUM_25,   // Stepper step pin
    .flags = {
        .invert_pwm = 0,
    },
};

mcpwm_gen_handle_t generator = NULL;
ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));
```

### 3.5 Setting Generator Actions (Replacing the old duty type concept)

The old API had preset duty types (`MCPWM_DUTY_MODE_0`, etc.). The new API requires explicit action setup:

```c
// Set HIGH on timer empty (start of period), going UP
ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
    generator,
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
));

// Set LOW when compare value is reached, going UP
ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
    generator,
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)
));
```

This replaces the old `mcpwm_set_duty()` and `mcpwm_set_duty_type()` calls.

### 3.6 Setting Period at Runtime

```c
// Update timer period (equivalent to mcpwm->timer[t].period.period = ticks)
ESP_ERROR_CHECK(mcpwm_timer_set_period(timer, new_period_ticks));
```

### 3.7 Force Level (Equivalent to mcpwm_set_signal_high/low)

```c
// Force output LOW (equivalent to mcpwm_set_signal_low)
ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generator, 0, true));

// Force output HIGH
ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generator, 1, true));

// Release force (resume normal PWM)
ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generator, -1, false));
```

### 3.8 Enable/Disable and Start/Stop

```c
// Enable the timer (allocates interrupt, power management lock)
ESP_ERROR_CHECK(mcpwm_timer_enable(timer));

// Start the timer
ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

// Stop at next zero
ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY));

// Disable the timer (releases resources)
ESP_ERROR_CHECK(mcpwm_timer_disable(timer));
```

---

## 4. MCPWM ISR Callbacks

### 4.1 Timer Event Callbacks

```c
typedef bool (*mcpwm_timer_event_cb_t)(
    mcpwm_timer_handle_t timer,
    const mcpwm_timer_event_data_t *edata,
    void *user_ctx
);

typedef struct {
    mcpwm_timer_event_cb_t on_full;  // Timer counts to peak
    mcpwm_timer_event_cb_t on_empty; // Timer counts to zero
    mcpwm_timer_event_cb_t on_stop;  // Timer stops
} mcpwm_timer_event_callbacks_t;

// Event data includes count value and direction:
typedef struct {
    uint32_t count_value;
    mcpwm_timer_direction_t direction;
} mcpwm_timer_event_data_t;
```

**Usage:**
```c
// MUST be called BEFORE mcpwm_timer_enable()
ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timer, &cbs, user_data));
```

### 4.2 Comparator Event Callbacks (KEY for FastAccelStepper!)

This is the replacement for the old `CMR_TEA_INT` (compare match on timer equal A) interrupt that the legacy driver used for pulse counting.

```c
typedef bool (*mcpwm_compare_event_cb_t)(
    mcpwm_cmpr_handle_t comparator,
    const mcpwm_compare_event_data_t *edata,
    void *user_ctx
);

typedef struct {
    uint32_t compare_ticks;            // The compare value
    mcpwm_timer_direction_t direction; // Count direction
} mcpwm_compare_event_data_t;

typedef struct {
    mcpwm_compare_event_cb_t on_reach; // Fires when timer counter equals compare value
} mcpwm_comparator_event_callbacks_t;

// Register callback
ESP_ERROR_CHECK(mcpwm_comparator_register_event_callbacks(comparator, &cbs, user_data));
```

**Important**: Callbacks run in ISR context. Must return `bool` indicating if a high-priority task was woken.

### 4.3 Operator Brake Event Callbacks

```c
typedef struct {
    mcpwm_brake_event_cb_t on_brake_cbc; // Cycle-by-cycle brake
    mcpwm_brake_event_cb_t on_brake_ost; // One-shot brake
} mcpwm_operator_event_callbacks_t;

ESP_ERROR_CHECK(mcpwm_operator_register_event_callbacks(oper, &cbs, user_data));
```

### 4.4 ISR Priority Constraints

Events within the same MCPWM group share a common interrupt source. When registering multiple interrupt events within the same group, the driver uses the interrupt priority of the **first registered event**. All subsequent event registrations must use the same priority.

---

## 5. New PCNT API Architecture

### 5.1 Component Structure

The new PCNT driver is in `esp_driver_pcnt` component. Include path: `driver/pulse_cnt.h`.

### 5.2 Object Model

```c
typedef struct pcnt_unit_t *pcnt_unit_handle_t;   // Opaque unit handle
typedef struct pcnt_chan_t *pcnt_channel_handle_t;  // Opaque channel handle
```

### 5.3 Creating a PCNT Unit

```c
pcnt_unit_config_t unit_config = {
    .clk_src = PCNT_CLK_SRC_DEFAULT,  // Usually APB clock
    .low_limit = -100,                 // Must be < 0
    .high_limit = 32767,               // Must be > 0
    .intr_priority = 0,
    .flags = {
        .accum_count = 1,  // Enable accumulator for overflow compensation (RECOMMENDED for stepper counting!)
    },
};

pcnt_unit_handle_t pcnt_unit = NULL;
ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
```

### 5.4 Creating a PCNT Channel

```c
pcnt_chan_config_t channel_config = {
    .edge_gpio_num = GPIO_NUM_26,    // Step signal from MCPWM generator
    .level_gpio_num = -1,            // No direction pin needed (or set to dir pin for quadrature)
    .flags = {
        .invert_edge_input = 0,
        .invert_level_input = 0,
    },
};

pcnt_channel_handle_t pcnt_channel = NULL;
ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &channel_config, &pcnt_channel));
```

### 5.5 Setting Channel Actions

```c
// Edge actions (on step signal edges)
ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
    pcnt_channel,
    PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // Rising edge: increment
    PCNT_CHANNEL_EDGE_ACTION_HOLD       // Falling edge: hold
));

// Level actions (on direction signal)
ESP_ERROR_CHECK(pcnt_channel_set_level_action(
    pcnt_channel,
    PCNT_CHANNEL_LEVEL_ACTION_INVERSE,  // High: reverse count direction
    PCNT_CHANNEL_LEVEL_ACTION_KEEP      // Low: keep current direction
));
```

### 5.6 Glitch Filter

```c
pcnt_glitch_filter_config_t filter_config = {
    .max_glitch_ns = 1000,  // Ignore pulses < 1µs
};
ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
```

### 5.7 Watch Points (Replaces the old event system)

```c
// Add watch point for zero crossing
ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, 0));
// Add watch point at high limit
ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, 32767));
// After adding watch point, clear count to make it effective
ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
```

### 5.8 Event Callbacks (ISR)

```c
typedef bool (*pcnt_watch_cb_t)(
    pcnt_unit_handle_t unit,
    const pcnt_watch_event_data_t *edata,
    void *user_ctx
);

typedef struct {
    int watch_point_value;                       // Which watch point triggered
    pcnt_unit_zero_cross_mode_t zero_cross_mode; // How zero was crossed
} pcnt_watch_event_data_t;

typedef struct {
    pcnt_watch_cb_t on_reach;  // Called when counter reaches any watch point
} pcnt_event_callbacks_t;

// MUST be called BEFORE pcnt_unit_enable()
ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, user_data));
```

### 5.9 Enable/Disable and Start/Stop

```c
ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));   // Enables interrupts, PM lock
ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));    // Start counting

ESP_ERROR_CHECK(pcnt_unit_stop(pcnt_unit));     // Stop (keeps count)
ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));  // Reset counter to zero

ESP_ERROR_CHECK(pcnt_unit_disable(pcnt_unit));  // Releases resources
```

### 5.10 ISR-Safe Functions

The following can be called from ISR context:
- `pcnt_unit_start()` / `pcnt_unit_stop()`
- `pcnt_unit_clear_count()`
- `pcnt_unit_get_count()`

### 5.11 Overflow Compensation (`accum_count`)

This is critical for stepper motor applications where pulse counts can exceed the 16-bit hardware counter:

1. Enable `accum_count` flag in `pcnt_unit_config_t`
2. Add high/low limit as watch points
3. `pcnt_unit_get_count()` now returns accumulated value including overflows

**Important**: Use as large high/low limits as possible to avoid frequent ISR triggering.

---

## 6. Register-Level Access Status

### What's Still Available

The register-level headers are **still present** in ESP-IDF 5.x and 6.x:
- `soc/mcpwm_reg.h` - Register address definitions
- `soc/mcpwm_struct.h` - Register struct access (e.g., `MCPWM0.timer[0].period.period`)
- `soc/pcnt_reg.h` - PCNT register addresses
- `soc/pcnt_struct.h` - PCNT struct access (e.g., `PCNT.conf_unit[0].conf2.cnt_h_lim`)

### What's Changed

The new driver **does not lock registers** - it uses mutex/locking internally. However:
- The legacy `driver/mcpwm.h` and `driver/pcnt.h` are still present but deprecated
- Direct register access still works but bypasses the driver's state management
- On ESP32-S3, register struct layouts changed between IDF 4.4 and later versions (see the `__ESP32_IDF_V44__` guards in the legacy code)

### Recommendation for FastAccelStepper

**Hybrid approach is possible**: Use the new driver API for initialization and ISR registration, but still access registers directly in the ISR for maximum speed. However, this is fragile:

```c
// Still possible but NOT recommended:
#include <soc/mcpwm_struct.h>
MCPWM0.timer[0].timer_cfg0.timer_period = new_period;
```

**Better approach**: Use the new API exclusively. The API functions are designed to be fast (many are IRAM-safe with `CONFIG_MCPWM_CTRL_FUNC_IN_IRAM` / `CONFIG_PCNT_CTRL_FUNC_IN_IRAM`). For the ISR critical path:
- `mcpwm_comparator_set_compare_value()` - sets compare value
- `mcpwm_timer_set_period()` - sets timer period
- `mcpwm_generator_set_force_level()` - force output high/low
- `pcnt_unit_get_count()` - read pulse count (ISR-safe)
- `pcnt_unit_clear_count()` - clear pulse count (ISR-safe)

---

## 7. Up/Down Counting Mode (MCPWM_TIMER_COUNT_MODE_UP_DOWN)

### Availability

**YES, `MCPWM_TIMER_COUNT_MODE_UP_DOWN` is fully supported** in the new API. This is the triangular wave mode where the timer counts up to peak, then back down to zero, then up again.

### Period Interpretation

From the header documentation:
> For up-down mode, the timer peak value is half of the `period_ticks`.

So if `period_ticks = 1000`, the timer counts 0→500→0→500→... The full cycle takes 1000 ticks.

### Implications for FastAccelStepper

FastAccelStepper's mode 3 (up/down counting) generates a symmetric pulse pattern:
- Timer counts up: generates rising edge at compare match
- Timer counts down: generates falling edge at compare match

In the new API, generator actions can be set per-direction:

```c
// Going UP: set HIGH on compare match
mcpwm_generator_set_action_on_compare_event(gen,
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_HIGH));
// Going DOWN: set LOW on compare match
mcpwm_generator_set_action_on_compare_event(gen,
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, comparator, MCPWM_GEN_ACTION_LOW));
```

The comparator callback `on_reach` fires in **both** directions (up and down), and `edata->direction` tells you which direction the timer was counting when the compare event occurred.

---

## 8. Critical ISR Performance Concerns for FastAccelStepper

### The Problem

FastAccelStepper's ISR must be extremely fast - it processes stepper pulse commands and updates the MCPWM timer period and PCNT limits for the next pulse. The legacy code does this by directly writing to hardware registers.

### API Function Overhead

The new driver API functions have additional overhead:
1. **Handle validation** (checking for NULL handles)
2. **Mutex/locking** for thread safety
3. **State management** (checking if timer/operator is in correct state)

### IRAM-Safe Configuration

Enable these Kconfig options for ISR-critical code:
- `CONFIG_MCPWM_ISR_IRAM_SAFE` - Puts ISR handler and related functions into IRAM
- `CONFIG_MCPWM_CTRL_FUNC_IN_IRAM` - Puts control functions into IRAM
- `CONFIG_PCNT_ISR_IRAM_SAFE` - Same for PCNT ISR
- `CONFIG_PCNT_CTRL_FUNC_IN_IRAM` - Same for PCNT control functions

### Potential Workaround: Direct Register Access in ISR

If the API overhead is too high for the ISR, a hybrid approach could work:
1. Use new API for initialization, ISR registration, and power management
2. In the ISR callback, directly write to hardware registers (still accessible via `soc/mcpwm_struct.h`)
3. The driver won't know about register changes, but this only matters if:
   - `update_period_on_empty` / `update_cmp_on_tez` flags are set (shadow registers used)
   - The driver needs to restore registers after sleep (only if `allow_pd` is set)

**Key insight**: With `flags.update_period_on_empty = 0` (immediate update), there are no shadow registers. Direct register writes in the ISR will work identically to the legacy approach.

---

## 9. ESP32 Resource Limits

| Resource | ESP32 | ESP32-S2 | ESP32-S3 | ESP32-C3 |
|----------|-------|----------|----------|----------|
| MCPWM Groups | 2 | 1 | 1 | 0 |
| Timers/Group | 3 | 3 | 3 | 0 |
| Operators/Group | 3 | 3 | 3 | 0 |
| Comparators/Oper | 2 | 2 | 2 | 0 |
| Generators/Oper | 2 | 2 | 2 | 0 |
| PCNT Units | 4 (8 with group) | 4 | 4 | 4 |
| PCNT Channels/Unit | 4 | 4 | 4 | 4 |

### Maximum Stepper Queues via MCPWM/PCNT

Each stepper needs:
- 1 MCPWM timer + 1 operator + 1 comparator + 1 generator (for PWM output)
- 1 PCNT unit (for pulse counting feedback)

**ESP32**: Up to 6 steppers (3 timers × 2 groups, limited by PCNT units = 4 per group, 8 total)
**ESP32-S3**: Up to 3 steppers (3 timers × 1 group, limited by PCNT units = 4)

---

## 10. Initialization Sequence for New API

The initialization order matters:

```
1. Create MCPWM timer(s)           mcpwm_new_timer()
2. Create MCPWM operator(s)        mcpwm_new_operator()
3. Connect timer to operator       mcpwm_operator_connect_timer()
4. Create comparator(s) on oper    mcpwm_new_comparator()
5. Create generator(s) on oper     mcpwm_new_generator()
6. Set generator actions           mcpwm_generator_set_action_on_timer_event()
                                    mcpwm_generator_set_action_on_compare_event()
7. Register ISR callbacks          mcpwm_comparator_register_event_callbacks()
8. Create PCNT unit                pcnt_new_unit()
9. Create PCNT channel(s)          pcnt_new_channel()
10. Set PCNT channel actions       pcnt_channel_set_edge_action()
11. Set PCNT glitch filter         pcnt_unit_set_glitch_filter()
12. Add PCNT watch points          pcnt_unit_add_watch_point()
13. Register PCNT callbacks        pcnt_unit_register_event_callbacks()
14. Enable MCPWM timer             mcpwm_timer_enable()
15. Enable PCNT unit               pcnt_unit_enable()
16. Clear PCNT count               pcnt_unit_clear_count()
17. Start PCNT unit                pcnt_unit_start()
18. Start MCPWM timer              mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP)
```

**CRITICAL**: Steps 7 and 13 (callback registration) MUST happen before steps 14 and 15 (enable). Otherwise `ESP_ERR_INVALID_STATE` is returned.

---

## 11. Comparison: Legacy vs New API for FastAccelStepper's ISR

| Operation | Legacy (Direct Register) | New API |
|-----------|-------------------------|---------|
| Set timer period | `mcpwm->timer[t].period.period = ticks` | `mcpwm_timer_set_period(timer, ticks)` |
| Set compare value | `mcpwm->channel[t].cmpr_value[0].a = val` | `mcpwm_comparator_set_compare_value(cmpr, val)` |
| Set generator action | `mcpwm->channel[t].generator[0].utea = 2` | `mcpwm_generator_set_action_on_compare_event(gen, ...)` |
| Read PCNT count | `PCNT.cnt_unit[u].cnt_val` | `pcnt_unit_get_count(unit, &val)` |
| Clear PCNT count | `REG_SET_BIT(PCNT_CTRL_REG, ...)` | `pcnt_unit_clear_count(unit)` |
| Set PCNT high limit | `PCNT.conf_unit[u].conf2.cnt_h_lim = steps` | NOT in new PCNT API (see note) |
| Enable MCPWM interrupt | `mcpwm->int_ena.val \|= ...` | Automatic via callback registration |
| Clear MCPWM interrupt | `mcpwm->int_clr.val = ...` | Automatic via driver ISR handler |

### Missing: PCNT High Limit Dynamic Update

**This is a significant gap for FastAccelStepper.** The legacy code dynamically updates `PCNT.conf_unit[u].conf2.cnt_h_lim` in the ISR to set the number of pulses for the next command. The new PCNT API does NOT expose a function to change the high/low limits at runtime - they are only set during unit creation via `pcnt_unit_config_t::high_limit` / `low_limit`.

**Possible workarounds:**
1. **Direct register access**: Still access `PCNT.conf_unit[u].conf2.cnt_h_lim_un` directly in ISR (register structs may have changed name on newer chips)
2. **Use watch_step**: The `pcnt_unit_add_watch_step()` function can set a step interval notification - this may serve a similar purpose
3. **Use watch points**: Add/remove watch points dynamically (but this has more overhead)

---

## 12. Key Gotchas and Limitations

1. **Callback registration before enable**: ISR callbacks MUST be registered before `mcpwm_timer_enable()` / `pcnt_unit_enable()`. You cannot add callbacks to an already-enabled peripheral.

2. **Shared interrupt per group**: All MCPWM events within a group share one interrupt. The priority of the first registered callback is used for the group.

3. **Prescale conflicts**: The group prescaler is set by the first timer's resolution. If you allocate timers with different resolutions in the same group, you may get conflicts. Allocate timers in order of resolution (highest first or lowest first).

4. **PCNT limit cannot be changed at runtime**: The new PCNT API does not expose a function to change `high_limit` / `low_limit` after creation. This is a blocker for FastAccelStepper's pattern of dynamically setting pulse count.

5. **Dead time constraint**: You can't set rising edge delay for both generator 0 and 1 at the same time (hardware limitation).

6. **Generator update timing**: If `update_gen_action_on_tez` is set, generator action changes only take effect when timer counts to zero. Set to 0 for immediate updates.

7. **Compare value update timing**: Similarly, `update_cmp_on_tez`/`update_cmp_on_tep` control when compare value changes take effect.

8. **IRAM placement**: For real-time stepper control, enable `CONFIG_MCPWM_ISR_IRAM_SAFE` and `CONFIG_MCPWM_CTRL_FUNC_IN_IRAM` to ensure ISR runs even during flash operations.

9. **Power management**: The driver acquires PM locks automatically. If using direct register access alongside the new API, be aware that the driver may power down the peripheral.

10. **ESP32-S3 PCNT register names changed**: The legacy code already handles this with `HAVE_ESP32S3_PULSE_COUNTER` guards. The new API abstracts this away.

11. **No mode 3 (up/down) for basic pulse counting**: While `MCPWM_TIMER_COUNT_MODE_UP_DOWN` exists, for FastAccelStepper's simple pulse generation mode (mode 0 - up counting only), the simpler `MCPWM_TIMER_COUNT_MODE_UP` is appropriate.

---

## 13. Recommended Porting Strategy

### Option A: Pure New API (Recommended)

1. Create wrapper functions that mirror the legacy ISR operations
2. Use `mcpwm_timer_set_period()` and `mcpwm_comparator_set_compare_value()` in ISR
3. For the PCNT high-limit problem, investigate `pcnt_unit_add_watch_step()` or accept direct register access just for this one operation
4. Use `mcpwm_generator_set_force_level()` for the "stop output" case (equivalent to old `utea = 1`)

### Option B: Hybrid (New API init + Direct Register ISR)

1. Use new API for all initialization, ISR registration, GPIO setup
2. In ISR callbacks, directly access MCPWM and PCNT registers for maximum speed
3. Keep `flags.update_period_on_empty = 0` and `flags.update_cmp_on_tez = 0` to avoid shadow register conflicts
4. This is closest to the legacy approach but uses the new API for all the "setup" code

### Option C: RMT-Only (Current IDF 5.x/6.x approach)

The current approach already works - use only RMT. MCPWM/PCNT is disabled. This works but loses the MCPWM/PCNT advantages (dedicated hardware, no DMA contention).

---

## 14. API Reference Links

- MCPWM docs: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html
- PCNT docs: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html
- Migration guide (4.4→5.0): https://docs.espressif.com/projects/esp-idf/en/latest/esp32/migration-guides/release-5.x/5.0/peripherals.html
- MCPWM headers: `components/esp_driver_mcpwm/include/driver/mcpwm_*.h`
- PCNT headers: `components/esp_driver_pcnt/include/driver/pulse_cnt.h`
- MCPWM types: `components/hal/include/hal/mcpwm_types.h`
