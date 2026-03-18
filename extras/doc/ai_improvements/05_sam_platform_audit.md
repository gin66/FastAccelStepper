# SAM Platform Audit: sam_queue.cpp Against SAM3X8E Datasheet

## Reference Sources

- SAM3X8E CMSIS headers from `arduino/ArduinoCore-sam`:
  - `pio_sam3x8e.h` — PIO peripheral B mux definitions
  - `component_tc.h` — TC register definitions
  - `component_pwm.h` — PWM register definitions
  - `instance_tc1.h` — TC1 peripheral instance
- Arduino Due variant pin table (`variant.cpp`)

## Verified Correct

| Item | Detail |
|------|--------|
| PWM prescaler | `PWM_CMR_CPRE_MCK_DIV_4` = MCK/4 = 84 MHz/4 = 21 MHz. Matches `TICKS_PER_S = 21000000` |
| PWM clock enable | `pmc_enable_periph_clk(PWM_INTERFACE_ID)` |
| PWM CH_NUM[0..7] register layout | Correct 8-channel array at offset 0x200 |
| PWM_ISR1 usage | Reads channel 0-7 counter events, masked with `0xFF` |
| Duty cycle 210 | 10 us pulse width at 21 MHz |
| PWMC_SetPeriod | Uses CPRD directly (channel disabled at that point) |
| Pin-to-channel mapping (excluding bugs below) | All pins verified against `pio_sam3x8e.h` CMSIS header |
| PIO peripheral B config | ABSR set, ODR/PDR correctly configured for peripheral mux |
| TC5 is TC1 Channel 2 | `TC1->TC_CHANNEL[2]` confirmed |
| TC5 NVIC setup | Priority 1, IRQ enabled |
| TC5 interrupt handler | Reads SR, checks CPCS, calls manageSteppers(), re-triggers |
| Channel mutual exclusion | `channelsUsed[]` + `isValidStepPin()` prevents double-use |
| `startQueue()` CPRD vs CPRDUPD | Correctly checks `PWM_SR` to choose update register |

## Bugs

### BUG 1 (Critical): Pin 72 has no PWM function

**File:** `sam_queue.cpp:213`  
**Code:** `pinToChannel(72)` returns channel 1

Pin 72 is PC30, the **RX LED** on the Arduino Due. The CMSIS header `pio_sam3x8e.h` contains no `PIO_PC30*PWM*` definition. The Arduino variant confirms `NOT_ON_PWM`. If a user passes pin 72 as `step_pin`, the code configures PC30 for PIO peripheral B, but no PWM signal exists on that pin — no step pulses will be generated.

**Documentation also affected:** `FastAccelStepperEngine.h:116` lists pin 72 as a valid step pin.

**Fix:**
- Remove `case 72:` from `pinToChannel()` in `sam_queue.cpp:213`
- Remove pin 72 from the documentation table in `FastAccelStepperEngine.h:116`

### BUG 2 (Critical): TC5 CMR register value is wrong

**File:** `sam_queue.cpp:265`  
**Code:** `TC1->TC_CHANNEL[2].TC_CMR = 0x80 | 0x40`

From `component_tc.h`:
- `0x40` = bit 6 = `TC_CMR_LDBSTOP` (Counter Clock Stopped with RB Loading)
- `0x80` = bit 7 = `TC_CMR_LDBDIS` (Counter Clock Disable with RB Loading)

These are waveform-mode bits that configure when to stop/disable the clock. The intended configuration was almost certainly:
- `TC_CMR_CPCTRG` = bit 14 = `0x4000` — auto-reset counter on RC compare
- `TC_CMR_TCCLKS_TIMER_CLOCK2` = bits [2:0] = `0x01` — MCK/8 = 84 MHz/8 = 10.5 MHz

**Current value `0xC0`:**
- Clock source: TIMER_CLOCK1 = MCK/2 = 42 MHz (bits [2:0] = 0)
- CPCTRG not set — counter does not auto-reset on RC compare (the `SWTRG` in `TC5_Handler` compensates)
- Period: 84000 / 42 MHz = 2 ms (manageSteppers called at 500 Hz)

**Intended value `0x4001`:**
- Clock source: TIMER_CLOCK2 = MCK/8 = 10.5 MHz
- CPCTRG set — counter auto-resets on RC compare
- Period: 84000 / 10.5 MHz = 8 ms (manageSteppers called at 125 Hz)

The code works because `SWTRG` in the ISR re-triggers the counter manually, and `CPCS` fires on RC compare regardless. But the 2 ms period means `manageSteppers()` runs 4x more frequently than intended, wasting CPU time.

**Fix:**
```cpp
TC1->TC_CHANNEL[2].TC_CMR = TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK2;
```

### BUG 3 (Documentation): Pin 74 should be pin 73

**File:** `FastAccelStepperEngine.h:116`  
**Code:** Lists `34/67/74/35` for channel 0

- Pin 74 = PA25 (MISO) — `NOT_ON_PWM` in Arduino variant. No PWM function on this pin.
- Pin 73 = PA21 — has `PIO_PA21B_PWML0` (PWM channel 0, left-aligned). Valid.

The code's `pinToChannel()` already uses pin 73 at line 207 — only the documentation is wrong.

**Fix:** Change `74` to `73` in `FastAccelStepperEngine.h:116`.

### BUG 4 (Documentation): Channel 2 pins missing from docs

**File:** `FastAccelStepperEngine.h:116`

The documentation lists only 7 groups (channels 0, 1, 3, 4, 5, 6, 7). Channel 2 is entirely absent. Valid channel 2 pins verified against CMSIS:
- Pin 38 → PC6 → `PIO_PC6B_PWML2`
- Pin 43 → PA20 → `PIO_PA20B_PWML2`
- Pin 63 → PB18 → `PIO_PB18B_PWML2`
- Pin 39 → PC7 → `PIO_PC7B_PWMH2`

The code in `pinToChannel()` correctly maps these pins to channel 2.

**Fix:** Add `38/43/63/39` to the documentation table for channel 2.

## Minor Issues

### MINOR 1: TC5 clock enabled before CMR write (datasheet violation)

**File:** `sam_queue.cpp:263-265`

```cpp
TC1->TC_CHANNEL[2].TC_CCR = 1;              // Enable clock (CLKEN=1)
TC1->TC_CHANNEL[2].TC_CMR = 0;              // Write CMR with CLKEN=1
TC1->TC_CHANNEL[2].TC_CMR = 0x80 | 0x40;    // Protected fields written
```

The SAM3X datasheet states that TCCLKS (bits [2:0]) and BURST (bits [5:4]) can only be written when `CLKEN = 0`. The code enables the clock first, then writes the CMR. In practice this works because no trigger has fired yet (no clock edges counted), but it is technically a violation.

**Fix:** Write CMR before setting `TC_CCR_CLKEN`, or disable the clock before reconfiguring.

### MINOR 2: PWM_DIS written as no-op

**File:** `sam_queue.cpp:302,304,306`

```cpp
PWM_INTERFACE->PWM_DIS = PWM_INTERFACE->PWM_DIS & (~mapping->channelMask);
```

`PWM_DIS` is a write-only register (writes of 1 disable the corresponding channel). Reading `PWM_DIS` returns 0 (or undefined). The expression `PWM_DIS & ~mask` always evaluates to 0, which means no channels are ever disabled by these lines.

The `connect()` function appears to intend to disable the channel before reconfiguring it. These writes are silently ineffective.

**Fix:** Replace with `PWM_INTERFACE->PWM_DIS = mapping->channelMask;` where a channel disable is intended, or remove the lines if the disable is handled elsewhere.

## Pin-to-Channel Mapping Verification Table

Cross-referenced `pinToChannel()` against `pio_sam3x8e.h` CMSIS header:

| Pin | GPIO | CMSIS PWM Mux | Channel | Code | Variant `NOT_ON_PWM` | Status |
|-----|------|---------------|---------|------|---------------------|--------|
| 34 | PC2 | `PIO_PC2B_PWML0` | 0 | ✓ | No | Correct |
| 35 | PC3 | `PIO_PC3B_PWMH0` | 0 | ✓ | No | Correct |
| 67 | PB16 | `PIO_PB16B_PWML0` | 0 | ✓ | No | Correct |
| **73** | PA21 | `PIO_PA21B_PWML0` | **0** | ✓ | No | **Doc says 74** |
| **74** | PA25 | none | — | — | **Yes** | **Bug in doc** |
| 17 | PA12 | `PIO_PA12B_PWML1` | 1 | ✓ | No | Correct |
| 36 | PC4 | `PIO_PC4B_PWML1` | 1 | ✓ | No | Correct |
| 37 | PC5 | `PIO_PC5B_PWMH1` | 1 | ✓ | No | Correct |
| 42 | PA19 | `PIO_PA19B_PWMH1` | 1 | ✓ | No | Correct |
| **72** | PC30 | **none** | — | ✓ | **Yes** | **Bug in code+doc** |
| 38 | PC6 | `PIO_PC6B_PWML2` | 2 | ✓ | No | Correct (missing from doc) |
| 39 | PC7 | `PIO_PC7B_PWMH2` | 2 | ✓ | No | Correct (missing from doc) |
| 43 | PA20 | `PIO_PA20B_PWML2` | 2 | ✓ | No | Correct (missing from doc) |
| 63 | PB18 | `PIO_PB18B_PWML2` | 2 | ✓ | No | Correct (missing from doc) |
| 40 | PC8 | `PIO_PC8B_PWMH3` | 3 | ✓ | No | Correct |
| 41 | PC9 | `PIO_PC9B_PWMH3` | 3 | ✓ | No | Correct |
| 64 | PB19 | `PIO_PB19B_PWMH3` | 3 | ✓ | No | Correct |
| 69 | PA4 | none | — | — | Yes | **Bug in doc** |
| 9 | PC21 | `PIO_PC21B_PWML4` | 4 | ✓ | No | Correct |
| 8 | PC22 | `PIO_PC22B_PWML5` | 5 | ✓ | No | Correct |
| 44 | PC19 | `PIO_PC19B_PWMH5` | 5 | ✓ | No | Correct |
| 7 | PC23 | `PIO_PC23B_PWML6` | 6 | ✓ | No | Correct |
| 45 | PC18 | `PIO_PC18B_PWMH6` | 6 | ✓ | No | Correct |
| 6 | PC24 | `PIO_PC24B_PWML7` | 7 | ✓ | No | Correct |

### Additional Bug Found During Table Verification

**BUG 5 (Critical): Pin 69 has no PWM function**

**File:** `FastAccelStepperEngine.h:116`  
Pin 69 is listed as a valid step pin (group `40/64/69/41` for channel 3).

Pin 69 is PA0, the CANTX0 pin. The CMSIS header has no `PIO_PA0*PWM*` definition. The Arduino variant confirms `NOT_ON_PWM`.

The code's `pinToChannel()` does NOT include pin 69 (correct), but the documentation claims it is valid. This is a doc-only bug that would mislead users.

**Fix:** Remove `69` from the documentation table in `FastAccelStepperEngine.h:116`.

## PWM ISR Channel Event Logic

`PWM_Handler()` reads `PWM_ISR1`, masks to 8 bits, then iterates over set bits using `__CLZ`. This correctly handles multiple channels interrupting simultaneously. The `PWM_ISR1` bits map 1:1 to channels 0-7 (CHID0..CHID7), which aligns with the `channelMask` (1 << channel). Verified correct.

## Corrected Documentation Table

The corrected pin table for `FastAccelStepperEngine.h:116`:

```
| Atmel SAM | This can be one of each group of pins:
             CH0: 34/35/67/73
             CH1: 17/36/37/42
             CH2: 38/39/43/63
             CH3: 40/41/64
             CH4: 9
             CH5: 8/44
             CH6: 7/45
             CH7: 6 |
```
