# Design Review: ESP32 I2S Output Driver (`04_esp32_i2s_driver.md`)

Reviewer perspective: embedded SW designer with focus on timing-critical DMA
data paths, ISR constraints, and bit-level I/O.

This document consolidates the review findings with owner feedback (marked
**DECISION**) and derives concrete action items for the design document.

**Reference test**: `extras/tests/pc_based/test_21.cpp` is the PC-based
test bed for the fill function. All design changes to the fill algorithm
must be reflected in test_21 updates. Current status: **does not compile**
(struct field name mismatch, see §6).

---

## 1. Critical Design Issues

### 1.1 ISR-Context `i2s_channel_write()` Will Deadlock

**Location**: §DMA Callback Flow, step 2

The ESP-IDF `i2s_channel_write()` internally acquires a mutex. The `on_sent`
callback runs in ISR context. Calling `i2s_channel_write()` from ISR will
deadlock or trigger a FreeRTOS assertion.

**DECISION**: Option A — high-priority FreeRTOS task signalled by ISR via
`xTaskNotifyFromISR()`. The CPU core to run this task on shall be
configurable at engine initialization.

**Action**: Update §DMA Callback Flow to describe the two-stage architecture:

1. ISR: updates `busy_block`, calls `xTaskNotifyFromISR()` to wake I2S task
2. I2S task (high-priority): calls `fill_i2s_buffer()` for all steppers,
   then `i2s_channel_write()` (blocking, timeout=portMAX_DELAY) to re-queue
   the filled block

Add CPU core parameter to `initI2sSingleStepper()` / `initI2sMultiStepper()`
config structs.

### 1.2 DMA Re-Queue Before Fill — Asynchronous Fill Model

**Location**: §DMA Callback Flow, step 2

**DECISION**: Filling is strictly asynchronous. The fill routine only
receives the info which block is currently being transmitted. No ordering
guarantee between fill and DMA consumption.

With the Option A architecture (§1.1), the sequence becomes:

1. ISR fires → `busy_block` updated → task notified
2. Task wakes → fills the available blocks (all blocks except the busy one)
   → calls `i2s_channel_write()` (blocking) to push filled block(s)

Since `i2s_channel_write()` is blocking with portMAX_DELAY, the task
naturally paces itself with the DMA. The filled block is submitted before
DMA needs it, as long as fill computation takes less than one block period
(500 µs). At 240 MHz ESP32 that is 120k cycles — generous for the fill
algorithm.

**Action**: Document this timing budget and the natural pacing property of
blocking `i2s_channel_write()`. Clarify that when block N becomes busy,
both block (N+1)%3 and block (N+2)%3 are available for filling.

### 1.3 Multi-Stepper Shared Buffer — Per-Stepper Bit Tracking

**Location**: §Buffer Fill Algorithm

**DECISION**: **No memset**. Each stepper must track its own generated pulses
(via the `pulse_positions` array) and individually clear them before writing
new ones. At low speeds (e.g., 1 kHz), only 1–few bytes need to be touched.

**Action**: Update the fill algorithm to specify:

1. Before filling a block, the stepper clears its own previous HIGH bits
   using the stored `pulse_positions` array from the last fill of that block.
2. Then writes new HIGH bits and records their positions.
3. The buffer is never bulk-zeroed. Each stepper is responsible only for its
   own bit slots.

This requires the `pulse_positions` array to be per-block (one array per
block in the triple buffer), so the stepper knows which positions it set
in any given block. The design document's `i2s_fill_state` must carry
`pulse_positions[I2S_BLOCK_COUNT][MAX_PULSES]` or equivalent.

---

## 2. Design Gaps

### 2.1 Bit-Level Resolution — Single vs. Multi-Stepper Distinction

**Location**: §Single-Stepper Mode, §Buffer Fill Algorithm

**DECISION**: The two modes have fundamentally different bit semantics:

- **Multi-stepper**: effective bit rate for one stepper = frame rate.
  One frame = one output sample. The bitstream `0/1/0` means one HIGH frame
  (64 ticks = 4 µs pulse). Bit-level positions within the frame carry
  different steppers' data, not time resolution.

- **Single-stepper**: effective bit rate = 2 × 16 × frame_rate = 8 Mbit/s.
  The bitstream `0/1/0` at bit level = 125 ns HIGH, which is too short for
  a step pulse. Practical minimum step pulse = multiple consecutive 1-bits.

**Action**: The fill algorithm specification must:

1. For multi-stepper: describe frame-level fill only. The stepper sets its
   bit-slot in the frame byte. One step = one frame with bit-slot HIGH.
2. For single-stepper: describe bit-span fill. A step pulse = N consecutive
   1-bits where N = pulse_width_ticks / 2 (each bit = 2 ticks = 125 ns).
   A 4 µs pulse = 32 consecutive 1-bits = one full frame of all-ones.
   Sub-frame edge placement gives positioning resolution of 2 ticks.

Add a concrete function spec: `tick_to_bit_position(tick_within_frame) →
(byte_index, bit_mask)` accounting for MSB-first L/R ordering.

### 2.2 Bit Serialization Formula Is Wrong

**Location**: §Implementation Notes, Bit Serialization Order

**DECISION**: Acknowledged, formula was AI-generated and not reviewed.

**Action**: Replace with corrected mapping:

```
Frame layout in memory (4 bytes): [L_hi, L_lo, R_hi, R_lo]
I2S outputs: L_hi[7] first ... L_lo[0], then R_hi[7] ... R_lo[0]

For multi-stepper (slot = stepper's bit position, 0..31):
  byte_in_frame = slot / 8;          // 0..3
  bit_in_byte   = 7 - (slot % 8);    // MSB first
  byte_in_buf   = frame_index * 4 + byte_in_frame;
  bit_mask      = 1 << bit_in_byte;

For single-stepper (bit_time_index = tick_pos / 2, 0..31):
  same mapping, but bit_time_index represents temporal position
  within the frame, not a stepper's slot
```

Add a worked example for both modes.

### 2.3 Driver Contract: Command Execution and Queue Pointer Advancement

**Location**: Missing from document

**DECISION**: The I2S driver does NOT feed back position to the ramp
generator. No driver does. The driver contract is:

1. Execute commands as stored in the command queue exactly as specified
2. Correctly advance the `read_idx` pointer
3. Maintain synchronicity with the µC timebase by counting cycles

The I2S clock may differ slightly from the 16 MHz command queue timebase.
An adjustment value (1–64 ticks range) may be needed to compensate for this
discrepancy.

**Action**: Add a §Driver Contract section to the design document stating:

- The driver executes commands and advances `read_idx`. Nothing else.
- Define how the 16 MHz ↔ I2S clock drift is handled: e.g., accumulate
  fractional-tick error per frame and insert/skip a tick when the error
  exceeds ±1 tick (Bresenham-style correction).
- Specify that `TICKS_PER_S = 16_000_000` and `I2S frame rate = 250_000`
  → 64 ticks/frame → exact. No drift if the I2S clock is derived from
  the same PLL. Verify this on hardware.

### 2.4 ForceStop: Selective Pulse Clearing via Stored Positions

**Location**: §Graceful Stop (partially), missing for forceStop

**DECISION**: On forceStop, all HIGH pulses belonging to that stepper must
be cleared from the buffer — even if a block is currently being transmitted.
Use the stored `pulse_positions` array to avoid scanning all 1500 frames.
Other steppers must not be affected (multi-stepper mode). In single-stepper
mode, DMA can be stopped, buffer cleared entirely (memset is acceptable for
single-stepper since only one stepper owns the buffer), and DMA restarted.

**Action**: Add §ForceStop Algorithm to the design document:

```
forceStop(stepper):
  if single-stepper mode:
    stop DMA
    memset(all blocks, 0)
    clear fill state
    restart DMA
  else (multi-stepper):
    for each block b in 0..2:
      for each pos in stepper.pulse_positions[b]:
        clear stepper's bit-slot at buf[b][pos]
      stepper.pulse_positions[b].count = 0
    clear stepper.fill_state
```

Note: clearing bits in the currently-transmitting block is a race but
acceptable — worst case, one extra pulse is output. This is inherent to
the async DMA model and consistent with the "forceStop does not guarantee
position" contract.

### 2.5 Direction/Enable Signals in Multi-Stepper Mode

**Location**: §Implementation Notes, Direction Pin Timing

**DECISION**: Direction and enable timing is NOT in scope of the pulse
driver. The I2S driver only provides a **service** to allow direction and
enable signals to be placed in the buffer by higher-level code, in the
case of multi-stepper serial demultiplexing.

**Action**: Add a §Multi-Stepper Signal Service section:

- Define service functions: `setSlotBit(block, frame, slot, value)` and
  `getSlotBit(block, frame, slot)` for DIR/ENABLE manipulation.
- The caller (stepper logic) is responsible for asserting DIR with
  sufficient setup time before STEP frames.
- Document the constraint: direction or enable must not be changed during
  frames where STEP is asserted. The service does not enforce this — caller
  responsibility.
- This interaction (DIR/ENABLE vs STEP timing in the shared buffer) is
  non-trivial and deserves its own design sub-section.

### 2.6 Pulse Width Constraints in Multi-Stepper Mode

**Location**: §Timing Constraints

In multi-stepper mode, pulse width is quantized to frame multiples (4 µs).

**DECISION**: Corrected frequency calculation: a 2-frame pulse (8 µs HIGH)
with a 2-frame pause (8 µs LOW) = 16 µs period = 62.5 kHz, not 27 kHz.

The constraint table for multi-stepper mode:

| Pulse Width | Min Period (pulse + 1 frame gap) | Max Freq |
|-------------|----------------------------------|----------|
| 1 frame (4 µs) | 2 frames (8 µs) | 125 kHz theoretical |
| 1 frame (4 µs) | 3 frames (12 µs) | 83.3 kHz |
| 2 frames (8 µs) | 4 frames (16 µs) | 62.5 kHz |

Practical max with acceptable granularity remains ~40 kHz as stated in the
design document.

**Action**: Document that in multi-stepper mode,
`setI2sPulseWidth(ticks)` rounds up to the next frame boundary and
that multi-stepper minimum pulse width is 1 frame (4 µs).

### 2.7 DMA Descriptor Configuration and IDF Driver Usage

**Location**: §Triple Buffering

**DECISION**: Preference is to use IDF drivers (espressif restricts
low-level HW access in new releases). The application queues the same
three buffers 0/1/2 repeatedly. With high-priority task + blocking
`i2s_channel_write()`, only 2 DMA descriptors are needed (the one being
transmitted and the one pre-queued). No 32 KB IDF ring buffer exists —
`i2s_channel_write()` with prefetch/DMA likely does not copy.

**Action**: Update the document:

- Set `I2S_DMA_DESC_NUM = 2` (minimum for continuous streaming)
- Set `I2S_DMA_FRAME_NUM = I2S_FRAMES_PER_BLOCK` (one block per descriptor)
- Remove the 16 × 500 DMA configuration
- Clarify that `i2s_channel_write()` pushes a block reference to the DMA
  queue without copying (verify on hardware)
- Document total memory: 3 × block_size bytes for the application buffers
  plus per-stepper fill state. No additional large allocations.

---

## 3. Logical Errors / Corrections

### 3.1 Max Pulses Per Block

**Location**: §Pulse Tracking and Dynamic Allocation

**DECISION**: 200 kHz max is guaranteed. Max frequency is ensured by
`addQueueEntry()`, not by the I2S driver.

**Action**: Size the pulse array to the exact maximum:

```c
#define I2S_BLOCK_DURATION_US  500
#define MAX_STEP_FREQ_HZ       200000
#define I2S_MAX_PULSES_PER_BLOCK  ((I2S_BLOCK_DURATION_US * MAX_STEP_FREQ_HZ / 1000000UL) + 1)
// = 101
```

Document that `addQueueEntry()` enforces the maximum frequency. The I2S
driver relies on this guarantee for array sizing.

### 3.2 Tick Pointer Width

**Location**: §Buffer Fill Algorithm, step 1

**DECISION**: Either constrain max block duration to keep `tp` within
`uint16_t`, or use `uint32_t`.

**Action**: Add design constraint:

```
With uint16_t tp:
  max total ticks = 3 × FRAMES_PER_BLOCK × 64 ≤ 65535
  → FRAMES_PER_BLOCK ≤ 341 → max block duration = 1364 µs

With uint32_t tp:
  no practical limit
```

Recommend `uint32_t tp` for flexibility. The fill routine is not in a
tight inner loop — the extra 2 bytes per stepper are negligible.

### 3.3 HIGH Phase Carry-Over Across Blocks

**Location**: §Buffer Fill Algorithm, steps 3–4

**DECISION**: Memset is wrong (§1.3). Carry-over of high ticks must place
1-bits starting at current `tp`, which can be the new block start.

**Action**: Add explicit carry-over step in the fill pseudo-code:

```
At block entry:
  1. Clear previous pulse positions for this block (from stored array)
  2. If remaining_high_ticks > 0:
       Write 1-bits from tp (= block start) for remaining_high_ticks
       Record these positions in pulse_positions array
       Advance tp by remaining_high_ticks (or to block end, whichever first)
  3. Continue with normal fill loop
```

### 3.4 Step Counting at Fill Time

**Location**: §Buffer Fill Algorithm, step 3

**DECISION**: ForceStop does not guarantee maintaining position. Counting
steps at fill time is acceptable.

**Action**: Document this explicitly: "Step count is decremented at fill
time when the pulse HIGH phase is committed to the buffer. ForceStop may
result in position error of up to (block_count - 1) × max_steps_per_block
steps."

---

## 4. Required Additions to Design Document

### 4.1 Sequence Diagram: DMA ↔ I2S Task ↔ Fill ↔ StepperTask

**DECISION**: Add this diagram.

The following diagram reflects the Option A architecture. Key invariant:
when block N is busy (being transmitted by DMA), the other two blocks are
**free** and available for filling.

```
Time →
         Block 0 plays    Block 1 plays    Block 2 plays    Block 0 plays
DMA:     ████████████████  ████████████████  ████████████████  ████████████████
                        ↑                 ↑                 ↑
ISR:                    │ on_sent         │ on_sent         │ on_sent
                        │ busy=1          │ busy=2          │ busy=0
                        │ notify task     │ notify task     │ notify task
                        ↓                 ↓                 ↓
I2S Task:          ┌──────────┐      ┌──────────┐      ┌──────────┐
                   │B1 = busy │      │B2 = busy │      │B0 = busy │
                   │fill B2,B0│      │fill B0,B1│      │fill B1,B2│
                   │write to  │      │write to  │      │write to  │
                   │DMA       │      │DMA       │      │DMA       │
                   └──────────┘      └──────────┘      └──────────┘

StepperTask:  ─── manageSteppers() ──── manageSteppers() ────
                  (adds cmds to queue)   (adds cmds to queue)

Queue:        [cmd1|cmd2|cmd3|...]  ← write_idx (StepperTask writes)
              read_idx → [cmd1|...]  ← fill advances read_idx
```

When ISR fires with busy=N:

- Block N is currently being transmitted (do not touch)
- Block (N+1)%3 and (N+2)%3 are free and can be filled
- The I2S task fills the free blocks, then submits them via
  `i2s_channel_write()` (blocking)

**Action**: Add this as a proper diagram in the design document.

### 4.2 Memory Budget

**Action**: Add corrected memory table:

| Component | Single-Stepper | Multi-Stepper (16×) |
|-----------|---------------|---------------------|
| Triple buffer (3 × 500B) | 1500 bytes | 1500 bytes (shared) |
| DMA descriptors (2×) | ~32 bytes | ~32 bytes |
| Per-stepper fill state | ~220 bytes × 1 | ~220 bytes × 16 |
| I2sManager singleton | ~50 bytes | ~50 bytes |
| **Total** | **~1800 bytes** | **~5100 bytes** |

(Fill state = tick_pos + remaining ticks + pulse_positions array per block.)

### 4.3 Error Handling

**DECISION**: Minimal error handling. DMA underrun is accepted risk.

**Action**: Add brief §Error Handling:

- `i2s_channel_write()` failure: retry once, then log warning
- DMA underrun (fill can't keep up): accepted risk, log warning
- Stepper disconnect while DMA runs: no-op for I2S driver
- `init()` after DMA started: undefined (application bug)

### 4.4 Thread Safety

**DECISION**: `read_idx` / `next_write_idx` are single bytes → atomically
safe on ESP32. Contract: `read_idx` updated only by driver (fill routine),
`next_write_idx` only by ramp generator (StepperTask).

**Action**: State this contract explicitly in the design document to confirm
no additional synchronization (mutex/spinlock) is needed for the I2S path.

### 4.5 Multi-Stepper DIR/ENABLE Service Functions

**DECISION**: The DMA buffer doesn't care what semantics the bits carry.
Only the stepper knows which slot is STEP vs DIR vs ENABLE. Direction and
enable assertion/deassertion need separate service functions. Must ensure
no direction change or disable happens during frames where STEP is asserted.

**Action**: Design these service functions. Key constraint: the service must
refuse or defer a DIR/ENABLE change if the stepper currently has STEP
asserted in the same or adjacent frames (within the dir setup time window).
This is a non-trivial interaction that deserves its own sub-section.

---

## 5. Improvement Suggestions (Accepted)

### 5.1 Verify `i2s_channel_write()` Zero-Copy Behavior

**DECISION**: Likely no copy for blocking writes. Verify on hardware.

**Action**: Create a minimal Arduino sketch that:

1. Inits I2S with 2 DMA descriptors
2. Writes the same 3 rotating buffers via `i2s_channel_write(blocking)`
3. Modifies a buffer after write returns and before the next write
4. Observes on logic analyzer whether the modification appears in the output

If the modification appears → zero-copy confirmed. If not → copy semantics,
and option B (direct DMA descriptors) should be reconsidered.

### 5.2 Unify Block Size Constants

**DECISION**: Makes sense.

**Action**: Derive all constants from `I2S_BLOCK_DURATION_US`:

```c
#define I2S_BLOCK_DURATION_US     500
#define I2S_FRAMES_PER_BLOCK      (I2S_BLOCK_DURATION_US * I2S_SAMPLE_RATE_HZ / 1000000UL)
#define I2S_BLOCK_TICKS           (I2S_FRAMES_PER_BLOCK * I2S_TICKS_PER_FRAME)
#define I2S_BYTES_PER_BLOCK       (I2S_FRAMES_PER_BLOCK * I2S_BYTES_PER_FRAME)
#define I2S_MAX_PULSES_PER_BLOCK  ((I2S_BLOCK_DURATION_US * 200000UL / 1000000UL) + 1)
```

### 5.3 8-Bit Mode as Future Extension

**DECISION**: 8-bit mode doubles time resolution for multi-stepper (32
ticks/frame instead of 64), halves max slots (16 instead of 32). Not a
design priority now but keep the architecture extensible.

**Action**: Note in §Open Items that the frame format (8-bit vs 16-bit
stereo) should be a compile-time or init-time parameter, and that the
bit-serialization mapping (§2.2) and fill algorithm must be parameterized
accordingly.

---

## 6. Test Infrastructure: `test_21.cpp`

### 6.1 Current Status: Does Not Compile

The test file references `state.pulse_count` and `state.pulse_positions`,
but the struct `i2s_fill_state` in `i2s_fill.h` declares these as
`prev_pulse_count` and `prev_pulse_positions`. The `i2s_fill.cpp`
implementation has the same mismatch. Result: 11 compilation errors.

**Action**: Resolve the naming. Recommended: rename the struct fields to
`pulse_count` and `pulse_positions` (the names used by fill code and tests)
since the "prev_" prefix is misleading — they track the *current* block's
pulse positions, not a previous block's.

With the per-block tracking decision (§1.3), the struct needs to evolve to:

```c
struct i2s_fill_state {
  uint32_t tick_pos;                // changed to uint32_t per §3.2
  uint16_t remaining_high_ticks;
  uint16_t remaining_low_ticks;
  uint16_t pulse_positions[I2S_BLOCK_COUNT][I2S_MAX_PULSES_PER_BLOCK];
  uint8_t  pulse_count[I2S_BLOCK_COUNT];
};
```

### 6.2 Tests Use `memset` — Conflicts with §1.3

All fill tests currently do `memset(buf, 0, I2S_BYTES_PER_BLOCK)` before
each fill call. This conflicts with the "no memset" decision.

**Action**: Tests must be updated to match the new fill contract:

1. Remove `memset` before fill calls
2. The fill function itself must clear previous pulse positions before
   writing new ones (using stored `pulse_positions[block]` array)
3. Add new test: verify that fill clears only its own previous pulses
   and leaves other bytes untouched (critical for multi-stepper)
4. Add new test: verify that old pulse data from a previous fill cycle
   of the same block is properly cleared

### 6.3 Tests Missing for Multi-Stepper Shared Buffer

No tests exercise multiple steppers writing to the same buffer with
different bit slots. This is the most complex and error-prone path.

**Action**: Add test cases:

- Two steppers filling same buffer, different slots — verify no interference
- ForceStop on one stepper while another continues — verify other stepper's
  bits are preserved
- Direction bit service function (when designed, §2.5 / §4.5)

### 6.4 Tests Missing for Block Carry-Over with Per-Block Tracking

The current tests fill blocks sequentially using a single buffer. With
triple buffering and per-block pulse tracking, tests must verify:

- Pulse positions from block 0 fill cycle N are correctly cleared when
  block 0 is filled again in cycle N+1
- Carry-over state (remaining_high_ticks, remaining_low_ticks) is correct
  across block boundaries without memset
- The fill function handles the case where the previous fill of this block
  had more pulses than the current fill (all old positions must be cleared)

---

## Summary of Design Document Changes Required

| # | Section to Add/Change | Priority |
|---|----------------------|----------|
| 1 | §DMA Callback Flow → two-stage ISR + high-prio task | Critical |
| 2 | §Buffer Fill → no memset, per-stepper bit tracking + clear | Critical |
| 3 | §Buffer Fill → HIGH carry-over at block boundary | Critical |
| 4 | Fix `i2s_fill.h`/`.cpp` struct names → compile test_21 | Critical |
| 5 | §Bit Serialization → corrected formula + examples | High |
| 6 | New §Driver Contract (cmd execution, read_idx, clock sync) | High |
| 7 | New §ForceStop Algorithm (single vs multi-stepper) | High |
| 8 | New §Multi-Stepper Signal Service (DIR/ENABLE functions) | High |
| 9 | §DMA config → 2 descriptors, verify zero-copy | Medium |
| 10 | §Fill Algorithm → single vs multi bit semantics | Medium |
| 11 | §Constants → derive from I2S_BLOCK_DURATION_US | Medium |
| 12 | New §Sequence Diagram (corrected block availability) | Medium |
| 13 | Update test_21 → remove memset, add multi-stepper tests | Medium |
| 14 | New §Memory Budget | Low |
| 15 | New §Error Handling | Low |
| 16 | New §Thread Safety contract | Low |
| 17 | §API → CPU core parameter in init config | Low |
