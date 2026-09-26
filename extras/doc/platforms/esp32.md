# ESP32 Platform

[Back to README](../../../README.md) | [Platform index](../hardware.md) | [I2S driver](../esp32_i2s_driver.md)

Supported variants: esp32, esp32s2, esp32s3, esp32c3, esp32c6, esp32p4.

## Arduino core compatibility

Arduino core v3.0.x are using esp-idf v5.0 up to v5.1 and FastAccelStepper will
fail to compile.

Arduino core 3.1.0 will support ESP-IDF V5.3.0 (based on RC1).

## Variants overview

| Variant     | IDF 4.x Driver     | IDF 4.x Max | IDF 5.3+ Driver            | IDF 5.3+ Max               | I2S Ctrl | I2S Mux | I2S Direct |
|-------------|--------------------|-------------|---------------------------|----------------------------|----------|---------|------------|
| ESP32       | MCPWM/PCNT + RMT   | 14 (6+8)    | MCPWM/PCNT + RMT + I2S    | 14 (6+8) + 32              | 2        | 32 pins | 2 ch       |
| ESP32-S2    | RMT                | 4           | RMT + I2S                 | 4 + 32                     | 2        | 32 pins | 2 ch       |
| ESP32-S3    | MCPWM/PCNT + RMT   | 8           | MCPWM/PCNT + RMT + I2S    | 4 (4) + 32                  | 2        | 32 pins | 2 ch       |
| ESP32-C3    | RMT                | 2           | RMT + I2S                 | 2 + 32                     | 1        | 32 pins | 1 ch       |
| ESP32-C6    | -                  | -           | MCPWM/PCNT + RMT + I2S    | 2 (2) + 32                  | 1        | 32 pins | 1 ch       |
| ESP32-P4    | -                  | -           | RMT + I2S                 | 4 + 32                     | 3        | 32 pins | 3 ch       |

**Notes:**

- IDF 5.3+ Max = MCPWM/PCNT + RMT channels + I2S Mux slots (can be combined)
- MCPWM/PCNT is available on IDF 5.3+ and IDF 6.1 for ESP32, ESP32-S3, ESP32-C6, ESP32-H2
  - ESP32-S2 has PCNT but no MCPWM, ESP32-C3 has neither, so only RMT/I2S are available there
  - ESP32-P4 has both (2 MCPWM groups, 4 PCNT units in IDF 5.3+), but MCPWM/PCNT is not implemented for P4 yet
- I2S Mux requires ESP-IDF >=5.3 and uses one I2S controller
- Step rate: 200 kHz applies to MCPWM/PCNT and RMT. I2S Mux is limited to 40 kHz, I2S Direct reaches up to 200 kHz
- I2S Mux slots are shared: if step/dir/enable all use I2S Mux, each stepper consumes 1-3 slots
  - Step only: up to 32 steppers
  - Step + Dir: up to 16 steppers
  - Step + Dir + Enable: up to 10 steppers
- I2S Direct channels = number of I2S controllers (experimental)
- C6 and P4 require IDF >=5.3 (no IDF 4.x support)

## Usage limits

### ESP-IDF version 4.x.y

* allows up to 200000 generated steps per second
* supports up to 14 stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Steppers' command queue depth: 32

### ESP-IDF version >=5.3.0

* allows up to 200000 generated steps per second for RMT. 40kHz for I2S-MUX.
* supports up to 8+32 stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Steppers' command queue depth: 32

## ESP32 I2S Mux Driver (ESP-IDF >=5.3.0 only)

The I2S Mux driver provides an alternative approach for driving multiple stepper
motors using the ESP32's I2S peripheral. This is especially useful when you need
more steppers than RMT channels provide.

**Overview:**
The I2S transmitter outputs a 32-bit data word at 250kHz. Each bit corresponds to
one output slot (0-31). These signals can be used in two ways:

1. **With external demultiplexer**: Connect a decoder IC (e.g., 74HC154, 74HC138
   cascade, or shift registers like 74HC595) to the I2S data pin to decode the
   32-bit stream into individual output pins for step, direction, and enable
   signals.

2. **Direct connection**: Use individual I2S output slots directly as stepper
   driver inputs. Each slot provides one output signal that toggles at the I2S
   frame rate.

**Key Features:**

* Up to 32 output pins from a single I2S transmitter
* I2S runs at 250kHz sample rate (4µs frame time)
* Step, direction, and enable pins can all use I2S outputs
* Suitable for applications requiring many coordinated steppers

**Limitations:**

* Requires ESP-IDF 5.3 or later (I2S driver API changed significantly)
* Minimum speed for I2S Mux is 25µs period (40kHz max step rate)
* Minimum speed for I2S Direct is 5µs period (200kHz max step rate)
* Step pulse width is 2.5us for I2S Direct and 4us for I2S MUX
* I2S Direct: stepper speed adjustable in 1/8us deltas.
* I2S MUX: stepper speed adjustable in 4us deltas e.g. speed 50us will be 52/48/52/48...
* `forceStop()` will still emit commands already in the DMA buffer (up to ~500µs latency)
* Direction change: GPIO DIR needs ~1 ms of pause before the change
  (`2*I2S_BLOCK_TICKS`, both DMA blocks without steps). Mux-slot DIR
  (`PIN_I2S_FLAG`) needs 500 µs after the change (mask applies to the next
  block). Not inserted if the caller already queued enough pause.

**I2S Direct vs I2S Mux:**

* **I2S Mux**: Single I2S transmitter drives up to 32 pins. All pins share the same timing.
* **I2S Direct**: Each stepper gets its own I2S channel. Higher precision timing.

**Initialization:**

```cpp
// Initialize I2S Mux with data, bclk, and word select pins
// Not needed, if I2S direct mode is used
engine.initI2sMux(data_pin, bclk_pin, ws_pin);

// Connect stepper to I2S mux slot (0-31)
// PIN_I2S_FLAG automatically selects I2S Mux mode
stepper = engine.stepperConnectToPin(slot | PIN_I2S_FLAG);

// Direction and enable can also use I2S mux slots
stepper->setDirectionPin(dir_slot | PIN_I2S_FLAG, dirHighCountsUp);
stepper->setEnablePin(enable_slot | PIN_I2S_FLAG, activeLow);
```

**Pin Allocation:**
The I2S mux uses slot numbers 0-31, which are output on the I2S data line. These
can be used in two ways:

* **With external demultiplexer**: Connect a decoder (e.g., 74HC154, 74HC138 cascade)
  to the I2S data pin to decode the 32-bit frame into 32 individual output pins
  for step/direction/enable signals
* **Direct connection**: Use the I2S data pin directly as a step signal

**Testing and Demo Application:**
A comprehensive platformio test application with web interface is available in
`extras/Esp32StepperDemo/`. This demonstrates:

* Configuration of multiple steppers via Web UI
* Real-time position and status monitoring via WebSocket
* Each I2S slot can be toggled individually
* I2S mux pin management
* serial console 115200 asks at startup for Wifi credentials.
* Sequence automation (not tested)

To build and test:

```bash
cd extras/Esp32StepperDemo
pio run -t upload && pio device monitor
```

This test application was AI crafted....

## Implementation

### ESP-IDF version 4.x.y

This stepper driver uses mcpwm modules of the esp32: for the first three stepper
motors mcpwm0, and mcpwm1 for the steppers four to six. In addition, the pulse
counter module is used starting from `unit_0` to `unit_5`. This driver uses the
`pcnt_isr_service`, so unallocated modules can still be used by the application.
The mcpwm modules' outputs are fed into the pulse counter by direct
gpio-matrix-modification.

For the other stepper motors, the rmt module comes into use.

### ESP-IDF version >=5.3.0

RMT, I2S Mux/Direct, and MCPWM/PCNT (ESP32, ESP32-S3, ESP32-C6, ESP32-H2)
drivers are supported.

MCPWM/PCNT works on both ESP-IDF 5.3+ and ESP-IDF 6.1; the ESP-IDF 6.1 API uses
a dedicated ISR (`StepperISR_idf6_esp32_mcpwm_pcnt.cpp`) and the split
`mcpwm_timer/oper/cmpr/gen` plus `hal/mcpwm_ll` and `pulse_cnt` headers. All
three driver families (MCPWM/PCNT, RMT and I2S) are available and tested on
ESP32/ESP32-S3 with ESP-IDF 6.1.

### I2S Mux driver implementation

The I2S Mux driver uses the ESP32's I2S transmitter in 16-bit stereo mode at
250kHz sample rate. Each I2S frame (32 bits = 16-bit left + 16-bit right
channel) represents one time slot, with each bit corresponding to one output
pin.

**Technical Details:**

* Sample rate: 250kHz, giving 4µs per frame (64 ticks at 16MHz reference)
* 32 output slots per frame, each slot is one bit
* Bits are transmitted MSB-first within each byte
* DMA buffers are filled in the I2S TX-done callback
* Direction and enable pins are stored in a 32-bit state word, written to each frame's data

**Timing Considerations:**

* Minimum step period: 25µs for I2S Mux, 5µs for I2S Direct
* Direction/enable changes have up to 4µs latency
* All steppers sharing I2S mux are inherently synchronized (same DMA buffer)

See [ESP32 I2S Output Driver](../esp32_i2s_driver.md) for the full design.

### Both ESP-IDF versions

A note to `MIN_CMD_TICKS` using mcpwm/pcnt: The current implementation uses one
interrupt per command in the command queue. This is much less interrupt rate
than for avr. Nevertheless at 200kSteps/s the switch from one command to the
next one should be ideally serviced before the next step. This means within 5us.
As this cannot be guaranteed, the driver remedies an overrun (at least by design)
to deduct the overrun pulses from the next command. The overrun pulses will then
be run at the former command's tick rate. For real life stepper application, this
should be ok. To be considered for raw access: Do not run many steps at high rate
e.g. 200kSteps/s followed by a pause.

What are the differences between mcpwm/pcnt, rmt, and i2s mux?

|                            | mcpwm/pcnt                              | rmt                                                                           | i2s mux                         |
|:---------------------------|:----------------------------------------|:------------------------------------------------------------------------------|:--------------------------------|
|Interrupt rate/stepper      | one interrupt per command               | min: one interrupt per command, max: one interrupt per 31 steps at high speed | one interrupt per 500µs (all)   |
|Required interrupt response | at high speed: time between two steps   | at high speed: time between 31 steps                                          | 500µs for all steppers combined |
|Module usage                | 1 or 2 mcpcms, up to 6 channels of pcnt | rmt                                                                           | 1 i2s                           |
|Max steppers                | 6 + 8 rmt                               | 8 (ESP32), 4 (ESP32S3)                                                        | 32 (I2S slots) + rmt            |
|esp32 notes                 | available pcnt modules can be connected | no pcnt module used, so can be attached to rmt output as realtime position    | synchronized outputs            |
|Min step period             | ~5µs                                    | ~5µs                                                                          | ~25µs                           |

If the interrupt load is not an issue, then rmt is the better choice. With rmt
the multi-axis mention of loss of synchonicity at high speeds can be avoided. The
rmt driver is - besides some rmt modules perks - less complex and way more
straightforward.

As of now, allocation of steppers on esp32 are: first all 6 mcpwm/pcnt drivers
and then the 8 rmt drivers. In future this may be under application control.
Starting with 0.29.2, the module can be directly selected on call of
`stepperConnectToPin()`. So the allocation gets more flexible.

One specific note for the rmt: If a direction pin toggle is needed directly after
a command with steps, then the driver will add before that direction pin toggle
another pause of `MIN_CMD_TICKS` ticks.

MCPWM/PCNT inserts one `MIN_CMD_TICKS` pause (old DIR) before a direction change.
`apply_command()` (and DIR) runs at MCPWM compare tick 1 (TEA), when STEP has
just gone high and stays high until TEP. The pause defers the DIR toggle until
STEP is low (#370). Independent of `dir_change_delay_us`. ESP-IDF 5+
additionally latches generator actions at TEZ/TEP; `startQueue` applies the first
command immediately so leftover step action cannot pulse during a leading pause.

### ESP32S2

This stepper driver uses rmt module only.

### ESP32S3

The ESP32S3's rmt module is similar to esp32c3 with 4 instead of 2 channels and
with different register names.

* **ESP-IDF version 4.x.y:** This stepper driver uses mcpwm/pcnt + rmt modules.
  Can drive up to 8 motors. Tested with 6 motors (not by me).
* **ESP-IDF version >=5.3.0:** This stepper driver uses rmt modules. Can drive up
  to 4 motors. MCPWM/PCNT is available on ESP32-S3 with IDF 5.3+ (4 steppers).
  I2S Mux driver is also available with same capabilities as ESP32 (see
  [ESP32 I2S Mux Driver](#esp32-i2s-mux-driver-esp-idf-530-only) above).

### ESP32C3

This stepper driver uses rmt module and can drive up to 2 motors. Not thoroughly
tested, so only experimental support.

### ESP32P4

This stepper driver uses rmt module and can drive up to 4 motors. Not thoroughly
tested, so only experimental support.

### ESP32-MINI-1

Compatibility with ESP32-MINI-1: At least mcpwm and pulse counter modules are
listed in the datasheet. So there are chances, that this lib works.
