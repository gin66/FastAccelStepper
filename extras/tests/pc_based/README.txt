Tests;

- test_01
  check queue functionality

- test_02
  checks ramp timing

- log2_test
  checks Log2Representation implementation

- test_04
  one test case with speed change during ramp

- test_05
  check for move/moveTo while ramp is processing
  Introduce concept of interrupt generation during noInterrupts call

- test_06
  check for stop during move

- test_07
  test case with varying speed

- test_08
  ramp to max speed with step wise increased length

- test_09
  simple test case for V30 A1000000 R53 W R53

- test_10
  test case for V30 a17164 w2000 a-1000

- test 11
  test case for M1 A1000 V10000 f w300 V100000 U
  This is stuck in state RED.
  Revised test: M1 A1000 V10000 P100 w300 V100000 U

- test 12
  ramp up with 1 step/s^2 to 1000us/step

- test 13
  tests with maximum high acceleration

- test 14
  test case for issue #178: Speed jump instead of decrease

- test 15
  test case for setForwardPlanningTimeInMs()

- test 16
  tests for moveTimed()

- test 17
  test to specifically test for esp32 rmt implementation

- test 18
  RMT fill buffer test with comprehensive validation
  Tests rmt_fill_buffer() with various command patterns and PART_SIZE values
  
  Part 1: Fixed Command Inputs
  - Single step at different tick values (16000, 32000, 65535, 8000)
  - Multi-step commands (10@1000, 20@500, 10@8000)
  - Pause and step combinations
  - Mixed command sequences
  - Ramp generator outputs (from ramp_helper)
  Validates: step count, total ticks, high/low ratio, min symbol period
  
  Part 2: Step Count Limits (0-255 steps)
  - Tests 0-255 steps at MIN_CMD_TICKS (3200 ticks = 5 kHz)
  - Tests 0-255 steps at 16000 ticks (1000 us = 1 kHz)
  - Tests 0-255 steps at 32000 ticks (2000 us = 500 Hz)
  - Tests 0-255 steps at 65535 ticks (4095 us = ~244 Hz, special case)
  
  Part 3: High/Low Ratio Check
  - Validates 50% duty cycle for step commands
  - Tests tick values: 1000, 5000, 8000, 10000, 16000, 32000, 65535
  - Ratio must be between 30-200% for valid step generation
  
  Part 4: Reported Hardware Failure Analysis
  - Tests cases reported to fail on real hardware:
    * v=1000us (16000 ticks) => no pulses on real hardware
    * v=2000us (32000 ticks) => no pulses on real hardware  
    * v=500us (8000 ticks) => pulses but incorrect pattern (2:1 high:low ratio)
  - Compares simulation results with hardware reports
  
  Part 5: Additional Validation
  - Steps 0-255 with MIN_CMD_TICKS (3200 ticks) - comprehensive test
  - Steps 0-255 with period 65535 ticks - edge case validation
  - 200 kHz (80 ticks) with steps 1-255 - max frequency test
    Note: 80 ticks may be below stretching threshold for some PART_SIZE values
  
  Test runs with multiple even PART_SIZE values: 22, 24, 30, 32
  - ESP32C6 uses PART_SIZE=24 (RMT_SIZE=48)
  - ESP32/ESP32S3 use PART_SIZE=32 (RMT_SIZE=64)
  - PART_SIZE must be even (enforced by preprocessor check)
  
  Preprocessor options:
  - DUMP_RMT_SYMBOLS: Set to true/false to enable/disable condensed RMT symbol
    dumps for each test (default: true)
  
  Key validation criteria:
  1. Step count must match command steps
  2. Total ticks must match command ticks
  3. High/Low ratio should be ~1:1 (50% duty cycle) for step commands
  4. Minimum RMT symbol period must be >= 2 ticks (hardware limit)
  5. For ticks=65535, special encoding uses 2 RMT entries per step

- test 19
  ramp up with 100 step/s^2 to max speed. should not coast (#347)

- test 20
  test case for motor stopping at low speed with acceleration=8164, speed=81Hz
  
  Part 1: Motor stops at low speed
  - Tests motor movement with accel=8164, speed=81Hz, target=1720 steps
  - Verifies motor does not stop prematurely due to empty queue
  
  Part 2: Speed change from high to low
  - Tests speed reduction from 89Hz to 81Hz during movement
  - Reproduces reported bug where motor stops after speed reduction
  
  Part 3: Multiple speed changes
  - Tests sequence: 73Hz -> 81Hz -> 89Hz -> 97Hz -> 81Hz
  - Verifies motor continues through all speed changes
  
  Part 4: Reduced acceleration
  - Tests with acceleration=8164/3 (user-reported workaround)
  - Verifies motor reaches target with reduced acceleration

- test 21
  test case for ESP32 I2S stepper implementation with 2µs pulse generation
  
  Part 1: I2S init
  - Tests I2sManager initialization and work buffer allocation
  
  Part 2: Direct write
  - Tests writing and verifying 2µs pulses (both L and R bytes = 0xFF)
  
  Part 3: Queue fill
  - Tests filling I2S buffer from stepper queue commands
  - Validates pulse positions match expected frames
  
  Part 4: Ring buffer clear
  - Tests old pulse clearing when new pulses are added
  - Verifies ring buffer tracking of pulse positions
  
  Part 5: Frame boundary
  - Tests pulses near buffer boundary (frames 1990, 1995, 1998)
  - Validates no off-by-one errors in frame calculation
  
  Part 6: Empty queue handling
  - Tests _isRunning and _i2s_drain state when queue empties
  - Verifies drain counter decrements correctly
  
  Part 7: Ring buffer wrap
  - Tests ring buffer wrap-around at I2S_TEST_PULSE_MAX capacity
  
  Part 8: Edge Cases
  - 1 pulse at 65535 ticks (max uint16_t)
    * Tests pulse beyond buffer boundary (2047 frames > 1000 frame limit)
    * Verifies no pulse written when frame_pos exceeds buffer
  - 1 pulse at MIN_CMD_TICKS (3200 ticks)
    * Tests minimum command period
    * Validates frame calculation: 3200/32 = 100 frames
  - 255 pulses at 65535 ticks
    * Tests partial entry processing when pulses don't fit
    * Validates remaining steps tracked correctly
  - 255 pulses at 200kHz (80 ticks each)
    * Tests high-frequency pulse generation
    * Validates tick carry accumulation for sub-frame periods
  - Pulses at ticks < I2S_TEST_TICKS_PER_FRAME (16 ticks)
    * Tests carry accumulation when ticks < 32
    * Validates multiple pulses per frame handling

- ramp_helper
  Helper tool to generate and dump ramp commands for given speed and acceleration
  Usage: make ramp_helper && ./ramp_helper <speed_us> <acceleration> <steps>
  Example: ./ramp_helper 1000 10000 1000
  Outputs commands until coasting reached, useful for creating test inputs

- ESP32 RMT Notes:
  * queue_entry.ticks is uint16_t, max value 65535 ticks (4095us at 16MHz)
  * For periods >65535 ticks, ramp generator uses pause commands
  * ticks=65535 is special case in rmt_fill_buffer (uses 2 entries per step)
  * ESP32C6 has PART_SIZE=24, ESP32/ESP32S3 use PART_SIZE=32
  * PART_SIZE must be even (enforced by preprocessor check)
  * Minimum RMT symbol period is 2 ticks (hardware limit)
