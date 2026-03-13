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
  ESP32 I2S low-level DMA simulation test
   
  Focus: Pure DMA infrastructure tests - callbacks, triple buffering, 
  bit stream analysis. No driver code involved.
   
  Part 1: DMA Callback Tests
  - Callback invocation on consume
   
  Part 2: DMA Block Management
  - Block rotation (0 -> 1 -> 2 -> 0)
  - Write block advance
  - Buffer clearing on consume
  - Write block availability with DMA synchronization
   
  Part 3: Bit Stream Analysis
  - Single pulse detection (rising/falling edges)
  - Multiple pulses (7 pulses at constant period)
  - Total ticks per block verification
   
  Part 4: DMA Round Tests
  - 10 rounds with 7 pulses each (70 total pulses)
  - Buffer content verification
   
  Key validation criteria:
  1. Callbacks must be invoked on each DMA consume
  2. Triple buffer must rotate correctly (0->1->2->0)
  3. Write block availability depends on DMA position
  4. Bit stream analyzer must correctly detect pulses
  5. Buffer must be cleared after DMA consume

- test_23
  Direction pin and external pin handling tests
  
  Part 1: MIN_DIR_DELAY_US / MAX_DIR_DELAY_US tests
  - MIN_DIR_DELAY_US value verification (platform-specific minimum)
  - MAX_DIR_DELAY_US value verification (4095us max delay)
  - Clamping of delays below minimum to MIN_DIR_DELAY_US
  - Clamping of delays above maximum to MAX_DIR_DELAY_US
  
  Part 2: dir_change_delay_us tests for regular direction pins
  - Pause insertion on direction change with delay configured
  - No pause on pause-only commands (steps=0)
  - Behavior with/without delay configured
  - No extra delay if prior pause has sufficient ticks
  - No extra delay if prior single step has sufficient ticks
  
  Part 3: External direction pin behavior tests
  - External dir pin (pin >= 128) with empty queue succeeds
  - External dir pin with steps in queue returns AQE_DIR_PIN_2MS_PAUSE_ADDED
  - Queue drain and retry mechanism for external pins
  - 2ms pause insertion allows queue to drain
  
  Part 4: External enable pin behavior tests
  - External enable pin callback invocation
  - External enable pin disable callback
  
  Part 5: Integration tests
  - Queue ticks tracking across multiple commands
  - hasStepsInQueue() detection (steps vs pause commands)
  - Realistic direction change scenario with delays
  
  Key validation criteria:
  1. Direction delays must be clamped to valid range
  2. Pause commands inserted on direction change (non-external pins)
  3. External pins return AQE_DIR_PIN_2MS_PAUSE_ADDED if queue has steps
  4. External pins succeed when queue is empty of steps
  5. Callback mechanism works for external enable/disable

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
