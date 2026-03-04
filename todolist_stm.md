# STM32F103C6T6 Support - TODO List

## Phase 1: Hardware Bring-up
- [ ] Verify LED pin (PC13 or other?)
- [ ] Confirm upload method (ST-Link / USB DFU / Serial bootloader)
- [ ] Upload and test blink sketch: `cd pio_dirs/Blink_STM32 && pio run -t upload`
- [ ] Verify LED blinks at 1Hz

## Phase 2: Architecture Discussion
- [ ] Review STM32F103C6T6 timer resources:
  - TIM1 (advanced, 16-bit) - channels on PA8, PA9, PA10, PA11
  - TIM2 (general, 16-bit) - channels on PA0, PA1, PA2, PA3
  - TIM3 (general, 16-bit) - channels on PA6, PA7, PB0, PB1
  - TIM4 (general, 16-bit) - channels on PB6, PB7, PB8, PB9 (if available on C6)
- [ ] Decide: HAL timer API vs direct register access
- [ ] Decide: PWM output compare vs interrupt-driven GPIO toggle
- [ ] Decide: How many steppers to support (limited by timers/RAM)
- [ ] Define TICKS_PER_S based on timer clock and prescaler

## Phase 3: Stub Files (compile-safe)
- [ ] Create `src/fas_arch/arduino_stm32.h` with constants only
- [ ] Create `src/StepperISR_stm32.cpp` with empty guarded implementation
- [ ] Modify `src/StepperISR.h` - add SUPPORT_STM32 section
- [ ] Modify `src/fas_arch/common.h` - add STM32 include
- [ ] Verify existing platforms still compile

## Phase 4: Timer Test Sketch
- [ ] Create test sketch using HardwareTimer to generate pulses
- [ ] Verify pulse timing with logic analyzer / oscilloscope
- [ ] Test interrupt latency and jitter

## Phase 5: Driver Implementation
- [ ] Implement `StepperQueue::init()` for STM32
- [ ] Implement `StepperQueue::startQueue()`
- [ ] Implement `StepperQueue::forceStop()`
- [ ] Implement `StepperQueue::isRunning()`
- [ ] Implement `StepperQueue::isValidStepPin()`
- [ ] Implement timer ISR for step generation
- [ ] Implement `fas_init_engine()`

## Phase 6: Testing
- [ ] Test single stepper with StepperDemo
- [ ] Verify acceleration/deceleration ramps
- [ ] Test position tracking accuracy
- [ ] Test multiple steppers (if supported)
- [ ] Run with actual stepper motor + driver

## Phase 7: Integration
- [ ] Add STM32 env to other pio_dirs projects
- [ ] Update build-platformio.sh
- [ ] Update README.md with STM32 support info
- [ ] Update CHANGELOG.md

## Notes
- F103C6T6: 32KB Flash, 10KB RAM, 72MHz
- Current blink uses ~32% Flash, ~11% RAM
- Consider smaller QUEUE_LEN (16 vs 32) to save RAM
