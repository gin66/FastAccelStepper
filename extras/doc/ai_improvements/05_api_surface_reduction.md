# API Surface Reduction

## Current State Analysis

### Problem
The `FastAccelStepper` class has **100+ public methods**, making the API complex and hard to use correctly:

**API Complexity Issues:**
1. **Method overload**: Multiple ways to do the same thing
2. **Poor grouping**: Related functionality scattered
3. **Complex parameter patterns**: Inconsistent parameter ordering
4. **Hidden dependencies**: Methods that must be called in specific order
5. **Learning curve**: Too many methods to understand

### Current API Surface

**Excessive method count in FastAccelStepper.h:**
- **Configuration methods**: 15+ methods for pin and timing setup
- **Motion control methods**: 20+ methods for movement
- **Speed/acceleration methods**: 15+ methods with multiple units
- **Queue management methods**: 10+ low-level methods
- **State query methods**: 20+ getter methods
- **Platform-specific methods**: 10+ ESP32-only methods

**Example of method overload:**
```cpp
// Multiple ways to set speed
int8_t setSpeedInUs(uint32_t min_step_us);
int8_t setSpeedInTicks(uint32_t min_step_ticks);
int8_t setSpeedInHz(uint32_t speed_hz);
int8_t setSpeedInMilliHz(uint32_t speed_mhz);

// Multiple ways to get speed
uint32_t getSpeedInUs();
uint32_t getSpeedInTicks();
uint32_t getSpeedInMilliHz();
int32_t getCurrentSpeedInUs(bool realtime = true);
int32_t getCurrentSpeedInMilliHz(bool realtime = true);
```

**Complex parameter patterns:**
```cpp
// Inconsistent parameter ordering
void setDirectionPin(uint8_t dirPin, bool dirHighCountsUp = true,
                    uint16_t dir_change_delay_us = 0);
                    
void setEnablePin(uint8_t enablePin, bool low_active_enables_stepper = true);

// Some methods return error codes, some return bool, some void
MoveResultCode move(int32_t move, bool blocking = false);
bool enableOutputs();
void setAutoEnable(bool auto_enable);
```

## Proposed Solution

### Grouped API Design

Create focused classes for related functionality:

```cpp
// src/api/StepperConfiguration.h
#pragma once

#include <stdint.h>

class StepperConfiguration {
public:
    // Pin configuration
    struct Pins {
        uint8_t step;
        uint8_t direction;
        uint8_t enable;
    };
    
    StepperConfiguration& setPins(const Pins& pins);
    StepperConfiguration& setDirectionConfig(bool high_counts_up = true,
                                            uint16_t change_delay_us = 0);
    StepperConfiguration& setEnableConfig(bool low_active = true);
    
    // Timing configuration
    StepperConfiguration& setSpeed(uint32_t speed_hz);
    StepperConfiguration& setAcceleration(uint32_t acceleration_hz_s);
    StepperConfiguration& setLinearAcceleration(uint32_t steps);
    StepperConfiguration& setJumpStart(uint32_t steps);
    
    // Auto-enable configuration
    StepperConfiguration& setAutoEnable(bool enable,
                                       uint32_t enable_delay_us = 0,
                                       uint16_t disable_delay_ms = 0);
    
    // Validation
    bool validate() const;
    
private:
    Pins pins_;
    bool dir_high_counts_up_;
    uint16_t dir_change_delay_us_;
    bool enable_low_active_;
    uint32_t speed_hz_;
    uint32_t acceleration_hz_s_;
    uint32_t linear_accel_steps_;
    uint32_t jump_start_steps_;
    bool auto_enable_;
    uint32_t enable_delay_us_;
    uint16_t disable_delay_ms_;
};
```

```cpp
// src/api/StepperMotion.h
#pragma once

#include <stdint.h>
#include "fas_arch/result_codes.h"

class StepperMotion {
public:
    // Basic movement
    MoveResultCode moveTo(int32_t position, bool blocking = false);
    MoveResultCode move(int32_t steps, bool blocking = false);
    
    // Continuous movement
    MoveResultCode runForward();
    MoveResultCode runBackward();
    void keepRunning();
    
    // Controlled movement
    MoveResultCode moveByAcceleration(int32_t acceleration,
                                     bool allow_reverse = true);
    
    // Stopping
    void stop();           // Normal deceleration
    void emergencyStop();  // Immediate stop
    
    // Single steps
    void stepForward(bool blocking = false);
    void stepBackward(bool blocking = false);
    
    // State queries
    bool isRunning() const;
    bool isMoving() const;  // Actually producing steps
    bool isAccelerating() const;
    bool isDecelerating() const;
    bool isStopping() const;
    
    // Position information
    int32_t getPosition() const;
    int32_t getTargetPosition() const;
    uint32_t stepsToStop() const;
    
private:
    // Implementation details
};
```

```cpp
// src/api/StepperStatus.h
#pragma once

#include <stdint.h>

struct StepperStatus {
    // Current state
    int32_t position;
    int32_t target_position;
    int32_t current_speed_hz;
    int32_t current_acceleration_hz_s;
    uint32_t steps_to_stop;
    
    // Movement state
    bool is_running;
    bool is_moving;
    bool is_accelerating;
    bool is_decelerating;
    bool is_stopping;
    bool is_continuous;
    
    // Queue information
    uint8_t queue_entries;
    uint32_t queue_ticks;
    bool queue_full;
    bool queue_empty;
    bool queue_running;
    
    // Configuration
    uint32_t configured_speed_hz;
    uint32_t configured_acceleration_hz_s;
    bool auto_enable;
};

class StepperStatusProvider {
public:
    StepperStatus getStatus() const;
    StepperStatus getDetailedStatus() const;  // More expensive
    
    // Individual queries (cheaper than full status)
    int32_t getPosition() const;
    int32_t getCurrentSpeed() const;
    bool isRunning() const;
    
private:
    // Implementation
};
```

### Simplified FastAccelStepper Class

```cpp
// src/FastAccelStepper.h (simplified)
#pragma once

#include "api/StepperConfiguration.h"
#include "api/StepperMotion.h"
#include "api/StepperStatus.h"

class FastAccelStepper {
public:
    // Factory method (replaces complex constructor)
    static FastAccelStepper* create(uint8_t step_pin);
    
    // Configuration (one method to configure everything)
    bool configure(const StepperConfiguration& config);
    
    // Motion control (delegated to StepperMotion)
    MoveResultCode moveTo(int32_t position, bool blocking = false) {
        return motion_.moveTo(position, blocking);
    }
    
    MoveResultCode move(int32_t steps, bool blocking = false) {
        return motion_.move(steps, blocking);
    }
    
    void stop() { motion_.stop(); }
    void emergencyStop() { motion_.emergencyStop(); }
    
    // Status (delegated to StepperStatusProvider)
    StepperStatus getStatus() const { return status_.getStatus(); }
    int32_t getPosition() const { return status_.getPosition(); }
    bool isRunning() const { return status_.isRunning(); }
    
    // Advanced features (optional, for power users)
    class Advanced {
    public:
        // Queue management
        AqeResultCode addQueueEntry(const stepper_command_s* cmd,
                                   bool start = true);
        bool isQueueEmpty() const;
        bool isQueueFull() const;
        uint32_t ticksInQueue() const;
        
        // Platform-specific features
        #ifdef SUPPORT_ESP32_PULSE_COUNTER
        bool attachToPulseCounter(/* params */);
        int16_t readPulseCounter();
        #endif
    };
    
    Advanced& advanced() { return advanced_; }
    
private:
    StepperConfiguration config_;
    StepperMotion motion_;
    StepperStatusProvider status_;
    Advanced advanced_;
    
    // Private constructor
    FastAccelStepper(uint8_t step_pin);
};
```

### Builder Pattern for Configuration

```cpp
// src/api/StepperBuilder.h
#pragma once

#include "StepperConfiguration.h"

class StepperBuilder {
public:
    StepperBuilder& withStepPin(uint8_t pin) {
        config_.pins.step = pin;
        return *this;
    }
    
    StepperBuilder& withDirectionPin(uint8_t pin, bool high_counts_up = true) {
        config_.pins.direction = pin;
        config_.dir_high_counts_up = high_counts_up;
        return *this;
    }
    
    StepperBuilder& withEnablePin(uint8_t pin, bool low_active = true) {
        config_.pins.enable = pin;
        config_.enable_low_active = low_active;
        return *this;
    }
    
    StepperBuilder& withSpeed(uint32_t speed_hz) {
        config_.speed_hz = speed_hz;
        return *this;
    }
    
    StepperBuilder& withAcceleration(uint32_t acceleration_hz_s) {
        config_.acceleration_hz_s = acceleration_hz_s;
        return *this;
    }
    
    StepperBuilder& withAutoEnable(bool enable = true,
                                  uint32_t enable_delay_us = 0,
                                  uint16_t disable_delay_ms = 0) {
        config_.auto_enable = enable;
        config_.enable_delay_us = enable_delay_us;
        config_.disable_delay_ms = disable_delay_ms;
        return *this;
    }
    
    // Create the stepper
    FastAccelStepper* create() {
        if (!config_.validate()) {
            return nullptr;
        }
        auto stepper = FastAccelStepper::create(config_.pins.step);
        if (stepper) {
            stepper->configure(config_);
        }
        return stepper;
    }
    
private:
    StepperConfiguration config_;
};
```

### Usage Examples

**Simple usage (80% of cases):**
```cpp
// Current way (complex)
FastAccelStepperEngine engine;
engine.init();
auto stepper = engine.stepperConnectToPin(step_pin);
if (stepper) {
    stepper->setDirectionPin(dir_pin);
    stepper->setEnablePin(enable_pin);
    stepper->setAutoEnable(true);
    stepper->setSpeedInHz(500);
    stepper->setAcceleration(100);
    stepper->moveTo(1000);
}

// Proposed way (simpler)
auto stepper = StepperBuilder()
    .withStepPin(step_pin)
    .withDirectionPin(dir_pin)
    .withEnablePin(enable_pin)
    .withSpeed(500)
    .withAcceleration(100)
    .withAutoEnable()
    .create();

if (stepper) {
    stepper->moveTo(1000);
}
```

**Advanced usage (power users):**
```cpp
// Current way (scattered methods)
stepper->setSpeedInUs(200);
stepper->setLinearAcceleration(100);
stepper->setJumpStart(10);
stepper->moveByAcceleration(50, false);
auto speed = stepper->getCurrentSpeedInMilliHz(true);
auto queue_ticks = stepper->ticksInQueue();

// Proposed way (grouped)
stepper->configure(StepperConfiguration()
    .setSpeed(5000)  // 5000 Hz
    .setLinearAcceleration(100)
    .setJumpStart(10));

stepper->moveByAcceleration(50, false);

auto status = stepper->getStatus();
// status.current_speed_hz
// status.queue_ticks
```

### Benefits

1. **Simplified Learning Curve**: Fewer methods to learn
2. **Better Organization**: Related functionality grouped
3. **Type Safety**: Configuration validated before use
4. **Cleaner Code**: Builder pattern for complex setup
5. **Progressive Disclosure**: Simple API for common cases, advanced for power users
6. **Testable**: Smaller, focused classes easier to test

### Implementation Strategy

**Phase 1: New API Structure**
1. Create configuration, motion, and status classes
2. Implement builder pattern
3. Add validation logic

**Phase 2: Backward Compatibility**
1. Create wrapper that implements old interface
2. Deprecate old methods with warnings
3. Provide migration guide

**Phase 3: Gradual Migration**
1. Update examples to use new API
2. Update documentation
3. Encourage new code to use new API

### Performance Considerations

**Zero-cost abstractions:**
- **Inline delegation**: Simple methods inline to existing implementation
- **Compile-time configuration**: Template-based builders
- **Minimal overhead**: New classes are thin wrappers

**Memory impact:**
- **No additional allocation**: Configuration passed by value/reference
- **Stack-based builders**: No heap allocation
- **Shared implementation**: Core logic unchanged

### Testing Strategy

**Unit tests for each API class:**
```cpp
TEST(StepperConfiguration, Validation) {
    StepperConfiguration config;
    config.setPins({.step = 9, .direction = 8, .enable = 7});
    config.setSpeed(1000);
    config.setAcceleration(100);
    
    EXPECT_TRUE(config.validate());
    
    config.setSpeed(0);
    EXPECT_FALSE(config.validate());
}
```

**Integration tests:**
```cpp
TEST(FastAccelStepper, SimpleUsage) {
    auto stepper = StepperBuilder()
        .withStepPin(9)
        .withSpeed(500)
        .create();
    
    ASSERT_NE(stepper, nullptr);
    EXPECT_EQ(stepper->moveTo(1000), MOVE_OK);
    EXPECT_TRUE(stepper->isRunning());
}
```

### Migration Plan

**Step 1: Add new API alongside old**
- Implement new classes
- Keep old API fully functional
- Add deprecation warnings

**Step 2: Update documentation and examples**
- Create new examples using new API
- Update getting started guide
- Provide migration examples

**Step 3: Gradual phase-out**
- After sufficient adoption, mark old API as deprecated
- Provide automatic migration tools
- Eventually remove old API (major version)

### Expected Outcomes

- **Reduced API Surface**: 100+ methods → ~30 core methods
- **Better Usability**: Clear, grouped functionality
- **Improved Safety**: Validation before use
- **Cleaner Code**: Builder pattern for configuration
- **Maintained Performance**: Thin wrappers over existing implementation
- **Easier Maintenance**: Smaller, focused classes