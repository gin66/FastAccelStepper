# Queue Management Refactoring

## Current State Analysis

### Problem
The `StepperQueue` class has platform-specific conditional compilation mixed with core queue logic, making it hard to maintain and test:

**Current StepperQueue issues:**
1. **Platform-specific code in core class**: Conditional compilation for AVR, ESP32, SAM, Pico
2. **Complex state management**: Multiple volatile variables for ISR synchronization
3. **Hard to test**: ISR dependencies make unit testing difficult
4. **Code duplication**: Similar queue logic across platforms with minor variations
5. **Poor separation of concerns**: Hardware control mixed with queue algorithms

### Code Complexity Examples

**Platform-specific conditional compilation in StepperQueue:**
```cpp
// From StepperISR.h - mixed platform code
class StepperQueue {
public:
    struct queue_entry entry[QUEUE_LEN];
    
    // Platform-specific members
#if defined(SUPPORT_RP_PICO)
    bool _isStarting;
    PIO pio;
    uint sm;
    bool claim_pio_sm(FastAccelStepperEngine* engine);
#endif
    
#if defined(SUPPORT_ESP32)
    volatile bool _isRunning;
    bool _nextCommandIsPrepared;
    bool use_rmt;
#endif
    
#if defined(SUPPORT_AVR)
    // AVR-specific members
#endif
    
    // Common methods with platform-specific implementations
    bool isRunning();
    bool isReadyForCommands();
    void startQueue();
    void stopQueue();
};
```

**Complex ISR synchronization:**
```cpp
// Multiple volatile variables for ISR communication
volatile uint8_t read_idx;      // ISR reads this
volatile uint8_t next_write_idx; // Application writes this
volatile bool ignore_commands;   // Force stop synchronization
```

## Proposed Solution

### Strategy Pattern for Queue Operations

Separate queue algorithms from hardware control using the Strategy pattern:

```cpp
// src/queue/QueueStrategy.h
#pragma once

#include <stdint.h>
#include "fas_arch/common.h"

class QueueStrategy {
public:
    virtual ~QueueStrategy() = default;
    
    // Queue operations
    virtual AqeResultCode addEntry(const stepper_command_s& cmd) = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void emergencyStop() = 0;
    
    // State queries
    virtual bool isRunning() const = 0;
    virtual bool isReadyForCommands() const = 0;
    virtual bool isQueueFull() const = 0;
    virtual bool isQueueEmpty() const = 0;
    
    // Queue information
    virtual uint8_t entries() const = 0;
    virtual uint32_t ticksInQueue() const = 0;
    virtual bool hasTicks(uint32_t min_ticks) const = 0;
    
    // Position tracking
    virtual int32_t getCurrentPosition() const = 0;
    virtual void setCurrentPosition(int32_t pos) = 0;
    virtual int32_t getPositionAfterCommands() const = 0;
    
    // Callbacks (for ISR/task notifications)
    virtual void setCommandProcessedCallback(void (*callback)(void*), void* ctx) = 0;
    virtual void setQueueEmptyCallback(void (*callback)(void*), void* ctx) = 0;
};
```

### Hardware-Agnostic Queue Implementation

```cpp
// src/queue/CircularQueue.h
#pragma once

#include <stdint.h>
#include <array>

template<size_t QueueSize>
class CircularQueue {
    static_assert((QueueSize & (QueueSize - 1)) == 0, 
                  "QueueSize must be power of two");
    
private:
    std::array<queue_entry, QueueSize> entries_;
    volatile uint8_t read_idx_;
    volatile uint8_t write_idx_;
    volatile bool running_;
    
    // Position tracking
    volatile int32_t current_position_;
    volatile bool current_direction_;
    
public:
    CircularQueue() : read_idx_(0), write_idx_(0), running_(false),
                     current_position_(0), current_direction_(true) {}
    
    // Queue operations
    bool push(const queue_entry& entry) {
        if (isFull()) return false;
        
        entries_[write_idx_] = entry;
        write_idx_ = (write_idx_ + 1) & (QueueSize - 1);
        return true;
    }
    
    bool pop(queue_entry& entry) {
        if (isEmpty()) return false;
        
        entry = entries_[read_idx_];
        read_idx_ = (read_idx_ + 1) & (QueueSize - 1);
        return true;
    }
    
    // State queries
    bool isEmpty() const { return read_idx_ == write_idx_; }
    bool isFull() const { return ((write_idx_ + 1) & (QueueSize - 1)) == read_idx_; }
    
    uint8_t size() const {
        return (write_idx_ - read_idx_) & (QueueSize - 1);
    }
    
    // Position tracking
    void updatePosition(int32_t delta) {
        current_position_ += delta;
    }
    
    int32_t getPosition() const { return current_position_; }
    void setPosition(int32_t pos) { current_position_ = pos; }
    
    // Thread/ISR safe operations
    void start() { running_ = true; }
    void stop() { running_ = false; }
    bool isRunning() const { return running_; }
};
```

### Platform-Specific Strategy Implementations

**AVR Strategy:**
```cpp
// src/queue/strategies/AVRQueueStrategy.h
#pragma once

#include "QueueStrategy.h"
#include "CircularQueue.h"
#include "hardware/avr/AVRTimer.h"

class AVRQueueStrategy : public QueueStrategy {
private:
    CircularQueue<16> queue_;  // AVR has 16-entry queues
    AVRTimer timer_;
    uint8_t step_pin_;
    uint8_t dir_pin_;
    
public:
    AVRQueueStrategy(uint8_t step_pin, uint8_t dir_pin);
    
    // QueueStrategy implementation
    AqeResultCode addEntry(const stepper_command_s& cmd) override;
    void start() override;
    void stop() override;
    // ... other overrides
    
private:
    void configureTimerForEntry(const queue_entry& entry);
    void isrHandler();  // Timer ISR
};
```

**ESP32 Strategy:**
```cpp
// src/queue/strategies/ESP32QueueStrategy.h
#pragma once

#include "QueueStrategy.h"
#include "CircularQueue.h"
#include "hardware/esp32/ESP32Driver.h"

template<typename DriverType>
class ESP32QueueStrategy : public QueueStrategy {
private:
    CircularQueue<32> queue_;  // ESP32 has 32-entry queues
    DriverType driver_;
    TaskHandle_t task_handle_;
    
public:
    ESP32QueueStrategy(uint8_t step_pin, uint8_t dir_pin);
    
    // QueueStrategy implementation
    AqeResultCode addEntry(const stepper_command_s& cmd) override;
    void start() override;
    void stop() override;
    // ... other overrides
    
private:
    static void stepperTask(void* param);
    void taskLoop();
    void processNextCommand();
};
```

### StepperQueue Refactored

```cpp
// src/StepperQueue.h (refactored)
#pragma once

#include "queue/QueueStrategy.h"
#include "config/PlatformConfig.h"

class StepperQueue {
private:
    std::unique_ptr<QueueStrategy> strategy_;
    PlatformConfig config_;
    
    // Common state (no platform-specific conditionals)
    uint8_t queue_num_;
    bool dir_high_counts_up_;
    uint8_t dir_pin_;
    
public:
    StepperQueue(uint8_t queue_num, uint8_t step_pin, uint8_t dir_pin);
    
    // Factory method for platform-specific strategy
    static std::unique_ptr<QueueStrategy> createStrategy(
        uint8_t queue_num, uint8_t step_pin, uint8_t dir_pin);
    
    // Delegated methods
    AqeResultCode addQueueEntry(const stepper_command_s* cmd, bool start = true) {
        if (!cmd) {
            if (start) strategy_->start();
            return AQE_OK;
        }
        return strategy_->addEntry(*cmd);
    }
    
    bool isQueueEmpty() const { return strategy_->isQueueEmpty(); }
    bool isQueueFull() const { return strategy_->isQueueFull(); }
    bool isQueueRunning() const { return strategy_->isRunning(); }
    
    uint32_t ticksInQueue() const { return strategy_->ticksInQueue(); }
    bool hasTicksInQueue(uint32_t min_ticks) const {
        return strategy_->hasTicks(min_ticks);
    }
    
    uint8_t queueEntries() const { return strategy_->entries(); }
    
    // Position tracking
    int32_t getPositionAfterCommandsCompleted() const {
        return strategy_->getPositionAfterCommands();
    }
    
    void setPositionAfterCommandsCompleted(int32_t new_pos) {
        strategy_->setCurrentPosition(new_pos);
    }
    
    // Platform-specific operations (delegated)
    void startQueue() { strategy_->start(); }
    void stopQueue() { strategy_->stop(); }
    bool isReadyForCommands() const { return strategy_->isReadyForCommands(); }
};
```

### Benefits

1. **Clean Separation**: Queue algorithms separate from hardware
2. **Testable**: Mock strategies for unit testing
3. **Reusable**: Common queue logic across platforms
4. **Maintainable**: Platform-specific code isolated
5. **Extensible**: Easy to add new queue strategies

### Implementation Strategy

**Phase 1: Core Queue Infrastructure**
1. Create `QueueStrategy` interface
2. Implement `CircularQueue` template
3. Add queue algorithm utilities

**Phase 2: Strategy Implementations**
1. Implement AVR strategy (simplest)
2. Implement ESP32 strategy (most complex)
3. Implement other platform strategies

**Phase 3: StepperQueue Refactoring**
1. Update StepperQueue to use strategies
2. Migrate existing code to new structure
3. Update all callers

### Performance Considerations

**Zero-cost abstractions for embedded:**
- **Template-based queue size**: Compile-time optimization
- **Static polymorphism**: CRTP instead of virtual functions for hot paths
- **Inline functions**: Critical methods inlined
- **ISR-friendly design**: Minimal overhead in interrupt context

**Memory optimization:**
- **Static allocation**: No dynamic allocation in critical paths
- **Cache-friendly layout**: Queue entries packed efficiently
- **Minimal state**: Only essential volatile variables

### Testing Strategy

**Unit tests for queue algorithms:**
```cpp
TEST(CircularQueue, BasicOperations) {
    CircularQueue<16> queue;
    
    queue_entry entry = {.ticks = 100, .steps = 1, .count_up = true};
    EXPECT_TRUE(queue.push(entry));
    EXPECT_FALSE(queue.isEmpty());
    
    queue_entry popped;
    EXPECT_TRUE(queue.pop(popped));
    EXPECT_EQ(popped.ticks, 100);
    EXPECT_TRUE(queue.isEmpty());
}
```

**Integration tests with mock hardware:**
```cpp
class MockQueueStrategy : public QueueStrategy {
    // Mock implementation for testing
};

TEST(StepperQueue, Integration) {
    auto mock_strategy = std::make_unique<MockQueueStrategy>();
    StepperQueue queue(std::move(mock_strategy));
    
    // Test queue operations without real hardware
}
```

### Migration Plan

**Step 1: Add new system alongside old**
- Create strategy interface and implementations
- Keep existing StepperQueue for compatibility
- Add factory methods for strategy creation

**Step 2: Gradual migration**
- Migrate one platform at a time
- Test thoroughly after each migration
- Update dependent code gradually

**Step 3: Clean up**
- Remove old StepperQueue implementation
- Clean up platform-specific conditionals
- Update documentation

### Backward Compatibility

**Compatibility layer during transition:**
```cpp
// Temporary compatibility
class CompatibleStepperQueue : public StepperQueue {
    // Exposes same interface as old StepperQueue
    // Uses new strategy internally
};
```

### Expected Outcomes

- **Cleaner Code**: No platform conditionals in core logic
- **Better Testing**: Mockable strategies for unit tests
- **Improved Maintenance**: Isolated platform code
- **Performance Maintained**: Optimized for each platform
- **Easier Extensions**: New platforms just implement strategy
- **Reusable Algorithms**: Common queue logic shared