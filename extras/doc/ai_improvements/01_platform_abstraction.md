# Platform Abstraction Layer Consolidation

## Current State Analysis

### Problem
FastAccelStepper has **10+ platform-specific StepperISR implementations** with significant code duplication:
- `StepperISR_avr.cpp` (368 lines)
- `StepperISR_due.cpp` (714 lines) 
- `StepperISR_esp32.cpp` (140 lines)
- `StepperISR_esp32xx_rmt.cpp` (255 lines)
- `StepperISR_idf4_esp32_mcpwm_pcnt.cpp` (574 lines)
- `StepperISR_idf4_esp32_rmt.cpp` (358 lines)
- `StepperISR_idf4_esp32c3_rmt.cpp` (489 lines)
- `StepperISR_idf4_esp32s3_rmt.cpp` (490 lines)
- `StepperISR_idf5_esp32_rmt.cpp` (225 lines)
- `StepperISR_rp_pico.cpp` (281 lines)

Each file implements similar patterns for:
- Timer configuration and control
- Pulse generation
- Interrupt/task handling
- Queue management

### Code Duplication Examples

**Common patterns found across all implementations:**
```cpp
// Similar structure in multiple files
void StepperQueue::startQueue() {
    // Platform-specific hardware initialization
    // Timer configuration
    // Interrupt setup
}

bool StepperQueue::isRunning() {
    // Platform-specific running state check
}

void StepperQueue::stopQueue() {
    // Platform-specific stop procedure
}
```

## Proposed Solution

### Unified Hardware Abstraction Interface

Create a clean abstraction layer that separates hardware-specific code from the core stepper logic:

```cpp
// src/platform/HardwareAbstraction.h
#pragma once

#include <stdint.h>
#include "fas_arch/common.h"

class HardwareTimer {
public:
    virtual ~HardwareTimer() = default;
    
    // Configuration
    virtual bool configure(uint8_t pin, uint32_t base_frequency) = 0;
    virtual bool setPeriod(uint32_t ticks) = 0;
    
    // Control
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void pause() = 0;
    virtual void resume() = 0;
    
    // State
    virtual bool isRunning() const = 0;
    virtual uint32_t getRemainingTicks() const = 0;
    
    // Callback setup
    virtual void setInterruptCallback(void (*callback)(void*), void* context) = 0;
};

class HardwarePulseGenerator {
public:
    virtual ~HardwarePulseGenerator() = default;
    
    // Pulse generation
    virtual bool generatePulses(uint8_t pin, uint32_t count, 
                               uint32_t period_ticks, bool count_up) = 0;
    virtual bool generatePulseTrain(uint8_t pin, const uint32_t* periods,
                                   uint32_t count, bool count_up) = 0;
    
    // Control
    virtual void stop() = 0;
    virtual void emergencyStop() = 0;
    
    // State
    virtual bool isBusy() const = 0;
    virtual uint32_t getRemainingPulses() const = 0;
    
    // Callback setup
    virtual void setCompletionCallback(void (*callback)(void*), void* context) = 0;
};

// Factory for creating platform-specific implementations
class HardwareFactory {
public:
    static HardwareTimer* createTimer();
    static HardwarePulseGenerator* createPulseGenerator();
    
    // Platform capabilities
    static uint32_t getTicksPerSecond();
    static uint32_t getMinCommandTicks();
    static uint32_t getMaxSteppers();
    static uint32_t getQueueLength();
};
```

### Concrete Implementations

**AVR Implementation:**
```cpp
// src/platform/avr/AVRTimer.cpp
class AVRTimer : public HardwareTimer {
    // Use Timer1/Timer3/Timer5 based on pin
    // Implement AVR-specific register access
};

class AVRPulseGenerator : public HardwarePulseGenerator {
    // Use output compare units for pulse generation
};
```

**ESP32 Implementation:**
```cpp
// src/platform/esp32/ESP32Timer.cpp
template<typename DriverType>
class ESP32Timer : public HardwareTimer {
    // DriverType = MCPWMDriver or RMTDriver
    // Common ESP32 timer logic
};
```

### Benefits

1. **Reduced Code Duplication**: Common logic moves to base classes
2. **Cleaner Architecture**: Clear separation between hardware and application logic
3. **Easier Testing**: Mock implementations for unit testing
4. **Simplified Platform Addition**: New platforms implement clear interfaces
5. **Better Maintainability**: Changes to core logic affect all platforms

### Implementation Strategy

**Phase 1: Interface Definition**
1. Define abstraction interfaces
2. Create platform capability detection system
3. Update build system to support new structure

**Phase 2: Platform Migration**
1. Migrate AVR implementation as reference
2. Migrate ESP32 implementations (biggest win)
3. Migrate remaining platforms

**Phase 3: Integration**
1. Update StepperQueue to use abstraction
2. Update FastAccelStepperEngine
3. Comprehensive testing

### Performance Considerations

For embedded performance:
- Use **CRTP (Curiously Recurring Template Pattern)** instead of virtual functions for critical paths
- Provide **compile-time specialization** for common platforms
- Maintain **inline functions** for hot paths
- Keep **ISR code minimal** and hardware-specific

### Memory Impact

The abstraction should maintain or reduce memory usage:
- **No dynamic allocation** in critical paths
- **Static dispatch** where possible
- **Template-based configuration** for compile-time optimization

### Testing Strategy

1. **Unit Tests**: Test each abstraction interface
2. **Integration Tests**: Test platform implementations
3. **Performance Tests**: Ensure no regression in timing
4. **Platform Tests**: Verify on actual hardware

### Migration Plan

1. Start with new platforms using the abstraction
2. Gradually migrate existing platforms
3. Maintain backward compatibility during transition
4. Provide migration guides for each platform

### Expected Outcomes

- **Code Reduction**: 30-40% reduction in platform-specific code
- **Improved Maintainability**: Clearer separation of concerns
- **Faster Development**: Easier to add new platforms
- **Better Testing**: More testable architecture
- **Performance Maintained**: No regression in timing accuracy