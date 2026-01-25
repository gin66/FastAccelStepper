# ESP32 Driver Unification

## Current State Analysis

### Problem
ESP32 support is fragmented across **7 different implementation files** with significant duplication:

1. **Driver Types**:
   - `StepperISR_esp32.cpp` (140 lines) - Common ESP32 logic
   - `StepperISR_esp32xx_rmt.cpp` (255 lines) - Common RMT logic
   - `StepperISR_idf4_esp32_mcpwm_pcnt.cpp` (574 lines) - IDF4 MCPWM/PCNT
   - `StepperISR_idf4_esp32_rmt.cpp` (358 lines) - IDF4 RMT
   - `StepperISR_idf4_esp32c3_rmt.cpp` (489 lines) - IDF4 ESP32-C3 RMT
   - `StepperISR_idf4_esp32s3_rmt.cpp` (490 lines) - IDF4 ESP32-S3 RMT
   - `StepperISR_idf5_esp32_rmt.cpp` (225 lines) - IDF5 RMT

2. **IDF Version Fragmentation**:
   - Separate implementations for IDF v4, v5, v6
   - Different API calls for similar functionality
   - Version-specific conditional compilation

3. **Code Duplication Examples**:

**Common RMT initialization pattern (found in 4+ files):**
```cpp
// Similar code in multiple RMT implementations
void StepperQueue::start_rmt() {
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(step_pin, channel);
    config.clk_div = 80;  // 1 MHz resolution
    config.mem_block_num = 1;
    rmt_config(&config);
    rmt_driver_install(channel, 0, 0);
}
```

**Common queue management (duplicated across files):**
```cpp
bool StepperQueue::add_rmt_entry(const struct queue_entry* entry) {
    // Similar logic in multiple files with minor variations
    if (entry->steps == 0) {
        // Add pause
        add_rmt_pause(entry->ticks);
    } else {
        // Add pulses
        add_rmt_pulses(entry->steps, entry->ticks, entry->countUp);
    }
}
```

## Proposed Solution

### Template-Based Driver Architecture

Create a unified ESP32 driver system using templates and policy-based design:

```cpp
// src/platform/esp32/ESP32Driver.h
#pragma once

#include <stdint.h>
#include "fas_arch/common.h"

// Driver type tags
struct MCPWMDriverTag {};
struct RMTDriverTag {};
struct RMTv2DriverTag {};

// Timer policy
template<typename DriverTag>
class ESP32TimerPolicy;

// Pulse counter policy  
template<typename DriverTag>
class ESP32PulseCounterPolicy;

// Main ESP32 driver template
template<typename TimerPolicy, typename PulseCounterPolicy>
class ESP32StepperDriver {
private:
    TimerPolicy timer_;
    PulseCounterPolicy pulse_counter_;
    
public:
    ESP32StepperDriver(uint8_t step_pin, uint8_t dir_pin = 255);
    
    // Common interface
    bool init();
    void start();
    void stop();
    bool addCommand(const stepper_command_s& cmd);
    bool isRunning() const;
    
    // Platform-specific optimizations
    uint32_t getActualPosition() const;
    void emergencyStop();
};
```

### Concrete Implementations

**MCPWM/PCNT Driver:**
```cpp
// src/platform/esp32/MCPWMDriver.cpp
template<>
class ESP32TimerPolicy<MCPWMDriverTag> {
    // MCPWM-specific timer implementation
    // Common across IDF versions
};

template<>
class ESP32PulseCounterPolicy<MCPWMDriverTag> {
    // PCNT-specific pulse counting
    // Handles IDF version differences internally
};
```

**RMT Driver (unified):**
```cpp
// src/platform/esp32/RMTDriver.cpp
class RMTDriverBase {
protected:
    // Common RMT functionality
    bool configureChannel(uint8_t pin, uint8_t channel);
    bool addPulseTrain(const uint32_t* durations, size_t count);
    
    // IDF version abstraction
    virtual void rmt_config_impl(/* params */) = 0;
    virtual void rmt_write_impl(/* params */) = 0;
};

class RMTv4Driver : public RMTDriverBase {
    // IDF v4 specific implementation
};

class RMTv5Driver : public RMTDriverBase {
    // IDF v5 specific implementation  
};

class RMTv6Driver : public RMTDriverBase {
    // IDF v6 specific implementation
};
```

### IDF Version Abstraction

Create a version-aware abstraction layer:

```cpp
// src/platform/esp32/IDFVersion.h
#pragma once

#include <esp_idf_version.h>

class IDFVersion {
public:
    static constexpr int MAJOR = ESP_IDF_VERSION_MAJOR;
    static constexpr int MINOR = ESP_IDF_VERSION_MINOR;
    static constexpr int PATCH = ESP_IDF_VERSION_PATCH;
    
    // Feature detection
    static bool supportsRMTv2();
    static bool supportsMCPWM();
    static bool hasNewPCNTAPI();
    
    // API abstraction
    template<typename FuncV4, typename FuncV5, typename FuncV6>
    static auto callVersionSpecific(FuncV4 v4, FuncV5 v5, FuncV6 v6) {
        if constexpr (MAJOR == 4) {
            return v4();
        } else if constexpr (MAJOR == 5) {
            return v5();
        } else if constexpr (MAJOR == 6) {
            return v6();
        }
    }
};
```

### Driver Selection System

```cpp
// src/platform/esp32/DriverSelector.h
#pragma once

#include "IDFVersion.h"
#include "MCPWMDriver.h"
#include "RMTDriver.h"

class ESP32DriverSelector {
public:
    using DriverType = /* selected based on configuration */;
    
    static DriverType createDriver(uint8_t step_pin, FasDriver preferred = DRIVER_DONT_CARE) {
        // Auto-select based on:
        // 1. User preference
        // 2. IDF version capabilities
        // 3. Chip features (ESP32, ESP32-C3, ESP32-S3)
        // 4. Performance requirements
        
        if (preferred == DRIVER_MCPWM_PCNT && IDFVersion::supportsMCPWM()) {
            return createMCPWMDriver(step_pin);
        } else if (preferred == DRIVER_RMT || !IDFVersion::supportsMCPWM()) {
            return createRMTDriver(step_pin);
        }
        
        // Default to RMT for newer IDF versions
        return createRMTDriver(step_pin);
    }
    
private:
    static DriverType createMCPWMDriver(uint8_t step_pin);
    static DriverType createRMTDriver(uint8_t step_pin);
};
```

### Benefits

1. **Eliminate Duplication**: Common logic in base classes
2. **Simplify Maintenance**: One code path per feature
3. **Better Chip Support**: Clear handling of ESP32 variants
4. **Future-Proof**: Easy to add new IDF versions
5. **Performance Optimization**: Compile-time driver selection

### Implementation Strategy

**Phase 1: Common Infrastructure**
1. Create IDF version abstraction
2. Build common RMT/MCPWM base classes
3. Define unified driver interface

**Phase 2: Driver Migration**
1. Migrate MCPWM/PCNT driver (largest file)
2. Migrate RMT drivers (consolidate 4 files)
3. Update chip-specific variations

**Phase 3: Integration**
1. Update StepperQueue to use unified driver
2. Update FastAccelStepperEngine driver selection
3. Comprehensive testing across IDF versions

### Chip Variant Support

**Unified approach for ESP32 family:**
```cpp
class ESP32ChipVariant {
public:
    enum Type {
        ESP32,
        ESP32C3,
        ESP32S2,
        ESP32S3,
        ESP32C6,
        ESP32H2
    };
    
    static Type detect();
    static bool hasRMT();
    static bool hasMCPWM();
    static bool hasPCNT();
    static uint32_t getCPUFrequency();
};
```

### Performance Considerations

**Maintain ESP32-specific optimizations:**
- **RMT block mode** for continuous pulse trains
- **MCPWM symmetry** for precise timing
- **PCNT hardware filtering** for noise immunity
- **Cache-friendly data structures** for ISR performance

**Memory optimization:**
- **Static allocation** for driver instances
- **Shared buffers** between drivers
- **Compile-time configuration** for feature selection

### Testing Strategy

**Multi-version testing matrix:**
1. **IDF v4.4** on ESP32, ESP32-C3, ESP32-S3
2. **IDF v5.x** on all supported chips
3. **IDF v6.x** on newer chips
4. **Cross-driver compatibility**: MCPWM ↔ RMT

**Automated testing:**
```python
# Example test matrix
test_matrix = [
    {"chip": "esp32", "idf": "4.4", "driver": "mcpwm"},
    {"chip": "esp32", "idf": "4.4", "driver": "rmt"},
    {"chip": "esp32-c3", "idf": "5.3", "driver": "rmt"},
    # ... all combinations
]
```

### Migration Plan

**Step-by-step migration:**
1. Add new unified driver alongside existing
2. Test new driver thoroughly
3. Gradually migrate examples and tests
4. Remove old implementations after validation
5. Update documentation

**Backward compatibility:**
- Maintain existing API
- Auto-migrate configuration
- Provide migration scripts
- Detailed change logs

### Expected Outcomes

- **Code Reduction**: 60-70% reduction in ESP32-specific code
- **Simplified Build**: Clear driver selection
- **Better Performance**: Optimized for each chip variant
- **Easier Maintenance**: Single implementation per feature
- **Future Compatibility**: Ready for new ESP32 chips and IDF versions