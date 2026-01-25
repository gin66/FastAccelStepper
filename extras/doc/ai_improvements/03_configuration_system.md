# Configuration System Overhaul

## Current State Analysis

### Problem
FastAccelStepper uses complex preprocessor macros for platform configuration, making the code hard to understand and maintain:

**Current macro-based configuration:**
```cpp
// Examples from fas_arch/common_esp32.h
#define SUPPORT_ESP32
#define SUPPORT_EXTERNAL_DIRECTION_PIN
#define SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING
#define QUEUE_LEN 32
#define TICKS_PER_S 16000000L
#define MIN_CMD_TICKS (TICKS_PER_S / 5000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
```

**Issues with current approach:**
1. **Macro spaghetti**: Complex conditional compilation
2. **No type safety**: Macros are text substitution
3. **Hard to debug**: Preprocessor errors are cryptic
4. **Limited validation**: No compile-time checking
5. **Poor IDE support**: Macros confuse code analysis

### Configuration Complexity

**Platform-specific configuration files:**
- `fas_arch/arduino_avr.h` (80+ lines of macros)
- `fas_arch/common_esp32.h` (50+ lines of macros)
- `fas_arch/common_esp32_idf4.h` (separate file)
- `fas_arch/common_esp32_idf5.h` (separate file)
- `fas_arch/common_esp32_idf6.h` (separate file)

**Configuration propagation:**
```cpp
// Complex macro chains
#if defined(ARDUINO_ARCH_ESP32)
    #include "fas_arch/arduino_esp32.h"
#elif defined(ESP_PLATFORM)
    #include "fas_arch/espidf_esp32.h"
#elif defined(ARDUINO_ARCH_SAM)
    // ... and so on
#endif
```

## Proposed Solution

### Type-Safe Configuration System

Replace macros with `constexpr` configuration structures:

```cpp
// src/config/PlatformConfig.h
#pragma once

#include <cstdint>
#include <type_traits>

struct PlatformConfig {
    // Core timing
    uint32_t ticks_per_second;
    uint32_t min_command_ticks;
    uint32_t min_dir_delay_us;
    uint32_t max_dir_delay_us;
    
    // Queue configuration
    uint16_t queue_length;
    uint8_t max_steppers;
    uint8_t num_queues;
    
    // Feature flags (type-safe enums)
    enum class Feature : uint8_t {
        ExternalDirectionPin = 1 << 0,
        PulseCounter = 1 << 1,
        RMTDriver = 1 << 2,
        MCPWMDriver = 1 << 3,
        AutoEnable = 1 << 4,
        LinearAcceleration = 1 << 5,
        JumpStart = 1 << 6
    };
    
    uint8_t features;
    
    // Validation
    constexpr bool isValid() const {
        return ticks_per_second > 0 &&
               queue_length > 0 &&
               max_steppers > 0 &&
               num_queues > 0;
    }
    
    // Feature checking
    constexpr bool hasFeature(Feature f) const {
        return (features & static_cast<uint8_t>(f)) != 0;
    }
    
    // Derived values (computed at compile time)
    constexpr uint32_t us_to_ticks(uint32_t us) const {
        return (us * ticks_per_second) / 1000000;
    }
    
    constexpr uint32_t ticks_to_us(uint32_t ticks) const {
        return (ticks * 1000000) / ticks_per_second;
    }
};
```

### Platform-Specific Configurations

**AVR Configuration:**
```cpp
// src/config/avr/AVRConfig.h
#pragma once

#include "PlatformConfig.h"

namespace avr {
    // ATmega328P configuration
    constexpr PlatformConfig atmega328p = {
        .ticks_per_second = 16000000UL,
        .min_command_ticks = 16000000UL / 25000,  // 640 ticks
        .min_dir_delay_us = 40,
        .max_dir_delay_us = 65535 / (16000000UL / 1000000),
        .queue_length = 16,
        .max_steppers = 2,
        .num_queues = 2,
        .features = static_cast<uint8_t>(
            PlatformConfig::Feature::ExternalDirectionPin |
            PlatformConfig::Feature::AutoEnable
        )
    };
    
    // ATmega2560 configuration
    constexpr PlatformConfig atmega2560 = {
        .ticks_per_second = 16000000UL,
        .min_command_ticks = 16000000UL / 25000,
        .min_dir_delay_us = 40,
        .max_dir_delay_us = 65535 / (16000000UL / 1000000),
        .queue_length = 16,
        .max_steppers = 3,
        .num_queues = 3,
        .features = static_cast<uint8_t>(
            PlatformConfig::Feature::ExternalDirectionPin |
            PlatformConfig::Feature::AutoEnable
        )
    };
}
```

**ESP32 Configuration:**
```cpp
// src/config/esp32/ESP32Config.h
#pragma once

#include "PlatformConfig.h"
#include "IDFVersion.h"

namespace esp32 {
    // Common ESP32 configuration
    template<typename IDFVersion>
    constexpr PlatformConfig common_config = {
        .ticks_per_second = 16000000UL,
        .min_command_ticks = 16000000UL / 5000,  // 3200 ticks
        .min_dir_delay_us = 200,
        .max_dir_delay_us = 65535 / (16000000UL / 1000000),
        .queue_length = 32,
        .max_steppers = IDFVersion::MAX_STEPPERS,
        .num_queues = IDFVersion::NUM_QUEUES,
        .features = static_cast<uint8_t>(
            PlatformConfig::Feature::ExternalDirectionPin |
            PlatformConfig::Feature::PulseCounter |
            PlatformConfig::Feature::AutoEnable |
            PlatformConfig::Feature::LinearAcceleration |
            PlatformConfig::Feature::JumpStart
        )
    };
    
    // Add RMT or MCPWM feature based on selection
    constexpr auto add_rmt_feature(PlatformConfig config) {
        config.features |= static_cast<uint8_t>(PlatformConfig::Feature::RMTDriver);
        return config;
    }
    
    constexpr auto add_mcpwm_feature(PlatformConfig config) {
        config.features |= static_cast<uint8_t>(PlatformConfig::Feature::MCPWMDriver);
        return config;
    }
}
```

### Configuration Detection System

```cpp
// src/config/ConfigDetector.h
#pragma once

#include "PlatformConfig.h"

#ifdef ARDUINO_ARCH_AVR
    #include "avr/AVRConfig.h"
    #include "avr/ChipDetector.h"
#endif

#ifdef ARDUINO_ARCH_ESP32
    #include "esp32/ESP32Config.h"
    #include "esp32/ChipDetector.h"
#endif

// ... other platforms

class ConfigDetector {
public:
    // Get configuration for current platform
    static constexpr PlatformConfig detect() {
        #ifdef ARDUINO_ARCH_AVR
            return avr::ChipDetector::getConfig();
        #elif defined(ARDUINO_ARCH_ESP32)
            return esp32::ChipDetector::getConfig();
        #elif defined(ARDUINO_ARCH_SAM)
            return sam::ChipDetector::getConfig();
        #elif defined(PICO_RP2040)
            return pico::ChipDetector::getConfig();
        #else
            #error "Unsupported platform"
        #endif
    }
    
    // Runtime configuration access
    static const PlatformConfig& get() {
        static constexpr PlatformConfig config = detect();
        static_assert(config.isValid(), "Invalid platform configuration");
        return config;
    }
};
```

### Compile-Time Validation

```cpp
// src/config/ConfigValidator.h
#pragma once

#include "PlatformConfig.h"

template<PlatformConfig Config>
class ConfigValidator {
    static_assert(Config.ticks_per_second > 0, 
                  "Ticks per second must be positive");
    
    static_assert(Config.queue_length > 0 && 
                  (Config.queue_length & (Config.queue_length - 1)) == 0,
                  "Queue length must be power of two");
    
    static_assert(Config.max_steppers > 0 && 
                  Config.max_steppers <= 255,
                  "Max steppers must be 1-255");
    
    static_assert(Config.min_command_ticks > 0,
                  "Min command ticks must be positive");
    
    // Queue length validation for mask operations
    static_assert(Config.queue_length <= 256,
                  "Queue length too large for 8-bit index");
                  
public:
    constexpr static bool valid = true;
};
```

### Benefits

1. **Type Safety**: Compile-time type checking
2. **Better Debugging**: Clear error messages
3. **IDE Support**: Proper code completion and navigation
4. **Compile-Time Validation**: Catch errors early
5. **Cleaner Code**: No macro spaghetti
6. **Runtime Access**: Can inspect configuration at runtime

### Implementation Strategy

**Phase 1: Configuration Structure**
1. Define `PlatformConfig` structure
2. Create platform-specific configurations
3. Add compile-time validation

**Phase 2: Migration**
1. Replace macros with `constexpr` configurations
2. Update platform detection
3. Fix compilation errors

**Phase 3: Integration**
1. Update all source files to use new system
2. Add backward compatibility layer
3. Comprehensive testing

### Performance Considerations

**Zero-cost abstractions:**
- `constexpr` evaluations happen at compile time
- No runtime overhead for configuration access
- Inlined functions for derived values
- Static dispatch for platform-specific code

**Memory impact:**
- Configuration stored in read-only memory
- No dynamic allocation
- Minimal memory footprint

### Testing Strategy

**Compile-time tests:**
```cpp
// Example compile-time test
static_assert(ConfigValidator<avr::atmega328p>::valid,
              "AVR configuration must be valid");

static_assert(avr::atmega328p.us_to_ticks(1000) == 16000,
              "US to ticks conversion must be correct");
```

**Runtime tests:**
```cpp
TEST(PlatformConfig, AVRConfiguration) {
    auto config = ConfigDetector::get();
    EXPECT_EQ(config.ticks_per_second, 16000000UL);
    EXPECT_EQ(config.queue_length, 16);
    EXPECT_TRUE(config.hasFeature(PlatformConfig::Feature::AutoEnable));
}
```

### Migration Plan

**Step 1: Add new system alongside old**
- Create configuration structures
- Add detection system
- Keep macros for backward compatibility

**Step 2: Gradual migration**
- Update one platform at a time
- Test thoroughly after each migration
- Update dependent code

**Step 3: Remove old system**
- Remove macro definitions
- Clean up conditional compilation
- Update documentation

### Backward Compatibility

**Temporary compatibility layer:**
```cpp
// Compatibility macros during transition
#ifdef USING_NEW_CONFIG
    #define TICKS_PER_S ConfigDetector::get().ticks_per_second
    #define QUEUE_LEN ConfigDetector::get().queue_length
#else
    // Old macro definitions
#endif
```

### Expected Outcomes

- **Cleaner Code**: Eliminate macro spaghetti
- **Better Maintenance**: Clear configuration structure
- **Improved Debugging**: Type-safe error messages
- **Enhanced Safety**: Compile-time validation
- **Future-Proof**: Easy to add new platforms
- **Performance Maintained**: Zero-cost abstractions