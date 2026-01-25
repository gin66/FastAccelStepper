# Motion Mathematics Simplification

## Current State Analysis

### Problem
The motion mathematics in FastAccelStepper is overly complex with multiple calculation versions and custom log2 representation:

**Complexity Issues:**
1. **Multiple calculation versions**: 8 different `calculate_ticks_vX()` functions
2. **Custom log2 representation**: Complex `Log2Representation` class with approximations
3. **Platform-specific optimizations**: Different math for AVR vs ESP32
4. **Hard to understand**: Non-standard mathematical representations
5. **Testing difficulty**: Complex to verify correctness

### Current Mathematical Complexity

**Log2Representation complexity:**
```cpp
// From Log2Representation.h
typedef int16_t pmf_logarithmic;
#define LOG2_CONST_INVALID ((pmf_logarithmic)0x8000)
#define LOG2_CONST_MAX ((pmf_logarithmic)0x7fff)

// Complex approximations
#define log2_pow_div_3(x) ((x) / 2 + (x) / 4 + (x) / 16 + (x) / 256 + 1) / 2
#define log2_pow_2_div_3(x) ((x) - log2_pow_div_3(x))
#define log2_pow_3_div_2(x) ((x) + (x) / 2)
```

**Multiple calculation versions:**
```cpp
// From RampCalculator.h
#ifdef TEST_TIMING
uint32_t calculate_ticks_v1(uint32_t steps, float acceleration);
uint32_t calculate_ticks_v2(uint32_t steps, float acceleration);
uint32_t calculate_ticks_v3(uint32_t steps, float pre_calc);
uint32_t calculate_ticks_v4(uint32_t steps, uint32_t acceleration);
uint32_t calculate_ticks_v5(uint32_t steps, pmf_logarithmic pre_calc);
uint32_t calculate_ticks_v6(uint32_t steps, pmf_logarithmic pre_calc);
uint32_t calculate_ticks_v7(uint32_t steps, pmf_logarithmic pre_calc);
uint32_t calculate_ticks_v8(uint32_t steps, pmf_logarithmic pre_calc);
#endif
```

**Platform-specific math macros:**
```cpp
// From RampCalculator.h
#if (TICKS_PER_S == 16000000L)
    #define LOG2_TICKS_PER_S LOG2_CONST_16E6
    #define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 LOG2_CONST_16E6_DIV_SQRT_OF_2
    #define LOG2_ACCEL_FACTOR LOG2_CONST_128E12
    #define US_TO_TICKS(u32) ((u32) * 16)
    #define TICKS_TO_US(u32) ((u32) / 16)
#elif (TICKS_PER_S == 21000000L)
    #define LOG2_TICKS_PER_S LOG2_CONST_21E6
    #define LOG2_TICKS_PER_S_DIV_SQRT_OF_2 LOG2_CONST_21E6_DIV_SQRT_OF_2
    #define LOG2_ACCEL_FACTOR LOG2_CONST_2205E11
    #define US_TO_TICKS(u32) ((u32) * 21)
    #define TICKS_TO_US(u32) ((u32) / 21)
#else
    // Dynamic calculation with potential overflow
#endif
```

## Proposed Solution

### Simplified Motion Mathematics

Create a clear, maintainable motion mathematics system:

```cpp
// src/math/MotionMath.h
#pragma once

#include <cstdint>
#include <type_traits>

namespace motion_math {
    
// Fixed-point arithmetic for embedded systems
template<int FractionalBits = 16>
class FixedPoint {
private:
    int32_t value_;
    
public:
    constexpr FixedPoint() : value_(0) {}
    constexpr explicit FixedPoint(int32_t val) : value_(val) {}
    constexpr FixedPoint(float f) : value_(static_cast<int32_t>(f * (1 << FractionalBits))) {}
    
    // Basic arithmetic
    constexpr FixedPoint operator+(FixedPoint other) const { return FixedPoint(value_ + other.value_); }
    constexpr FixedPoint operator-(FixedPoint other) const { return FixedPoint(value_ - other.value_); }
    constexpr FixedPoint operator*(FixedPoint other) const { 
        return FixedPoint((static_cast<int64_t>(value_) * other.value_) >> FractionalBits);
    }
    constexpr FixedPoint operator/(FixedPoint other) const {
        return FixedPoint((static_cast<int64_t>(value_) << FractionalBits) / other.value_);
    }
    
    // Conversions
    constexpr int32_t to_int() const { return value_ >> FractionalBits; }
    constexpr float to_float() const { return static_cast<float>(value_) / (1 << FractionalBits); }
    constexpr int32_t raw() const { return value_; }
    
    // Constants
    static constexpr FixedPoint zero() { return FixedPoint(0); }
    static constexpr FixedPoint one() { return FixedPoint(1 << FractionalBits); }
    static constexpr FixedPoint half() { return FixedPoint(1 << (FractionalBits - 1)); }
};

// Motion calculations using clear physics formulas
class MotionCalculator {
public:
    struct MotionParameters {
        uint32_t acceleration_hz_s;  // steps/s²
        uint32_t max_speed_hz;       // steps/s
        uint32_t linear_accel_steps; // steps for linear acceleration phase
        uint32_t jump_start_steps;   // steps to jump start from standstill
    };
    
    struct MotionState {
        int32_t position;           // current position (steps)
        int32_t speed_hz;           // current speed (steps/s)
        int32_t acceleration_hz_s;  // current acceleration (steps/s²)
        uint32_t steps_remaining;   // steps to target
    };
    
    // Calculate time to accelerate from v0 to v1
    static uint32_t acceleration_time(uint32_t v0_hz, uint32_t v1_hz, 
                                     uint32_t acceleration_hz_s) {
        if (acceleration_hz_s == 0) return 0;
        uint32_t delta_v = (v1_hz > v0_hz) ? (v1_hz - v0_hz) : (v0_hz - v1_hz);
        return (delta_v * 1000000) / acceleration_hz_s;  // microseconds
    }
    
    // Calculate distance to accelerate from v0 to v1
    static uint32_t acceleration_distance(uint32_t v0_hz, uint32_t v1_hz,
                                         uint32_t acceleration_hz_s) {
        if (acceleration_hz_s == 0) return 0;
        // s = (v1² - v0²) / (2a)
        uint64_t v0_sq = static_cast<uint64_t>(v0_hz) * v0_hz;
        uint64_t v1_sq = static_cast<uint64_t>(v1_hz) * v1_hz;
        uint64_t numerator = (v1_sq > v0_sq) ? (v1_sq - v0_sq) : (v0_sq - v1_sq);
        return static_cast<uint32_t>(numerator / (2 * acceleration_hz_s));
    }
    
    // Calculate trapezoidal profile
    static bool calculate_trapezoidal_profile(
        uint32_t distance_steps,
        const MotionParameters& params,
        uint32_t& acceleration_steps,
        uint32_t& constant_speed_steps,
        uint32_t& deceleration_steps,
        uint32_t& acceleration_time_us,
        uint32_t& constant_speed_time_us,
        uint32_t& deceleration_time_us);
    
    // Calculate S-curve profile (for linear acceleration)
    static bool calculate_scurve_profile(
        uint32_t distance_steps,
        const MotionParameters& params,
        uint32_t& linear_accel_steps,
        uint32_t& constant_accel_steps,
        uint32_t& linear_decel_steps);
};
```

### Platform-Optimized Implementations

```cpp
// src/math/platform/AVRMotionMath.h
#pragma once

#include "MotionMath.h"

// AVR-optimized implementations using 16-bit math
namespace avr {
    class MotionMath : public motion_math::MotionCalculator {
    public:
        // 16-bit optimized version for AVR
        static uint16_t acceleration_time_16(uint16_t v0_hz, uint16_t v1_hz,
                                            uint16_t acceleration_hz_s) {
            if (acceleration_hz_s == 0) return 0;
            uint16_t delta_v = (v1_hz > v0_hz) ? (v1_hz - v0_hz) : (v0_hz - v1_hz);
            return (static_cast<uint32_t>(delta_v) * 1000) / acceleration_hz_s;
        }
        
        // Lookup table for common values
        static uint16_t ticks_to_us_lut(uint16_t ticks) {
            // Pre-calculated lookup table for 16MHz clock
            static const uint16_t lut[] = {
                // ticks -> microseconds
                16,  // 1 tick = 0.0625us, but we store us
                32,  // 2 ticks
                // ... up to max needed
            };
            if (ticks < sizeof(lut)/sizeof(lut[0])) {
                return lut[ticks];
            }
            return ticks / 16;  // Fallback
        }
    };
}
```

```cpp
// src/math/platform/ESP32MotionMath.h
#pragma once

#include "MotionMath.h"

// ESP32-optimized implementations using 32-bit math
namespace esp32 {
    class MotionMath : public motion_math::MotionCalculator {
    public:
        // 32-bit optimized version for ESP32
        static uint32_t acceleration_time_32(uint32_t v0_hz, uint32_t v1_hz,
                                            uint32_t acceleration_hz_s) {
            if (acceleration_hz_s == 0) return 0;
            uint32_t delta_v = (v1_hz > v0_hz) ? (v1_hz - v0_hz) : (v0_hz - v1_hz);
            return (static_cast<uint64_t>(delta_v) * 1000000) / acceleration_hz_s;
        }
        
        // Use hardware multiplication for better performance
        static uint32_t fast_multiply(uint32_t a, uint32_t b) {
            // ESP32 has hardware multiplier
            return a * b;
        }
    };
}
```

### Clear Physics-Based Calculations

```cpp
// src/math/PhysicsBasedMotion.h
#pragma once

#include <cstdint>
#include <cmath>

class PhysicsBasedMotion {
public:
    // Clear physics formulas with proper units
    
    // Kinematic equation: v² = u² + 2as
    static uint32_t final_velocity_squared(uint32_t initial_velocity_hz,
                                          uint32_t acceleration_hz_s,
                                          uint32_t distance_steps) {
        // v² = u² + 2as
        uint64_t u_sq = static_cast<uint64_t>(initial_velocity_hz) * initial_velocity_hz;
        uint64_t term = 2 * static_cast<uint64_t>(acceleration_hz_s) * distance_steps;
        return static_cast<uint32_t>(u_sq + term);
    }
    
    // Time to travel distance with constant acceleration
    static uint32_t time_with_constant_acceleration(uint32_t initial_velocity_hz,
                                                   uint32_t acceleration_hz_s,
                                                   uint32_t distance_steps) {
        if (acceleration_hz_s == 0) {
            // Constant velocity
            return (distance_steps * 1000000) / initial_velocity_hz;
        }
        
        // Solve quadratic: s = ut + ½at²
        // t = [-u ± sqrt(u² + 2as)] / a
        uint64_t discriminant = static_cast<uint64_t>(initial_velocity_hz) * initial_velocity_hz +
                               2 * static_cast<uint64_t>(acceleration_hz_s) * distance_steps;
        
        // Use integer square root approximation
        uint32_t sqrt_val = integer_sqrt(discriminant);
        uint32_t numerator = sqrt_val - initial_velocity_hz;
        
        return (numerator * 1000000) / acceleration_hz_s;
    }
    
    // Integer square root (for embedded)
    static uint32_t integer_sqrt(uint64_t n) {
        if (n < 2) return static_cast<uint32_t>(n);
        
        uint64_t small = integer_sqrt(n >> 2) << 1;
        uint64_t large = small + 1;
        return static_cast<uint32_t>((large * large > n) ? small : large);
    }
    
    // Linear acceleration (S-curve) calculations
    static uint32_t linear_acceleration_velocity(uint32_t acceleration_hz_s,
                                                uint32_t linear_steps,
                                                uint32_t current_step) {
        if (linear_steps == 0 || current_step == 0) {
            return 0;
        }
        
        // During linear acceleration phase:
        // a(t) = (a_max / t_accel) * t
        // v(t) = ½ * (a_max / t_accel) * t²
        
        // Convert to fixed-point for accuracy
        uint64_t step_sq = static_cast<uint64_t>(current_step) * current_step;
        uint64_t numerator = step_sq * acceleration_hz_s;
        uint64_t denominator = 2 * static_cast<uint64_t>(linear_steps);
        
        // v = sqrt(1.5 * a * s) approximation for linear phase
        // Use pre-calculated constants for common values
        return static_cast<uint32_t>(numerator / denominator);
    }
};
```

### Benefits

1. **Clear Physics**: Standard kinematic equations
2. **Maintainable Code**: No custom log2 representation
3. **Testable**: Standard math easier to verify
4. **Platform Optimized**: Specialized implementations where needed
5. **Documented Formulas**: Clear mathematical basis

### Implementation Strategy

**Phase 1: New Math Foundation**
1. Create clear physics-based motion calculations
2. Implement fixed-point arithmetic for embedded
3. Add platform-optimized versions

**Phase 2: Migration**
1. Replace log2 calculations with new system
2. Update RampGenerator to use new math
3. Test thoroughly for correctness

**Phase 3: Optimization**
1. Add platform-specific optimizations
2. Implement lookup tables for common values
3. Profile and optimize hot paths

### Performance Considerations

**Embedded optimizations:**
- **Fixed-point arithmetic**: Faster than floating point on embedded
- **Lookup tables**: Pre-calculated values for common cases
- **Platform-specific math**: Use hardware multipliers where available
- **Integer approximations**: Fast integer square root

**Memory optimization:**
- **Compile-time constants**: Pre-calculated where possible
- **Small lookup tables**: Cache-friendly sizes
- **Stack allocation**: No heap allocation in math functions

### Testing Strategy

**Unit tests for mathematical correctness:**
```cpp
TEST(MotionMath, AccelerationTime) {
    // v = u + at => t = (v - u)/a
    EXPECT_EQ(MotionMath::acceleration_time(0, 1000, 100), 10000); // 10ms
    EXPECT_EQ(MotionMath::acceleration_time(1000, 0, 100), 10000); // 10ms
}

TEST(MotionMath, AccelerationDistance) {
    // v² = u² + 2as => s = (v² - u²)/(2a)
    EXPECT_EQ(MotionMath::acceleration_distance(0, 1000, 100), 5000); // 5000 steps
}
```

**Numerical accuracy tests:**
```cpp
TEST(FixedPoint, Arithmetic) {
    FixedPoint<16> a(1.5f);
    FixedPoint<16> b(2.0f);
    
    EXPECT_NEAR((a * b).to_float(), 3.0f, 0.001f);
    EXPECT_NEAR((a / b).to_float(), 0.75f, 0.001f);
}
```

### Migration Plan

**Step 1: Add new math system alongside old**
- Implement new motion mathematics
- Keep old system for backward compatibility
- Add validation to ensure same results

**Step 2: Gradual replacement**
- Replace one calculation at a time
- Test equivalence at each step
- Update dependent code

**Step 3: Remove old system**
- Remove Log2Representation
- Clean up old calculation functions
- Update documentation

### Expected Outcomes

- **Simplified Code**: Clear physics instead of custom math
- **Better Maintainability**: Standard mathematical concepts
- **Improved Accuracy**: Proper fixed-point arithmetic
- **Maintained Performance**: Platform-optimized implementations
- **Easier Testing**: Standard math easier to verify
- **Better Documentation**: Clear mathematical basis