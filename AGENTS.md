# FastAccelStepper Agent Guidelines

## Build Commands
- **PC-based tests**: `make -C extras/tests/pc_based`
- **SimAVR tests**: `make -C extras/tests/simavr_based`
- **PlatformIO builds**: `bash extras/scripts/build-platformio.sh [target]`
- **ESP-IDF**: Standard CMake build process
- **Format code**: `bash extras/scripts/format_code.sh`
- **Memory footprint**: `bash extras/scripts/measure_library_size.sh` (generates memory_report.md)

## Test Commands
- **Run all PC tests**: `make test` in `extras/tests/pc_based/`
- **Run single PC test**: `make test_XX` where XX is test number (01-17)
- **Run SimAVR tests**: `make test` in `extras/tests/simavr_based/`
- **ESP32 HW tests**: Run scripts in `extras/tests/esp32_hw_based/`

## Code Style Guidelines
- **Formatting**: clang-format with Google style, 80 column limit
- **Variables**: camelCase (e.g., `stepperCount`)
- **Functions**: snake_case (e.g., `get_current_position()`)
- **Classes**: PascalCase (e.g., `FastAccelStepper`)
- **Macros**: ALL_CAPS_WITH_UNDERSCORES (e.g., `F_CPU`)
- **Includes**: Use `#include <stdint.h>` for standard types
- **Error handling**: Return error codes or NULL for failures
- **Comments**: None in implementation code
- **Architecture-specific code**: Use preprocessor conditionals like `#if defined(ARDUINO_ARCH_ESP32)`

## Development Workflow
1. Format code with clang-format before committing
2. Run PC-based tests to verify changes
3. Test on target platforms using PlatformIO
4. Use cppcheck for style validation with naming rules

## Quality Checks
- **Linting**: `cppcheck --enable=style --rule-file=extras/scripts/naming_rules.xml --language=c++ --std=c++11 src/`
- **Type checking**: Compile with `-Werror -Wall` flags
- **Memory safety**: Check for NULL pointer usage and proper error handling

## Platform-Specific Notes
- **AVR**: Limited to specific timer pins (Timer 1: pins 9/10, Timer 3: pins 5/2/3, Timer 5: pins 46/45/44)
- **ESP32**: Supports up to 14 stepper motors, use RMT driver for better performance
- **ESP32-IDF versions**: v4.x uses MCPWM/PCNT, v5.3+ uses RMT only
- **Pico**: GPIO pins 0-31, up to 8 steppers (4 with Arduino framework)

## Common Patterns
- **Stepper initialization**: Always check return value of `stepperConnectToPin()`
- **Auto-enable**: Use `setAutoEnable(true)` with appropriate delays
- **Direction pins**: Set direction delay if needed with `setDirectionPin(pin, delay_us)`
- **Position tracking**: Use 32-bit signed integers, handles overflow correctly