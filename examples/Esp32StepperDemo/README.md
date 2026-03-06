# ESP32 StepperDemo with WebUI

A modern, browser-based interface for controlling stepper motors on ESP32 platforms using the FastAccelStepper library.

## Overview

This is a comprehensive web-based stepper motor controller designed specifically for ESP32 microcontrollers. Unlike the traditional serial-based StepperDemo, this version provides:

- **Dynamic Configuration**: Add, modify, and remove steppers on-the-fly
- **Web Interface**: Full browser-based control panel accessible via WiFi
- **Extended Pin Support**: Native GPIO plus I2S expander support (32 additional outputs via 74HC595)
- **Multiple Driver Types**: Support for RMT, I2S_DIRECT, and I2S_MUX drivers
- **Persistent Configuration**: Save and load configurations to flash memory
- **Move Sequences**: Create, save, and execute complex move sequences

## Documentation

- **[DESIGN.md](DESIGN.md)** - Complete design document with architecture, API specifications, and implementation details

## Key Features

### Pin Management
- Automatic GPIO pin discovery and capability detection
- Support for input, output, ADC, touch, RMT, and I2S pins
- I2S expander support for up to 32 additional output pins
- Pin toggle and read functionality
- Visual pin mapping interface

### Stepper Control
- Dynamic stepper creation and configuration
- Support for all ESP32 driver types:
  - **RMT**: ESP32's remote control peripheral (recommended for most uses)
  - **I2S_DIRECT**: Direct I2S output for high-speed single steppers
  - **I2S_MUX**: Multiple steppers via 74HC595 shift register chain
- Flexible pin assignment:
  - Step, direction, and enable pins can be GPIO or I2S
  - Support for external pin callback for custom hardware
  - Configurable polarity for all pins
- Full motion control:
  - Absolute and relative positioning
  - Continuous running (forward/backward)
  - Speed and acceleration control
  - Linear acceleration and jump start
  - Auto-enable with configurable delays

### Web Interface
- Real-time status updates via WebSocket
- Responsive dashboard with system status
- Intuitive stepper control panels
- Visual pin configuration
- Move sequence editor
- Configuration import/export

### Configuration Management
- JSON-based configuration files stored in SPIFFS
- Separate configuration for:
  - System settings (WiFi, network)
  - I2S expander
  - Stepper definitions
  - Move sequences
- Factory reset capability

## Quick Start

### Hardware Requirements
- ESP32, ESP32-S2, ESP32-S3, ESP32-C3, or ESP32-C6
- Stepper motor driver (e.g., A4988, DRV8825, TMC2209)
- Optional: 74HC595 shift register for I2S expander

### Software Requirements
- PlatformIO or Arduino IDE
- ESP-IDF v4.4+ or Arduino ESP32 Core v2.0+
- Required libraries:
  - ESPAsyncWebServer
  - ArduinoJson
  - SPIFFS or LittleFS

### Installation

1. Clone or download this example
2. Install required libraries via PlatformIO or Library Manager
3. Open `Esp32StepperDemo.ino` in your IDE
4. Configure your WiFi credentials in the code or via the web interface
5. Upload to your ESP32
6. Access the web interface at `http://<esp32-ip-address>`

## Architecture

See [DESIGN.md](DESIGN.md) for complete architectural details.

```
┌─────────────────┐
│  Web Browser    │
│  (Client UI)    │
└────────┬────────┘
         │ HTTP + WebSocket
         ▼
┌─────────────────┐
│   Web Server    │
│  (ESPAsyncWeb)  │
├─────────────────┤
│  REST API       │
│  WebSocket      │
│  Config Manager │
├─────────────────┤
│ FastAccelStepper│
│     Engine      │
├─────────────────┤
│ Hardware Layer  │
│ - GPIO Manager  │
│ - I2S Expander  │
│ - RMT/I2S Driver│
└─────────────────┘
```

## Configuration Files

Configuration is stored in SPIFFS as JSON files:

```
/config/
  ├── system.json       # WiFi and network settings
  ├── i2s.json          # I2S expander configuration
  ├── steppers.json     # Stepper definitions
  └── sequences.json    # Move sequences
```

## Example Configurations

### Basic RMT Stepper

```json
{
  "id": 0,
  "name": "X-Axis",
  "driver": "RMT",
  "step_pin": {"source": "GPIO", "pin": 15},
  "dir_pin": {"source": "GPIO", "pin": 18},
  "enable_pin_low": {"source": "GPIO", "pin": 26, "active_low": true},
  "speed_us": 50,
  "acceleration": 10000,
  "auto_enable": true
}
```

### I2S MUX Stepper with Mixed Pin Sources

```json
{
  "id": 1,
  "name": "Y-Axis",
  "driver": "I2S_MUX",
  "step_pin": {"source": "I2S", "pin": 3},
  "dir_pin": {"source": "I2S", "pin": 4},
  "enable_pin_low": {"source": "I2S", "pin": 5, "active_low": true},
  "speed_us": 100,
  "acceleration": 5000
}
```

## API Overview

### REST Endpoints

- `GET /api/steppers` - List all steppers
- `POST /api/steppers` - Create new stepper
- `GET /api/steppers/{id}` - Get stepper details
- `PUT /api/steppers/{id}` - Update stepper configuration
- `DELETE /api/steppers/{id}` - Delete stepper
- `POST /api/steppers/{id}/move` - Move stepper
- `POST /api/steppers/{id}/stop` - Stop stepper

See [DESIGN.md](DESIGN.md) for complete API documentation.

### WebSocket Events

Real-time status updates are pushed via WebSocket:

```json
{
  "event": "stepper_status",
  "data": {
    "id": 0,
    "position": 1234,
    "speed": 1000,
    "running": true
  }
}
```

## Hardware Wiring Examples

### Basic RMT Setup
```
ESP32 GPIO15 → Step (stepper driver)
ESP32 GPIO18 → Dir
ESP32 GPIO26 → Enable (active low)
```

### I2S Expander with 74HC595
```
ESP32 GPIO32 → 74HC595 Data
ESP32 GPIO33 → 74HC595 Clock
ESP32 GPIO14 → 74HC595 Latch

74HC595 Q0-Q7 → Stepper 0 pins (step, dir, enable, ...)
74HC595 Q8-Q15 → Stepper 1 pins
...
```

## Development Status

This project is currently in the **design phase**. Implementation will follow the phases outlined in [DESIGN.md](DESIGN.md).

### Roadmap
- [ ] Phase 1: Core Infrastructure
- [ ] Phase 2: Pin Management
- [ ] Phase 3: Stepper Management
- [ ] Phase 4: Web UI Development
- [ ] Phase 5: Sequence System
- [ ] Phase 6: Polish and Testing

## Contributing

Contributions are welcome! Please ensure any code follows the FastAccelStepper code style guidelines (see `AGENTS.md` in the repository root).

## License

This example is provided under the same license as the FastAccelStepper library.

## Related Resources

- [FastAccelStepper Library](https://github.com/gin66/FastAccelStepper)
- [FastAccelStepper Documentation](https://github.com/gin66/FastAccelStepper/wiki)
- [Traditional StepperDemo](../StepperDemo/)
- [I2S Usage Example](../UsageEspI2S/)

## Support

For issues and feature requests, please use the [FastAccelStepper GitHub Issues](https://github.com/gin66/FastAccelStepper/issues) page.
