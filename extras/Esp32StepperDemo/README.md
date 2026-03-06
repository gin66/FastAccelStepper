# ESP32 StepperDemo with WebUI

A modern, web-based interface for controlling stepper motors on ESP32 using the FastAccelStepper library.

## Status

**Current Status: Core Implementation Complete**

✅ Implemented:
- WiFi configuration and connection
- Configuration management (load/save to LittleFS)
- Web server with REST API
- WebSocket for real-time status updates
- Basic HTML UI for monitoring and control
- Stepper command API

🚧 Planned/Partial:
- Sequence execution engine
- JSON body parsing for POST/PUT endpoints (currently uses parameters)
- Advanced UI features
- I2S expander support
- Automatic stepper initialization from config

## Features

- **Web-based UI**: Control steppers from any browser
- **Real-time updates**: WebSocket-based status updates
- **Configuration management**: JSON-based config files stored in LittleFS
- **WiFi setup**: Easy WiFi configuration via serial or AP mode
- **REST API**: Full REST API for integration with other systems
- **Multi-stepper support**: Control up to 14 steppers on ESP32

## Target Platform

- ESP-IDF v5.3+
- Arduino framework for ESP32
- Tested with pioarduino platform 53.03.11

## Quick Links

- [Design Document](DESIGN.md) - Complete technical design
- [README.md](../README.md) - Project overview (parent directory)
- [platformio.ini](platformio.ini) - PlatformIO configuration

## Directory Structure

```
Esp32StepperDemo/
├── DESIGN.md              # Complete design document
├── README.md              # This file
├── platformio.ini         # PlatformIO configuration
├── src/
│   ├── main.cpp          # Main entry point
│   ├── config/           # Configuration management
│   │   ├── ConfigManager.h/.cpp
│   │   ├── SystemConfig.h
│   │   ├── StepperConfig.h
│   │   ├── SequenceConfig.h
│   │   └── I2SExpanderConfig.h
│   └── web/              # Web server
│       ├── WebServer.h/.cpp
├── data/                  # File system data
│   └── index.html        # Web UI
└── lib/                   # Local libraries (symlink to main library)
```

## Building

```bash
# Create symlink to FastAccelStepper library
make

# Build for ESP32 with ESP-IDF v5.3
pio run -e esp32_pioarduino_V53_03_11

# Build and upload
pio run -e esp32_pioarduino_V53_03_11 -t upload

# Monitor serial output
pio device monitor
```

## Initial Setup

1. **First boot**: The device will enter WiFi configuration mode
   - Connect via serial (115200 baud) to enter WiFi credentials
   - Or use AP mode (SSID: "ESP32-Stepper", password: "stepper123")

2. **Access Web UI**: 
   - Once connected, open browser to http://[device-ip]
   - The UI allows basic stepper control and monitoring

## Memory Usage

- RAM: ~47KB (14.5% of 327KB)
- Flash: ~1.15MB (88.1% of 1.31MB)

## Development

See [DESIGN.md](DESIGN.md) for implementation details and future development phases.
