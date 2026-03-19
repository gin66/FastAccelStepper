# ESP32 StepperDemo with WebUI - Design Document

## 1. Overview

### 1.1 Purpose
A modern, web-based interface for controlling stepper motors on ESP32 platforms. Unlike the traditional StepperDemo (which is memory-constrained and uses serial interface), this version leverages ESP32's capabilities to provide a rich browser-based UI with dynamic configuration.

### 1.2 Key Differentiators from StepperDemo
- **Dynamic Configuration**: Start with no steppers, add/configure on-the-fly
- **Web Interface**: Browser-based UI accessible via WiFi
- **Extended Pin Support**: Native GPIO + I2S expander (32 additional outputs)
- **Multiple Driver Types**: RMT, I2S_DIRECT, I2S_MUX
- **Persistent Configuration**: Save/load to LittleFS
- **Real-time Monitoring**: Live status updates via WebSocket
- **Move Sequences**: Load and execute pre-defined move sequences

### 1.3 Target Platform
- ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6
- **ESP-IDF v5.3+** (uses latest RMT driver with improved performance)
- Minimum 4MB Flash (with LittleFS partition)

---

## 2. Architecture

### 2.1 System Components

```
┌─────────────────────────────────────────────────────────────┐
│                      Web Browser (Client)                    │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ Dashboard│  │Pin Config│  │  Stepper │  │  Move    │   │
│  │          │  │   Panel  │  │  Control │  │ Sequencer│   │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘   │
└─────────────────────────────────────────────────────────────┘
                          │ HTTP + WebSocket
┌─────────────────────────────────────────────────────────────┐
│                     ESP32 Web Server                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │  HTTP Server │  │  WebSocket   │  │  REST API    │      │
│  │  (ESPAsync)  │  │   Server     │  │  Handler     │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              Configuration Manager                     │  │
│  │  - Pin definitions  - Stepper configs  - Sequences    │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │          FastAccelStepper Engine Integration          │  │
│  │  - RMT Driver    - I2S Direct    - I2S MUX           │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
  │  │ GPIO Manager │  │ I2S Expander │  │  File System │      │
│  │              │  │   Manager    │  │  (LittleFS)  │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Data Flow

1. **Configuration Flow**:
   ```
   Web UI → REST API → Config Manager → FAS Engine → Hardware
                    ↓
                LittleFS (persistent)
   ```

2. **Real-time Status Flow**:
   ```
   Hardware → FAS Engine → Status Manager → WebSocket → Web UI
   ```

3. **Move Sequence Flow**:
   ```
   Web UI → REST API → Sequence Manager → Command Queue → FAS Engine
   ```

---

## 3. Web Server Architecture

### 3.1 HTTP Endpoints

#### Configuration Endpoints
```
GET  /api/config                    - Get full configuration
POST /api/config                    - Update configuration
POST /api/config/save               - Save to LittleFS
GET  /api/config/load               - Load from LittleFS
DELETE /api/config                  - Reset to defaults

GET  /api/pins                      - Get all pin states
POST /api/pins/{pin}/toggle         - Toggle a GPIO pin
GET  /api/pins/{pin}                - Read single pin state
POST /api/pins/{pin}                - Set pin mode and value

GET  /api/i2s/expander              - Get I2S expander status
POST /api/i2s/expander/enable       - Enable I2S expander
POST /api/i2s/expander/disable      - Disable I2S expander
GET  /api/i2s/expander/pins         - Get I2S pin states
POST /api/i2s/expander/pins/{slot}  - Set I2S pin value
```

#### Stepper Endpoints
```
GET  /api/steppers                  - List all steppers
POST /api/steppers                  - Create new stepper
GET  /api/steppers/{id}             - Get stepper details
PUT  /api/steppers/{id}             - Update stepper config
DELETE /api/steppers/{id}           - Delete stepper

POST /api/steppers/{id}/move        - Move stepper
POST /api/steppers/{id}/moveTo      - Move to position
POST /api/steppers/{id}/run         - Run continuous
POST /api/steppers/{id}/stop        - Stop stepper
POST /api/steppers/{id}/enable      - Enable outputs
POST /api/steppers/{id}/disable     - Disable outputs
POST /api/steppers/{id}/position    - Set current position

GET  /api/steppers/{id}/status      - Get real-time status
```

#### Sequence Endpoints
```
GET  /api/sequences                 - List all sequences
POST /api/sequences                 - Create new sequence
GET  /api/sequences/{id}            - Get sequence details
PUT  /api/sequences/{id}            - Update sequence
DELETE /api/sequences/{id}          - Delete sequence
POST /api/sequences/{id}/execute    - Execute sequence
POST /api/sequences/{id}/stop       - Stop sequence execution
POST /api/sequences/upload          - Upload sequence file
```

### 3.2 WebSocket Events

#### Server → Client
```json
{
  "event": "stepper_status",
  "data": {
    "id": 0,
    "position": 1234,
    "speed": 1000,
    "running": true,
    "ramp_state": "ACCELERATE"
  }
}

{
  "event": "pin_change",
  "data": {
    "pin": 15,
    "value": 1
  }
}

{
  "event": "sequence_progress",
  "data": {
    "id": 0,
    "step": 5,
    "total": 10,
    "status": "running"
  }
}
```

#### Client → Server
```json
{
  "command": "subscribe",
  "topics": ["steppers", "pins", "sequences"]
}

{
  "command": "unsubscribe",
  "topics": ["steppers"]
}
```

---

## 4. Configuration Management

### 4.1 Data Structures

#### System Configuration
```cpp
struct SystemConfig {
  char wifi_ssid[32];
  char wifi_password[64];
  char hostname[32];
  bool ap_mode;                    // Access Point mode if true
  uint16_t web_server_port;
  uint16_t websocket_port;
  uint32_t status_update_interval_ms;
};
```

#### I2S Expander Configuration
```cpp
struct I2SExpanderConfig {
  bool enabled;
  uint8_t data_pin;
  uint8_t bclk_pin;
  uint8_t ws_pin;
  uint8_t num_chips;              // Number of 74HC595 chips in chain
  uint8_t pin_states[32];         // Current state of each pin
};
```

#### Pin Definition
```cpp
enum PinType {
  PIN_TYPE_NONE = 0,
  PIN_TYPE_GPIO_INPUT,
  PIN_TYPE_GPIO_OUTPUT,
  PIN_TYPE_I2S_OUTPUT
};

enum PinCapability {
  PIN_CAP_NONE = 0,
  PIN_CAP_INPUT = 1,
  PIN_CAP_OUTPUT = 2,
  PIN_CAP_ADC = 4,
  PIN_CAP_TOUCH = 8,
  PIN_CAP_RMT = 16,
  PIN_CAP_I2S = 32
};

struct PinDefinition {
  uint8_t pin_number;
  PinType type;
  uint8_t capabilities;           // Bitmask of PinCapability
  char name[16];                  // "GPIO15", "I2S_0", etc.
  bool is_reserved;               // System reserved (flash, etc.)
  bool is_assigned;               // Assigned to a stepper
};
```

#### Stepper Configuration
```cpp
enum DriverType {
  DRIVER_RMT,
  DRIVER_I2S_DIRECT,
  DRIVER_I2S_MUX
};

enum PinSource {
  PIN_SOURCE_NONE = 0,
  PIN_SOURCE_GPIO,
  PIN_SOURCE_I2S,
  PIN_SOURCE_EXTERNAL            // Via external callback
};

struct StepperPinConfig {
  PinSource source;
  uint8_t pin;                   // GPIO number or I2S slot
  bool active_low;
  uint16_t delay_us;             // For direction pin only
};

struct StepperConfig {
  uint8_t id;
  char name[32];
  
  // Step pin configuration
  DriverType driver;
  StepperPinConfig step_pin;
  
  // Direction pin configuration
  StepperPinConfig dir_pin;
  bool dir_high_counts_up;
  
  // Enable pin(s) configuration
  StepperPinConfig enable_pin_low;
  StepperPinConfig enable_pin_high;
  
  // Motion parameters
  uint32_t speed_us;
  uint32_t acceleration;
  uint32_t linear_accel_steps;
  uint32_t jump_start_steps;
  
  // Auto-enable settings
  bool auto_enable;
  uint32_t delay_to_enable_us;
  uint16_t delay_to_disable_ms;
  
  // Current state
  int32_t current_position;
  bool is_enabled;
};
```

#### Move Sequence
```cpp
enum MoveCommandType {
  MOVE_TYPE_MOVE,
  MOVE_TYPE_MOVE_TO,
  MOVE_TYPE_RUN_FORWARD,
  MOVE_TYPE_RUN_BACKWARD,
  MOVE_TYPE_STOP,
  MOVE_TYPE_WAIT,
  MOVE_TYPE_SET_SPEED,
  MOVE_TYPE_SET_ACCEL,
  MOVE_TYPE_SET_POSITION,
  MOVE_TYPE_ENABLE,
  MOVE_TYPE_DISABLE
};

struct MoveCommand {
  MoveCommandType type;
  uint8_t stepper_id;
  int32_t value1;                 // steps, position, speed, etc.
  int32_t value2;                 // acceleration, etc.
  uint32_t delay_ms;              // For WAIT command
};

struct MoveSequence {
  uint8_t id;
  char name[32];
  bool loop;
  uint8_t num_commands;
  MoveCommand commands[MAX_SEQUENCE_COMMANDS];
  bool is_executing;
  uint8_t current_command_index;
};
```

### 4.2 Configuration File Format (JSON)

#### /config/system.json
```json
{
  "wifi": {
    "ssid": "MyNetwork",
    "password": "MyPassword",
    "hostname": "stepper-demo"
  },
  "network": {
    "ap_mode": false,
    "port": 80,
    "ws_port": 81
  },
  "update_interval_ms": 100
}
```

#### /config/i2s.json
```json
{
  "enabled": true,
  "data_pin": 32,
  "bclk_pin": 33,
  "ws_pin": 14,
  "num_chips": 4
}
```

#### /config/steppers.json
```json
{
  "steppers": [
    {
      "id": 0,
      "name": "X-Axis",
      "driver": "RMT",
      "step_pin": {
        "source": "GPIO",
        "pin": 15,
        "active_low": false
      },
      "dir_pin": {
        "source": "GPIO",
        "pin": 18,
        "active_low": false,
        "delay_us": 0
      },
      "enable_pin_low": {
        "source": "I2S",
        "pin": 2,
        "active_low": true
      },
      "speed_us": 50,
      "acceleration": 10000,
      "auto_enable": true,
      "delay_to_enable_us": 1000,
      "delay_to_disable_ms": 100
    },
    {
      "id": 1,
      "name": "Y-Axis",
      "driver": "I2S_MUX",
      "step_pin": {
        "source": "I2S",
        "pin": 3
      },
      "dir_pin": {
        "source": "I2S",
        "pin": 4
      },
      "enable_pin_low": {
        "source": "I2S",
        "pin": 5,
        "active_low": true
      },
      "speed_us": 100,
      "acceleration": 5000
    }
  ]
}
```

#### /config/sequences.json
```json
{
  "sequences": [
    {
      "id": 0,
      "name": "Test Pattern",
      "loop": false,
      "commands": [
        {
          "type": "MOVE_TO",
          "stepper_id": 0,
          "value1": 1000
        },
        {
          "type": "WAIT",
          "delay_ms": 500
        },
        {
          "type": "MOVE_TO",
          "stepper_id": 0,
          "value1": 0
        },
        {
          "type": "SET_SPEED",
          "stepper_id": 1,
          "value1": 100
        },
        {
          "type": "MOVE",
          "stepper_id": 1,
          "value1": 2000
        }
      ]
    }
  ]
}
```

---

## 5. Web UI Design

### 5.1 Dashboard Page
- System status overview
- WiFi information
- Free heap memory
- Uptime
- Quick stepper status cards
- Pin monitor (collapsible)

### 5.2 Pin Configuration Panel
- Visual pin map (ESP32 pinout diagram)
- Pin mode selector (Input/Output/I2S)
- Toggle buttons for output pins
- Real-time value indicators
- Pin capability badges (ADC, Touch, RMT, I2S)
- I2S expander section (if enabled)

### 5.3 Stepper Management Page

#### Stepper List View
- Card-based layout showing all configured steppers
- Quick actions: Enable/Disable, Stop, Jog
- Status indicators: Running, Enabled, Error

#### Stepper Configuration Form
```
┌────────────────────────────────────────────────────┐
│ Stepper Configuration                               │
├────────────────────────────────────────────────────┤
│ Name: [__________________]                          │
│                                                     │
│ Driver Type:                                        │
│   ○ RMT (ESP32 Hardware)                           │
│   ○ I2S Direct (One per I2S peripheral)           │
│   ○ I2S MUX (Via 74HC595)                          │
│                                                     │
│ Step Pin:                                           │
│   Source: [GPIO ▼]  Pin: [___]                     │
│   □ Active Low                                      │
│                                                     │
│ Direction Pin:                                      │
│   Source: [GPIO ▼]  Pin: [___]                     │
│   □ Active Low  Delay (us): [____]                 │
│   □ High counts up                                  │
│                                                     │
│ Enable Pin (Low Active):                            │
│   Source: [I2S ▼]   Pin/Slot: [___]                │
│   ☑ Active Low                                      │
│                                                     │
│ Enable Pin (High Active):                           │
│   Source: [None ▼]                                 │
│                                                     │
│ Motion Parameters:                                  │
│   Speed (us/step): [______]                         │
│   Acceleration (steps/s²): [________]               │
│   Linear Accel Steps: [______]                      │
│   Jump Start Steps: [______]                        │
│                                                     │
│ Auto Enable:                                        │
│   ☑ Enable auto enable                              │
│   Delay to enable (us): [______]                    │
│   Delay to disable (ms): [____]                     │
│                                                     │
│ [Save] [Cancel] [Delete]                            │
└────────────────────────────────────────────────────┘
```

#### Stepper Control Panel
```
┌────────────────────────────────────────────────────┐
│ X-Axis - Status                                     │
├────────────────────────────────────────────────────┤
│ Position: [_________] [Set]                         │
│                                                     │
│ ▶ Move To: [______] [Go]                           │
│ ▶ Move By: [______] [Go]                           │
│                                                     │
│ [◄◄] [◄] [Stop] [►] [►►]                          │
│                                                     │
│ [Run FWD] [Run REV] [Keep Running]                 │
│                                                     │
│ Speed: [▲][▼]  Accel: [▲][▼]                       │
│                                                     │
│ [Enable] [Disable] [Auto Enable: ON]               │
│                                                     │
│ Status: RUNNING (Accelerating)                      │
│ Position: 1234  Speed: 5000 steps/s                │
│ Queue: 12 entries  Target: 10000                    │
└────────────────────────────────────────────────────┘
```

### 5.4 Move Sequencer Page

#### Sequence List
- Table of all sequences
- Actions: Edit, Delete, Execute, Duplicate

#### Sequence Editor
```
┌────────────────────────────────────────────────────┐
│ Sequence: Test Pattern                              │
├────────────────────────────────────────────────────┤
│ Name: [__________________]                          │
│ ☑ Loop continuously                                │
│                                                     │
│ Commands:                                           │
│ ┌──┬─────────┬────────┬───────┬────────┐          │
│ │# │Command  │Stepper │Value  │Actions │          │
│ ├──┼─────────┼────────┼───────┼────────┤          │
│ │1 │MOVE_TO  │X-Axis  │1000   │[▲][▼][X]│         │
│ │2 │WAIT     │-       │500ms  │[▲][▼][X]│         │
│ │3 │MOVE_TO  │X-Axis  │0      │[▲][▼][X]│         │
│ │4 │SET_SPEED│Y-Axis  │100    │[▲][▼][X]│         │
│ │5 │MOVE     │Y-Axis  │2000   │[▲][▼][X]│         │
│ └──┴─────────┴────────┴───────┴────────┘          │
│                                                     │
│ [+ Add Command] [Save] [Execute] [Stop]            │
└────────────────────────────────────────────────────┘
```

### 5.5 Settings Page
- WiFi configuration
- I2S expander setup
- System settings
- Import/Export configuration
- Factory reset

---

## 6. Implementation Details

### 6.1 GPIO Manager

#### Pin Discovery
```cpp
class GPIOPinManager {
private:
  PinDefinition pins[NUM_GPIO_PINS];
  
  void discoverPins() {
    for (int i = 0; i < NUM_GPIO_PINS; i++) {
      pins[i].pin_number = i;
      pins[i].capabilities = getPinCapabilities(i);
      pins[i].is_reserved = isPinReserved(i);
      pins[i].is_assigned = false;
      pins[i].type = PIN_TYPE_NONE;
    }
  }
  
  uint8_t getPinCapabilities(uint8_t pin) {
    uint8_t caps = PIN_CAP_NONE;
    
    // Check if pin can be input
    if (gpio_input_enabled(pin)) caps |= PIN_CAP_INPUT;
    
    // Check if pin can be output
    if (gpio_output_enabled(pin)) caps |= PIN_CAP_OUTPUT;
    
    // Check ADC capability
    if (adc1_channel_enabled(pin) || adc2_channel_enabled(pin)) {
      caps |= PIN_CAP_ADC;
    }
    
    // Check touch capability
    if (touch_channel_enabled(pin)) caps |= PIN_CAP_TOUCH;
    
    // Check RMT capability
    if (rmt_channel_enabled(pin)) caps |= PIN_CAP_RMT;
    
    // Check I2S capability
    if (i2s_channel_enabled(pin)) caps |= PIN_CAP_I2S;
    
    return caps;
  }
  
  bool isPinReserved(uint8_t pin) {
    // Check for flash pins, strapping pins, etc.
    return (pin >= 6 && pin <= 11) ||  // Flash
           (pin == 0) ||                // Boot strap
           (pin == 2) ||                // Boot strap
           (pin == 12) ||               // Boot strap
           (pin == 15);                 // Boot strap
  }
  
public:
  bool setPinMode(uint8_t pin, PinType type);
  bool setPinValue(uint8_t pin, bool value);
  bool getPinValue(uint8_t pin);
  bool togglePin(uint8_t pin);
  PinDefinition* getPinDefinition(uint8_t pin);
  std::vector<PinDefinition*> getAvailablePins(PinCapability cap);
};
```

### 6.2 I2S Expander Manager

```cpp
class I2SExpanderManager {
private:
  I2SExpanderConfig config;
  bool initialized;
  
public:
  bool init(uint8_t data_pin, uint8_t bclk_pin, uint8_t ws_pin, uint8_t num_chips);
  void deinit();
  
  bool setBit(uint8_t slot, bool value);
  bool getBit(uint8_t slot);
  void setAllBits(uint32_t value);
  uint32_t getAllBits();
  
  bool isInitialized() { return initialized; }
  uint8_t getNumPins() { return config.num_chips * 8; }
};
```

### 6.3 Configuration Manager

```cpp
class ConfigManager {
private:
  SystemConfig system_config;
  I2SExpanderConfig i2s_config;
  std::vector<StepperConfig> steppers;
  std::vector<MoveSequence> sequences;
  
  bool loadFromFile(const char* path, JsonDocument& doc);
  bool saveToFile(const char* path, const JsonDocument& doc);
  
public:
  bool begin();
  
  // System config
  bool loadSystemConfig();
  bool saveSystemConfig();
  SystemConfig& getSystemConfig() { return system_config; }
  
  // I2S config
  bool loadI2SConfig();
  bool saveI2SConfig();
  I2SExpanderConfig& getI2SConfig() { return i2s_config; }
  
  // Stepper configs
  bool loadStepperConfigs();
  bool saveStepperConfigs();
  bool addStepper(const StepperConfig& config);
  bool updateStepper(uint8_t id, const StepperConfig& config);
  bool deleteStepper(uint8_t id);
  StepperConfig* getStepper(uint8_t id);
  std::vector<StepperConfig>& getAllSteppers() { return steppers; }
  
  // Sequence configs
  bool loadSequences();
  bool saveSequences();
  bool addSequence(const MoveSequence& seq);
  bool updateSequence(uint8_t id, const MoveSequence& seq);
  bool deleteSequence(uint8_t id);
  MoveSequence* getSequence(uint8_t id);
  std::vector<MoveSequence>& getAllSequences() { return sequences; }
  
  // Export/Import all
  bool exportAll(const char* path);
  bool importAll(const char* path);
  bool factoryReset();
};
```

### 6.4 Stepper Controller

```cpp
class StepperController {
private:
  FastAccelStepperEngine* engine;
  FastAccelStepper* steppers[MAX_STEPPER];
  StepperConfig configs[MAX_STEPPER];
  
  bool applyConfig(uint8_t id);
  bool setupExternalPin(StepperPinConfig& pin_config);
  
public:
  bool begin();
  
  bool createStepper(const StepperConfig& config);
  bool updateStepper(uint8_t id, const StepperConfig& config);
  bool deleteStepper(uint8_t id);
  
  // Movement commands
  int8_t move(uint8_t id, int32_t steps);
  int8_t moveTo(uint8_t id, int32_t position);
  int8_t runForward(uint8_t id);
  int8_t runBackward(uint8_t id);
  void stopMove(uint8_t id);
  void forceStop(uint8_t id);
  
  // Enable/Disable
  bool enableOutputs(uint8_t id);
  bool disableOutputs(uint8_t id);
  void setAutoEnable(uint8_t id, bool auto_enable);
  
  // Position
  int32_t getCurrentPosition(uint8_t id);
  void setCurrentPosition(uint8_t id, int32_t position);
  
  // Speed and acceleration
  int8_t setSpeed(uint8_t id, uint32_t speed_us);
  int8_t setAcceleration(uint8_t id, uint32_t accel);
  void applySpeedAcceleration(uint8_t id);
  
  // Status
  bool isRunning(uint8_t id);
  StepperStatus getStatus(uint8_t id);
};
```

### 6.5 Sequence Executor

```cpp
class SequenceExecutor {
private:
  ConfigManager* config_manager;
  StepperController* stepper_controller;
  bool executing;
  uint8_t current_sequence_id;
  uint32_t last_command_time;
  
  bool executeCommand(const MoveCommand& cmd);
  
public:
  void begin(ConfigManager* cfg, StepperController* ctrl);
  
  bool startSequence(uint8_t id);
  void stopSequence();
  void pauseSequence();
  void resumeSequence();
  
  void update();  // Call from loop()
  
  bool isExecuting() { return executing; }
  uint8_t getCurrentSequence() { return current_sequence_id; }
  float getProgress();
};
```

### 6.6 WebSocket Status Broadcaster

```cpp
class StatusBroadcaster {
private:
  AsyncWebSocket* ws;
  uint32_t last_update;
  uint32_t update_interval;
  
  bool subscribed_to_steppers;
  bool subscribed_to_pins;
  bool subscribed_to_sequences;
  
  void sendStepperStatus();
  void sendPinChanges();
  void sendSequenceProgress();
  
public:
  void begin(AsyncWebSocket* websocket, uint32_t interval_ms);
  void update();  // Call from loop()
  
  void onConnect(AsyncWebSocketClient* client);
  void onDisconnect(AsyncWebSocketClient* client);
  void onMessage(AsyncWebSocketClient* client, AwsFrameInfo* info, uint8_t* data, size_t len);
};
```

### 6.7 REST API Handler

```cpp
class RestAPIHandler {
private:
  ConfigManager* config_manager;
  StepperController* stepper_controller;
  SequenceExecutor* sequence_executor;
  GPIOPinManager* gpio_manager;
  I2SExpanderManager* i2s_manager;
  
  void handleGetConfig(AsyncWebServerRequest* request);
  void handleUpdateConfig(AsyncWebServerRequest* request, uint8_t* data, size_t len);
  void handleSaveConfig(AsyncWebServerRequest* request);
  
  void handleGetPins(AsyncWebServerRequest* request);
  void handleTogglePin(AsyncWebServerRequest* request);
  void handleSetPin(AsyncWebServerRequest* request, uint8_t* data, size_t len);
  
  void handleGetSteppers(AsyncWebServerRequest* request);
  void handleCreateStepper(AsyncWebServerRequest* request, uint8_t* data, size_t len);
  void handleGetStepper(AsyncWebServerRequest* request);
  void handleUpdateStepper(AsyncWebServerRequest* request, uint8_t* data, size_t len);
  void handleDeleteStepper(AsyncWebServerRequest* request);
  
  void handleStepperMove(AsyncWebServerRequest* request, uint8_t* data, size_t len);
  void handleStepperStop(AsyncWebServerRequest* request);
  
  void handleGetSequences(AsyncWebServerRequest* request);
  void handleCreateSequence(AsyncWebServerRequest* request, uint8_t* data, size_t len);
  void handleExecuteSequence(AsyncWebServerRequest* request);
  
public:
  void begin(AsyncWebServer* server, 
             ConfigManager* cfg,
             StepperController* stepper,
             SequenceExecutor* seq,
             GPIOPinManager* gpio,
             I2SExpanderManager* i2s);
};
```

---

## 7. External Pin Callback

For I2S pins and external pins, we need to implement the external callback:

```cpp
bool externalPinCallback(uint8_t pin, uint8_t value) {
  // Check if pin is in I2S range
  if (pin & PIN_I2S_FLAG) {
    uint8_t slot = pin & ~PIN_I2S_FLAG;
    return i2s_manager->setBit(slot, value);
  }
  
  // Check if pin is external (via PIN_EXTERNAL_FLAG)
  if (pin & PIN_EXTERNAL_FLAG) {
    // Handle external pins via GPIO manager
    return gpio_manager->setPinValue(pin & ~PIN_EXTERNAL_FLAG, value);
  }
  
  return false;
}

// In setup:
engine.setExternalCallForPin(externalPinCallback);
```

---

## 8. File Structure

```
examples/Esp32StepperDemo/
├── DESIGN.md                          # This document
├── Esp32StepperDemo.ino              # Main Arduino sketch
├── src/
│   ├── config/
│   │   ├── ConfigManager.h
│   │   ├── ConfigManager.cpp
│   │   ├── SystemConfig.h
│   │   ├── StepperConfig.h
│   │   └── SequenceConfig.h
│   ├── managers/
│   │   ├── GPIOPinManager.h
│   │   ├── GPIOPinManager.cpp
│   │   ├── I2SExpanderManager.h
│   │   └── I2SExpanderManager.cpp
│   ├── stepper/
│   │   ├── StepperController.h
│   │   ├── StepperController.cpp
│   │   ├── SequenceExecutor.h
│   │   └── SequenceExecutor.cpp
│   ├── web/
│   │   ├── WebServer.h
│   │   ├── WebServer.cpp
│   │   ├── RestAPIHandler.h
│   │   ├── RestAPIHandler.cpp
│   │   ├── WebSocketHandler.h
│   │   ├── WebSocketHandler.cpp
│   │   └── StatusBroadcaster.h
│   └── utils/
│       ├── JsonHelpers.h
│       └── PinHelpers.h
├── data/
│   ├── index.html                     # Main HTML file
│   ├── css/
│   │   ├── style.css
│   │   └── dashboard.css
│   ├── js/
│   │   ├── app.js
│   │   ├── api.js
│   │   ├── websocket.js
│   │   ├── stepper-control.js
│   │   ├── pin-manager.js
│   │   └── sequence-editor.js
│   └── assets/
│       └── esp32-pinout.svg           # Pin diagram
└── platformio.ini                     # PlatformIO configuration
```

---

## 9. Dependencies

### Arduino Libraries
- **ESPAsyncWebServer** - Async HTTP and WebSocket server
- **ArduinoJson** - JSON parsing and creation
- **LittleFS** - File system for configuration storage (SPIFFS deprecated in ESP-IDF v5.x)

### PlatformIO Configuration
```ini
[env:esp32]
platform = espressif32
board = esp32dev
framework = arduino, espidf
platform_packages = 
    framework-espidf @ https://github.com/espressif/esp-idf.git#v5.3
lib_deps = 
    espressif/esp32-camera
    me-no-dev/ESP Async WebServer
    bblanchon/ArduinoJson
monitor_speed = 115200
board_build.partitions = huge_app.csv
build_flags = 
    -DCORE_DEBUG_LEVEL=ARDUHAL_LOG_LEVEL_INFO
```

---

## 10. Development Phases

### Phase 1: Core Infrastructure (Week 1-2)
- [ ] Basic web server setup
- [ ] LittleFS file system integration
- [ ] Configuration manager skeleton
- [ ] JSON serialization/deserialization
- [ ] Basic REST API framework

### Phase 2: Pin Management (Week 3)
- [ ] GPIO pin discovery
- [ ] Pin capability detection
- [ ] I2S expander initialization
- [ ] Pin toggle/read functionality
- [ ] Pin state persistence

### Phase 3: Stepper Management (Week 4-5)
- [ ] Stepper creation/deletion
- [ ] Multiple driver support (RMT, I2S_DIRECT, I2S_MUX)
- [ ] External pin callback integration
- [ ] Basic movement commands
- [ ] Speed/acceleration control
- [ ] Stepper configuration persistence

### Phase 4: Web UI Development (Week 6-7)
- [ ] Dashboard page
- [ ] Pin configuration interface
- [ ] Stepper management UI
- [ ] Stepper control panel
- [ ] Real-time status updates via WebSocket

### Phase 5: Sequence System (Week 8)
- [ ] Move sequence definition
- [ ] Sequence storage
- [ ] Sequence execution engine
- [ ] Sequence editor UI
- [ ] Sequence progress tracking

### Phase 6: Polish and Testing (Week 9-10)
- [ ] Error handling
- [ ] Configuration import/export
- [ ] Factory reset
- [ ] Performance optimization
- [ ] Documentation
- [ ] Testing on multiple ESP32 variants

---

## 11. Security Considerations

### 11.1 WiFi Security
- Support WPA2/WPA3
- Optional password protection for web interface
- Disable AP mode after initial setup

### 11.2 Input Validation
- Validate all JSON inputs
- Sanitize pin numbers (prevent access to reserved pins)
- Limit stepper count and sequence size
- Rate limiting on REST API

### 11.3 Safe Defaults
- All outputs disabled on boot
- Steppers stopped on boot
- Safe pin states on errors

---

## 12. Performance Targets

### 12.1 Response Times
- REST API response: < 50ms
- WebSocket status update: 100ms interval (configurable)
- Pin toggle latency: < 5ms
- Stepper command latency: < 10ms

### 12.2 Resource Usage
- Heap usage: < 100KB (leaving > 200KB free on ESP32)
- LittleFS usage: < 64KB for configuration
- Flash usage: < 2MB total

### 12.3 Concurrency
- Support 5+ simultaneous WebSocket clients
- Handle concurrent stepper operations
- Non-blocking web server

---

## 13. Future Enhancements

### 13.1 Potential Features
- **OTA Updates**: Over-the-air firmware updates
- **MQTT Integration**: Connect to MQTT broker for IoT integration
- **G-code Interpreter**: Parse and execute G-code files
- **Multi-axis Coordination**: Synchronized multi-axis moves
- **Homing Routines**: Automated homing sequences
- **Temperature Monitoring**: Monitor driver temperature
- **Current Control**: Dynamic current adjustment (if driver supports)
- **Backup/Restore**: Cloud backup of configurations
- **User Authentication**: Multi-user support with permissions

### 13.2 Extended Hardware Support
- **LCD/OLED Display**: Local status display
- **Rotary Encoder**: Local manual control
- **SD Card**: Extended storage for sequences
- **CAN Bus**: Multi-controller coordination

---

## 14. Compatibility Notes

### 14.1 ESP32 Variants
- **ESP32**: Full support (RMT + I2S)
- **ESP32-S2**: Full support (RMT + I2S)
- **ESP32-S3**: Full support (RMT + I2S)
- **ESP32-C3**: Limited (RMT only, no I2S on some variants)
- **ESP32-C6**: Limited (RMT only)

### 14.2 IDF Version Compatibility
- **ESP-IDF v4.x**: MCPWM/PCNT + RMT (ESP32, ESP32-S3), RMT only (ESP32-S2, ESP32-C3)
- **ESP-IDF v5.3+**: MCPWM/PCNT (ESP32, ESP32-S3, ESP32-C6, ESP32-H2) + RMT + I2S
- **I2S Support**: Via new I2S driver API in v5.3+

### 14.3 Arduino ESP32 Core
- **v3.0+**: Required (for ESP-IDF v5.3 support)

---

## 15. Troubleshooting Guide

### 15.1 Common Issues

#### I2S Expander Not Working
- Check wiring (DATA, BCLK, WS pins)
- Verify 74HC595 power supply
- Check pin conflicts with other peripherals
- Ensure I2S is not used by other libraries

#### Stepper Not Moving
- Verify step pin is configured correctly
- Check driver type matches hardware
- Ensure enable pin is properly configured
- Verify speed/acceleration are set

#### Web Interface Not Accessible
- Check WiFi connection
- Verify hostname/IP address
- Check firewall settings
- Ensure AP mode is off (or configured correctly)

#### Configuration Not Saving
- Check LittleFS is mounted
- Verify sufficient space available
- Check file permissions
- Look for JSON syntax errors

### 15.2 Debug Mode
Enable verbose logging for troubleshooting:
```cpp
#define DEBUG_SERIAL Serial
#define DEBUG_LEVEL DEBUG_VERBOSE
```

---

## 16. API Reference

### 16.1 Complete REST API Documentation

See separate file: [API_REFERENCE.md](API_REFERENCE.md)

### 16.2 WebSocket Protocol Documentation

See separate file: [WEBSOCKET_PROTOCOL.md](WEBSOCKET_PROTOCOL.md)

---

## 17. License

This example code is provided under the same license as FastAccelStepper library.

---

## 18. Credits

- FastAccelStepper Library: [https://github.com/gin66/FastAccelStepper](https://github.com/gin66/FastAccelStepper)
- ESPAsyncWebServer: [https://github.com/me-no-dev/ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)
- ArduinoJson: [https://github.com/bblanchon/ArduinoJson](https://github.com/bblanchon/ArduinoJson)

---

## Appendix A: Pin Mapping Reference

### ESP32-WROOM-32 Pin Capabilities

| Pin | Input | Output | ADC | Touch | RMT | I2S | Notes |
|-----|-------|--------|-----|-------|-----|-----|-------|
| 0   | ✓     | ✓      | -   | -     | ✓   | -   | Boot (strapping) |
| 1   | ✓     | ✓      | -   | -     | ✓   | -   | TX0 |
| 2   | ✓     | ✓      | -   | -     | ✓   | -   | Boot (strapping) |
| 3   | ✓     | ✓      | -   | -     | ✓   | -   | RX0 |
| 4   | ✓     | ✓      | -   | ✓     | ✓   | ✓   | |
| 5   | ✓     | ✓      | -   | -     | ✓   | ✓   | Boot (strapping) |
| 6-11| -     | -      | -   | -     | -   | -   | Flash (reserved) |
| 12  | ✓     | ✓      | ✓   | ✓     | ✓   | ✓   | Boot (strapping) |
| 13  | ✓     | ✓      | ✓   | ✓     | ✓   | ✓   | |
| 14  | ✓     | ✓      | ✓   | ✓     | ✓   | ✓   | |
| 15  | ✓     | ✓      | ✓   | ✓     | ✓   | ✓   | Boot (strapping) |
| 16-17| ✓   | ✓      | ✓   | -     | ✓   | -   | |
| 18  | ✓     | ✓      | -   | -     | ✓   | ✓   | |
| 19  | ✓     | ✓      | -   | -     | ✓   | ✓   | |
| 21  | ✓     | ✓      | -   | -     | ✓   | ✓   | I2C SDA |
| 22  | ✓     | ✓      | -   | -     | ✓   | ✓   | I2C SCL |
| 23  | ✓     | ✓      | -   | -     | ✓   | ✓   | |
| 25  | ✓     | ✓      | ✓   | -     | ✓   | ✓   | DAC1 |
| 26  | ✓     | ✓      | ✓   | -     | ✓   | ✓   | DAC2 |
| 27  | ✓     | ✓      | ✓   | ✓     | ✓   | ✓   | |
| 32-39| ✓   | ✓      | ✓   | ✓     | ✓   | ✓   | Input only (32-39) |

---

## Appendix B: Move Sequence Command Reference

| Command | Parameters | Description |
|---------|------------|-------------|
| MOVE | stepper_id, steps | Move stepper by relative steps |
| MOVE_TO | stepper_id, position | Move stepper to absolute position |
| RUN_FORWARD | stepper_id | Start continuous forward motion |
| RUN_BACKWARD | stepper_id | Start continuous backward motion |
| STOP | stepper_id | Stop stepper with deceleration |
| FORCE_STOP | stepper_id | Stop stepper immediately |
| WAIT | delay_ms | Wait for specified milliseconds |
| WAIT_FOR_STOP | stepper_id | Wait until stepper stops |
| SET_SPEED | stepper_id, speed_us | Set stepper speed |
| SET_ACCEL | stepper_id, accel | Set stepper acceleration |
| SET_POSITION | stepper_id, position | Set current position |
| ENABLE | stepper_id | Enable stepper outputs |
| DISABLE | stepper_id | Disable stepper outputs |
| KEEP_RUNNING | stepper_id | Set keep running flag |

---

## Appendix C: Configuration Migration from StepperDemo

For users migrating from the old StepperDemo, a migration tool can convert hardcoded configurations:

```cpp
// Old StepperDemo config
const struct stepper_config_s stepper_config[MAX_STEPPER] = {
  {
    step : 17,
    enable_low_active : 26,
    enable_high_active : PIN_UNDEFINED,
    direction : 18,
    dir_change_delay : 0,
    direction_high_count_up : true,
    auto_enable : true,
    on_delay_us : 50,
    off_delay_ms : 1000
  }
};

// Converted to WebUI config (JSON)
{
  "steppers": [
    {
      "id": 0,
      "name": "Stepper 0",
      "driver": "RMT",
      "step_pin": {"source": "GPIO", "pin": 17},
      "dir_pin": {"source": "GPIO", "pin": 18, "delay_us": 0},
      "enable_pin_low": {"source": "GPIO", "pin": 26, "active_low": true},
      "speed_us": 50,
      "acceleration": 10000,
      "auto_enable": true,
      "delay_to_enable_us": 50,
      "delay_to_disable_ms": 1000
    }
  ]
}
```

---

## Appendix D: Web UI Screenshots

*Placeholder for UI mockups*

### Dashboard
*Mockup of main dashboard showing system status and stepper cards*

### Pin Manager
*Mockup of pin configuration interface with ESP32 pinout diagram*

### Stepper Control
*Mockup of stepper configuration and control panel*

### Sequence Editor
*Mockup of move sequence editor interface*

---

## Appendix E: Hardware Setup Examples

### E.1 Basic RMT Stepper
```
ESP32 GPIO15 ──► Step Pin
ESP32 GPIO18 ──► Dir Pin
ESP32 GPIO26 ──► Enable Pin (active low)
```

### E.2 I2S Direct Stepper
```
ESP32 GPIO15 ──► I2S Data (to stepper driver step input)
ESP32 GPIO18 ──► Dir Pin (GPIO)
ESP32 GPIO26 ──► Enable Pin (GPIO)
```

### E.3 I2S MUX with 74HC595
```
ESP32 GPIO32 ──► 74HC595 Data (DS)
ESP32 GPIO33 ──► 74HC595 Clock (SH_CP)
ESP32 GPIO14 ──► 74HC595 Latch (ST_CP)

74HC595 Outputs:
  Q0 ──► Stepper 0 Step
  Q1 ──► Stepper 0 Dir
  Q2 ──► Stepper 0 Enable
  Q3 ──► Stepper 1 Step
  Q4 ──► Stepper 1 Dir
  Q5 ──► Stepper 1 Enable
  ...
```

---

**End of Design Document**
