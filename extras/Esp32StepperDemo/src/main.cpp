/**
 * ESP32 StepperDemo with WebUI
 *
 * A modern, web-based interface for controlling stepper motors on ESP32
 * using the FastAccelStepper library.
 *
 * Target Platform: ESP-IDF v5.3+
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include "FastAccelStepper.h"
#include "config/ConfigManager.h"
#include "web/WebServer.h"
#include "managers/GPIOPinManager.h"
#include "managers/I2SPinManager.h"

#define VERSION "0.1.0-dev"
#define NAMESPACE "stepper_demo"

#define STATUS_LED 2

#define PREF_SSID "wifi_ssid"
#define PREF_PASSWORD "wifi_password"
#define PREF_HOSTNAME "hostname"

FastAccelStepperEngine stepper_engine = FastAccelStepperEngine();
FastAccelStepper* stepper[MAX_STEPPER];

ConfigManager* configManager = nullptr;
WebServerManager* webServer = nullptr;
GPIOPinManager* gpioManager = nullptr;
I2SPinManager* i2sManager = nullptr;

bool externalPinCallback(uint8_t pin, uint8_t value) {
  if (i2sManager && i2sManager->isInitialized() && pin < I2S_NUM_PINS) {
    return i2sManager->setPinValue(pin, value == HIGH);
  }
  return false;
}

void initializeSteppers() {
  Serial.println("Initializing steppers from config...");

  StepperConfig* configs = configManager->getAllSteppers();
  int initialized = 0;

  for (int i = 0; i < MAX_STEPPERS; i++) {
    if (configs[i].id == 255) continue;

    StepperConfig& cfg = configs[i];

    bool isI2SStep = (cfg.step_pin.source == PIN_SOURCE_I2S);
    bool isI2SDir = (cfg.dir_pin.source == PIN_SOURCE_I2S);
    bool isI2SEnableLow = (cfg.enable_pin_low.source == PIN_SOURCE_I2S);
    bool isI2SEnableHigh = (cfg.enable_pin_high.source == PIN_SOURCE_I2S);

    if (cfg.step_pin.pin == 255) {
      Serial.printf("  Stepper %d: Invalid step pin, skipping\n", cfg.id);
      continue;
    }

    if ((isI2SStep || isI2SDir || isI2SEnableLow || isI2SEnableHigh) &&
        (i2sManager == nullptr || !i2sManager->isInitialized())) {
      Serial.printf(
          "  Stepper %d: Uses I2S pins but I2S not initialized, skipping\n",
          cfg.id);
      continue;
    }

    Serial.printf("  Stepper %d: %s (step=%s%d, dir=%s%d, driver=%s)\n", cfg.id,
                  cfg.name, isI2SStep ? "I2S:" : "GPIO:", cfg.step_pin.pin,
                  isI2SDir ? "I2S:" : "GPIO:", cfg.dir_pin.pin,
                  cfg.driver == STEPPER_DRIVER_I2S_MUX      ? "I2S_MUX"
                  : cfg.driver == STEPPER_DRIVER_I2S_DIRECT ? "I2S_DIRECT"
                                                            : "RMT");

    FastAccelStepper* s = nullptr;

    if (cfg.driver == STEPPER_DRIVER_I2S_MUX) {
      uint8_t stepPin = cfg.step_pin.pin;
      if (isI2SStep) {
        stepPin |= PIN_I2S_FLAG;
      }
      s = stepper_engine.stepperConnectToPin(stepPin, DRIVER_I2S_MUX);
    } else if (cfg.driver == STEPPER_DRIVER_I2S_DIRECT) {
      uint8_t stepPin = cfg.step_pin.pin;
      if (isI2SStep) {
        stepPin |= PIN_I2S_FLAG;
      }
      s = stepper_engine.stepperConnectToPin(stepPin, DRIVER_I2S_DIRECT);
    } else {
      if (!isI2SStep) {
        s = stepper_engine.stepperConnectToPin(cfg.step_pin.pin, DRIVER_RMT);
      }
    }

    if (!s) {
      Serial.printf("    Failed to connect stepper\n");
      continue;
    }

    if (cfg.dir_pin.pin != 255) {
      if (isI2SDir) {
        s->setDirectionPin(cfg.dir_pin.pin, cfg.dir_high_counts_up,
                           cfg.dir_pin.delay_us);
        i2sManager->assignPinToStepper(cfg.dir_pin.pin, cfg.id);
      } else {
        s->setDirectionPin(cfg.dir_pin.pin, cfg.dir_high_counts_up,
                           cfg.dir_pin.delay_us);
        gpioManager->assignPinToStepper(cfg.dir_pin.pin, cfg.id);
      }
    }

    if (isI2SStep) {
      i2sManager->assignPinToStepper(cfg.step_pin.pin, cfg.id);
    } else {
      gpioManager->assignPinToStepper(cfg.step_pin.pin, cfg.id);
    }

    s->setSpeedInUs(cfg.speed_us);
    s->setAcceleration(cfg.acceleration);

    if (cfg.auto_enable) {
      s->setAutoEnable(true);
      s->setDelayToEnable(cfg.delay_to_enable_us);
      s->setDelayToDisable(cfg.delay_to_disable_ms);
    }

    if (cfg.enable_pin_low.pin != 255) {
      if (isI2SEnableLow) {
        s->setEnablePin(cfg.enable_pin_low.pin, true);
        i2sManager->assignPinToStepper(cfg.enable_pin_low.pin, cfg.id);
      } else {
        s->setEnablePin(cfg.enable_pin_low.pin, true);
        gpioManager->assignPinToStepper(cfg.enable_pin_low.pin, cfg.id);
      }
    }

    if (cfg.enable_pin_high.pin != 255) {
      if (isI2SEnableHigh) {
        s->setEnablePin(cfg.enable_pin_high.pin, false);
        i2sManager->assignPinToStepper(cfg.enable_pin_high.pin, cfg.id);
      } else {
        s->setEnablePin(cfg.enable_pin_high.pin, false);
        gpioManager->assignPinToStepper(cfg.enable_pin_high.pin, cfg.id);
      }
    }

    s->setCurrentPosition(cfg.current_position);

    stepper[cfg.id] = s;
    initialized++;

    Serial.printf("    OK: speed=%luus, accel=%lu\n", cfg.speed_us,
                  cfg.acceleration);
  }

  Serial.printf("Initialized %d steppers\n", initialized);
}

enum WiFiState {
  WIFI_NOT_CONFIGURED,
  WIFI_CONNECTING,
  WIFI_CONNECTED,
  WIFI_AP_MODE
};

WiFiState wifiState = WIFI_NOT_CONFIGURED;
unsigned long lastWifiAttempt = 0;
String wifiSSID;
String wifiPassword;
String hostname;
String inputBuffer;
String readStringFromSerial(const char* prompt) {
  Serial.print(prompt);
  Serial.flush();

  inputBuffer = "";
  unsigned long timeout = millis() + 30000;  // 30 second timeout

  while (millis() < timeout) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        break;
      }
      inputBuffer += c;
    }
    delay(10);
  }

  inputBuffer.trim();
  return inputBuffer;
}
void saveWiFiConfig(const String& ssid, const String& password,
                    const String& host) {
  Preferences prefs;
  prefs.begin(NAMESPACE, false);
  prefs.putString(PREF_SSID, ssid);
  prefs.putString(PREF_PASSWORD, password);
  prefs.putString(PREF_HOSTNAME, host);
  prefs.end();

  Serial.println("WiFi configuration saved!");
}
bool loadWiFiConfig() {
  Preferences prefs;
  prefs.begin(NAMESPACE, true);
  wifiSSID = prefs.getString(PREF_SSID, "");
  wifiPassword = prefs.getString(PREF_PASSWORD, "");
  hostname = prefs.getString(PREF_HOSTNAME, "");
  prefs.end();

  return wifiSSID.length() > 0;
}
void configureWiFiViaSerial() {
  Serial.println();
  Serial.println("=====================================");
  Serial.println("WiFi Configuration Required");
  Serial.println("=====================================");
  Serial.println();
  Serial.println("No WiFi credentials found in NVM.");
  Serial.println("Please enter WiFi credentials via serial console.");
  Serial.println("(30 second timeout for each input)");
  Serial.println();

  wifiSSID = readStringFromSerial("Enter WiFi SSID (or 'skip' for AP mode): ");

  if (wifiSSID.length() == 0 || wifiSSID.equalsIgnoreCase("skip")) {
    Serial.println("Starting in Access Point mode...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP32-Stepper", "stepper123");
    wifiState = WIFI_AP_MODE;
    hostname = "esp32-stepper";
    saveWiFiConfig("", "", hostname);
    return;
  }

  wifiPassword = readStringFromSerial("Enter WiFi Password: ");
  hostname = readStringFromSerial(
      "Enter Hostname (or press Enter for 'esp32-stepper'): ");

  if (hostname.length() == 0) {
    hostname = "esp32-stepper";
  }

  saveWiFiConfig(wifiSSID, wifiPassword, hostname);

  Serial.println();
  Serial.println("Configuration saved! Connecting to WiFi...");
}
void setupWiFi() {
  if (loadWiFiConfig()) {
    Serial.println("Found WiFi configuration in NVM:");
    Serial.printf("  SSID: %s\n", wifiSSID.c_str());
    Serial.printf("  Hostname: %s\n", hostname.c_str());

    WiFi.mode(WIFI_STA);
    if (hostname.length() > 0) {
      WiFi.setHostname(hostname.c_str());
    }
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    wifiState = WIFI_CONNECTING;
  } else {
    Serial.println("No WiFi configuration found in NVM");
    configureWiFiViaSerial();
  }
}
void checkWiFiConnection() {
  if (wifiState == WIFI_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiState = WIFI_CONNECTED;
      Serial.println();
      Serial.println("=====================================");
      Serial.println("WiFi Connected!");
      Serial.println("=====================================");
      Serial.print("  IP Address: ");
      Serial.println(WiFi.localIP());
      Serial.print("  Hostname: ");
      Serial.println(hostname);
      Serial.println();
      digitalWrite(STATUS_LED, HIGH);

      Serial.println("Initializing GPIO manager...");
      gpioManager = new GPIOPinManager();

      initializeSteppers();

      Serial.println("Starting web server...");
      webServer = new WebServerManager(configManager, &stepper_engine, stepper,
                                       gpioManager, i2sManager);
      if (webServer->begin(80)) {
        Serial.println("Web server started successfully!");
      } else {
        Serial.println("Failed to start web server!");
      }
    } else {
      unsigned long now = millis();
      if (now - lastWifiAttempt > 10000) {
        Serial.print(".");
        lastWifiAttempt = now;
        if (WiFi.status() == WL_CONNECT_FAILED) {
          Serial.println();
          Serial.println("WiFi connection failed. Retrying...");
          WiFi.disconnect();
          delay(1000);
          WiFi.reconnect();
        }
      }
    }
  } else if (wifiState == WIFI_AP_MODE) {
    static bool webServerStarted = false;
    if (!webServerStarted) {
      Serial.println();
      Serial.println("=====================================");
      Serial.println("Access Point Mode Active");
      Serial.println("=====================================");
      Serial.print("  Connect to SSID: ");
      Serial.println(WiFi.softAPSSID());
      Serial.print("  AP IP: ");
      Serial.println(WiFi.softAPIP());
      Serial.println();
      digitalWrite(STATUS_LED, HIGH);

      initializeSteppers();

      Serial.println("Starting web server...");
      webServer = new WebServerManager(configManager, &stepper_engine, stepper,
                                       gpioManager, i2sManager);
      if (webServer->begin(80)) {
        Serial.println("Web server started successfully!");
      }
      webServerStarted = true;
    }
  }
}
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  Serial.println();
  Serial.println("=====================================");
  Serial.println("ESP32 StepperDemo with WebUI");
  Serial.println("=====================================");
  Serial.print("Version: ");
  Serial.println(VERSION);
  Serial.print("Platform: ESP-IDF ");
  Serial.print(ESP_IDF_VERSION_MAJOR);
  Serial.print(".");
  Serial.print(ESP_IDF_VERSION_MINOR);
  Serial.print(".");
  Serial.print(ESP_IDF_VERSION_PATCH);
  Serial.println();
  Serial.println();

  Serial.println("Initializing configuration manager...");
  configManager = new ConfigManager();
  if (!configManager->begin()) {
    Serial.println("Warning: Failed to mount LittleFS, using defaults");
  }

  Serial.println("Initializing GPIO manager...");
  gpioManager = new GPIOPinManager();

  I2SExpanderConfig& i2sCfg = configManager->getI2SConfig();
  Serial.printf("I2S Config: enabled=%d, data=%d, bclk=%d, ws=%d\n",
                i2sCfg.enabled ? 1 : 0, i2sCfg.data_pin, i2sCfg.bclk_pin,
                i2sCfg.ws_pin);

  if (i2sCfg.enabled) {
    PinDefinition* dataPin = gpioManager->getPinDefinition(i2sCfg.data_pin);
    PinDefinition* bclkPin = gpioManager->getPinDefinition(i2sCfg.bclk_pin);
    PinDefinition* wsPin = gpioManager->getPinDefinition(i2sCfg.ws_pin);
    if (dataPin) dataPin->is_reserved = true;
    if (bclkPin) bclkPin->is_reserved = true;
    if (wsPin) wsPin->is_reserved = true;
    Serial.println("I2S pins marked as reserved");
  }

  Serial.println("Initializing FastAccelStepper engine...");
  stepper_engine.init();
  for (int i = 0; i < MAX_STEPPER; i++) {
    stepper[i] = nullptr;
  }

  if (i2sCfg.enabled) {
    Serial.println("Initializing I2S Expander...");
    i2sManager = new I2SPinManager();
    if (i2sManager->init(&stepper_engine, &i2sCfg)) {
      stepper_engine.setExternalCallForPin(externalPinCallback);
      Serial.println("I2S Expander initialized successfully");
    } else {
      Serial.println("I2S Expander initialization FAILED");
      delete i2sManager;
      i2sManager = nullptr;
    }
  } else {
    Serial.println("I2S Expander not enabled in config");
  }

  SystemConfig& sysConfig = configManager->getSystemConfig();
  if (strlen(sysConfig.wifi_ssid) > 0) {
    wifiSSID = String(sysConfig.wifi_ssid);
    wifiPassword = String(sysConfig.wifi_password);
    hostname = String(sysConfig.hostname);
  } else {
    if (loadWiFiConfig()) {
      Serial.println("Found WiFi configuration in NVM:");
      Serial.printf("  SSID: %s\n", wifiSSID.c_str());
      Serial.printf("  Hostname: %s\n", hostname.c_str());
    } else {
      Serial.println("No WiFi configuration found in NVM");
      configureWiFiViaSerial();
    }
  }

  Serial.println("Setting up WiFi...");
  if (wifiSSID.length() > 0 && !sysConfig.ap_mode) {
    WiFi.mode(WIFI_STA);
    if (hostname.length() > 0) {
      WiFi.setHostname(hostname.c_str());
    }
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    wifiState = WIFI_CONNECTING;
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP32-Stepper", "stepper123");
    wifiState = WIFI_AP_MODE;
    hostname = "esp32-stepper";
  }

  Serial.println();
  Serial.println("ESP32 StepperDemo ready!");
  Serial.println();
}
void loop() {
  static uint32_t lastPrint = 0;

  checkWiFiConnection();

  if (wifiState == WIFI_NOT_CONFIGURED) {
    return;
  }

  if (webServer) {
    webServer->loop();
  }

  uint32_t now = millis();
  if (now - lastPrint > 5000) {
    Serial.printf("[%lu] Running... Free heap: %lu bytes\n", now,
                  ESP.getFreeHeap());
    lastPrint = now;
  }

  delay(10);
}
