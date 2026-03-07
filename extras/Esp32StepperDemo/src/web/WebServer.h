#pragma once

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "../config/ConfigManager.h"
#include "../managers/GPIOPinManager.h"
#include "../managers/I2SPinManager.h"
#include "FastAccelStepper.h"

class WebServerManager {
 private:
  AsyncWebServer* server;
  AsyncWebSocket* ws;
  ConfigManager* configManager;
  FastAccelStepperEngine* stepperEngine;
  FastAccelStepper** steppers;
  GPIOPinManager* gpioManager;
  I2SPinManager* i2sManager;

  bool running;
  uint32_t lastStatusUpdate;
  uint32_t lastCleanup;
  uint16_t statusUpdateInterval;
  bool prevRunningState[MAX_STEPPER];
  bool enabledState[MAX_STEPPER];

  void setupStaticFiles();
  void setupApiRoutes();
  void setupWebSocket();

  void handleGetSystemConfig(AsyncWebServerRequest* request);
  void handleSetSystemConfig(AsyncWebServerRequest* request);

  void handleGetSteppers(AsyncWebServerRequest* request);
  void handleGetStepper(AsyncWebServerRequest* request);
  void handleCreateStepper(AsyncWebServerRequest* request);
  void handleUpdateStepper(AsyncWebServerRequest* request);
  void handleDeleteStepper(AsyncWebServerRequest* request);

  void handleStepperCommand(AsyncWebServerRequest* request);
  void handleGetPosition(AsyncWebServerRequest* request);
  void handleGetStatus(AsyncWebServerRequest* request);

  void handleGetSequences(AsyncWebServerRequest* request);
  void handleGetSequence(AsyncWebServerRequest* request);
  void handleCreateSequence(AsyncWebServerRequest* request);
  void handleUpdateSequence(AsyncWebServerRequest* request);
  void handleDeleteSequence(AsyncWebServerRequest* request);
  void handleRunSequence(AsyncWebServerRequest* request);

  void handleExportConfig(AsyncWebServerRequest* request);
  void handleImportConfig(AsyncWebServerRequest* request);
  void handleFactoryReset(AsyncWebServerRequest* request);

  void handleGetPins(AsyncWebServerRequest* request);
  void handleGetPin(AsyncWebServerRequest* request);
  void handleSetPin(AsyncWebServerRequest* request);
  void handleTogglePin(AsyncWebServerRequest* request);

  void handleGetI2SConfig(AsyncWebServerRequest* request);
  void handleSetI2SConfig(AsyncWebServerRequest* request);

  void sendJsonResponse(AsyncWebServerRequest* request, int code,
                        const JsonDocument& doc);
  void sendErrorResponse(AsyncWebServerRequest* request, int code,
                         const char* message);

  void broadcastStatus();
  void notifyWebSocketClients(const char* message);

 public:
  WebServerManager(ConfigManager* cfgMgr, FastAccelStepperEngine* engine,
                   FastAccelStepper** stp, GPIOPinManager* gpioMgr,
                   I2SPinManager* i2sMgr = nullptr);
  ~WebServerManager();

  bool begin(uint16_t port = 80);
  void loop();
  void setStatusUpdateInterval(uint16_t interval_ms);
};
