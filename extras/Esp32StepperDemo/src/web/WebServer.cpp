#include "WebServer.h"
#include "WebUI.h"
#include <LittleFS.h>

WebServerManager::WebServerManager(ConfigManager* cfgMgr,
                                   FastAccelStepperEngine* engine,
                                   FastAccelStepper** stp,
                                   GPIOPinManager* gpioMgr,
                                   I2SPinManager* i2sMgr)
    : server(nullptr),
      ws(nullptr),
      configManager(cfgMgr),
      stepperEngine(engine),
      steppers(stp),
      gpioManager(gpioMgr),
      i2sManager(i2sMgr),
      running(false),
      lastStatusUpdate(0),
      lastCleanup(0),
      statusUpdateInterval(500) {
  memset(prevRunningState, 0, sizeof(prevRunningState));
}

WebServerManager::~WebServerManager() {
  if (ws) {
    delete ws;
  }
  if (server) {
    delete server;
  }
}

bool WebServerManager::begin(uint16_t port) {
  server = new AsyncWebServer(port);
  ws = new AsyncWebSocket("/ws");

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods",
                                       "GET, POST, OPTIONS");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers",
                                       "Content-Type");

  setupStaticFiles();
  setupApiRoutes();
  setupWebSocket();

  server->begin();
  running = true;

  Serial.printf("Web server started on port %d\n", port);
  return true;
}

void WebServerManager::setupStaticFiles() {
  server->on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/html", FPSTR(index_html));
  });

  server->onNotFound([](AsyncWebServerRequest* request) {
    if (request->method() == HTTP_OPTIONS) {
      request->send(200);
    } else {
      request->send(404, "text/plain", "Not found");
    }
  });
}

void WebServerManager::setupApiRoutes() {
  server->on("/api/system", HTTP_GET, [this](AsyncWebServerRequest* request) {
    handleGetSystemConfig(request);
  });

  server->on("/api/system", HTTP_POST, [this](AsyncWebServerRequest* request) {
    handleSetSystemConfig(request);
  });

  server->on("/api/steppers", HTTP_GET, [this](AsyncWebServerRequest* request) {
    handleGetSteppers(request);
  });

  server->on(
      "^/api/steppers/([0-9]+)$", HTTP_GET,
      [this](AsyncWebServerRequest* request) { handleGetStepper(request); });

  server->on("/api/steppers/command", HTTP_POST,
             [this](AsyncWebServerRequest* request) {
               handleStepperCommand(request);
             });

  server->on(
      "/api/steppers", HTTP_POST,
      [this](AsyncWebServerRequest* request) { handleCreateStepper(request); });

  server->on(
      "^/api/steppers/([0-9]+)$", HTTP_PUT,
      [this](AsyncWebServerRequest* request) { handleUpdateStepper(request); });

  server->on(
      "^/api/steppers/([0-9]+)$", HTTP_DELETE,
      [this](AsyncWebServerRequest* request) { handleDeleteStepper(request); });

  server->on(
      "/api/steppers/position", HTTP_GET,
      [this](AsyncWebServerRequest* request) { handleGetPosition(request); });

  server->on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
    handleGetStatus(request);
  });

  server->on(
      "/api/sequences", HTTP_GET,
      [this](AsyncWebServerRequest* request) { handleGetSequences(request); });

  server->on(
      "^/api/sequences/([0-9]+)$", HTTP_GET,
      [this](AsyncWebServerRequest* request) { handleGetSequence(request); });

  server->on("/api/sequences", HTTP_POST,
             [this](AsyncWebServerRequest* request) {
               handleCreateSequence(request);
             });

  server->on("^/api/sequences/([0-9]+)$", HTTP_PUT,
             [this](AsyncWebServerRequest* request) {
               handleUpdateSequence(request);
             });

  server->on("^/api/sequences/([0-9]+)$", HTTP_DELETE,
             [this](AsyncWebServerRequest* request) {
               handleDeleteSequence(request);
             });

  server->on(
      "^/api/sequences/([0-9]+)/run$", HTTP_POST,
      [this](AsyncWebServerRequest* request) { handleRunSequence(request); });

  server->on(
      "/api/config/export", HTTP_GET,
      [this](AsyncWebServerRequest* request) { handleExportConfig(request); });

  server->on(
      "/api/config/import", HTTP_POST,
      [this](AsyncWebServerRequest* request) { handleImportConfig(request); });

  server->on(
      "/api/factory-reset", HTTP_POST,
      [this](AsyncWebServerRequest* request) { handleFactoryReset(request); });

  server->on("/api/pins", HTTP_GET, [this](AsyncWebServerRequest* request) {
    handleGetPins(request);
  });

  server->on("^/api/pins/([0-9]+)$", HTTP_GET,
             [this](AsyncWebServerRequest* request) { handleGetPin(request); });

  server->on("^/api/pins/([0-9]+)$", HTTP_POST,
             [this](AsyncWebServerRequest* request) { handleSetPin(request); });

  server->on(
      "^/api/pins/([0-9]+)/toggle$", HTTP_POST,
      [this](AsyncWebServerRequest* request) { handleTogglePin(request); });

  server->on("/api/i2s", HTTP_GET, [this](AsyncWebServerRequest* request) {
    handleGetI2SConfig(request);
  });

  server->on("/api/i2s", HTTP_POST, [this](AsyncWebServerRequest* request) {
    handleSetI2SConfig(request);
  });
}

void WebServerManager::setupWebSocket() {
  ws->onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client,
                     AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      Serial.printf("WebSocket client #%lu connected\n",
                    (unsigned long)client->id());
    } else if (type == WS_EVT_DISCONNECT) {
      Serial.printf("WebSocket client #%lu disconnected\n",
                    (unsigned long)client->id());
    } else if (type == WS_EVT_DATA) {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len) {
        if (info->opcode == WS_TEXT) {
          data[len] = 0;
          JsonDocument doc;
          DeserializationError error = deserializeJson(doc, data);
          if (!error) {
            const char* cmd = doc["cmd"];
            if (cmd && strcmp(cmd, "set_interval") == 0) {
              statusUpdateInterval = doc["interval"] | 100;
            }
          }
        }
      }
    }
  });

  server->addHandler(ws);
}

void WebServerManager::handleGetSystemConfig(AsyncWebServerRequest* request) {
  JsonDocument doc;
  SystemConfig& cfg = configManager->getSystemConfig();

  JsonObject wifi = doc["wifi"].to<JsonObject>();
  wifi["ssid"] = cfg.wifi_ssid;
  wifi["password"] = cfg.wifi_password;
  wifi["hostname"] = cfg.hostname;

  JsonObject network = doc["network"].to<JsonObject>();
  network["ap_mode"] = cfg.ap_mode;
  network["port"] = cfg.web_server_port;
  network["ws_port"] = cfg.websocket_port;

  sendJsonResponse(request, 200, doc);
}

void WebServerManager::handleSetSystemConfig(AsyncWebServerRequest* request) {
  sendErrorResponse(request, 501, "Not yet implemented - use serial config");
}

void WebServerManager::handleGetSteppers(AsyncWebServerRequest* request) {
  JsonDocument doc;
  JsonArray arr = doc["steppers"].to<JsonArray>();

  StepperConfig* steppersCfg = configManager->getAllSteppers();
  for (int i = 0; i < MAX_STEPPERS; i++) {
    if (steppersCfg[i].id != 255) {
      JsonObject s = arr.add<JsonObject>();
      s["id"] = steppersCfg[i].id;
      s["name"] = steppersCfg[i].name;
      s["driver"] =
          steppersCfg[i].driver == STEPPER_DRIVER_I2S_DIRECT ? "I2S_DIRECT"
          : steppersCfg[i].driver == STEPPER_DRIVER_I2S_MUX  ? "I2S_MUX"
                                                             : "RMT";
      s["step_pin"] = steppersCfg[i].step_pin.pin;
      s["step_source"] =
          steppersCfg[i].step_pin.source == PIN_SOURCE_I2S ? "I2S" : "GPIO";
      s["dir_pin"] = steppersCfg[i].dir_pin.pin;
      s["dir_source"] =
          steppersCfg[i].dir_pin.source == PIN_SOURCE_I2S ? "I2S" : "GPIO";
      s["enable_pin_low"] = steppersCfg[i].enable_pin_low.pin;
      s["enable_pin_low_source"] =
          steppersCfg[i].enable_pin_low.source == PIN_SOURCE_I2S ? "I2S"
                                                                 : "GPIO";
      s["enable_pin_high"] = steppersCfg[i].enable_pin_high.pin;
      s["enable_pin_high_source"] =
          steppersCfg[i].enable_pin_high.source == PIN_SOURCE_I2S ? "I2S"
                                                                  : "GPIO";
      s["speed_us"] = steppersCfg[i].speed_us;
      s["acceleration"] = steppersCfg[i].acceleration;
      s["auto_enable"] = steppersCfg[i].auto_enable;
    }
  }

  sendJsonResponse(request, 200, doc);
}

void WebServerManager::handleGetStepper(AsyncWebServerRequest* request) {
  String idStr = request->pathArg(0);
  uint8_t id = idStr.toInt();

  StepperConfig* cfg = configManager->getStepper(id);
  if (!cfg) {
    sendErrorResponse(request, 404, "Stepper not found");
    return;
  }

  JsonDocument doc;
  doc["id"] = cfg->id;
  doc["name"] = cfg->name;
  doc["driver"] = cfg->driver == STEPPER_DRIVER_I2S_DIRECT ? "I2S_DIRECT"
                  : cfg->driver == STEPPER_DRIVER_I2S_MUX  ? "I2S_MUX"
                                                           : "RMT";

  JsonObject stepPin = doc["step_pin"].to<JsonObject>();
  stepPin["source"] = cfg->step_pin.source == PIN_SOURCE_I2S ? "I2S" : "GPIO";
  stepPin["pin"] = cfg->step_pin.pin;
  stepPin["active_low"] = cfg->step_pin.active_low;

  JsonObject dirPin = doc["dir_pin"].to<JsonObject>();
  dirPin["source"] = cfg->dir_pin.source == PIN_SOURCE_I2S ? "I2S" : "GPIO";
  dirPin["pin"] = cfg->dir_pin.pin;
  dirPin["active_low"] = cfg->dir_pin.active_low;
  dirPin["delay_us"] = cfg->dir_pin.delay_us;

  doc["dir_high_counts_up"] = cfg->dir_high_counts_up;
  doc["speed_us"] = cfg->speed_us;
  doc["acceleration"] = cfg->acceleration;
  doc["linear_accel_steps"] = cfg->linear_accel_steps;
  doc["jump_start_steps"] = cfg->jump_start_steps;
  doc["auto_enable"] = cfg->auto_enable;
  doc["delay_to_enable_us"] = cfg->delay_to_enable_us;
  doc["delay_to_disable_ms"] = cfg->delay_to_disable_ms;

  sendJsonResponse(request, 200, doc);
}

void WebServerManager::handleCreateStepper(AsyncWebServerRequest* request) {
  if (!request->hasParam("name", true) ||
      !request->hasParam("step_pin", true)) {
    sendErrorResponse(request, 400,
                      "Missing required parameters: name, step_pin");
    return;
  }

  StepperConfig cfg;
  cfg.setDefaults();

  int8_t slot = configManager->findFreeStepperSlot();
  if (slot < 0 || slot >= MAX_STEPPERS) {
    sendErrorResponse(request, 500, "No free stepper slots");
    return;
  }

  cfg.id = slot;
  strlcpy(cfg.name, request->getParam("name", true)->value().c_str(),
          sizeof(cfg.name));
  cfg.step_pin.pin = request->getParam("step_pin", true)->value().toInt();

  if (request->hasParam("step_source", true)) {
    String src = request->getParam("step_source", true)->value();
    cfg.step_pin.source = (src == "I2S") ? PIN_SOURCE_I2S : PIN_SOURCE_GPIO;
  } else {
    cfg.step_pin.source = PIN_SOURCE_GPIO;
  }

  if (request->hasParam("driver", true)) {
    String driver = request->getParam("driver", true)->value();
    if (driver == "I2S_DIRECT") {
      cfg.driver = STEPPER_DRIVER_I2S_DIRECT;
    } else if (driver == "I2S_MUX") {
      cfg.driver = STEPPER_DRIVER_I2S_MUX;
    } else {
      cfg.driver = STEPPER_DRIVER_RMT;
    }
  }

  if (request->hasParam("dir_pin", true)) {
    cfg.dir_pin.pin = request->getParam("dir_pin", true)->value().toInt();
    if (request->hasParam("dir_source", true)) {
      String src = request->getParam("dir_source", true)->value();
      cfg.dir_pin.source = (src == "I2S") ? PIN_SOURCE_I2S : PIN_SOURCE_GPIO;
    } else {
      cfg.dir_pin.source = PIN_SOURCE_GPIO;
    }
  }

  if (request->hasParam("enable_pin_low", true)) {
    cfg.enable_pin_low.pin =
        request->getParam("enable_pin_low", true)->value().toInt();
    if (request->hasParam("enable_pin_low_source", true)) {
      String src = request->getParam("enable_pin_low_source", true)->value();
      cfg.enable_pin_low.source =
          (src == "I2S") ? PIN_SOURCE_I2S : PIN_SOURCE_GPIO;
    } else {
      cfg.enable_pin_low.source = PIN_SOURCE_GPIO;
    }
  }

  if (request->hasParam("enable_pin_high", true)) {
    cfg.enable_pin_high.pin =
        request->getParam("enable_pin_high", true)->value().toInt();
    if (request->hasParam("enable_pin_high_source", true)) {
      String src = request->getParam("enable_pin_high_source", true)->value();
      cfg.enable_pin_high.source =
          (src == "I2S") ? PIN_SOURCE_I2S : PIN_SOURCE_GPIO;
    } else {
      cfg.enable_pin_high.source = PIN_SOURCE_GPIO;
    }
  }

  if (request->hasParam("speed_us", true)) {
    cfg.speed_us = request->getParam("speed_us", true)->value().toInt();
  }
  if (request->hasParam("acceleration", true)) {
    cfg.acceleration = request->getParam("acceleration", true)->value().toInt();
  }
  if (request->hasParam("auto_enable", true)) {
    cfg.auto_enable = request->getParam("auto_enable", true)->value() == "true";
  }

  if (!configManager->addStepper(cfg)) {
    sendErrorResponse(request, 500, "Failed to add stepper config");
    return;
  }

  configManager->saveStepperConfigs();

  JsonDocument doc;
  doc["success"] = true;
  doc["id"] = cfg.id;
  doc["message"] = "Stepper created. Restart device to initialize.";
  sendJsonResponse(request, 200, doc);
}

void WebServerManager::handleUpdateStepper(AsyncWebServerRequest* request) {
  String idStr = request->pathArg(0);
  uint8_t id = idStr.toInt();

  StepperConfig* cfg = configManager->getStepper(id);
  if (!cfg) {
    sendErrorResponse(request, 404, "Stepper not found");
    return;
  }

  if (request->hasParam("name", true)) {
    strlcpy(cfg->name, request->getParam("name", true)->value().c_str(),
            sizeof(cfg->name));
  }
  if (request->hasParam("driver", true)) {
    String driver = request->getParam("driver", true)->value();
    if (driver == "I2S_DIRECT") {
      cfg->driver = STEPPER_DRIVER_I2S_DIRECT;
    } else if (driver == "I2S_MUX") {
      cfg->driver = STEPPER_DRIVER_I2S_MUX;
    } else {
      cfg->driver = STEPPER_DRIVER_RMT;
    }
  }
  if (request->hasParam("step_pin", true)) {
    cfg->step_pin.pin = request->getParam("step_pin", true)->value().toInt();
    if (request->hasParam("step_source", true)) {
      String src = request->getParam("step_source", true)->value();
      cfg->step_pin.source = (src == "I2S") ? PIN_SOURCE_I2S : PIN_SOURCE_GPIO;
    }
  }
  if (request->hasParam("dir_pin", true)) {
    cfg->dir_pin.pin = request->getParam("dir_pin", true)->value().toInt();
    if (request->hasParam("dir_source", true)) {
      String src = request->getParam("dir_source", true)->value();
      cfg->dir_pin.source = (src == "I2S") ? PIN_SOURCE_I2S : PIN_SOURCE_GPIO;
    }
  }
  if (request->hasParam("enable_pin_low", true)) {
    cfg->enable_pin_low.pin =
        request->getParam("enable_pin_low", true)->value().toInt();
    if (request->hasParam("enable_pin_low_source", true)) {
      String src = request->getParam("enable_pin_low_source", true)->value();
      cfg->enable_pin_low.source =
          (src == "I2S") ? PIN_SOURCE_I2S : PIN_SOURCE_GPIO;
    }
  }
  if (request->hasParam("enable_pin_high", true)) {
    cfg->enable_pin_high.pin =
        request->getParam("enable_pin_high", true)->value().toInt();
    if (request->hasParam("enable_pin_high_source", true)) {
      String src = request->getParam("enable_pin_high_source", true)->value();
      cfg->enable_pin_high.source =
          (src == "I2S") ? PIN_SOURCE_I2S : PIN_SOURCE_GPIO;
    }
  }
  if (request->hasParam("speed_us", true)) {
    cfg->speed_us = request->getParam("speed_us", true)->value().toInt();
  }
  if (request->hasParam("acceleration", true)) {
    cfg->acceleration =
        request->getParam("acceleration", true)->value().toInt();
  }
  if (request->hasParam("auto_enable", true)) {
    cfg->auto_enable =
        request->getParam("auto_enable", true)->value() == "true";
  }

  if (steppers[id]) {
    steppers[id]->setSpeedInUs(cfg->speed_us);
    steppers[id]->setAcceleration(cfg->acceleration);
  }

  configManager->saveStepperConfigs();

  JsonDocument doc;
  doc["success"] = true;
  doc["message"] = "Stepper updated";
  sendJsonResponse(request, 200, doc);
}

void WebServerManager::handleDeleteStepper(AsyncWebServerRequest* request) {
  String idStr = request->pathArg(0);
  uint8_t id = idStr.toInt();

  if (!configManager->deleteStepper(id)) {
    sendErrorResponse(request, 404, "Stepper not found");
    return;
  }

  configManager->saveStepperConfigs();

  JsonDocument doc;
  doc["success"] = true;
  sendJsonResponse(request, 200, doc);
}

void WebServerManager::handleStepperCommand(AsyncWebServerRequest* request) {
  Serial.printf("[CMD] %s %s params=%d\n", request->methodToString(),
                request->url().c_str(), request->params());

  for (int i = 0; i < request->params(); i++) {
    const AsyncWebParameter* p = request->getParam(i);
    Serial.printf("  param[%d]: %s=%s\n", i, p->name().c_str(),
                  p->value().c_str());
  }

  if (!request->hasParam("stepper_id", true) ||
      !request->hasParam("command", true)) {
    sendErrorResponse(request, 400, "Missing required parameters");
    return;
  }

  uint8_t id = request->getParam("stepper_id", true)->value().toInt();
  String cmd = request->getParam("command", true)->value();

  if (id >= MAX_STEPPER || !steppers[id]) {
    sendErrorResponse(request, 404, "Stepper not found or not initialized");
    return;
  }

  FastAccelStepper* s = steppers[id];
  int32_t value1 = 0;

  if (request->hasParam("value1", true)) {
    value1 = request->getParam("value1", true)->value().toInt();
  }

  if (cmd == "move") {
    s->move(value1);
  } else if (cmd == "move_to") {
    s->moveTo(value1);
  } else if (cmd == "run_forward") {
    s->runForward();
  } else if (cmd == "run_backward") {
    s->runBackward();
  } else if (cmd == "stop") {
    s->stopMove();
  } else if (cmd == "force_stop") {
    s->forceStop();
  } else if (cmd == "set_position") {
    s->setCurrentPosition(value1);
  } else if (cmd == "set_speed") {
    s->setSpeedInUs(value1);
  } else if (cmd == "set_acceleration") {
    s->setAcceleration(value1);
  } else if (cmd == "enable") {
    s->enableOutputs();
  } else if (cmd == "disable") {
    s->disableOutputs();
  } else {
    sendErrorResponse(request, 400, "Unknown command");
    return;
  }

  JsonDocument doc;
  doc["success"] = true;
  sendJsonResponse(request, 200, doc);
}

void WebServerManager::handleGetPosition(AsyncWebServerRequest* request) {
  JsonDocument doc;
  JsonArray arr = doc["positions"].to<JsonArray>();

  for (int i = 0; i < MAX_STEPPER; i++) {
    if (steppers[i]) {
      JsonObject p = arr.add<JsonObject>();
      p["id"] = i;
      p["position"] = steppers[i]->getCurrentPosition();
      p["target_position"] = steppers[i]->targetPos();
      p["speed"] = steppers[i]->getCurrentSpeedInUs();
      p["running"] = steppers[i]->isRunning();
    }
  }

  sendJsonResponse(request, 200, doc);
}

void WebServerManager::handleGetStatus(AsyncWebServerRequest* request) {
  JsonDocument doc;

  doc["wifi"]["connected"] = WiFi.isConnected();
  doc["wifi"]["ssid"] = WiFi.SSID();
  doc["wifi"]["ip"] = WiFi.localIP().toString();
  doc["heap"]["free"] = ESP.getFreeHeap();
  doc["heap"]["total"] = ESP.getHeapSize();
  doc["uptime_ms"] = millis();

  sendJsonResponse(request, 200, doc);
}

void WebServerManager::handleGetSequences(AsyncWebServerRequest* request) {
  JsonDocument doc;
  JsonArray arr = doc["sequences"].to<JsonArray>();

  MoveSequence* sequences = configManager->getAllSequences();
  for (int i = 0; i < MAX_SEQUENCES; i++) {
    if (sequences[i].id != 255) {
      JsonObject s = arr.add<JsonObject>();
      s["id"] = sequences[i].id;
      s["name"] = sequences[i].name;
      s["loop"] = sequences[i].loop;
      s["num_commands"] = sequences[i].num_commands;
    }
  }

  sendJsonResponse(request, 200, doc);
}

void WebServerManager::handleGetSequence(AsyncWebServerRequest* request) {
  String idStr = request->pathArg(0);
  uint8_t id = idStr.toInt();

  MoveSequence* seq = configManager->getSequence(id);
  if (!seq) {
    sendErrorResponse(request, 404, "Sequence not found");
    return;
  }

  JsonDocument doc;
  doc["id"] = seq->id;
  doc["name"] = seq->name;
  doc["loop"] = seq->loop;
  doc["num_commands"] = seq->num_commands;

  sendJsonResponse(request, 200, doc);
}

void WebServerManager::handleCreateSequence(AsyncWebServerRequest* request) {
  sendErrorResponse(request, 501, "Not yet implemented - use config file");
}

void WebServerManager::handleUpdateSequence(AsyncWebServerRequest* request) {
  sendErrorResponse(request, 501, "Not yet implemented - use config file");
}

void WebServerManager::handleDeleteSequence(AsyncWebServerRequest* request) {
  String idStr = request->pathArg(0);
  uint8_t id = idStr.toInt();

  if (!configManager->deleteSequence(id)) {
    sendErrorResponse(request, 404, "Sequence not found");
    return;
  }

  configManager->saveSequences();

  JsonDocument doc;
  doc["success"] = true;
  sendJsonResponse(request, 200, doc);
}

void WebServerManager::handleRunSequence(AsyncWebServerRequest* request) {
  sendErrorResponse(request, 501, "Sequence execution not yet implemented");
}

void WebServerManager::handleExportConfig(AsyncWebServerRequest* request) {
  if (configManager->exportAll("/config/export.json")) {
    request->send(LittleFS, "/config/export.json", "application/json");
  } else {
    sendErrorResponse(request, 500, "Failed to export configuration");
  }
}

void WebServerManager::handleImportConfig(AsyncWebServerRequest* request) {
  sendErrorResponse(request, 501,
                    "Not yet implemented - use config file upload");
}

void WebServerManager::handleFactoryReset(AsyncWebServerRequest* request) {
  configManager->factoryReset();

  JsonDocument doc;
  doc["success"] = true;
  doc["message"] = "Factory reset complete. Device will restart.";
  sendJsonResponse(request, 200, doc);

  delay(1000);
  ESP.restart();
}

void WebServerManager::sendJsonResponse(AsyncWebServerRequest* request,
                                        int code, const JsonDocument& doc) {
  String response;
  serializeJson(doc, response);
  Serial.printf("[HTTP] %s %s -> %d: %s\n", request->methodToString(),
                request->url().c_str(), code, response.c_str());
  request->send(code, "application/json", response);
}

void WebServerManager::sendErrorResponse(AsyncWebServerRequest* request,
                                         int code, const char* message) {
  Serial.printf("[HTTP] ERROR %s %s -> %d: %s\n", request->methodToString(),
                request->url().c_str(), code, message);
  JsonDocument doc;
  doc["success"] = false;
  doc["error"] = message;
  sendJsonResponse(request, code, doc);
}

void WebServerManager::broadcastStatus() {
  if (!ws || ws->count() == 0) {
    return;
  }

  JsonDocument doc;
  doc["type"] = "status";

  JsonArray positions = doc["positions"].to<JsonArray>();
  for (int i = 0; i < MAX_STEPPER; i++) {
    if (steppers[i]) {
      JsonObject p = positions.add<JsonObject>();
      p["id"] = i;
      p["position"] = steppers[i]->getCurrentPosition();
      p["running"] = steppers[i]->isRunning();
      p["speed"] = steppers[i]->getCurrentSpeedInUs();
    }
  }

  String message;
  serializeJson(doc, message);
  ws->textAll(message);
}

void WebServerManager::notifyWebSocketClients(const char* message) {
  if (ws) {
    ws->textAll(message);
  }
}

void WebServerManager::loop() {
  if (!running) {
    return;
  }

  uint32_t now = millis();

  bool stateChanged = false;
  for (int i = 0; i < MAX_STEPPER; i++) {
    if (steppers[i]) {
      bool isRunning = steppers[i]->isRunning();
      if (isRunning != prevRunningState[i]) {
        stateChanged = true;
        prevRunningState[i] = isRunning;
      }
    }
  }

  if (stateChanged || (now - lastStatusUpdate >= statusUpdateInterval)) {
    broadcastStatus();
    lastStatusUpdate = now;
  }

  if (now - lastCleanup >= 5000) {
    ws->cleanupClients();
    lastCleanup = now;
  }
}

void WebServerManager::setStatusUpdateInterval(uint16_t interval_ms) {
  statusUpdateInterval = interval_ms;
}

void WebServerManager::handleGetPins(AsyncWebServerRequest* request) {
  if (!gpioManager) {
    sendErrorResponse(request, 500, "GPIO manager not initialized");
    return;
  }

  JsonDocument doc;
  JsonArray arr = doc["pins"].to<JsonArray>();

  std::vector<PinDefinition*> pins = gpioManager->getAllPins();

  for (auto pin : pins) {
    JsonObject p = arr.add<JsonObject>();
    p["pin"] = pin->pin_number;
    p["name"] = pin->name;
    p["type"] = (int)pin->type;
    p["capabilities"] = pin->capabilities;
    p["is_reserved"] = pin->is_reserved;
    p["is_assigned"] = pin->is_assigned;
    p["source"] = "gpio";
    if (pin->is_assigned) {
      p["assigned_to"] = pin->assigned_to_stepper;
    }

    int value = -1;
    if (!pin->is_reserved && pin->type != PIN_TYPE_NONE) {
      value = digitalRead(pin->pin_number);
    }
    p["value"] = value;
  }

  if (i2sManager && i2sManager->isInitialized()) {
    std::vector<I2SPinDefinition*> i2sPins = i2sManager->getAllPins();
    for (auto pin : i2sPins) {
      JsonObject p = arr.add<JsonObject>();
      p["pin"] = pin->pin_number;
      char name[16];
      snprintf(name, sizeof(name), "I2S %d", pin->pin_number);
      p["name"] = name;
      p["type"] = (int)PIN_TYPE_I2S_OUTPUT;
      p["capabilities"] = PIN_CAP_OUTPUT | PIN_CAP_I2S;
      p["is_reserved"] = false;
      p["is_assigned"] = pin->is_assigned;
      p["source"] = "i2s";
      if (pin->is_assigned) {
        p["assigned_to"] = pin->assigned_to_stepper;
      }
      p["value"] = pin->current_value ? 1 : 0;
    }
  }

  sendJsonResponse(request, 200, doc);
}

void WebServerManager::handleGetPin(AsyncWebServerRequest* request) {
  if (!gpioManager) {
    sendErrorResponse(request, 500, "GPIO manager not initialized");
    return;
  }

  String pinStr = request->pathArg(0);
  uint8_t pin = pinStr.toInt();

  PinDefinition* pinDef = gpioManager->getPinDefinition(pin);

  if (!pinDef) {
    sendErrorResponse(request, 404, "Pin not found");
    return;
  }

  JsonDocument doc;
  doc["pin"] = pinDef->pin_number;
  doc["name"] = pinDef->name;
  doc["type"] = (int)pinDef->type;
  doc["capabilities"] = pinDef->capabilities;
  doc["is_reserved"] = pinDef->is_reserved;
  doc["is_assigned"] = pinDef->is_assigned;
  if (pinDef->is_assigned) {
    doc["assigned_to"] = pinDef->assigned_to_stepper;
  }

  int value = -1;
  if (!pinDef->is_reserved && pinDef->type != PIN_TYPE_NONE) {
    value = digitalRead(pinDef->pin_number);
  }
  doc["value"] = value;

  sendJsonResponse(request, 200, doc);
}

void WebServerManager::handleSetPin(AsyncWebServerRequest* request) {
  String pinStr = request->pathArg(0);
  uint8_t pin = pinStr.toInt();
  String source = "gpio";
  if (request->hasParam("source", true)) {
    source = request->getParam("source", true)->value();
  }

  if (source == "i2s") {
    if (!i2sManager || !i2sManager->isInitialized()) {
      sendErrorResponse(request, 500, "I2S manager not initialized");
      return;
    }

    I2SPinDefinition* pinDef = i2sManager->getPinDefinition(pin);
    if (!pinDef || pin >= I2S_NUM_PINS) {
      sendErrorResponse(request, 404, "I2S pin not found");
      return;
    }

    if (pinDef->is_assigned) {
      sendErrorResponse(request, 403, "I2S pin is assigned to stepper");
      return;
    }

    if (!request->hasParam("mode", true)) {
      sendErrorResponse(request, 400, "Missing 'mode' parameter");
      return;
    }

    String mode = request->getParam("mode", true)->value();
    int value = 0;
    if (request->hasParam("value", true)) {
      value = request->getParam("value", true)->value().toInt();
    }

    if (mode == "output") {
      if (i2sManager->setPinValue(pin, value == 1)) {
        JsonDocument doc;
        doc["success"] = true;
        doc["pin"] = pin;
        doc["source"] = "i2s";
        doc["mode"] = "output";
        doc["value"] = i2sManager->getPinValue(pin) ? 1 : 0;
        sendJsonResponse(request, 200, doc);
      } else {
        sendErrorResponse(request, 400, "Failed to set I2S pin value");
      }
    } else {
      sendErrorResponse(request, 400, "I2S pins only support 'output' mode");
    }
    return;
  }

  if (!gpioManager) {
    sendErrorResponse(request, 500, "GPIO manager not initialized");
    return;
  }

  PinDefinition* pinDef = gpioManager->getPinDefinition(pin);
  if (!pinDef) {
    sendErrorResponse(request, 404, "Pin not found");
    return;
  }

  if (pinDef->is_reserved || pinDef->is_assigned) {
    sendErrorResponse(request, 403, "Pin is reserved or assigned");
    return;
  }

  if (!request->hasParam("mode", true)) {
    sendErrorResponse(request, 400, "Missing 'mode' parameter");
    return;
  }

  String mode = request->getParam("mode", true)->value();
  int value = 0;
  if (request->hasParam("value", true)) {
    value = request->getParam("value", true)->value().toInt();
  }

  if (mode == "input") {
    if (gpioManager->setPinMode(pin, PIN_TYPE_GPIO_INPUT)) {
      JsonDocument doc;
      doc["success"] = true;
      doc["pin"] = pin;
      doc["source"] = "gpio";
      doc["mode"] = "input";
      sendJsonResponse(request, 200, doc);
    } else {
      sendErrorResponse(request, 400, "Failed to set pin mode");
    }
  } else if (mode == "output") {
    if (gpioManager->setPinMode(pin, PIN_TYPE_GPIO_OUTPUT)) {
      if (value == 0 || value == 1) {
        gpioManager->setPinValue(pin, value);
      }

      JsonDocument doc;
      doc["success"] = true;
      doc["pin"] = pin;
      doc["source"] = "gpio";
      doc["mode"] = "output";
      doc["value"] = digitalRead(pin);
      sendJsonResponse(request, 200, doc);
    } else {
      sendErrorResponse(request, 400, "Failed to set pin mode");
    }
  } else {
    sendErrorResponse(request, 400, "Invalid mode. Use 'input' or 'output'");
  }
}

void WebServerManager::handleTogglePin(AsyncWebServerRequest* request) {
  String pinStr = request->pathArg(0);
  uint8_t pin = pinStr.toInt();
  String source = "gpio";
  if (request->hasParam("source", true)) {
    source = request->getParam("source", true)->value();
  }

  if (source == "i2s") {
    if (!i2sManager || !i2sManager->isInitialized()) {
      sendErrorResponse(request, 500, "I2S manager not initialized");
      return;
    }

    I2SPinDefinition* pinDef = i2sManager->getPinDefinition(pin);
    if (!pinDef || pin >= I2S_NUM_PINS) {
      sendErrorResponse(request, 404, "I2S pin not found");
      return;
    }

    if (pinDef->is_assigned) {
      sendErrorResponse(request, 403, "I2S pin is assigned to stepper");
      return;
    }

    if (i2sManager->togglePin(pin)) {
      JsonDocument doc;
      doc["success"] = true;
      doc["pin"] = pin;
      doc["source"] = "i2s";
      doc["value"] = i2sManager->getPinValue(pin) ? 1 : 0;
      sendJsonResponse(request, 200, doc);
    } else {
      sendErrorResponse(request, 400, "Failed to toggle I2S pin");
    }
    return;
  }

  if (!gpioManager) {
    sendErrorResponse(request, 500, "GPIO manager not initialized");
    return;
  }

  PinDefinition* pinDef = gpioManager->getPinDefinition(pin);
  if (!pinDef) {
    sendErrorResponse(request, 404, "Pin not found");
    return;
  }

  if (pinDef->is_reserved || pinDef->is_assigned) {
    sendErrorResponse(request, 403, "Pin is reserved or assigned");
    return;
  }

  if (pinDef->type != PIN_TYPE_GPIO_OUTPUT) {
    sendErrorResponse(request, 400, "Pin must be in output mode to toggle");
    return;
  }

  if (gpioManager->togglePin(pin)) {
    JsonDocument doc;
    doc["success"] = true;
    doc["pin"] = pin;
    doc["source"] = "gpio";
    doc["value"] = digitalRead(pin);
    sendJsonResponse(request, 200, doc);
  } else {
    sendErrorResponse(request, 400, "Failed to toggle pin");
  }
}

void WebServerManager::handleGetI2SConfig(AsyncWebServerRequest* request) {
  I2SExpanderConfig& cfg = configManager->getI2SConfig();

  JsonDocument doc;
  doc["enabled"] = cfg.enabled;
  doc["data_pin"] = cfg.data_pin;
  doc["bclk_pin"] = cfg.bclk_pin;
  doc["ws_pin"] = cfg.ws_pin;
  doc["initialized"] = (i2sManager != nullptr && i2sManager->isInitialized());

  sendJsonResponse(request, 200, doc);
}

void WebServerManager::handleSetI2SConfig(AsyncWebServerRequest* request) {
  I2SExpanderConfig& cfg = configManager->getI2SConfig();

  if (request->hasParam("enabled", true)) {
    cfg.enabled = request->getParam("enabled", true)->value() == "true";
  }
  if (request->hasParam("data_pin", true)) {
    cfg.data_pin = request->getParam("data_pin", true)->value().toInt();
  }
  if (request->hasParam("bclk_pin", true)) {
    cfg.bclk_pin = request->getParam("bclk_pin", true)->value().toInt();
  }
  if (request->hasParam("ws_pin", true)) {
    cfg.ws_pin = request->getParam("ws_pin", true)->value().toInt();
  }

  configManager->saveI2SConfig();

  JsonDocument doc;
  doc["success"] = true;
  doc["message"] = "I2S configuration saved. Restart device to apply.";
  sendJsonResponse(request, 200, doc);
}
