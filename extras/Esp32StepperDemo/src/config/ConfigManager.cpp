#include "ConfigManager.h"

bool ConfigManager::fileExists(const char* path) {
  return LittleFS.exists(path);
}

bool ConfigManager::loadJsonFile(const char* path, JsonDocument& doc) {
  File file = LittleFS.open(path, "r");
  if (!file) {
    return false;
  }

  DeserializationError error = deserializeJson(doc, file);
  file.close();

  return error == DeserializationError::Ok;
}

bool ConfigManager::saveJsonFile(const char* path, const JsonDocument& doc) {
  File file = LittleFS.open(path, "w");
  if (!file) {
    return false;
  }

  size_t bytes = serializeJson(doc, file);
  file.close();

  return bytes > 0;
}

bool ConfigManager::begin() {
  if (!LittleFS.begin(true)) {
    Serial.println("Failed to mount LittleFS");
    return false;
  }

  fs_mounted = true;
  Serial.println("LittleFS mounted successfully");

  LittleFS.mkdir("/config");

  loadSystemConfig();
  loadI2SConfig();
  loadStepperConfigs();
  loadSequences();

  return true;
}

bool ConfigManager::loadSystemConfig() {
  JsonDocument doc;
  if (!loadJsonFile("/config/system.json", doc)) {
    Serial.println("No system config found, using defaults");
    system_config.setDefaults();
    return false;
  }

  JsonObject wifi = doc["wifi"];
  if (!wifi.isNull()) {
    strlcpy(system_config.wifi_ssid, wifi["ssid"] | "",
            sizeof(system_config.wifi_ssid));
    strlcpy(system_config.wifi_password, wifi["password"] | "",
            sizeof(system_config.wifi_password));
    strlcpy(system_config.hostname, wifi["hostname"] | "esp32-stepper",
            sizeof(system_config.hostname));
  }

  JsonObject network = doc["network"];
  if (!network.isNull()) {
    system_config.ap_mode = network["ap_mode"] | false;
    system_config.web_server_port = network["port"] | 80;
    system_config.websocket_port = network["ws_port"] | 81;
  }

  system_config.status_update_interval_ms = doc["update_interval_ms"] | 100;

  return true;
}

bool ConfigManager::saveSystemConfig() {
  JsonDocument doc;

  JsonObject wifi = doc["wifi"].to<JsonObject>();
  wifi["ssid"] = system_config.wifi_ssid;
  wifi["password"] = system_config.wifi_password;
  wifi["hostname"] = system_config.hostname;

  JsonObject network = doc["network"].to<JsonObject>();
  network["ap_mode"] = system_config.ap_mode;
  network["port"] = system_config.web_server_port;
  network["ws_port"] = system_config.websocket_port;

  doc["update_interval_ms"] = system_config.status_update_interval_ms;

  return saveJsonFile("/config/system.json", doc);
}

bool ConfigManager::loadI2SConfig() {
  JsonDocument doc;
  if (!loadJsonFile("/config/i2s.json", doc)) {
    Serial.println("No I2S config found, using defaults");
    i2s_config.setDefaults();
    return false;
  }

  i2s_config.enabled = doc["enabled"] | false;
  i2s_config.data_pin = doc["data_pin"] | 32;
  i2s_config.bclk_pin = doc["bclk_pin"] | 33;
  i2s_config.ws_pin = doc["ws_pin"] | 14;

  return true;
}

bool ConfigManager::saveI2SConfig() {
  JsonDocument doc;

  doc["enabled"] = i2s_config.enabled;
  doc["data_pin"] = i2s_config.data_pin;
  doc["bclk_pin"] = i2s_config.bclk_pin;
  doc["ws_pin"] = i2s_config.ws_pin;

  return saveJsonFile("/config/i2s.json", doc);
}

int8_t ConfigManager::findFreeStepperSlot() {
  for (int i = 0; i < MAX_STEPPERS; i++) {
    if (steppers[i].id == 255) {
      return i;
    }
  }
  return -1;
}

StepperConfig* ConfigManager::getStepper(uint8_t id) {
  for (int i = 0; i < MAX_STEPPERS; i++) {
    if (steppers[i].id == id) {
      return &steppers[i];
    }
  }
  return nullptr;
}

bool ConfigManager::addStepper(const StepperConfig& config) {
  int8_t slot = findFreeStepperSlot();
  if (slot < 0) {
    return false;
  }

  steppers[slot] = config;
  return true;
}

bool ConfigManager::updateStepper(uint8_t id, const StepperConfig& config) {
  StepperConfig* existing = getStepper(id);
  if (!existing) {
    return false;
  }

  *existing = config;
  return true;
}

bool ConfigManager::deleteStepper(uint8_t id) {
  StepperConfig* existing = getStepper(id);
  if (!existing) {
    return false;
  }

  existing->setDefaults();
  return true;
}

bool ConfigManager::loadStepperConfigs() {
  JsonDocument doc;
  if (!loadJsonFile("/config/steppers.json", doc)) {
    Serial.println("No stepper configs found");
    return false;
  }

  JsonArray steppersArray = doc["steppers"];
  if (steppersArray.isNull()) {
    return false;
  }

  int index = 0;
  for (JsonObject stepperObj : steppersArray) {
    if (index >= MAX_STEPPERS) break;

    StepperConfig& s = steppers[index];
    s.setDefaults();

    s.id = stepperObj["id"] | index;
    strlcpy(s.name, stepperObj["name"] | "", sizeof(s.name));

    String driver = stepperObj["driver"] | "RMT";
    if (driver == "I2S_DIRECT")
      s.driver = STEPPER_DRIVER_I2S_DIRECT;
    else if (driver == "I2S_MUX")
      s.driver = STEPPER_DRIVER_I2S_MUX;
    else
      s.driver = STEPPER_DRIVER_RMT;

    JsonObject stepPin = stepperObj["step_pin"];
    if (!stepPin.isNull()) {
      String source = stepPin["source"] | "GPIO";
      if (source == "I2S")
        s.step_pin.source = PIN_SOURCE_I2S;
      else
        s.step_pin.source = PIN_SOURCE_GPIO;
      s.step_pin.pin = stepPin["pin"] | 255;
      s.step_pin.active_low = stepPin["active_low"] | false;
    }

    JsonObject dirPin = stepperObj["dir_pin"];
    if (!dirPin.isNull()) {
      String source = dirPin["source"] | "GPIO";
      if (source == "I2S")
        s.dir_pin.source = PIN_SOURCE_I2S;
      else
        s.dir_pin.source = PIN_SOURCE_GPIO;
      s.dir_pin.pin = dirPin["pin"] | 255;
      s.dir_pin.active_low = dirPin["active_low"] | false;
      s.dir_pin.delay_us = dirPin["delay_us"] | 0;
    }

    s.dir_high_counts_up = stepperObj["dir_high_counts_up"] | true;

    JsonObject enableLow = stepperObj["enable_pin_low"];
    if (!enableLow.isNull()) {
      String source = enableLow["source"] | "GPIO";
      if (source == "I2S")
        s.enable_pin_low.source = PIN_SOURCE_I2S;
      else
        s.enable_pin_low.source = PIN_SOURCE_GPIO;
      s.enable_pin_low.pin = enableLow["pin"] | 255;
      s.enable_pin_low.active_low = enableLow["active_low"] | true;
    }

    JsonObject enableHigh = stepperObj["enable_pin_high"];
    if (!enableHigh.isNull()) {
      String source = enableHigh["source"] | "GPIO";
      if (source == "I2S")
        s.enable_pin_high.source = PIN_SOURCE_I2S;
      else
        s.enable_pin_high.source = PIN_SOURCE_GPIO;
      s.enable_pin_high.pin = enableHigh["pin"] | 255;
      s.enable_pin_high.active_low = enableHigh["active_low"] | false;
    }

    s.speed_us = stepperObj["speed_us"] | 50;
    s.acceleration = stepperObj["acceleration"] | 10000;
    s.linear_accel_steps = stepperObj["linear_accel_steps"] | 0;
    s.jump_start_steps = stepperObj["jump_start_steps"] | 0;
    s.auto_enable = stepperObj["auto_enable"] | false;
    s.delay_to_enable_us = stepperObj["delay_to_enable_us"] | 1000;
    s.delay_to_disable_ms = stepperObj["delay_to_disable_ms"] | 100;

    index++;
  }

  return true;
}

bool ConfigManager::saveStepperConfigs() {
  JsonDocument doc;
  JsonArray steppersArray = doc["steppers"].to<JsonArray>();

  for (int i = 0; i < MAX_STEPPERS; i++) {
    if (steppers[i].id == 255) continue;

    JsonObject s = steppersArray.add<JsonObject>();
    s["id"] = steppers[i].id;
    s["name"] = steppers[i].name;

    switch (steppers[i].driver) {
      case STEPPER_DRIVER_I2S_DIRECT:
        s["driver"] = "I2S_DIRECT";
        break;
      case STEPPER_DRIVER_I2S_MUX:
        s["driver"] = "I2S_MUX";
        break;
      default:
        s["driver"] = "RMT";
        break;
    }

    JsonObject stepPin = s["step_pin"].to<JsonObject>();
    stepPin["source"] =
        steppers[i].step_pin.source == PIN_SOURCE_I2S ? "I2S" : "GPIO";
    stepPin["pin"] = steppers[i].step_pin.pin;
    stepPin["active_low"] = steppers[i].step_pin.active_low;

    JsonObject dirPin = s["dir_pin"].to<JsonObject>();
    dirPin["source"] =
        steppers[i].dir_pin.source == PIN_SOURCE_I2S ? "I2S" : "GPIO";
    dirPin["pin"] = steppers[i].dir_pin.pin;
    dirPin["active_low"] = steppers[i].dir_pin.active_low;
    dirPin["delay_us"] = steppers[i].dir_pin.delay_us;

    s["dir_high_counts_up"] = steppers[i].dir_high_counts_up;

    if (steppers[i].enable_pin_low.pin != 255) {
      JsonObject enableLow = s["enable_pin_low"].to<JsonObject>();
      enableLow["source"] =
          steppers[i].enable_pin_low.source == PIN_SOURCE_I2S ? "I2S" : "GPIO";
      enableLow["pin"] = steppers[i].enable_pin_low.pin;
      enableLow["active_low"] = steppers[i].enable_pin_low.active_low;
    }

    if (steppers[i].enable_pin_high.pin != 255) {
      JsonObject enableHigh = s["enable_pin_high"].to<JsonObject>();
      enableHigh["source"] =
          steppers[i].enable_pin_high.source == PIN_SOURCE_I2S ? "I2S" : "GPIO";
      enableHigh["pin"] = steppers[i].enable_pin_high.pin;
      enableHigh["active_low"] = steppers[i].enable_pin_high.active_low;
    }

    s["speed_us"] = steppers[i].speed_us;
    s["acceleration"] = steppers[i].acceleration;
    s["linear_accel_steps"] = steppers[i].linear_accel_steps;
    s["jump_start_steps"] = steppers[i].jump_start_steps;
    s["auto_enable"] = steppers[i].auto_enable;
    s["delay_to_enable_us"] = steppers[i].delay_to_enable_us;
    s["delay_to_disable_ms"] = steppers[i].delay_to_disable_ms;
  }

  return saveJsonFile("/config/steppers.json", doc);
}

int8_t ConfigManager::findFreeSequenceSlot() {
  for (int i = 0; i < MAX_SEQUENCES; i++) {
    if (sequences[i].id == 255) {
      return i;
    }
  }
  return -1;
}

MoveSequence* ConfigManager::getSequence(uint8_t id) {
  for (int i = 0; i < MAX_SEQUENCES; i++) {
    if (sequences[i].id == id) {
      return &sequences[i];
    }
  }
  return nullptr;
}

bool ConfigManager::addSequence(const MoveSequence& seq) {
  int8_t slot = findFreeSequenceSlot();
  if (slot < 0) {
    return false;
  }

  sequences[slot] = seq;
  return true;
}

bool ConfigManager::updateSequence(uint8_t id, const MoveSequence& seq) {
  MoveSequence* existing = getSequence(id);
  if (!existing) {
    return false;
  }

  *existing = seq;
  return true;
}

bool ConfigManager::deleteSequence(uint8_t id) {
  MoveSequence* existing = getSequence(id);
  if (!existing) {
    return false;
  }

  existing->setDefaults();
  return true;
}

bool ConfigManager::loadSequences() {
  JsonDocument doc;
  if (!loadJsonFile("/config/sequences.json", doc)) {
    Serial.println("No sequences config found");
    Serial.flush();
    return false;
  }

  JsonArray seqArray = doc["sequences"];
  if (seqArray.isNull()) {
    return false;
  }

  int index = 0;
  for (JsonObject seqObj : seqArray) {
    if (index >= MAX_SEQUENCES) break;

    MoveSequence& seq = sequences[index];
    seq.setDefaults();

    seq.id = seqObj["id"] | index;
    strlcpy(seq.name, seqObj["name"] | "", sizeof(seq.name));
    seq.loop = seqObj["loop"] | false;

    JsonArray cmdArray = seqObj["commands"];
    if (!cmdArray.isNull()) {
      int cmdIndex = 0;
      for (JsonObject cmdObj : cmdArray) {
        if (cmdIndex >= MAX_SEQUENCE_COMMANDS) break;

        MoveCommand& cmd = seq.commands[cmdIndex];
        cmd.setDefaults();

        String type = cmdObj["type"] | "STOP";
        if (type == "MOVE")
          cmd.type = MOVE_TYPE_MOVE;
        else if (type == "MOVE_TO")
          cmd.type = MOVE_TYPE_MOVE_TO;
        else if (type == "RUN_FORWARD")
          cmd.type = MOVE_TYPE_RUN_FORWARD;
        else if (type == "RUN_BACKWARD")
          cmd.type = MOVE_TYPE_RUN_BACKWARD;
        else if (type == "FORCE_STOP")
          cmd.type = MOVE_TYPE_FORCE_STOP;
        else if (type == "WAIT")
          cmd.type = MOVE_TYPE_WAIT;
        else if (type == "WAIT_FOR_STOP")
          cmd.type = MOVE_TYPE_WAIT_FOR_STOP;
        else if (type == "SET_SPEED")
          cmd.type = MOVE_TYPE_SET_SPEED;
        else if (type == "SET_ACCEL")
          cmd.type = MOVE_TYPE_SET_ACCEL;
        else if (type == "SET_POSITION")
          cmd.type = MOVE_TYPE_SET_POSITION;
        else if (type == "ENABLE")
          cmd.type = MOVE_TYPE_ENABLE;
        else if (type == "DISABLE")
          cmd.type = MOVE_TYPE_DISABLE;
        else if (type == "KEEP_RUNNING")
          cmd.type = MOVE_TYPE_KEEP_RUNNING;
        else
          cmd.type = MOVE_TYPE_STOP;

        cmd.stepper_id = cmdObj["stepper_id"] | 0;
        cmd.value1 = cmdObj["value1"] | 0;
        cmd.value2 = cmdObj["value2"] | 0;
        cmd.delay_ms = cmdObj["delay_ms"] | 0;

        cmdIndex++;
      }
      seq.num_commands = cmdIndex;
    }

    index++;
  }

  return true;
}

bool ConfigManager::saveSequences() {
  JsonDocument doc;
  JsonArray seqArray = doc["sequences"].to<JsonArray>();

  for (int i = 0; i < MAX_SEQUENCES; i++) {
    if (sequences[i].id == 255) continue;

    JsonObject seq = seqArray.add<JsonObject>();
    seq["id"] = sequences[i].id;
    seq["name"] = sequences[i].name;
    seq["loop"] = sequences[i].loop;

    JsonArray cmdArray = seq["commands"].to<JsonArray>();
    for (int j = 0; j < sequences[i].num_commands; j++) {
      MoveCommand& cmd = sequences[i].commands[j];
      JsonObject cmdObj = cmdArray.add<JsonObject>();

      String type;
      switch (cmd.type) {
        case MOVE_TYPE_MOVE:
          type = "MOVE";
          break;
        case MOVE_TYPE_MOVE_TO:
          type = "MOVE_TO";
          break;
        case MOVE_TYPE_RUN_FORWARD:
          type = "RUN_FORWARD";
          break;
        case MOVE_TYPE_RUN_BACKWARD:
          type = "RUN_BACKWARD";
          break;
        case MOVE_TYPE_FORCE_STOP:
          type = "FORCE_STOP";
          break;
        case MOVE_TYPE_WAIT:
          type = "WAIT";
          break;
        case MOVE_TYPE_WAIT_FOR_STOP:
          type = "WAIT_FOR_STOP";
          break;
        case MOVE_TYPE_SET_SPEED:
          type = "SET_SPEED";
          break;
        case MOVE_TYPE_SET_ACCEL:
          type = "SET_ACCEL";
          break;
        case MOVE_TYPE_SET_POSITION:
          type = "SET_POSITION";
          break;
        case MOVE_TYPE_ENABLE:
          type = "ENABLE";
          break;
        case MOVE_TYPE_DISABLE:
          type = "DISABLE";
          break;
        case MOVE_TYPE_KEEP_RUNNING:
          type = "KEEP_RUNNING";
          break;
        default:
          type = "STOP";
          break;
      }

      cmdObj["type"] = type;
      cmdObj["stepper_id"] = cmd.stepper_id;
      cmdObj["value1"] = cmd.value1;
      cmdObj["value2"] = cmd.value2;
      cmdObj["delay_ms"] = cmd.delay_ms;
    }
  }

  return saveJsonFile("/config/sequences.json", doc);
}

bool ConfigManager::factoryReset() {
  LittleFS.remove("/config/system.json");
  LittleFS.remove("/config/i2s.json");
  LittleFS.remove("/config/steppers.json");
  LittleFS.remove("/config/sequences.json");

  system_config.setDefaults();
  i2s_config.setDefaults();
  for (int i = 0; i < MAX_STEPPERS; i++) {
    steppers[i].setDefaults();
  }
  for (int i = 0; i < MAX_SEQUENCES; i++) {
    sequences[i].setDefaults();
  }

  return true;
}

bool ConfigManager::exportAll(const char* path) {
  JsonDocument doc;

  doc["system"]["wifi"]["ssid"] = system_config.wifi_ssid;
  doc["system"]["wifi"]["password"] = system_config.wifi_password;
  doc["system"]["wifi"]["hostname"] = system_config.hostname;
  doc["system"]["network"]["ap_mode"] = system_config.ap_mode;
  doc["system"]["network"]["port"] = system_config.web_server_port;

  doc["i2s"]["enabled"] = i2s_config.enabled;
  doc["i2s"]["data_pin"] = i2s_config.data_pin;
  doc["i2s"]["bclk_pin"] = i2s_config.bclk_pin;
  doc["i2s"]["ws_pin"] = i2s_config.ws_pin;

  return saveJsonFile(path, doc);
}

bool ConfigManager::importAll(const char* path) {
  JsonDocument doc;
  if (!loadJsonFile(path, doc)) {
    return false;
  }

  JsonObject system = doc["system"];
  if (!system.isNull()) {
    JsonObject wifi = system["wifi"];
    if (!wifi.isNull()) {
      strlcpy(system_config.wifi_ssid, wifi["ssid"] | "",
              sizeof(system_config.wifi_ssid));
      strlcpy(system_config.wifi_password, wifi["password"] | "",
              sizeof(system_config.wifi_password));
      strlcpy(system_config.hostname, wifi["hostname"] | "esp32-stepper",
              sizeof(system_config.hostname));
    }
    JsonObject network = system["network"];
    if (!network.isNull()) {
      system_config.ap_mode = network["ap_mode"] | false;
      system_config.web_server_port = network["port"] | 80;
    }
  }

  JsonObject i2s = doc["i2s"];
  if (!i2s.isNull()) {
    i2s_config.enabled = i2s["enabled"] | false;
    i2s_config.data_pin = i2s["data_pin"] | 32;
    i2s_config.bclk_pin = i2s["bclk_pin"] | 33;
    i2s_config.ws_pin = i2s["ws_pin"] | 14;
  }

  return true;
}
