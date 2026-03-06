#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "../config/SystemConfig.h"
#include "../config/StepperConfig.h"
#include "../config/I2SExpanderConfig.h"
#include "../config/SequenceConfig.h"

#define MAX_STEPPERS 14
#define MAX_SEQUENCES 10

class ConfigManager {
 private:
  SystemConfig system_config;
  I2SExpanderConfig i2s_config;
  StepperConfig steppers[MAX_STEPPERS];
  MoveSequence sequences[MAX_SEQUENCES];
  bool fs_mounted;

  bool loadJsonFile(const char* path, JsonDocument& doc);
  bool saveJsonFile(const char* path, const JsonDocument& doc);
  bool fileExists(const char* path);

 public:
  ConfigManager() : fs_mounted(false) {
    system_config.setDefaults();
    i2s_config.setDefaults();
    for (int i = 0; i < MAX_STEPPERS; i++) {
      steppers[i].setDefaults();
    }
    for (int i = 0; i < MAX_SEQUENCES; i++) {
      sequences[i].setDefaults();
    }
  }

  bool begin();
  bool isFsMounted() { return fs_mounted; }

  SystemConfig& getSystemConfig() { return system_config; }
  bool loadSystemConfig();
  bool saveSystemConfig();

  I2SExpanderConfig& getI2SConfig() { return i2s_config; }
  bool loadI2SConfig();
  bool saveI2SConfig();

  int8_t findFreeStepperSlot();
  StepperConfig* getStepper(uint8_t id);
  StepperConfig* getAllSteppers() { return steppers; }
  bool addStepper(const StepperConfig& config);
  bool updateStepper(uint8_t id, const StepperConfig& config);
  bool deleteStepper(uint8_t id);
  bool loadStepperConfigs();
  bool saveStepperConfigs();

  int8_t findFreeSequenceSlot();
  MoveSequence* getSequence(uint8_t id);
  MoveSequence* getAllSequences() { return sequences; }
  bool addSequence(const MoveSequence& seq);
  bool updateSequence(uint8_t id, const MoveSequence& seq);
  bool deleteSequence(uint8_t id);
  bool loadSequences();
  bool saveSequences();

  bool exportAll(const char* path);
  bool importAll(const char* path);
  bool factoryReset();
};
