#pragma once

#include <Arduino.h>
#include <vector>
#include "FastAccelStepper.h"
#include "../config/I2SExpanderConfig.h"

struct I2SPinDefinition {
  uint8_t pin_number;
  bool is_assigned;
  int assigned_to_stepper;
  bool current_value;
};

class I2SPinManager {
 private:
  FastAccelStepperEngine* engine;
  I2SPinDefinition pins[I2S_NUM_PINS];
  bool _initialized;
  I2SExpanderConfig* _config;

 public:
  I2SPinManager();

  bool init(FastAccelStepperEngine* eng, I2SExpanderConfig* config);
  bool isInitialized() { return _initialized; }

  bool setPinValue(uint8_t pin, bool value);
  bool getPinValue(uint8_t pin);
  bool togglePin(uint8_t pin);

  I2SPinDefinition* getPinDefinition(uint8_t pin);
  std::vector<I2SPinDefinition*> getAllPins();
  std::vector<I2SPinDefinition*> getAvailablePins();

  bool assignPinToStepper(uint8_t pin, uint8_t stepper_id);
  void unassignPin(uint8_t pin);
  bool isPinAvailable(uint8_t pin);
};
