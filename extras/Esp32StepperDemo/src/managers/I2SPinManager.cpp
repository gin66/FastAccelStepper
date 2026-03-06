#include "I2SPinManager.h"

I2SPinManager::I2SPinManager() {
  _initialized = false;
  engine = nullptr;
  _config = nullptr;

  for (int i = 0; i < I2S_NUM_PINS; i++) {
    pins[i].pin_number = i;
    pins[i].is_assigned = false;
    pins[i].assigned_to_stepper = -1;
    pins[i].current_value = false;
  }
}

bool I2SPinManager::init(FastAccelStepperEngine* eng,
                         I2SExpanderConfig* config) {
  if (_initialized) {
    return true;
  }

  if (eng == nullptr || config == nullptr) {
    Serial.println("I2SPinManager: Invalid engine or config");
    return false;
  }

  if (!config->enabled) {
    Serial.println("I2SPinManager: I2S not enabled in config");
    return false;
  }

  engine = eng;
  _config = config;

  Serial.printf(
      "I2SPinManager: Initializing I2S Mux (data=%d, bclk=%d, ws=%d)\n",
      config->data_pin, config->bclk_pin, config->ws_pin);

  if (!engine->initI2sMux(config->data_pin, config->bclk_pin, config->ws_pin)) {
    Serial.println("I2SPinManager: Failed to initialize I2S Mux");
    return false;
  }

  _initialized = true;
  Serial.println("I2SPinManager: I2S Mux initialized successfully");

  for (int i = 0; i < I2S_NUM_PINS; i++) {
    engine->i2sMuxSetBit(i, false);
  }

  return true;
}

bool I2SPinManager::setPinValue(uint8_t pin, bool value) {
  if (!_initialized || pin >= I2S_NUM_PINS) {
    return false;
  }

  pins[pin].current_value = value;
  engine->i2sMuxSetBit(pin, value);
  return true;
}

bool I2SPinManager::getPinValue(uint8_t pin) {
  if (!_initialized || pin >= I2S_NUM_PINS) {
    return false;
  }

  return pins[pin].current_value;
}

bool I2SPinManager::togglePin(uint8_t pin) {
  if (!_initialized || pin >= I2S_NUM_PINS) {
    return false;
  }

  bool newValue = !pins[pin].current_value;
  return setPinValue(pin, newValue);
}

I2SPinDefinition* I2SPinManager::getPinDefinition(uint8_t pin) {
  if (pin >= I2S_NUM_PINS) {
    return nullptr;
  }
  return &pins[pin];
}

std::vector<I2SPinDefinition*> I2SPinManager::getAllPins() {
  std::vector<I2SPinDefinition*> result;
  for (int i = 0; i < I2S_NUM_PINS; i++) {
    result.push_back(&pins[i]);
  }
  return result;
}

std::vector<I2SPinDefinition*> I2SPinManager::getAvailablePins() {
  std::vector<I2SPinDefinition*> result;
  for (int i = 0; i < I2S_NUM_PINS; i++) {
    if (!pins[i].is_assigned) {
      result.push_back(&pins[i]);
    }
  }
  return result;
}

bool I2SPinManager::assignPinToStepper(uint8_t pin, uint8_t stepper_id) {
  if (!_initialized || pin >= I2S_NUM_PINS || pins[pin].is_assigned) {
    return false;
  }

  pins[pin].is_assigned = true;
  pins[pin].assigned_to_stepper = stepper_id;
  return true;
}

void I2SPinManager::unassignPin(uint8_t pin) {
  if (pin < I2S_NUM_PINS) {
    pins[pin].is_assigned = false;
    pins[pin].assigned_to_stepper = -1;
  }
}

bool I2SPinManager::isPinAvailable(uint8_t pin) {
  if (pin >= I2S_NUM_PINS) {
    return false;
  }
  return !pins[pin].is_assigned;
}
