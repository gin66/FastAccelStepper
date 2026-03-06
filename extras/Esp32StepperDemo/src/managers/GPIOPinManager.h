#pragma once

#include <Arduino.h>
#include <vector>

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
  uint8_t capabilities;
  char name[16];
  bool is_reserved;
  bool is_assigned;
  int assigned_to_stepper;
};

class GPIOPinManager {
 private:
  static const int NUM_GPIO_PINS = 40;
  PinDefinition pins[NUM_GPIO_PINS];

  void discoverPins();
  uint8_t getPinCapabilities(uint8_t pin);
  bool isPinReserved(uint8_t pin);
  const char* getPinName(uint8_t pin);

 public:
  GPIOPinManager();

  bool setPinMode(uint8_t pin, PinType type);
  bool setPinValue(uint8_t pin, bool value);
  bool getPinValue(uint8_t pin);
  bool togglePin(uint8_t pin);

  PinDefinition* getPinDefinition(uint8_t pin);
  std::vector<PinDefinition*> getAvailablePins(uint8_t capability_filter);
  std::vector<PinDefinition*> getAllPins();

  bool assignPinToStepper(uint8_t pin, uint8_t stepper_id);
  void unassignPin(uint8_t pin);
  bool isPinAvailable(uint8_t pin);
};
