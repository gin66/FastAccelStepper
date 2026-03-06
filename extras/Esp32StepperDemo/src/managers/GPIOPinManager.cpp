#include "GPIOPinManager.h"
#include "driver/gpio.h"

static bool isValidGpio(uint8_t pin) {
#if CONFIG_IDF_TARGET_ESP32
  if (pin >= 24 && pin <= 27) return false;
  if (pin >= 28 && pin <= 31) return false;
  if (pin == 20) return false;
  return pin < GPIO_NUM_MAX;
#else
  return pin < GPIO_NUM_MAX;
#endif
}

GPIOPinManager::GPIOPinManager() { discoverPins(); }

void GPIOPinManager::discoverPins() {
  for (int i = 0; i < NUM_GPIO_PINS; i++) {
    pins[i].pin_number = i;
    pins[i].capabilities = getPinCapabilities(i);
    pins[i].is_reserved = isPinReserved(i);
    pins[i].is_assigned = false;
    pins[i].assigned_to_stepper = -1;
    pins[i].type = PIN_TYPE_NONE;
    strlcpy(pins[i].name, getPinName(i), sizeof(pins[i].name));
  }
}

uint8_t GPIOPinManager::getPinCapabilities(uint8_t pin) {
  if (pin >= NUM_GPIO_PINS) {
    return PIN_CAP_NONE;
  }

  if (isPinReserved(pin)) {
    return PIN_CAP_NONE;
  }

  uint8_t caps = PIN_CAP_NONE;

  caps |= PIN_CAP_INPUT;

  if (pin < 34 && !isPinReserved(pin)) {
    caps |= PIN_CAP_OUTPUT;
  }

  if ((pin >= 32 && pin <= 39) || (pin >= 0 && pin <= 5) ||
      (pin >= 12 && pin <= 15) || (pin >= 25 && pin <= 27)) {
    caps |= PIN_CAP_ADC;
  }

  if (pin <= 9 || (pin >= 32 && pin <= 33)) {
    caps |= PIN_CAP_TOUCH;
  }

  if (!isPinReserved(pin) && pin < 34) {
    caps |= PIN_CAP_RMT;
    caps |= PIN_CAP_I2S;
  }

  return caps;
}

bool GPIOPinManager::isPinReserved(uint8_t pin) {
  if (pin >= 6 && pin <= 11) {
    return true;
  }
  if (pin == 1 || pin == 3) {
    return true;
  }
  if (pin == 0) {
    return true;
  }
  return false;
}

const char* GPIOPinManager::getPinName(uint8_t pin) {
  static char name[16];

  if (pin == 0) {
    snprintf(name, sizeof(name), "GPIO0 (Boot)");
  } else if (pin == 1) {
    snprintf(name, sizeof(name), "GPIO1 (TX)");
  } else if (pin == 3) {
    snprintf(name, sizeof(name), "GPIO3 (RX)");
  } else if (pin >= 6 && pin <= 11) {
    snprintf(name, sizeof(name), "GPIO%d (Flash)", pin);
  } else if (pin >= 34 && pin <= 39) {
    snprintf(name, sizeof(name), "GPIO%d (IN)", pin);
  } else {
    snprintf(name, sizeof(name), "GPIO%d", pin);
  }

  return name;
}

bool GPIOPinManager::setPinMode(uint8_t pin, PinType type) {
  if (pin >= NUM_GPIO_PINS || pins[pin].is_reserved) {
    return false;
  }

  switch (type) {
    case PIN_TYPE_GPIO_INPUT:
      ::pinMode(pin, INPUT);
      pins[pin].type = PIN_TYPE_GPIO_INPUT;
      break;
    case PIN_TYPE_GPIO_OUTPUT:
      ::pinMode(pin, OUTPUT);
      pins[pin].type = PIN_TYPE_GPIO_OUTPUT;
      break;
    case PIN_TYPE_NONE:
      pins[pin].type = PIN_TYPE_NONE;
      break;
    default:
      return false;
  }

  return true;
}

bool GPIOPinManager::setPinValue(uint8_t pin, bool value) {
  if (pin >= NUM_GPIO_PINS || pins[pin].is_reserved) {
    return false;
  }

  if (pins[pin].type != PIN_TYPE_GPIO_OUTPUT) {
    if (!setPinMode(pin, PIN_TYPE_GPIO_OUTPUT)) {
      return false;
    }
  }

  digitalWrite(pin, value ? HIGH : LOW);
  return true;
}

bool GPIOPinManager::getPinValue(uint8_t pin) {
  if (pin >= NUM_GPIO_PINS) {
    return false;
  }
  return digitalRead(pin) == HIGH;
}

bool GPIOPinManager::togglePin(uint8_t pin) {
  if (pin >= NUM_GPIO_PINS || pins[pin].is_reserved) {
    return false;
  }

  if (pins[pin].type != PIN_TYPE_GPIO_OUTPUT) {
    if (!setPinMode(pin, PIN_TYPE_GPIO_OUTPUT)) {
      return false;
    }
  }

  bool currentValue = digitalRead(pin) == HIGH;
  digitalWrite(pin, currentValue ? LOW : HIGH);
  return true;
}

PinDefinition* GPIOPinManager::getPinDefinition(uint8_t pin) {
  if (pin >= NUM_GPIO_PINS) {
    return nullptr;
  }
  return &pins[pin];
}

std::vector<PinDefinition*> GPIOPinManager::getAvailablePins(
    uint8_t capability_filter) {
  std::vector<PinDefinition*> result;

  for (int i = 0; i < NUM_GPIO_PINS; i++) {
    if (!pins[i].is_reserved && !pins[i].is_assigned) {
      if (capability_filter == PIN_CAP_NONE ||
          (pins[i].capabilities & capability_filter)) {
        result.push_back(&pins[i]);
      }
    }
  }

  return result;
}

std::vector<PinDefinition*> GPIOPinManager::getAllPins() {
  std::vector<PinDefinition*> result;

  for (int i = 0; i < NUM_GPIO_PINS; i++) {
    result.push_back(&pins[i]);
  }

  return result;
}

bool GPIOPinManager::assignPinToStepper(uint8_t pin, uint8_t stepper_id) {
  if (pin >= NUM_GPIO_PINS || pins[pin].is_reserved || pins[pin].is_assigned) {
    return false;
  }

  pins[pin].is_assigned = true;
  pins[pin].assigned_to_stepper = stepper_id;
  return true;
}

void GPIOPinManager::unassignPin(uint8_t pin) {
  if (pin < NUM_GPIO_PINS) {
    pins[pin].is_assigned = false;
    pins[pin].assigned_to_stepper = -1;
  }
}

bool GPIOPinManager::isPinAvailable(uint8_t pin) {
  if (pin >= NUM_GPIO_PINS) {
    return false;
  }
  return !pins[pin].is_reserved && !pins[pin].is_assigned;
}
