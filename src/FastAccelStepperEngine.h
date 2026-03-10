#ifndef FASTACCELSTEPPER_ENGINE_H
#define FASTACCELSTEPPER_ENGINE_H
#include <stdint.h>
#include "fas_arch/common.h"

// # FastAccelStepper
//
// FastAccelStepper is a high speed alternative for the
// [AccelStepper library](http://www.airspayce.com/mikem/arduino/AccelStepper/).
// Supported are avr (ATmega 168/328/P, ATmega2560), esp32 and atmelsam due.
//
// Here is a basic example to run a stepper from position 0 to 1000 and back
// again to 0.
// ```
// #include <FastAccelStepper.h>
//
// FastAccelStepperEngine engine = FastAccelStepperEngine();
// FastAccelStepper *stepper = NULL;
//
// #define dirPinStepper    5
// #define enablePinStepper 6
// #define stepPinStepper   9
// void setup() {
//    engine.init();
//    stepper = engine.stepperConnectToPin(stepPinStepper);
//    if (stepper) {
//       stepper->setDirectionPin(dirPinStepper);
//       stepper->setEnablePin(enablePinStepper);
//       stepper->setAutoEnable(true);
//
//       stepper->setSpeedInHz(500);
//       stepper->setAcceleration(100);
//       stepper->moveTo(1000, true);
//       stepper->moveTo(0, true);
//    }
// }
//
// void loop() {}
// ```

class FastAccelStepper;

class FastAccelStepperEngine {
  //
  // ## FastAccelStepperEngine
  //
  // This engine - actually a factory - provides you with instances of steppers.

 public:
  // ### Initialization
  //
  // The FastAccelStepperEngine is declared with FastAccelStepperEngine().
  // This is to occupy the needed memory.
  // ```cpp
  // FastAccelStepperEngine engine = FastAccelStepperEngine();
  // ```
  // But it still needs to be initialized.
  // For this init shall be used:
  // ```cpp
  // void setup() {
  //    engine.init();
  // }
  // ```

#if defined(SUPPORT_CPU_AFFINITY)
  // In a multitasking and multicore system like ESP32, the steppers are
  // controlled by a continuously running task. This task can be fixed to one
  // CPU core with this modified init()-call. ESP32 implementation detail: For
  // values 0 and 1, xTaskCreatePinnedToCore() is used, or else xTaskCreate()
  void init(uint8_t cpu_core = 255);
#else
  void init();
#endif

  // ### Creation of FastAccelStepper
  //
  // Using a call to `stepperConnectToPin()` a FastAccelStepper instance is
  // created. This call tells the stepper, which step pin to use. As the
  // hardware may have limitations - e.g. no stepper resources anymore, or the
  // step pin cannot be used, then NULL is returned. So it is advised to check
  // the return value of this call.
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  // For e.g. esp32, there are two types of driver.
  // One using mcpwm and pcnt module. And another using rmt module.
  // This call allows to select the respective driver
#define DRIVER_MCPWM_PCNT FasDriver::MCPWM_PCNT
#define DRIVER_RMT FasDriver::RMT
#if defined(SUPPORT_ESP32_I2S)
  // For esp32, there is also an I2S based driver available.
  // DRIVER_I2S_DIRECT uses the I2S module directly.
  // DRIVER_I2S_MUX uses the I2S module with a multiplexer.
  // initI2sMux() must be called before using DRIVER_I2S_MUX.
#define DRIVER_I2S_DIRECT FasDriver::I2S_DIRECT
#define DRIVER_I2S_MUX FasDriver::I2S_MUX
  bool initI2sMux(uint8_t data_pin, uint8_t bclk_pin, uint8_t ws_pin);
  void i2sMuxSetBit(uint8_t slot, bool value);
  bool i2sMuxGetBit(uint8_t slot);
#endif
#define DRIVER_DONT_CARE FasDriver::DONT_CARE
  FastAccelStepper* stepperConnectToPin(
      uint8_t step_pin, FasDriver driver_type = DRIVER_DONT_CARE);
#else
  FastAccelStepper* stepperConnectToPin(uint8_t step_pin);
#endif

  // Comments to valid pins:
  //
  // clang-format off
  // | Device          | Comment                                                                                           |
  // |:----------------|:--------------------------------------------------------------------------------------------------|
  // | ESP32           | Every output capable GPIO can be used                                                             |
  // | ESP32S2         | Every output capable GPIO can be used                                                             |
  // | Atmega168/328/p | Only the pins connected to OC1A and OC1B are allowed                                              |
  // | Atmega2560      | Only the pins connected to OC4A, OC4B and OC4C are allowed.                                       |
  // | Atmega32u4      | Only the pins connected to OC1A, OC1B and OC1C are allowed                                        |
  // | Atmel SAM       | This can be one of each group of pins: 34/67/74/35, 17/36/72/37/42, 40/64/69/41, 9, 8/44, 7/45, 6 |
  // clang-format on

#if defined(SUPPORT_TASK_RATE_CHANGE)
  // For e.g. esp32 the repetition rate of the stepper task can be changed.
  // The default delay is 4ms.
  //
  // The steppertask is looping with:
  //       manageSteppers()
  //       wdt_reset()
  //       delay()
  //
  // The actual repetition rate of the stepper task is delay + execution time of
  // manageSteppers()
  //
  // This function is primary of interest in conjunction with
  // setForwardPlanningTimeInMs(). If the delay is larger then forward planning
  // time, then the stepper queue will always run out of commands, which lead to
  // a sudden stop of the motor. If the delay is 0, then the stepper task will
  // constantly looping, which may lead to the task blocking other tasks.
  // Consequently, this function is intended for advanced users.
  //
  // There is not planned to test this functionality, because automatic testing
  // is only available for avr devices and those continue to use fixed 4ms rate.
  //
  // Please be aware, that the configured tick rate aka portTICK_PERIOD_MS is
  // relevant. Apparently, arduino-esp32 has FreeRTOS configured to have a
  // tick-rate of 1000Hz
  inline void task_rate(uint8_t delay_ms) { _delay_ms = delay_ms; };
  uint8_t _delay_ms;
#endif

  // ## External Pins
  //
  // If the direction/enable pins are e.g. connected via external HW (shift
  // registers), then an external callback function can be supplied. The
  // supplied value is either LOW or HIGH. The return value shall be the status
  // of the pin (false for LOW or true for HIGH). If returned value and supplied
  // value do not match, the stepper does not continue, but calls this function
  // again.
  //
  // This function is called from cyclic task/interrupt with 4ms rate, which
  // creates the commands to put into the command queue. Thus the supplied
  // function should take much less time than 4ms. Otherwise there is risk, that
  // other running steppers are running out of commands in the queue. If this
  // takes longer, then the function should be offloaded and return the new
  // status, after the pin change has been successfully completed.
  //
  // The callback has to be called on the FastAccelStepperEngine.
  // See examples/ExternalCall
  //
  // Stepperpins (enable or direction), which should use this external callback,
  // need to be or'ed with PIN_EXTERNAL_FLAG ! FastAccelStepper uses this flag
  // to determine, if a pin is external or internal.
  void setExternalCallForPin(bool (*func)(uint8_t pin, uint8_t value));

  // ### Debug LED
  //
  // If blinking of a LED is required to indicate, the stepper controller is
  // still running, then the port. to which the LED is connected, can be told to
  // the engine. The periodic task will let the associated LED blink with 1 Hz
  void setDebugLed(uint8_t ledPin);

  /* This should be only called from ISR or stepper task. So do not call it */
  void manageSteppers();

 private:
  bool isDirPinBusy(uint8_t dirPin, uint8_t except_stepper);

  uint8_t _stepper_cnt;
  FastAccelStepper* _stepper[MAX_STEPPER];

  bool (*_externalCallForPin)(uint8_t pin, uint8_t value);

#if defined(SUPPORT_RP_PICO)
  uint8_t claimed_pios;
  PIO pio[NUM_PIOS];
#endif

  friend class FastAccelStepper;
  friend class StepperQueue;
};

#endif
