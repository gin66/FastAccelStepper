# Installation & Build Integration

[Back to README](../../README.md)

## Arduino

Available through the Arduino Library Manager.

[Arduino library manager log](https://downloads.arduino.cc/libraries/logs/github.com/gin66/FastAccelStepper/)

## PlatformIO

[Library on platformio](https://registry.platformio.org/libraries/gin66/FastAccelStepper)

If you prefer platformio and you are running Linux, then platformio version of
the examples are created by executing

```
ci/build-platformio.sh
```

This will create a directory pio_dirs, which contains all examples. Can be
executed by e.g.

```
cd pio_dirs/StepperDemo
pio run -e avr --target upload --upload-port /dev/ttyUSB0
```

## ESP-IDF

A `CMakeLists.txt` is provided to use FastAccelStepper as an ESP-IDF component.
Clone it into the `components/` directory in the root of your project and build
as usual. You must have Arduino available as a component.
[See this](https://docs.espressif.com/projects/arduino-esp32/en/latest/esp-idf_component.html)
for instructions on how to set that up. Tested as ESP-IDF component on PlatformIO
Espressif32 Platform v3.3.2.

For any questions/support please contact [gagank1](https://github.com/gagank1),
as I do not use esp-idf.

## Platform-specific notes

* AVR ATmega2560 timer selection: see [platforms/avr.md](platforms/avr.md).
* Pico platform workaround for PIO: see [platforms/pico.md](platforms/pico.md).
* ESP-IDF version compatibility: see [platforms/esp32.md](platforms/esp32.md).
