# Hardware & Pin Connection

[Back to README](../../README.md)

The stepper motors should be connected via a driver IC (like A4988) with a 1, 2
or 3-wire connection:

* Step Signal
  - avr atmega168/328/p: only Pin 9 and 10.
  - avr atmega32u4: only Pin 9, 10 and 11.
  - avr atmega2560: only Pin 6, 7 and 8.
      On platformio, this can be changed to other triples: 11/12/13 Timer 1,
      5/2/3 Timer 3 or 46/45/44 Timer 5 with `FAS_TIMER_MODULE` setting.
  - esp32: This can be any output capable port pin, or use I2S for additional steppers:
    * I2S Mux mode: up to 32 additional steppers via external demultiplexer (IDF ≥5.3)
    * I2S Direct mode: 1-3 additional steppers using I2S controllers directly (IDF ≥5.3)
  - pico: Any GPIO up to 31
  - atmel sam due: This can be one of each group of pins: 34/67/74/35, 17/36/72/37/42, 40/64/69/41, 9, 8/44, 7/45, 6
  - samd51: Any pin with a TCC waveform output in the board's variant table
    (checked at `stepperConnectToPin()`), one stepper per TCC instance
  - Step should be done on transition Low to High. High time will be only a few us.
  On esp32 the high time is for slow speed fixed to ~2ms and high speed to 50% duty cycle.
  For pico direction delay is recommended
* Direction Signal (optional)
  - This can be any output capable port pin.
  - esp32: Can also use I2S Mux slots for direction control (IDF ≥5.3)
  - pico: Any GPIO up to 31
  - Position counting up on direction pin high or low, as per optional parameter
    to `setDirectionPin()`. Default is high.
  - With external callback on esp32 derivates, even shift register outputs can be used
* Enable Signal (optional)
  - This can be any output capable port pin.
  - esp32: Can also use I2S Mux slots for enable control (IDF ≥5.3)
  - Stepper will be enabled on pin high or low, as per optional parameter to
    `setEnablePin()`. Default is low.
  - With external callback, even shift register outputs can be used

## Per-platform documentation

| Platform | Doc |
|----------|-----|
| AVR (ATmega 168/328/32u4/2560) | [avr.md](platforms/avr.md) |
| ESP32 / S2 / S3 / C3 / C6 / P4 | [esp32.md](platforms/esp32.md) |
| Raspberry Pi Pico / Pico 2 | [pico.md](platforms/pico.md) |
| Atmel SAM Due | [sam.md](platforms/sam.md) |
| Microchip SAMD51 | [samd51.md](platforms/samd51.md) |
| Teensy 4.0/4.1 (experimental) | [teensy.md](platforms/teensy.md) |
