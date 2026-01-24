#include <Arduino.h>

#include <pico/stdlib.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <FreeRTOS.h>

#include "pico_pio.h"

pio_program_t pio_program;
PIO pio;
uint sm;
uint offset;

// #define PIO_STEP_GPIO LED_BUILTIN
#define PIO_STEP_GPIO 14
#define PIO_DIR_GPIO 15

uint32_t start_ms;

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < 6; i++) {
    Serial.println(i);
    delay(1000);
  }

  // initialize the digital pin as an output.
  // pinMode(LED_BUILTIN, OUTPUT);

  stepper_pio_program* program = stepper_make_program();
  Serial.print("Program length at most 32: ");
  Serial.println(program->pc);

  if (program->pc > 32) {
    while (true) {
      Serial.print("Program too long:");
      Serial.println(program->pc);
      delay(1000);
    }
  }

  pio_program.instructions = program->code;
  pio_program.length = program->pc;
  pio_program.origin = 0;
  pio_program.pio_version = 0;
  pio_program.used_gpio_ranges = 0;
  bool rc = pio_claim_free_sm_and_add_program_for_gpio_range(
      &pio_program, &pio, &sm, &offset, PIO_STEP_GPIO, 1, true);

  pio_sm_config c = pio_get_default_sm_config();
  // Map the state machine's OUT pin group to one pin, namely the `pin`
  // parameter to this function.
  sm_config_set_jmp_pin(&c,
                        PIO_STEP_GPIO);  // Step pin read back for double period
  sm_config_set_out_pins(&c, PIO_DIR_GPIO, 1);   // Direction pin via out
  sm_config_set_set_pins(&c, PIO_STEP_GPIO, 1);  // Step pin via set
  sm_config_set_wrap(&c, program->wrap_target, program->wrap_at);
  // Set this pin's GPIO function (connect PIO to the pad)
  pio_gpio_init(pio, PIO_STEP_GPIO);
  pio_gpio_init(pio, PIO_DIR_GPIO);

  // Load our configuration, and jump to the start of the program
  pio_sm_init(pio, sm, offset, &c);
  // Set the pin direction to output at the PIO
  pio_sm_set_consecutive_pindirs(pio, sm, PIO_STEP_GPIO, 1, true);
  pio_sm_set_consecutive_pindirs(pio, sm, PIO_DIR_GPIO, 1, true);

  Serial.print("Clock=");
  Serial.println(clock_get_hz(clk_sys));  // 150.000.000/128.000.000=1+44/256
  //  pio_sm_set_clkdiv(pio,sm,150.0f/128.0f);

  // Set the state machine running
  pio_sm_set_enabled(pio, sm, true);  // sm is running, otherwise loop() stops

  pio_sm_clear_fifos(pio, sm);
  pio_sm_restart(pio, sm);

  Serial.print("Loaded=");
  Serial.println(rc ? "OK" : "ERROR");
}

// the loop routine runs over and over again forever:
uint32_t cnt = 0;  // wait fifo full
uint32_t pulses = 0;
bool forward = true;
void loop() {
  Serial.print("Hallo pc=");
  Serial.print(pio_sm_get_pc(pio, sm));
  Serial.print(' ');
  //
  // 5 steps and period 4*1000*1000:
  // expected is 128.000.000/(5*4.000.000*cycles)=6.4/1s/cycles
  // result is 937.5ms/command approx 6.8 cycles/command = 800ms*150/128
  //
  // without clkdiv we have 150.000.000/(5*4.000.000*cycles)=7.5/1s/cycles
  // result is 800ms/command approx 9.375 cycles/command
  // result is 160ms/step => 25.000.000/step
  //
  // 25 steps and period 4*1000*1000:
  // without clkdiv we have 150.000.000/(25*4.000.000*cycles)=1.5/1s/cycles
  // result is 4000ms/command
  // result is 160ms/step => 25.000.000/step
  //
  // 250 steps and period 4*1000:
  // without clkdiv we have 150.000.000/(250*4.000*cycles)=150/1s/cycles
  // result is 40.06ms/command
  uint8_t steps = 7;
  uint32_t period = 4 * 1000;

  period = stepper_calc_period(forward, steps, 64000);  // 4ms
  uint32_t entry = (period << 9) | (forward ? 0 : 256) | steps;
  pio_sm_put_blocking(pio, sm, entry);

  // this pause is toggling for couple of repeats beween all low and one
  // high/low part.
  steps = 0;
  period = stepper_calc_period(forward, steps, 64000);  // 4ms
  entry = (period << 9) | (forward ? 0 : 256) | steps;
  pio_sm_put_blocking(pio, sm, entry);

  uint32_t end = millis();
  if (cnt <= 10) {
    start_ms = end;
  } else {
    uint32_t dt = end - start_ms;
    Serial.print("cnt=");
    Serial.print(cnt);
    Serial.print(": ");
    Serial.print((float)dt / (float)(cnt - 10));
    Serial.print("ms/command ");
    float cmd_time;
    if (steps > 0) {
      uint32_t cycles = 6 * period + 26;
      if (!forward) {
        cycles += 2;
      }
      cycles *= steps;
      cycles += 8;
      cmd_time = (float)(cycles) / 150.0;
    } else {
      uint32_t cycles = 3 * period + 14;
      cmd_time = (float)(cycles) / 150.0;
    }
    Serial.print(cmd_time);
    Serial.print("µs");
  }
  cnt++;

  if ((cnt % 100) == 0) {
    forward = !forward;
  }

  if ((cnt % 10) == 0) {
    // Drop old values in fifo
    for (uint8_t i = 0; i <= 4; i++) {
      pio_sm_get(pio, sm);
    }
    // we expect a new value in the fifo within less than 1 us. So the
    // Serial.print is sufficient delay
    Serial.print("  position=");
    Serial.print(pio_sm_get(pio, sm));
  }
  Serial.println();
}
