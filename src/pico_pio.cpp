// Analyze the following code for raspberry pi pico to create a pio program. Understand the register changes and cycle count of the instructions. Based on your understanding create a mermaid diagram with states describing the current cycle count and register content. The transitions shall give info about the executed instructions and their respecive instruction cycles. Instructions without jump inbetween can be intelligently combined. Any cycle mismatch of parallel paths shall be highlighted. In the dir parallel path two cycle info may be needed. Important to note,that wrap is not in use. Every instruction is one cycle, unless an additional delay in cycles is given. The code is:
#if defined(PICO_RP2040) || defined(PICO_RP2350)
#include <Arduino.h>

#include <pico/stdlib.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>

#include "pico_pio.h"

static stepper_pio_program program;

static void add_step(uint instruction) {
  if (program.pc < 32) {
    program.code[program.pc++] = (uint16_t)instruction;
  }
}

// fifo data NEW is
//   FEDCBA9876543210fedcba9876543210 = 32bits
//          <- period  ->|  |loopcnt|
//                         ^-------------Dir pin value
//                        ^--------------Count up if 1
//
// For a pause the loop_cnt shall be 1.
// For steps, the loop_cnt = 2*steps
// The LSB of loop_cnt is the inverted output of the step pin.
// In other words, after loop_cnt decrement, the LSB shall equal step pin value.
//
// Timing: 27+3*period cycles per loop iteration
//         Pause: 27+3*period cycles
//         Steps: (27+3*period)*steps*2 cycles
//
//         Combined: (27+3*period)*loop_cnt = cycles per cmd
//
//
uint32_t stepper_calc_period(uint8_t steps,
                             uint16_t cycles_in_16th_us) {
  uint32_t cycles_in_80MHz = cycles_in_16th_us;
  // should be yielding multiplication with 5
  cycles_in_80MHz *= program.sys_clk/1000000;
  cycles_in_80MHz /= 16000000/1000000;
  if (steps > 1) {
    cycles_in_80MHz *= steps;
  }
  uint16_t loop_cnt = steps == 0 ? 1 : 2 * (uint16_t)steps; // pause or steps
  cycles_in_80MHz /= loop_cnt;
  cycles_in_80MHz -= 27; // 27 cycles for the loop overhead
  cycles_in_80MHz /= 3;
  return cycles_in_80MHz;
}
uint32_t stepper_make_fifo_entry(bool dir_high, bool count_up, uint8_t steps,
                                 uint16_t cycles_16th_us) {
  uint32_t period = stepper_calc_period(steps, cycles_16th_us);
  uint16_t loop_cnt = steps == 0 ? 1 : 2 * (uint16_t)steps; // pause or steps
  uint32_t entry =
      (period << 11) | (count_up ? 1024 : 0) | (dir_high ? 512 : 0) | loop_cnt;
  //char out[200];
  //sprintf(out, "stepper_make_fifo_entry: dir_high: %d, count_up: %d, steps: %d, cycles_16th_us: %d, period: %d, loop_cnt: %d, entry: 0x%08X",
  //       dir_high, count_up, steps, cycles_16th_us, period, loop_cnt, entry);
  //Serial.println(out);
  return entry;
}

// clang-format off
stepper_pio_program *stepper_make_program() {
  program.sys_clk = clock_get_hz(clk_sys);
  program.pc = 0;
  // We assume, that isr is cleared on sm start !
  // ISR=position=0, X=invalid, Y=invalid, OSR=invalid
  uint8_t label_main_loop = program.pc;
  // ISR=position, X=invalid, Y=invalid, OSR=invalid
  // Blocking load of next command
  add_step(pio_encode_pull(false, true));
  // ISR=position, X=invalid, Y=invalid, OSR=period:up:dir:loop_cnt
  // copy osr to x
  add_step(pio_encode_mov(pio_x, pio_osr));
  // ISR=position, X=period:up:dir:loop_cnt, Y=invalid, OSR=period:up:dir:loop_cnt
  // remove loop_cnt from osr (9 bits)
  add_step(pio_encode_out(pio_null, 9));
  // ISR=position, X=period:up:dir:loop_cnt, Y=invalid, OSR=period:up:dir
  uint8_t label_step_loop = program.pc;
  // Move the dir pin value in y
  add_step(pio_encode_out(pio_y, 1));
  // ISR=position, X=period:up:dir:loop_cnt, Y=dir, OSR=period:up
  add_step(pio_encode_jmp_not_y(program.pc + 3));
  add_step(pio_encode_set(pio_pins, 1));  // set dir pin to 1
  add_step(pio_encode_jmp(program.pc + 2));
  add_step(pio_encode_set(pio_pins, 0) | pio_encode_delay(1));  // set dir pin to 0
  // Get the UP flag into Y for position update below
  add_step(pio_encode_out(pio_y, 1));
  // ISR=position, X=period:up:dir:loop_cnt, Y=up, OSR=[period]
  // decrement X in order to reduce loop_cnt by 1
  add_step(pio_encode_jmp_x_dec(program.pc + 1));
  // ISR=position, X=period:up:dir:loop_cnt-1, Y=up, OSR=[period]
  // set step output to LSB of loop_cnt defined by sm_config_set_out_pins
  add_step(pio_encode_mov(pio_pins, pio_x));
  // restore period:remaining_loop_cnt:dir in osr
  add_step(pio_encode_mov(pio_osr, pio_x));
  // ISR=position, X=period:up:dir:loop_cnt-1, Y=up, OSR=period:up:dir:loop_cnt-1
  // If step pin is one, we need to update position
  add_step(pio_encode_jmp_pin(program.pc + 2));
  // Step or Pause is completed
  uint8_t forward_jump_1 = program.pc;
  add_step(pio_encode_jmp(0) | pio_encode_delay(7));
  //
  // Perform increment by one with only invert and decrement:
  //     position  0 1 2 .. d e f ef
  //     invert    f e d .. 2 1 0 10
  //     decrement e d c .. 1 0 f 0f
  //     invert    1 2 3 .. e f 0 f0
  //
  // The following code section needs always 7 cycles
  // Load from isr the position into x and use inverted position, if dir=0
  add_step(pio_encode_mov(pio_x, pio_isr));
  add_step(pio_encode_jmp_not_y(program.pc + 2));  // jump if y = 0
  add_step(pio_encode_mov_not(pio_x, pio_isr));
  // ISR=position, X=position/-position, Y=up, OSR=period:up:dir:loop_cnt-1
  add_step(pio_encode_jmp_x_dec(program.pc + 1));  // decrement x
  // Store updated position in isr and invert, if dir=0
  add_step(pio_encode_mov_not(pio_isr, pio_x));
  add_step(pio_encode_jmp_y_dec(program.pc + 2));  // jump if y != 0
  add_step(pio_encode_mov(pio_isr, pio_x));
  // ISR=position+/-1, X=[position+/-1], Y=[up+0/-1], OSR=period:up:dir:loop_cnt-1
  // restore x from osr
  add_step(pio_encode_mov(pio_x, pio_osr));
  // ISR=position+/-1, X=period:up:dir:loop_cnt-1, Y=[up+0/-1], OSR=period:up:dir:loop_cnt-1

  uint8_t forward_jump_1_target = program.pc; // T=20
  // ISR=position, X=period:up:dir:loop_cnt, Y=[up], OSR=period:up:dir:loop_cnt
  // isolate period into osr
  add_step(pio_encode_out(pio_null, 9+1+1));
  // ISR=position, X=period:up:dir:loop_cnt, Y=[up], OSR=period
  // copy period to y
  add_step(pio_encode_mov(pio_y, pio_osr));
  // ISR=position, X=period:up:dir:loop_cnt, Y=period, OSR=period
  // store period/remaining_loop_cnt in osr
  add_step(pio_encode_mov(pio_osr, pio_x));
  // ISR=position, X=[period:up:dir:loop_cnt], Y=period, OSR=period:up:dir:loop_cnt 
  // move position into x
  add_step(pio_encode_mov(pio_x, pio_isr));
  // ISR=position, X=position, Y=period, OSR=period:up:dir:loop_cnt
  uint8_t label_period_loop = program.pc;    // T=24
  // output position in fifo
  add_step(pio_encode_push(false, false));
  // ISR=0, X=position, Y=period, OSR=period:up:dir:loop_cnt
  // restore position in ISR
  add_step(pio_encode_mov(pio_isr, pio_x));
  // ISR=position, X=position, Y=period, OSR=period:up:dir:loop_cnt
  // loop until period in y is 0
  add_step(pio_encode_jmp_y_dec(label_period_loop)); // T=24+3*N
  // ISR=position, X=[position], Y=[0], OSR=period:up:dir:loop_cnt
  // store period:up:dir:loop_cnt in x
  add_step(pio_encode_mov(pio_x, pio_osr));
  // ISR=position, X=period:up:dir:loop_cnt, Y=0, OSR=period:up:dir:loop_cnt
  // get loop_cnt into y (9 bits)
  add_step(pio_encode_out(pio_y, 9));
  // ISR=position, X=period:up:dir:loop_cnt, Y=loop_cnt, OSR=period:up:dir
  // if loop_cnt is zero go to main loop
  add_step(pio_encode_jmp_not_y(label_main_loop)); // T=27+3*N
  // Otherwise continue loop using wrap around
  // restore x to osr
  add_step(pio_encode_jmp(label_step_loop) | pio_encode_delay(2));
  // ISR=position, X=[period:up:dir:loop_cnt], Y=loop_cnt, OSR=period:up:dir:loop_cnt
  // wrap around does not need a cycle
  
  // patch forward jump address
  program.code[forward_jump_1] |= forward_jump_1_target;

  // In this code we do not actually use the wrap around feature of PIO.
  program.wrap_at = program.pc - 1;
  program.wrap_target = label_step_loop;

  // delay(2000);
  // Serial.println("stepper_make_program: pc: " + String(program.pc) +
  //             ", wrap_at: " + String(program.wrap_at) +
  //             ", wrap_target: " + String(program.wrap_target));
  return &program;
}
// clang-format on
#endif
