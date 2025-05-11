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

// fifo data is
//   FEDCBA9876543210fedcba9876543210 = 32bits
//          <- period   ->|  |steps |
//                          ^------------Dir pin value
//                         ^-------------Count up if 1
// For a pause the period is executed once and for a step: steps*2*period
//
//  6 cycles from start to including first jump for pause
// 12 cycles to second unconditional jump
//  2 additional cycles on DIR=1
//  1 cycle for each second step loop
//  4 cycles before period loop
//  3 cycles for period loop including jump
//  2 cycles including jump for step loop
//  2 cycles including jump for per step repeat
//
// Cycles:
//   Pause: 6+1+4+3*period+2+2=15+3*period
//   Steps:
//   6+(12+2*DIR+(4+3*period+2)*2+2)*steps+2=8+6*period*steps+26*steps+2*DIR*steps
//
uint32_t stepper_calc_period(bool dir_high, uint8_t steps,
                             uint16_t cycles_16th_us) {
  float cmd_time_s = float(cycles_16th_us) / 16000000.0;
  if (steps == 0) {
    uint32_t target_cycles = uint32_t(cmd_time_s * program.sys_clk);
    target_cycles -= 15;
    target_cycles /= 3;
    return target_cycles;
  } else {
    cmd_time_s *= steps;
    uint32_t target_cycles = uint32_t(cmd_time_s * program.sys_clk);
    target_cycles -= 8;
    if (dir_high) {
      target_cycles -= 2 * steps;
    }
    target_cycles -= 26 * steps;
    target_cycles /= 6 * steps;
    return target_cycles;
  }
}
uint32_t stepper_make_fifo_entry(bool dir_high, bool count_up, uint8_t steps,
                                 uint16_t cycles_16th_us) {
  uint32_t period = stepper_calc_period(dir_high, steps, cycles_16th_us);
  uint32_t entry =
      (period << 10) | (count_up ? 512 : 0) | (dir_high ? 256 : 0) | steps;
  return entry;
}

//
stepper_pio_program *stepper_make_program() {
  program.sys_clk = clock_get_hz(clk_sys);
  program.pc = 0;
  // We assume, that isr is cleared on sm start !
  // ISR=position=0, X=invalid, Y=invalid, OSR=invalid
  uint8_t label_main_loop = program.pc;
  // ISR=position, X=invalid, Y=invalid, OSR=invalid
  // Blocking load of next command
  add_step(pio_encode_pull(false, true));
  // ISR=position, X=invalid, Y=invalid, OSR=period:up:dir:steps_to_do
  // copy osr to x
  add_step(pio_encode_mov(pio_x, pio_osr));
  // ISR=position, X=period:up:dir:steps_to_do, Y=invalid,
  // OSR=period:up:dir:steps_to_do get steps_to_do into y (8 bits)
  add_step(pio_encode_out(pio_y, 8));
  // ISR=position, X=period:up:dir:steps_to_do, Y=steps_to_do, OSR=period:up:dir
  // Move the dir pin value in lsb to pins
  add_step(pio_encode_out(pio_pins, 1));
  // ISR=position, X=period:up:dir:steps_to_do, Y=steps_to_do, OSR=[period:up]
  uint8_t label_loop_one_step = program.pc;
  // ISR=position, X=period:up:dir:steps_to_do, Y=steps_to_do, OSR=undefined
  // store period:remaining_steps_to_do:dir in osr
  add_step(pio_encode_mov(pio_osr, pio_x));
  // ISR=position, X=period:up:dir:steps_to_do, Y=steps_to_do,
  // OSR=period:up:dir:steps_to_do Forward jump, if steps_to_do is zero, which
  // clears step and enters loop
  uint8_t forward_jump_1 = program.pc;
  add_step(pio_encode_jmp_not_y(0));
  // ISR=position, X=period:up:dir:steps_to_do, Y=[steps_to_do],
  // OSR=period:up:dir:steps_to_do Get the up flag into Y for position update
  // below
  add_step(pio_encode_out(pio_null, 9));
  // ISR=position, X=period:up:dir:steps_to_do, Y=[steps_to_do], OSR=period:up
  add_step(pio_encode_out(pio_y, 1));
  // ISR=position, X=period:up:dir:steps_to_do, Y=up, OSR=[period]
  // decrement X in order to reduce steps_to_do by 1
  add_step(pio_encode_jmp_x_dec(program.pc + 1));
  // ISR=position, X=period:up:dir:steps_to_do-1, Y=up, OSR=[period]
  // store updated period:remaining_steps_to_do:dir in osr
  add_step(pio_encode_mov(pio_osr, pio_x));
  // ISR=position, X=period:up:dir:steps_to_do-1, Y=up,
  // OSR=period:up:dir:steps_to_do-1 set step output to 1 aka HIGH
  add_step(pio_encode_set(pio_pins, 1));
  // Step will be performed, so modify position using x
  //
  // Perform increment by one with only invert and decrement:
  //     position  0 1 2 .. d e f ef
  //     invert    f e d .. 2 1 0 10
  //     decrement e d c .. 1 0 f 0f
  //     invert    1 2 3 .. e f 0 f0
  //
  // Load from isr the position into x and use inverted position, if dir=0
  add_step(pio_encode_mov(pio_x, pio_isr));
  add_step(pio_encode_jmp_not_y(program.pc + 2));  // jump if y = 0
  add_step(pio_encode_mov_not(pio_x, pio_isr));
  // ISR=position, X=position/-position, Y=up, OSR=period:up:dir:steps_to_do-1
  add_step(pio_encode_jmp_x_dec(program.pc + 1));  // decrement x
  // Store updated position in isr and invert, if dir=0
  add_step(pio_encode_mov(pio_isr, pio_x));
  add_step(pio_encode_jmp_not_y(program.pc + 2));  // jump if y = 0
  add_step(pio_encode_mov_not(pio_isr, pio_x));
  // ISR=position+/-1, X=[position+/-1], Y=[up], OSR=period:up:dir:steps_to_do-1
  // restore x to osr
  add_step(pio_encode_mov(pio_x, pio_osr));
  // ISR=position+1, X=period:up:dir:steps_to_do, Y=[up],
  // OSR=period:up:dir:steps_to_do Jump to the start of the loop
  uint8_t forward_jump_2 = program.pc;
  add_step(pio_encode_jmp(0));

  // set step output to 0 aka LOW
  uint8_t label_loop_with_clear_step = program.pc;
  add_step(pio_encode_set(pio_pins, 0));

  // Now we combine the pause/steps_to_do command and perform the delay
  // After one time period, we set step pin to 0, which is no-op for a pause
  uint8_t label_no_steps = program.pc;
  // ISR=position, X=period:up:dir:steps_to_do, Y=undefined,
  // OSR=period:up:dir:steps_to_do isolate period into osr
  add_step(pio_encode_out(pio_null, 10));
  // ISR=position, X=period:up:dir:steps_to_do, Y=undefined, OSR=period
  // copy period to y
  add_step(pio_encode_mov(pio_y, pio_osr));
  // ISR=position, X=period:up:dir:steps_to_do, Y=period, OSR=period
  // store period/remaining_steps_to_do in osr
  add_step(pio_encode_mov(pio_osr, pio_x));
  // ISR=position, X=[period:up:dir:steps_to_do], Y=period,
  // OSR=period:up:dir:steps_to_do move position into x
  add_step(pio_encode_mov(pio_x, pio_isr));
  uint8_t label_period_loop = program.pc;
  // ISR=position, X=position, Y=period, OSR=period:up:dir:steps_to_do
  // output position in fifo
  add_step(pio_encode_push(false, false));
  // ISR=0, X=position, Y=period, OSR=period:up:dir:steps_to_do
  // restore position in ISR
  add_step(pio_encode_mov(pio_isr, pio_x));
  // ISR=position, X=position, Y=period, OSR=period:up:dir:steps_to_do
  // loop until period in y is 0
  add_step(pio_encode_jmp_y_dec(label_period_loop));
  // ISR=position, X=[position], Y=[0], OSR=period:up:dir:steps_to_do
  // store period:up:dir:steps_to_do in x
  add_step(pio_encode_mov(pio_x, pio_osr));
  // ISR=position, X=period:up:dir:steps_to_do, Y=0,
  // OSR=period:up:dir:steps_to_do If step pin is still one, we need to perform
  // another loop
  add_step(pio_encode_jmp_pin(label_loop_with_clear_step));
  // Step or Pause is completed

  // ISR=position, X=period:up:dir:steps_to_do, Y=0,
  // OSR=period:up:dir:steps_to_do get steps_to_do into y (8 bits)
  add_step(pio_encode_out(pio_y, 8));
  // ISR=position, X=period:up:dir:steps_to_do, Y=steps_to_do,
  // OSR=[period:up:dir] if steps_to_do is zero go to main loop
  add_step(pio_encode_jmp_not_y(label_main_loop));
  // Otherwise continue loop using wrap around

  // patch forward jump address
  program.code[forward_jump_1] |= label_loop_with_clear_step;
  program.code[forward_jump_2] |= label_no_steps;

  program.wrap_at = program.pc - 1;
  program.wrap_target = label_loop_one_step;
  return &program;
}
#endif
