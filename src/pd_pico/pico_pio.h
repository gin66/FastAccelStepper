#ifndef PD_PICO_PIO_H
#define PD_PICO_PIO_H

typedef struct pio_program_s {
  uint16_t code[32];  // at max 32 instructions
  uint8_t pc;
  uint8_t wrap_at;
  uint8_t wrap_target;
  uint32_t sys_clk;
} stepper_pio_program;
stepper_pio_program* stepper_make_program();

// One pass is LOOP_OVERHEAD + LOOP_CYCLES*period, plus LOOP_PREAMBLE once.
// The direction pin is not in this program (the filler writes it), so a pass
// is 22 + 3*period. 22 + 3 does not divide the old 80-cycle pause evenly:
// LOOPS_FOR_1US is 18 and that idle pause is 79 cycles.
#define LOOP_OVERHEAD 22
#define LOOP_PREAMBLE 3
#define LOOP_CYCLES 3
#define LOOPS_FOR_1US ((80 - LOOP_PREAMBLE - LOOP_OVERHEAD) / LOOP_CYCLES)
uint32_t pio_calc_loops(uint8_t steps, uint16_t cycles_16th_us,
                        uint16_t* adjust_80MHz);
uint32_t pio_make_fifo_entry(bool count_up, uint8_t steps, uint32_t loops);

#endif  // PD_PICO_PIO_H
