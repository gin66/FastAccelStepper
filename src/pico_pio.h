typedef struct pio_program_s {
  uint16_t code[32];  // at max 32 instructions
  uint8_t pc;
  uint8_t wrap_at;
  uint8_t wrap_target;
  uint32_t sys_clk;
} stepper_pio_program;
stepper_pio_program *stepper_make_program();

#define LOOPS_FOR_1US ((80-27)/3)
uint32_t pio_calc_loops(uint8_t steps, uint16_t cycles_16th_us, uint16_t *adjust_80MHz);
uint32_t pio_make_fifo_entry(bool dir_high, bool count_up, uint8_t steps,
                                 uint32_t loops);
