typedef struct pio_program_s {
  uint16_t code[32];  // at max 32 instructions
  uint8_t pc;
  uint8_t wrap_at;
  uint8_t wrap_target;
  uint32_t sys_clk;
} stepper_pio_program;
stepper_pio_program *stepper_make_program();

uint32_t stepper_calc_period(bool dir_high, uint8_t steps,
                             uint16_t cycles_16th_us);
uint32_t stepper_make_fifo_entry(bool dir_high, bool count_up, uint8_t steps,
                                 uint16_t cycles_16th_us);
