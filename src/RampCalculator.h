#include <stdint.h>
#include "PoorManFloat.h"

uint32_t calculate_ticks_v1(uint32_t steps, float acceleration);
uint32_t calculate_ticks_v2(uint32_t steps, float acceleration);
uint32_t calculate_ticks_v3(uint32_t steps, float pre_calc);
uint32_t calculate_ticks_v4(uint32_t steps, uint32_t acceleration);
uint32_t calculate_ticks_v5(uint32_t steps, upm_float pre_calc);
uint32_t calculate_ticks_v6(uint32_t steps, upm_float pre_calc);
