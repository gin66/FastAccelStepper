#include <stdint.h>

typedef int16_t pmf_logarithmic;

#define PMF_CONST_1 ((pmf_logarithmic)0x0000)
#define PMF_CONST_128E12 ((pmf_logarithmic)0x5bba)
#define PMF_CONST_16E6 ((pmf_logarithmic)0x2fdd)
#define PMF_CONST_500 ((pmf_logarithmic)0x11ee)
#define PMF_CONST_1000 ((pmf_logarithmic)0x13ee)
#define PMF_CONST_2000 ((pmf_logarithmic)0x15ee)
#define PMF_CONST_32000 ((pmf_logarithmic)0x1dee)
#define PMF_CONST_16E6_DIV_SQRT_OF_2 ((pmf_logarithmic)0x2edc)
#define PMF_CONST_21E6 ((pmf_logarithmic)0x30a5)
#define PMF_CONST_42000 ((pmf_logarithmic)0x1eb7)
#define PMF_CONST_21E6_DIV_SQRT_OF_2 ((pmf_logarithmic)0x2fa6)
#define PMF_CONST_2205E11 ((pmf_logarithmic)0x5d96)
#define PMF_CONST_1_DIV_500 ((pmf_logarithmic)0xee12)
pmf_logarithmic pmfl_from(uint8_t x);
pmf_logarithmic pmfl_from(uint16_t x);
pmf_logarithmic pmfl_from(uint32_t x);

uint16_t pmfl_to_u16(pmf_logarithmic x);
uint32_t pmfl_to_u32(pmf_logarithmic x);

pmf_logarithmic pmfl_shl(pmf_logarithmic x, uint8_t n);
pmf_logarithmic pmfl_shr(pmf_logarithmic x, uint8_t n);

pmf_logarithmic pmfl_multiply(pmf_logarithmic x, pmf_logarithmic y);
pmf_logarithmic pmfl_reciprocal(pmf_logarithmic x);
pmf_logarithmic pmfl_square(pmf_logarithmic x);
pmf_logarithmic pmfl_rsquare(pmf_logarithmic x);  // Reciprocal square = 1/(x*x)
pmf_logarithmic pmfl_rsqrt(pmf_logarithmic x);  // . Reciprocal sqrt() = 1/sqrt(x)
pmf_logarithmic pmfl_divide(pmf_logarithmic x, pmf_logarithmic y);
