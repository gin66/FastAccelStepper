#ifndef POORMANFLOAT_H
#define POORMANFLOAT_H
#include <stdint.h>

typedef int16_t pmf_logarithmic;

#define PMF_CONST_INVALID ((pmf_logarithmic)0x8000)
#define PMF_CONST_MAX ((pmf_logarithmic)0x7fff)
#define PMF_CONST_1 ((pmf_logarithmic)0x0000)
#define PMF_CONST_3_DIV_2 ((pmf_logarithmic)0x012c)
#define PMF_CONST_128E12 ((pmf_logarithmic)0x5dba)
#define PMF_CONST_16E6 ((pmf_logarithmic)0x2fdd)
#define PMF_CONST_500 ((pmf_logarithmic)0x11ee)
#define PMF_CONST_1000 ((pmf_logarithmic)0x13ee)
#define PMF_CONST_2000 ((pmf_logarithmic)0x15ee)
#define PMF_CONST_32000 ((pmf_logarithmic)0x1dee)
#define PMF_CONST_16E6_DIV_SQRT_OF_2 ((pmf_logarithmic)0x2edd)
#define PMF_CONST_21E6 ((pmf_logarithmic)0x30a5)
#define PMF_CONST_42000 ((pmf_logarithmic)0x1eb7)
#define PMF_CONST_21E6_DIV_SQRT_OF_2 ((pmf_logarithmic)0x2fa5)
#define PMF_CONST_2205E11 ((pmf_logarithmic)0x5f96)
pmf_logarithmic pmfl_from(uint8_t x);
pmf_logarithmic pmfl_from(uint16_t x);
pmf_logarithmic pmfl_from(uint32_t x);

uint16_t pmfl_to_u16(pmf_logarithmic x);
uint32_t pmfl_to_u32(pmf_logarithmic x);

#define pmfl_shl(x, n) ((x) + (((int16_t)(n)) << 9))
#define pmfl_shr(x, n) ((x) - (((int16_t)(n)) << 9))

#define pmfl_multiply(x, y) ((x) + (y))
#define pmfl_divide(x, y) ((x) - (y))
#define pmfl_reciprocal(x) (-(x))
#define pmfl_sqrt(x) ((x) / 2)
#define pmfl_rsqrt(x) (-(x) / 2)
#define pmfl_rsquare(x) pmfl_reciprocal(pmfl_square(x))

inline pmf_logarithmic pmfl_pow_div_3(pmf_logarithmic x) {
  // 1/3 ~ (1/4+1/16+1/64+1/256+1/1024+1/4096+1/16384)
  x /= 2;        // x/2
  x += x / 4;    // x/2 + x/8
  x += x / 16;   // x/2 + x/8 + x/32 + x/128
  x += x / 256;  // x/2 + x/8 + x/32 + x/128 + x/512 + x/2048 + x/8192 + x/32768
  x += 1;
  return x / 2;
}
#define pmfl_pow_2_div_3(x) ((x)-pmfl_pow_div_3(x))
#define pmfl_pow_3_div_2(x) ((x) + (x) / 2)

pmf_logarithmic pmfl_square(pmf_logarithmic x);

uint8_t leading_zeros(uint8_t x);
#endif
