#ifndef LOG2REPRESENTATION_H
#define LOG2REPRESENTATION_H
#include <stdint.h>

#include <Log2RepresentationConst.h>

typedef int16_t pmf_logarithmic;

#define LOG2_CONST_INVALID ((pmf_logarithmic)0x8000)
#define LOG2_CONST_MAX ((pmf_logarithmic)0x7fff)
#define LOG2_CONST_MIN ((pmf_logarithmic)0x8001)

pmf_logarithmic log2_from(uint8_t x);
pmf_logarithmic log2_from(uint16_t x);
pmf_logarithmic log2_from(uint32_t x);

uint16_t log2_to_u16(pmf_logarithmic x);
uint32_t log2_to_u32(pmf_logarithmic x);

#define log2_shl(x, n) ((x) + (((int16_t)(n)) << 9))
#define log2_shr(x, n) ((x) - (((int16_t)(n)) << 9))

#define log2_multiply(x, y) ((x) + (y))
#define log2_divide(x, y) ((x) - (y))
#define log2_reciprocal(x) (-(x))
#define log2_sqrt(x) ((x) / 2)
#define log2_rsqrt(x) (-(x) / 2)
#define log2_rsquare(x) log2_reciprocal(log2_square(x))

inline pmf_logarithmic log2_pow_div_3(pmf_logarithmic x) {
  // 1/3 ~ (1/4+1/16+1/64+1/256+1/1024+1/4096+1/16384)
  x /= 2;        // x/2
  x += x / 4;    // x/2 + x/8
  x += x / 16;   // x/2 + x/8 + x/32 + x/128
  x += x / 256;  // x/2 + x/8 + x/32 + x/128 + x/512 + x/2048 + x/8192 + x/32768
  x += 1;
  return x / 2;
}
#define log2_pow_2_div_3(x) ((x) - log2_pow_div_3(x))
#define log2_pow_3_div_2(x) ((x) + (x) / 2)

pmf_logarithmic log2_square(pmf_logarithmic x);

uint8_t leading_zeros(uint8_t x);
#endif
