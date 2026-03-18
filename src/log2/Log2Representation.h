#ifndef LOG2REPRESENTATION_H
#define LOG2REPRESENTATION_H
#include <stdint.h>

#include "log2/Log2RepresentationConst.h"

typedef int16_t log2_value_t;

#define LOG2_CONST_INVALID ((log2_value_t)0x8000)
#define LOG2_CONST_MAX ((log2_value_t)0x7fff)
#define LOG2_CONST_MIN ((log2_value_t)0x8001)

log2_value_t log2_from(uint8_t x);
log2_value_t log2_from(uint16_t x);
log2_value_t log2_from(uint32_t x);

uint16_t log2_to_u16(log2_value_t x);
uint32_t log2_to_u32(log2_value_t x);

#define log2_shl(x, n) ((x) + (((int16_t)(n)) << 9))
#define log2_shr(x, n) ((x) - (((int16_t)(n)) << 9))

#define log2_multiply(x, y) ((x) + (y))
#define log2_divide(x, y) ((x) - (y))
#define log2_reciprocal(x) (-(x))
#define log2_sqrt(x) ((x) / 2)
#define log2_rsqrt(x) (-(x) / 2)
#define log2_rsquare(x) log2_reciprocal(log2_square(x))

static inline log2_value_t log2_pow_div_3(log2_value_t x) {
  // 1/3 ~ (1/4+1/16+1/64+1/256+1/1024+1/4096+1/16384)
  uint16_t xu;
  xu = x < 0 ? -x : x;
  xu /= 2;        // x/2
  xu += xu / 4;    // x/2 + x/8
  xu += xu / 16;   // x/2 + x/8 + x/32 + x/128
  xu += xu / 256;  // x/2 + x/8 + x/32 + x/128 + x/512 + x/2048 + x/8192 + x/32768
  xu += 1;
  xu /= 2;
  x = (x < 0) ? -(int16_t)xu : xu;
  return x;
}
#define log2_pow_2_div_3(x) ((x) - log2_pow_div_3(x))
#define log2_pow_3_div_2(x) ((x) + (x) / 2)

log2_value_t log2_square(log2_value_t x);

uint8_t leading_zeros(uint8_t x);
#endif
