#include <stdint.h>

typedef uint16_t upm_float;
typedef int16_t upm_logarithmic;
typedef int16_t pmf_logarithmic;
#define UPM_CONST_1 ((upm_float)0x8000)
#define UPM_CONST_128E12 ((upm_float)0xaed0)
#define UPM_CONST_16E6 ((upm_float)0x97e8)
#define UPM_CONST_500 ((upm_float)0x88f4)
#define UPM_CONST_1000 ((upm_float)0x89f4)
#define UPM_CONST_2000 ((upm_float)0x8af4)
#define UPM_CONST_32000 ((upm_float)0x8ef4)
#define UPM_CONST_16E6_DIV_SQRT_OF_2 ((upm_float)0x9759)
#define UPM_CONST_21E6 ((upm_float)0x9840)
#define UPM_CONST_42000 ((upm_float)0x8f48)
#define UPM_CONST_21E6_DIV_SQRT_OF_2 ((upm_float)0x97c4)
#define UPM_CONST_2205E11 ((upm_float)0xaf91)
#define UPM_CONST_1_DIV_500 ((upm_float)0x7706)
upm_float upm_from(uint8_t x);
upm_float upm_from(uint16_t x);
upm_float upm_from(uint32_t x);

uint16_t upm_to_u16(upm_float x);
uint32_t upm_to_u32(upm_float x);

upm_float upm_shl(upm_float x, uint8_t n);
upm_float upm_shr(upm_float x, uint8_t n);

upm_float upm_multiply(upm_float x, upm_float y);  // TESTED
upm_float upm_reciprocal(upm_float x);             // TESTED
upm_float upm_square(upm_float x);                 // TESTED = x*x
upm_float upm_rsquare(upm_float x);  // TESTED Reciprocal square = 1/(x*x)
upm_float upm_rsqrt(upm_float x);    // TESTED. Reciprocal sqrt() = 1/sqrt(x)
upm_float upm_divide(upm_float x, upm_float y);

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

uint16_t pmfl_to_u16(upm_logarithmic x);
uint32_t pmfl_to_u32(upm_logarithmic x);

pmf_logarithmic pmfl_shl(pmf_logarithmic x, uint8_t n);
pmf_logarithmic pmfl_shr(pmf_logarithmic x, uint8_t n);

