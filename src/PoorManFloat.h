#include <stdint.h>
#if defined(ARDUINO_ARCH_ESP32)
#define min(a, b) ((a) > (b) ? (b) : (a))
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

typedef uint16_t upm_float;
#define UPM_CONST_128E12 ((upm_float)0xaee8)
#define UPM_CONST_16E6 ((upm_float)0x97f4)
#define UPM_CONST_500 ((upm_float)0x88fa)
#define UPM_CONST_1000 ((upm_float)0x89fa)
#define UPM_CONST_2000 ((upm_float)0x8afa)
#define UPM_CONST_32000 ((upm_float)0x8efa)

upm_float upm_from(uint8_t x);
upm_float upm_from(uint16_t x);
upm_float upm_from(uint32_t x);

uint16_t upm_to_u16(upm_float x);
uint32_t upm_to_u32(upm_float x);

upm_float upm_shl(upm_float x, uint8_t n);
upm_float upm_shr(upm_float x, uint8_t n);

upm_float upm_multiply(upm_float x, upm_float y); // TESTED
upm_float upm_divide(upm_float x, upm_float y);
upm_float upm_square(upm_float x); // TESTED = x*x
upm_float upm_rsquare(upm_float x); // TESTED Reciprocal square = 1/(x*x)
upm_float upm_sqrt(upm_float x);
upm_float upm_sqrt_after_divide(upm_float x, upm_float y);
upm_float upm_rsqrt(upm_float x); // TESTED. Reciprocal sqrt() = 1/sqrt(x)
