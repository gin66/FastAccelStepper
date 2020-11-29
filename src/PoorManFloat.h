#include <stdint.h>
#if defined(ARDUINO_ARCH_ESP32)
#define min(a, b) ((a) > (b) ? (b) : (a))
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

typedef uint16_t upm_float;

upm_float upm_from(uint8_t x);
upm_float upm_from(uint16_t x);
upm_float upm_from(uint32_t x);

uint16_t upm_to_u16(upm_float x);
uint32_t upm_to_u32(upm_float x);

upm_float upm_multiply(upm_float x, upm_float y);
upm_float upm_divide(upm_float x, upm_float y);
upm_float upm_abs_diff(upm_float x, upm_float y);
upm_float upm_sum(upm_float x, upm_float y);
upm_float upm_shl(upm_float x, uint8_t n);
upm_float upm_shr(upm_float x, uint8_t n);
upm_float upm_square(upm_float x);
upm_float upm_sqrt(upm_float x);
