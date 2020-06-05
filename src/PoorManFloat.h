
typedef uint16_t upm_float;

upm_float upm_from(uint8_t x);
upm_float upm_from(uint16_t x);
upm_float upm_from(uint32_t x);

uint16_t upm_to_u16(upm_float x);
uint32_t upm_to_u32(upm_float x);

upm_float multiply(upm_float x,upm_float y);
upm_float divide(upm_float x,upm_float y);
upm_float abs_diff(upm_float x,upm_float y);
