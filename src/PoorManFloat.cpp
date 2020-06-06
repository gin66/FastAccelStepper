#include <stdint.h>
#ifdef TEST
#include "stubs.h"
#else
#include <avr/pgmspace.h>
#endif
#include "PoorManFloat.h"

#define LOG_DIVIDE

// representation is:
//
//     76543210:76543210
//     XXXXXXXX:1XXXXXXX
//     exponent mantissa
//
// exponent is shifted by 128 in order to allow numbers < 1
//
//  0x8080 => is 1

const PROGMEM uint8_t isqrt_tab[256] = {
    128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 139, 140, 141,
    142, 143, 144, 145, 146, 147, 148, 148, 149, 150, 151, 152, 153, 153, 154,
    155, 156, 157, 158, 158, 159, 160, 161, 162, 162, 163, 164, 165, 166, 166,
    167, 168, 169, 169, 170, 171, 172, 172, 173, 174, 175, 175, 176, 177, 177,
    178, 179, 180, 180, 181, 182, 182, 183, 184, 185, 185, 186, 187, 187, 188,
    189, 189, 190, 191, 191, 192, 193, 193, 194, 195, 195, 196, 197, 197, 198,
    199, 199, 200, 200, 201, 202, 202, 203, 204, 204, 205, 206, 206, 207, 207,
    208, 209, 209, 210, 210, 211, 212, 212, 213, 213, 214, 215, 215, 216, 216,
    217, 218, 218, 219, 219, 220, 221, 221, 222, 222, 223, 223, 224, 225, 225,
    226, 226, 227, 227, 228, 229, 229, 230, 230, 231, 231, 232, 232, 233, 234,
    234, 235, 235, 236, 236, 237, 237, 238, 238, 239, 239, 240, 241, 241, 242,
    242, 243, 243, 244, 244, 245, 245, 246, 246, 247, 247, 248, 248, 249, 249,
    250, 250, 251, 251, 252, 252, 253, 253, 254, 254, 255, 255};
#ifdef LOG_DIVIDE
const PROGMEM uint8_t log_adjust[192] = {
    0,   3,   6,   8,   11,  14,  17,  19,  22,  24,  27,  29,  32,  34,  36,
    39,  41,  43,  46,  48,  50,  52,  55,  57,  59,  61,  63,  65,  67,  69,
    71,  73,  75,  77,  79,  80,  82,  84,  86,  88,  90,  91,  93,  95,  97,
    98,  100, 102, 103, 105, 106, 108, 110, 111, 113, 114, 116, 117, 119, 121,
    122, 123, 125, 126, 128, 129, 131, 132, 134, 135, 136, 138, 139, 140, 142,
    143, 144, 146, 147, 148, 150, 151, 152, 153, 155, 156, 157, 158, 160, 161,
    162, 163, 164, 166, 167, 168, 169, 170, 171, 172, 174, 175, 176, 177, 178,
    179, 180, 181, 182, 183, 184, 186, 187, 188, 189, 190, 191, 192, 193, 194,
    195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 206, 207, 208,
    209, 210, 211, 212, 213, 214, 215, 216, 217, 217, 218, 219, 220, 221, 222,
    223, 224, 224, 225, 226, 227, 228, 229, 229, 230, 231, 232, 233, 234, 234,
    235, 236, 237, 238, 238, 239, 240, 241, 241, 242, 243, 244, 245, 245, 246,
    247, 248, 248, 249, 250, 251, 251, 252, 253, 254, 254, 255};
const PROGMEM uint8_t exp_mul_64[256] = {
    64,  64,  65,  65,  65,  66,  66,  66,  67,  67,  68,  68,  68,  69,  69,
    69,  70,  70,  71,  71,  71,  72,  72,  72,  73,  73,  74,  74,  74,  75,
    75,  76,  76,  77,  77,  77,  78,  78,  79,  79,  79,  80,  80,  81,  81,
    82,  82,  83,  83,  83,  84,  84,  85,  85,  86,  86,  87,  87,  88,  88,
    89,  89,  90,  90,  91,  91,  92,  92,  93,  93,  94,  94,  95,  95,  96,
    96,  97,  97,  98,  98,  99,  99,  100, 100, 101, 101, 102, 103, 103, 104,
    104, 105, 105, 106, 107, 107, 108, 108, 109, 109, 110, 111, 111, 112, 112,
    113, 114, 114, 115, 116, 116, 117, 117, 118, 119, 119, 120, 121, 121, 122,
    123, 123, 124, 125, 125, 126, 127, 127, 128, 129, 129, 130, 131, 132, 132,
    133, 134, 135, 135, 136, 137, 137, 138, 139, 140, 140, 141, 142, 143, 144,
    144, 145, 146, 147, 147, 148, 149, 150, 151, 152, 152, 153, 154, 155, 156,
    157, 157, 158, 159, 160, 161, 162, 163, 163, 164, 165, 166, 167, 168, 169,
    170, 171, 172, 173, 174, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183,
    184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 196, 197, 198, 199,
    200, 201, 202, 203, 204, 205, 206, 208, 209, 210, 211, 212, 213, 214, 216,
    217, 218, 219, 220, 221, 223, 224, 225, 226, 228, 229, 230, 231, 233, 234,
    235, 236, 238, 239, 240, 242, 243, 244, 246, 247, 248, 250, 251, 252, 254,
    255};
#endif

upm_float upm_from(uint8_t x) {
  uint16_t res;
  if ((x & 0xf0) == 0) {
    if ((x & 0x0c) == 0) {
      if ((x & 0x02) == 0) {
        x <<= 7;
        res = x;
      } else {
        x <<= 6;
        res = x | 0x0100;
      }
    } else {
      if ((x & 0x08) == 0) {
        x <<= 5;
        res = x | 0x0200;
      } else {
        x <<= 4;
        res = x | 0x0300;
      }
    }
  } else {
    if ((x & 0xc0) == 0) {
      if ((x & 0x20) == 0) {
        x <<= 3;
        res = x | 0x0400;
      } else {
        x <<= 2;
        res = x | 0x0500;
      }
    } else {
      if ((x & 0x80) == 0) {
        x <<= 1;
        res = x | 0x0600;
      } else {
        res = x | 0x0700;
      }
    }
  }
  return res | 0x8000;
}
upm_float upm_from(uint16_t x) {
  uint8_t exponent;
  if ((x & 0xff00) == 0) {
    uint8_t b = x & 0xff;
    return upm_from(b);
  }
  if (x & 0xf000) {
    exponent = 8 + 4;
  } else {
    x <<= 4;
    exponent = 8 + 0;
  }
  if ((x & 0xc000) == 0) {
    x <<= 2;
  } else {
    exponent += 2;
  }
  if ((x & 0x8000) == 0) {
    x <<= 1;
  } else {
    exponent += 1;
  }
  x >>= 8;
  exponent |= 0x80;
  return x | (exponent << 8);
}
upm_float upm_from(uint32_t x) {
  if ((x & 0xffff0000) == 0) {
    uint16_t w = x & 0xffff;
    return upm_from(w);
  }
  if ((x & 0xff000000) == 0) {
    uint16_t w = x >> 8;
    return upm_from(w) + 0x0800;
  } else {
    uint16_t w = x >> 16;
    return upm_from(w) + 0x1000;
  }
}
upm_float multiply(upm_float x, upm_float y) {
  uint8_t a = x & 255;
  uint8_t b = y & 255;
  uint16_t ab = a * b;
  if (ab & 0x8000) {
    ab >>= 8;
    ab += 0x0100;
  } else {
    ab >>= 7;
  }
  ab += ((x & 0xff00) - 0x4000) + ((y & 0xff00) - 0x4000);
  return ab;
}
#ifdef LOG_DIVIDE
upm_float divide(upm_float x, upm_float y) {
  uint8_t exp_x = x >> 8;
  uint8_t exp_y = y >> 8;
  uint8_t mant_x = x & 255;
  uint8_t mant_y = y & 255;

  if (mant_x < mant_y) {
    mant_y >>= 1;
    exp_y += 1;
  }
  uint8_t log_x = pgm_read_byte_near(&log_adjust[mant_x - 64]);
  uint8_t log_y = pgm_read_byte_near(&log_adjust[mant_y - 64]);
  uint8_t log_x_y = log_x - log_y;
  uint8_t mant_res = pgm_read_byte_near(&exp_mul_64[log_x_y]);
  uint8_t exp_res = exp_x - exp_y + 128 - 6 +
                    7;  // 6 for table_64 factor. 7 for shift of upm_float

  if (mant_res < 128) {
    mant_res <<= 1;
    exp_res -= 1;
  }

  uint16_t res = exp_res;
  res <<= 8;
  res |= mant_res;
  return res;
}
#else
upm_float divide(upm_float x, upm_float y) {
  if (x < y) {
    return 0;
  }
  uint8_t a = x & 255;
  uint8_t b = y & 255;

  uint8_t exponent = (x >> 8) - (y >> 8);
  uint8_t mantissa = 0;
  uint8_t mask = 0x80;
  while (mask) {
    if (a >= b) {
      a -= b;
      mantissa |= mask;
    }
    if (a == 0) {
      break;
    }
    a <<= 1;
    mask >>= 1;
  }
  if ((mantissa & 0x80) == 0) {
    exponent -= 1;
  }
  uint16_t res = exponent + 128;
  res <<= 8;
  res |= mantissa;
  return res;
}
#endif
uint16_t upm_to_u16(upm_float x) {
  uint8_t exponent = x >> 8;
  if (exponent > 15 + 128) {
    return 0xffff;
  }
  if (exponent < 128) {
    return 0;
  }
  exponent -= 128;
  uint8_t mantissa = x & 0x00ff;
  uint16_t res = mantissa;
  if (exponent < 8) {
    res >>= (7 - exponent);
  } else {
    res <<= exponent - 7;
  }
  return res;
}
uint32_t upm_to_u32(upm_float x) {
  uint8_t exponent = x >> 8;
  if (exponent > 31 + 128) {
    return 0xffffffff;
  }
  if (exponent < 128) {
    return 0;
  }
  exponent -= 128;
  uint8_t mantissa = x & 0x00ff;
  uint32_t res = mantissa;
  if (exponent < 8) {
    res >>= (7 - exponent);
  } else {
    res <<= exponent - 7;
  }
  return res;
}
upm_float abs_diff(upm_float x, upm_float y) {
  uint8_t exp_x = x >> 8;
  uint8_t exp_y = y >> 8;
  uint8_t mantissa;
  uint8_t exponent;
  if (x > y) {
    exponent = exp_x;
    uint8_t m_y = y & 0xff;
    m_y >>= (exp_x - exp_y);
    mantissa = x & 0xff;
    mantissa -= m_y;
  } else if (x < y) {
    exponent = exp_y;
    uint8_t m_x = x & 0xff;
    m_x >>= (exp_y - exp_x);
    mantissa = y & 0xff;
    mantissa -= m_x;
  } else {
    return 0;
  }
  while ((mantissa & 0x80) == 0) {
    mantissa <<= 1;
    exponent--;
  }
  uint16_t res = exponent;
  res <<= 8;
  res |= mantissa;
  return res;
}
upm_float sum(upm_float x, upm_float y) {
  uint8_t exp_x = x >> 8;
  uint8_t exp_y = y >> 8;
  uint16_t mantissa;
  uint8_t exponent;
  if (x > y) {
    exponent = exp_x;
    uint8_t m_y = y & 0xff;
    m_y >>= (exp_x - exp_y);
    mantissa = x & 0xff;
    mantissa += m_y;
  } else if (x < y) {
    exponent = exp_y;
    uint8_t m_x = x & 0xff;
    m_x >>= (exp_y - exp_x);
    mantissa = y & 0xff;
    mantissa += m_x;
  } else {
    return x + 0x0100;
  }
  while (mantissa & 0xff00) {
    mantissa >>= 1;
    exponent++;
  }
  uint16_t res = exponent;
  res <<= 8;
  res |= (mantissa & 0x00ff);
  return res;
}
upm_float shl(upm_float x, uint8_t n) { return x + (((uint16_t)n) << 8); }
upm_float shr(upm_float x, uint8_t n) { return x - (((uint16_t)n) << 8); }
upm_float sqrt(upm_float x) {
  uint8_t mantissa = x & 0x00ff;
  uint8_t exponent = x >> 8;
  if ((exponent & 0x01) == 0) {
    exponent += 1;
    mantissa >>= 1;
  }
  uint8_t sqrt_mantissa = pgm_read_byte_near(&isqrt_tab[mantissa - 64]);
  if (exponent >= 128) {
    exponent -= 128;
    exponent >>= 1;
    exponent += 128;
  } else {
    exponent = 129 - exponent;  // this difference to 128 is needed
    exponent >>= 1;
    exponent = 128 - exponent;
  }
  uint16_t res = ((uint16_t)exponent) << 8;
  return res | sqrt_mantissa;
}
