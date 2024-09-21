#include <stdint.h>
#if defined(ARDUINO_ARCH_AVR)
#include <avr/pgmspace.h>
#else
#define PROGMEM
#define pgm_read_byte_near(x) (*(x))
#endif
#include "PoorManFloat.h"
#ifdef TEST
#include <stdio.h>
#endif

// In FastAccelStepper there is seldom the need for adding or
// subtracting floats, but multiplication/division/square and power
// are often in use. Negative numbers and even zero are not needed.
// Consequently, a purely logarithmic representation is completely
// sufficient and the necessary range can be achieved by 16 bit signed integers.
// The interpretation of a signed integer xi is:
//
//      x = 2^(xi/512)
//
// The signed integer range xi (-32768..32767) is mapped to the
// range (5e-20, 1.8e19). Please note: zero is not included.
// The greatest constant in use is 2.2e14. So the range is sufficient.
//
// In order to map x to xi, log2(x) needs to be calculated.
// For this we rewrite x to:
//     x = (1 + r) * 2^e
//
// With r being in the range 0 <= r < 1.
// This means e is the largest integer value with 2^e < x.
// Consequently, e can be derived by counting the leading numbers in
// the integer x.
//
// So xi = 512*log2(x) = 512 * (e + log(1+r))
//
// The first 7 bits is plain e with sign and the lower 9 bits is log(1+r)*512
//     xi  = eeee_eeem_mmmm_mmmm
//
// Example for 16 bit integer:
//    x = 15373 = 0b0011_1100_0000_1101
//
//    log2(15373)*512 = 7121 (rounded)
//                    = 0b0001_1011_1101_0001
//
//    two leading zeros => e = 15 - 2 = 13 = 0b1101
//    2^e = 8192
//
//    => x = 8192 * 1.1_1100_0000_1101
//    r is the decimal part without the leading 1
//    r = 1_1100_0000 / 512
//    log2(1+r) * 512 = 464 = 0b1_1101_0001
//
//    xi = eeee_eeem_mmmm_mmmm
//       = 0001_1011_1101_0001
//       = 0x1bd1
//
// The remaining task is to calculate log2(1+f).
//
// log2(1+r) is at the corners identical to r:
//      log2(1+0) = 0 and log2(1+1) = 1
// So it is interesting to look at function
//      f(1+r) = log2(1+r) - r
//
// This function is 0 for r at 0 and 1, positive over the range inbetween
// and reaches max value of 0.08607 at r = 0.442695.
// This max value is at r = 1/ln(2)-1
//
// As we need actually 512*log2(1+r), then the max value is 44.
// This allows to multiply with up to 8 to improve the resolution.
// Here we use factor 4, so two table values can be summed up without overflow.
//
// So the log2 can be calculated for e.g.:
//       x    = 0001_mmmm_mmmm_mmmm    three leading zeros
//
// Shift left by leading zeros+1 (removing leading 1) and then right by 5:
//       x    = 0001_mmmm_mmmm_mmmm    three leading zeros
//              mmmm_mmmm_mmmm_m000    shift left (3+1)
//              0000_0mmm_mmmm_mmmm    shift right (5)
//                                     if ninth bit is 0:
//            +           ttt_tttt0    add table value for mmmm_mmmm shifted one
//                                     else:
//            +           0ttt_tttt      add table value for mmmm_mmmm
//            +           0ttt_tttt      add table value for mmmm_mmmm+1
//            +           0000_0001      round
//     result:       mmmm_mmmm_m
//     final:   000e_eeem_mmmm_mmmm    For up to 32bits (e = 31)
//
// We are using a table of length 256, so first 8 bits are the index.
// In order to improve the resolution, the ninth bit is used to interpolate
// with the next table entry.
//
// Using python3 this can be calculated by:
//     [round(math.log2(i/256) * 256 - (i-256)) for i in range(256,512)]
//
// For better precision y_yyyy is shifted by 2 and can be calculated as:
//	   [round((math.log2(i/256) * 256 - (i-256))*4) for i in range(256,512)]
//
const PROGMEM uint8_t log2_minus_x_plus_one_shifted_by_2[256] = {
    0,  2,  3,  5,  7,  9,  10, 12, 13, 15, 17, 18, 20, 21, 23, 24, 26, 27, 28,
    30, 31, 32, 34, 35, 36, 38, 39, 40, 41, 43, 44, 45, 46, 47, 48, 49, 50, 51,
    52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 64, 65, 66, 67, 68, 68,
    69, 70, 70, 71, 72, 72, 73, 74, 74, 75, 75, 76, 77, 77, 78, 78, 79, 79, 80,
    80, 80, 81, 81, 82, 82, 83, 83, 83, 84, 84, 84, 84, 85, 85, 85, 86, 86, 86,
    86, 86, 87, 87, 87, 87, 87, 87, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88,
    88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 87, 87, 87, 87, 87, 87, 86, 86,
    86, 86, 86, 85, 85, 85, 85, 84, 84, 84, 84, 83, 83, 83, 82, 82, 82, 81, 81,
    81, 80, 80, 79, 79, 79, 78, 78, 77, 77, 76, 76, 75, 75, 74, 74, 73, 73, 72,
    72, 71, 71, 70, 70, 69, 68, 68, 67, 67, 66, 65, 65, 64, 63, 63, 62, 61, 61,
    60, 59, 59, 58, 57, 57, 56, 55, 54, 54, 53, 52, 51, 51, 50, 49, 48, 47, 47,
    46, 45, 44, 43, 42, 42, 41, 40, 39, 38, 37, 36, 35, 34, 34, 33, 32, 31, 30,
    29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11,
    10, 9,  8,  7,  6,  4,  3,  2,  1};

// For the inverse pow(2,x) needs to be calculated. Similarly it makes sense to
// evaluate instead
//     g(x) = x - pow(2,x-1)
//
// This function equals 0 for x at 1 and 2, with extremum 0.08607 at x
// = 1.528766. This max. value is at x = 1 - ln(ln(2))/ln(2)
//
// Noteworthy the max values of log2(x) - x + 1 and x - pow(2, x-1) are
// identical, calculated by (-1 + ln(2) - ln(ln(2)))/ln(2)
//
// Using python3 this can be calculated by:
//	   [round(i - math.pow(2,i/256-1)*256) for i in range(256,512)]
//
// Similarly shifted by two bits:
//     [round((i - math.pow(2,i/256-1)*256)*4) for i in range(256,512)]
//
const PROGMEM uint8_t x_minus_pow2_of_x_minus_one_shifted_by_2[256] = {
    0,  1,  2,  4,  5,  6,  7,  8,  10, 11, 12, 13, 14, 15, 16, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 46, 47, 48, 49, 50, 51, 52, 52, 53, 54, 55, 56, 56,
    57, 58, 59, 59, 60, 61, 62, 62, 63, 64, 64, 65, 66, 66, 67, 68, 68, 69, 69,
    70, 71, 71, 72, 72, 73, 73, 74, 74, 75, 76, 76, 76, 77, 77, 78, 78, 79, 79,
    80, 80, 80, 81, 81, 82, 82, 82, 83, 83, 83, 84, 84, 84, 84, 85, 85, 85, 85,
    86, 86, 86, 86, 87, 87, 87, 87, 87, 87, 87, 88, 88, 88, 88, 88, 88, 88, 88,
    88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 87, 87, 87, 87, 87,
    87, 86, 86, 86, 86, 86, 85, 85, 85, 84, 84, 84, 84, 83, 83, 83, 82, 82, 81,
    81, 81, 80, 80, 79, 79, 78, 78, 77, 77, 76, 76, 75, 75, 74, 74, 73, 72, 72,
    71, 71, 70, 69, 68, 68, 67, 66, 66, 65, 64, 63, 63, 62, 61, 60, 59, 58, 58,
    57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39,
    38, 36, 35, 34, 33, 32, 30, 29, 28, 27, 25, 24, 23, 22, 20, 19, 17, 16, 15,
    13, 12, 10, 9,  8,  6,  5,  3,  2};

uint8_t leading_zeros(uint8_t x) {
  uint8_t res;
  if ((x & 0xf0) == 0) {
    x <<= 4;
    res = 4;
  } else {
    res = 0;
  }
  if ((x & 0xc0) == 0) {
    x <<= 2;
    res += 2;
  }
  if ((x & 0x80) == 0) {
    res += 1;
    if (x == 0) {
      res += 1;
    }
  }
  return res;
}

pmf_logarithmic pmfl_from(uint8_t x) {
  // calling with x == 0 is considered an error.
  //
  // In a first step convert to
  //    0000_0eee_mmmm_mmmm
  //
  // Hereby mmmm_mmmm are the lower bits right from the first 1 in x
  // An example with only four valid mantissa bits:
  //	x = 0001_mmmm   =>  0000_0100_mmmm_0000
  //
  // eee is the exponent
  //
  // The second convert this to the logarithm of x
  //    1. Use mmmm_mmmm as index in the log2_minus_x_plus_one_shifted_by_1
  //    table
  //    2. shift left 0000_0eee_mmmm_mmmm by 1
  //    3. add the value from the log2_minus_x_plus_one_shifted_by_1 table
  uint8_t leading = leading_zeros(x);
  if (leading == 8) {
    return PMF_CONST_INVALID;
  }
  x <<= leading + 1;
  uint8_t e = 7 - leading;
  uint8_t offset = pgm_read_byte_near(&log2_minus_x_plus_one_shifted_by_2[x]);
  uint16_t res = (((uint16_t)e) << 8) | x;
  res <<= 1;
  offset += 1;
  res += offset >> 1;
  return res;
}

pmf_logarithmic pmfl_from(uint16_t x) {
  uint8_t leading = leading_zeros(x >> 8);
  if (leading == 8) {
    return pmfl_from((uint8_t)x);
  }
  // shift msb out
  x <<= leading + 1;
  uint8_t exponent = 15 - leading;
  x >>= 5;
  uint8_t index = x >> 3;
  uint8_t offset =
      pgm_read_byte_near(&log2_minus_x_plus_one_shifted_by_2[index]);
  // only with x & 7 > 2, the calculated constants are correct...
  if ((x & 7) > 2) {
    index++;  // overflow to 0 is ok. index is an uint8_t
    offset += pgm_read_byte_near(&log2_minus_x_plus_one_shifted_by_2[index]);
    offset += 1;
  } else {
    offset <<= 1;
    // offset += 1;
  }
  x += offset;
  x >>= 2;
  x += ((uint16_t)exponent) << 9;
  return x;
}
pmf_logarithmic pmfl_from(uint32_t x) {
  int16_t exp_offset;
  uint16_t w;
  if ((x & 0xff000000) == 0) {
    if ((x & 0x00ff0000) == 0) {
      w = (uint16_t)x;
      exp_offset = 0;
    } else if ((x & 0x00f00000) == 0) {
      w = x >> 4;
      exp_offset = 0x0800;
    } else {
      w = x >> 8;
      exp_offset = 0x1000;
    }
  } else if ((x & 0xf0000000) == 0) {
    w = x >> 12;
    exp_offset = 0x1800;
  } else {
    w = x >> 16;
    exp_offset = 0x2000;
  }
  return pmfl_from(w) + exp_offset;
}
uint16_t pmfl_to_u16(pmf_logarithmic x) {
  if (x < 0) {
    return 0;
  }
  if (x >= PMF_CONST_UINT16_MAX) {
    return __UINT16_MAX__;
  }
  uint8_t exponent = ((uint16_t)x) >> 9;
  x &= 0x01ff;
  x += 0x200;
  uint8_t index = ((uint16_t)x) >> 1;
  x <<= 1;
  uint8_t offset =
      pgm_read_byte_near(&x_minus_pow2_of_x_minus_one_shifted_by_2[index]);
  if ((x & 2) != 0) {
    index++;  // overflow to 0 is ok. index is an uint8_t
    offset +=
        pgm_read_byte_near(&x_minus_pow2_of_x_minus_one_shifted_by_2[index]);
    offset >>= 1;
  }
  x -= offset;
  if (exponent > 10) {
    x <<= exponent - 10;
  } else if (exponent < 10) {
    x += (exponent != 9) ? 2 : 1;
    x >>= 10 - exponent;
  }
  return x;
}
uint32_t pmfl_to_u32(pmf_logarithmic x) {
  if (x < 0) {
    return 0;
  }
  if (x >= PMF_CONST_UINT32_MAX) {
    return __UINT32_MAX__;
  }
  uint8_t exponent = ((uint16_t)x) >> 9;
  if (exponent < 0x10) {
    return pmfl_to_u16(x);
  }
  uint8_t shift = exponent - 0x0f;
  x = pmfl_shr(x, shift);
  uint32_t res = pmfl_to_u16(x);
  res <<= shift;
  return res;
}
pmf_logarithmic pmfl_square(pmf_logarithmic x) {
  if (x > 0x4000) {
    return PMF_CONST_MAX;
  }
  if (x <= -0x4000) {
    return PMF_CONST_MIN;
  }
  return x + x;
}
