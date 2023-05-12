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

// upm implementation is dropped from 0.29 onwards
//
// upm representation is:
//
//     76543210: 76543210
//     XXXXXXXX:1XXXXXXXX
//     exponent mantissa
//
// The mantissa represents encodes 256..511 and interpreted as 1.0 to ~2.0.
// The exponent uses an offset 128 to encode negative values and base 2
//
// 0x8000 => is 1
//
// Negative numbers and zero are not available
//
//
// I want to expound on the original comment, and include an example to make
// this less tedious for someone else.  You probably need to go lookup/refresh
// your memory on floating point formats to make this make sense.  In this
// format we will use an 8 bit exponent, and 8 bits of the data type as
// the mantissa, but as the diagram above shows, the first bit will always be 1
// So lets walk through an example.  For the Due, I needed the PWM clock in
// this format, so 21,000,000.
// First off, remember that floating point wants the entire number in the range
// of the mantissa, so we right shift the value until it is within the range of
// 256-511.  So for 21,000,000, thats 21000000 >> 16 = 0x140.  Remember that the
// leading one is hidden.  so the low byte is 0x40.  You'd be wrong to assume
// the exponent is simply 16.  again, in floating point, we want the mantissa
// to be treated as less than 1, so we shift past the bits.  This means we want
// it to look like 0.0x140, or 0.101000000.  Thats what mantissa means.  So, to
// get the correct value for the exponent, we need the exponent that makes this
// (as close to) the original value as we can get, that is 20,971,520 or
// 0001 0100 0000 0000 0000 0000 0000.  So again, we need to shift past our
// number, all the way to the above, so 0.101000000 << 25 is what we want.  Or
// 0x19, (remember we had 16 shifts, but 9 bits of mantissa including the
// hidden bit 16+9=25).  Now, we get a bit tricky.  We know the hidden bit is
// always there, so we don't include it in our exponent!  Thus, our exponent
// becomes 0x18 or 24.  So the almost final value is 0x1840, but we forgot
// about the sign bit.  We need to or 0x80 to the exponent to get the sign bit
// making for a final value of 0x9840 as seen in PoorManFloat.h.  To reverse
// this, we take our value of the mantissa enter it in calculator.  We take
// the exponent and subtract 8 from it.  Enter the mantissa with the added 1
// into a "programmer calculator", 0x140 and use the LSH operator to do 0x140
// << 16, and you should get 20,971,520.  There, now there's no need to be
// afraid of making new constants for the system to use different clock rates!
// I had to do this for the Due because the PWM clock could be divisions of 2
// off the main system clock of 84MHz.  I'd need a 5.25 divider to get 16 MHz.
// Thats not an even number, so even with the arbitrary clock divider, the best
// I could have managed was 16.8 MHz.  Easier to just use the modulo 4 divider
// and add the constants.
//
//
// In order to support cubic functions, more tables would be necessary.
// Using logarithmic/exponential functions the number of tables can be limited.
// For example:
//    square/cubic root would be simply: exp(log(x) * a), with a = 1/2 for
//    square and 1/3 for cubic
//
// In addition, within FastAccelStepper there is no need for adding or
// subtracting floats. Even negative numbers are not needed and not supported by
// upm_float. Consequently, a purely logarithmic representation is completely
// sufficient.
//
// As base for exp/log we will use 2  (not 10 or e). This way the exponent of an
// upm float being 2^n needs no calculation, as log2(2^n) = n. Still the offset
// by 128 needs to be considered
//
// The mantissa of an upm float with the leading 1 is in the range 1 <= m < 2.
//
// As log2(1) = 0 and log2(2) = 1 is at the corners identical to x + 1. So it is
// interesting to look at function f(x) = log2(x) - x + 1. This function is 0
// for x at 1 and 2, positive over the range inbetween and reaches max value of
// 0.08607 at x = 1.442695. This max value is at x = 1/ln(2)
//
// The upm mantissa is in total nine bits:
//       m = 1_xxxx_xxxx
//
// So the log2 of the mantissa can be calculated by:
//       m    = 1_xxxx_xxxx
//     - 1    = 1_0000_0000
//     + f(m) = 0_000y_yyyy
//     yields = 0_zzzz_zzzz
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
    if ((x & 0x40) == 0) {
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
  uint16_t res = (((uint16_t)e) << 8) | x;
  uint8_t index = res & 0x00ff;
  uint8_t offset =
      pgm_read_byte_near(&log2_minus_x_plus_one_shifted_by_2[index]);
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
  x >>= 6;
  uint8_t exponent = 15 - leading;
  uint8_t index = x >> 2;
  uint8_t offset =
      pgm_read_byte_near(&log2_minus_x_plus_one_shifted_by_2[index]);
  if (((x & 2) != 0) && (index++ != 255)) {
    offset += pgm_read_byte_near(&log2_minus_x_plus_one_shifted_by_2[index]);
    offset += 1;
    offset >>= 1;
  }
  x += offset;
  x >>= 1;
  x += ((uint16_t)exponent) << 9;
  return x;
}
pmf_logarithmic pmfl_from(uint32_t x) {
  if ((x & 0xffff0000) == 0) {
    return pmfl_from((uint16_t)x);
  }
  if ((x & 0xff000000) == 0) {
    uint16_t w = x >> 8;
    return pmfl_from(w) + 0x1000;
  } else {
    uint16_t w = x >> 16;
    return pmfl_from(w) + 0x2000;
  }
}
uint16_t pmfl_to_u16(pmf_logarithmic x) {
  if (x < 0) {
    return 0;
  }
  if (x >= 0x2000) {
    return 0xffff;
  }
  uint8_t exponent = ((uint16_t)x) >> 9;
  x &= 0x01ff;
  x += 0x200;
  uint8_t index = ((uint16_t)x) >> 1;
  x <<= 1;
  uint8_t offset =
      pgm_read_byte_near(&x_minus_pow2_of_x_minus_one_shifted_by_2[index]);
  if (((x & 2) != 0) && (index++ != 255)) {
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
  if (x >= 0x4000) {
    return 0xffffffff;
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
  if (x >= 0x4000) {
    return 0x7fff;
  }
  if (x <= -0x4000) {
    return (int16_t)0x8001;
  }
  return x + x;
}
