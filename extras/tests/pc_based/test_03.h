#include <math.h>

struct const_tab {
  uint32_t val_nom;
  uint32_t val_denom;
  bool squared;
  pmf_logarithmic c;
};

bool perform_test() {
#define NR_OF_CONSTANTS 13
  static const struct const_tab constants[NR_OF_CONSTANTS] = {
      {1, 1, false, PMF_CONST_1},
      {16000000, 1, false, PMF_CONST_16E6},
      {3, 2, false, PMF_CONST_3_DIV_2},
      {500, 1, false, PMF_CONST_500},
      {1000, 1, false, PMF_CONST_1000},
      {2000, 1, false, PMF_CONST_2000},
      {32000, 1, false, PMF_CONST_32000},
      {11313708, 1, false, PMF_CONST_16E6_DIV_SQRT_OF_2},
      {21000000, 1, false, PMF_CONST_21E6},
      {42000, 1, false, PMF_CONST_42000},
      {14849242, 1, false, PMF_CONST_21E6_DIV_SQRT_OF_2},
      {16000000, 2, true, PMF_CONST_128E12},  // (16e6)^2 / 2
      {22100000, 2, true, PMF_CONST_2205E11}  // (21e6)^2 / 2
  };
  uint16_t l1;
  pmf_logarithmic p1;

  trace("Check leading_zeros()");
  for (int16_t x_8 = 0; x_8 <= 255; x_8++) {
    uint8_t leading = leading_zeros(x_8);
    test(x_8 < (1 << (8 - leading)), "leading zeros too much");
    test(x_8 >= (0x80 >> leading), "leading zeros too less");
  }

  trace("Check conversion u8 <=> pmfl");
  p1 = pmfl_from((uint8_t)1);
  l1 = pmfl_to_u16(p1);
  xprintf("%x %d\n", p1, l1);
  test(p1 == 0x0000, "value 1");
  test(l1 == 1, "value 1");

  trace("Check conversion u8 <=> pmfl by shift 8bit");
  for (uint8_t n = 1; n < 8; n++) {
    uint8_t v = 1 << n;
    p1 = pmfl_from((uint8_t)v);
    l1 = pmfl_to_u16(p1);
    xprintf("8bit: %x %d\n", p1, l1);
    test(p1 == ((int16_t)n) << 9, "value");
    test(l1 == v, "value");
  }

  trace("Check conversion u8 <=> pmfl by shift 16bit");
  for (uint8_t n = 1; n < 16; n++) {
    uint16_t v = 1 << n;
    p1 = pmfl_from((uint16_t)v);
    l1 = pmfl_to_u16(p1);
    xprintf("16bit: %x %d\n", p1, l1);
    test(p1 == ((int16_t)n) << 9, "value");
    test(l1 == v, "value");
  }

  trace("Check conversion u8 <=> pmfl by shift 32bit");
  for (uint8_t n = 1; n < 32; n++) {
    uint32_t v = 1;
    v <<= n;
    p1 = pmfl_from((uint32_t)v);
    uint32_t res = pmfl_to_u32(p1);
    xprintf("32bit: %x %u\n", p1, res);
    test(p1 == (((int16_t)n) << 9), "value");
    test(res == v, "value");
  }

  trace("Check conversion u8 <=> pmfl for all values");
  for (uint8_t x_8 = 255; x_8 > 0; x_8--) {
    p1 = pmfl_from((uint8_t)x_8);
    uint16_t res_16 = pmfl_to_u16(p1);
    if (res_16 != x_8) {
      xprintf("%u => %x => %u\n", x_8, p1, res_16);
    }
    test(res_16 == x_8, "conversion error from uint8_t and back to uint16_t");
  }

  for (uint8_t n = 1; n <= 8; n++) {
    for (uint8_t x_8 = 255; x_8 > 0; x_8--) {
      uint16_t x_16 = x_8;
      x_16 <<= n;
      p1 = pmfl_from((uint8_t)x_8);
      p1 = pmfl_shl(p1, n);
      uint16_t res_16 = pmfl_to_u16(p1);
      uint16_t delta = x_16 - res_16;
      if (res_16 > x_16) {
        delta = res_16 - x_16;
      }
      uint16_t limit = 1;
      limit <<= n - 1;
      if (delta > limit) {
        xprintf("%u: %u => %x => %u, shifted: %d\n", x_8, x_16, p1, res_16, n);
      }
      test(delta <= limit,
           "conversion error from uint8_t and back to uint16_t with shift");
    }
  }

  for (uint8_t n = 1; n <= 24; n++) {
    for (uint8_t x_8 = 255; x_8 > 0; x_8--) {
      uint32_t x_32 = x_8;
      x_32 <<= n;
      p1 = pmfl_from((uint8_t)x_8);
      p1 = pmfl_shl(p1, n);
      uint32_t res_32 = pmfl_to_u32(p1);
      uint32_t delta = x_32 - res_32;
      if (res_32 > x_32) {
        delta = res_32 - x_32;
      }
      uint32_t limit = 1;
      limit <<= n - 1;
      if (delta > limit) {
        xprintf("%u: %u => %x => %u, shifted: %d\n", x_8, x_32, p1, res_32, n);
      }
      test(delta <= limit,
           "conversion error from uint8_t and back to uint32_t with shift");
    }
  }
  trace("Check conversion u16 <=> pmfl");
  uint16_t limit = 0x100;
  uint16_t trigger_16 = 0x8000;
  for (uint16_t x_16 = 0xffff; x_16 > 0; x_16--) {
    if ((x_16 & trigger_16) == 0) {
      limit >>= 1;
      trigger_16 >>= 1;
    }
    pmf_logarithmic p = pmfl_from((uint16_t)x_16);
    uint16_t res_16 = pmfl_to_u16(p);
    uint16_t delta = x_16 - res_16;
    if (res_16 > x_16) {
      delta = res_16 - x_16;
    }
    if (delta > limit) {
      xprintf("%x => %x => %x  (limit=%x)\n", x_16, p, res_16, limit);
    }
    test(delta <= limit, "conversion error from uint16_t and back to uint16_t");
  }

  for (uint8_t n = 1; n <= 16; n++) {
    uint32_t msb = 32768;
    for (uint16_t x_16 = 65535; x_16 > 256; x_16--) {
      if ((x_16 & msb) == 0) {
        msb >>= 1;
      }
      uint32_t x_32 = x_16;
      x_32 <<= n;
      p1 = pmfl_from((uint16_t)x_16);
      p1 = pmfl_shl(p1, n);
      uint32_t res_32 = pmfl_to_u32(p1);
      uint32_t delta = x_32 - res_32;
      uint32_t limit = (msb << n) >> 8;
      limit += limit >> 2;
      if (res_32 > x_32) {
        delta = res_32 - x_32;
      }
      if (delta > limit) {
        xprintf("%u: %u => %x => %u, shifted: %d, delta: %d > %d\n", x_16, x_32,
                p1, res_32, n, delta, limit);
      }
      test(delta <= limit,
           "conversion error from uint16_t and back to uint32_t with shift");
    }
  }

  p1 = pmfl_from((uint32_t)0x10000);
  test(pmfl_to_u16(p1) == 0xffff, "wrong overflow 16bit");
  p1 = pmfl_from((uint32_t)0x80000000);
  p1 = pmfl_shl(p1, 1);
  test(pmfl_to_u32(p1) == 0xffffffff, "wrong overflow 32bit");

#ifndef SIMULATOR
  trace("Check conversion u32 <=> pmfl");
  uint32_t trigger_32 = 0x80000000;
  uint32_t delta_32 = 0x01000000;
  for (uint32_t x_32 = 0xffffffff; x_32 > 0; x_32 -= delta_32) {
    if ((x_32 & trigger_32) == 0) {
      trigger_32 >>= 1;
      delta_32 >>= 1;
      if (delta_32 == 0) {
        delta_32 = 1;
      }
    }
    pmf_logarithmic px = pmfl_from((uint32_t)x_32);
    uint32_t res_32 = pmfl_to_u32(px);
    uint32_t delta = x_32 - res_32;
    if (res_32 > x_32) {
      delta = res_32 - x_32;
    }
    if (delta > delta_32 + 1) {
      xprintf("%x => %x => %x  (delta=%x > %x)\n", x_32, px, res_32, delta,
              delta_32);
    }
    test(delta <= delta_32 + 1,
         "conversion error from uint32_t and back to uint32_t");
  }

  trace("Check multiply");
  for (int16_t sa = -40; sa <= 40; sa++) {
    for (uint32_t a_32 = 1; a_32 <= 0x1ff; a_32++) {
      for (uint32_t b_32 = 1; b_32 <= 0x1ff; b_32++) {
        p1 = pmfl_from(a_32);
        pmf_logarithmic p2 = pmfl_from(b_32);
        if (sa > 0) {
          p1 = pmfl_shl(p1, sa);
        } else if (sa < 0) {
          p1 = pmfl_shr(p1, -sa);
        }
        pmf_logarithmic p = pmfl_multiply(p1, p2);
        if (sa > 0) {
          p = pmfl_shr(p, sa);
        } else if (sa < 0) {
          p = pmfl_shl(p, -sa);
        }
        uint32_t res = pmfl_to_u32(p);
        uint32_t real_res = a_32 * b_32;
        uint32_t repr_real = pmfl_to_u32(pmfl_from(real_res));
        uint32_t delta = res - repr_real;
        if (res < repr_real) {
          delta = repr_real - res;
        }
        uint32_t limit = real_res >> 7;
        if (delta > limit) {
          xprintf("%d*%d=%d ~ %d =?= %d, diff=%d\n", a_32, b_32, a_32 * b_32,
                  repr_real, res, (int32_t)res - (int32_t)repr_real);
        }
        test(delta <= limit, "pmfl_multiply error");
      }
    }
  }
#endif

  trace("Check pmf constants");
  bool error = false;
  for (uint8_t i = 0; i < NR_OF_CONSTANTS; i++) {
    const struct const_tab *dut = &constants[i];
    pmf_logarithmic val = pmfl_from(dut->val_nom);
    if (dut->squared) {
      val += val;
    }
    if (dut->val_denom > 1) {
      pmf_logarithmic val_denom = pmfl_from(dut->val_denom);
      val -= val_denom;
    }
    pmf_logarithmic c = dut->c;
    if (c != val) {
      xprintf("(%d/%d)^%d => %x != %x\n", dut->val_nom, dut->val_denom,
              dut->squared ? 2 : 1, val, c);
      error = true;
    }
  }
  test(!error, "constants");

  trace("Check rsqrt");
  for (int16_t sa = -20; sa <= 20; sa++) {
    for (uint32_t a_32 = 1; a_32 <= 0x1ff; a_32++) {
      p1 = pmfl_from(a_32);
      if (sa > 0) {
        p1 = pmfl_shl(p1, sa);
      } else if (sa < 0) {
        p1 = pmfl_shr(p1, -sa);
      }
      pmf_logarithmic p = pmfl_rsqrt(p1);
      pmf_logarithmic pe =
          pmfl_multiply(p1, pmfl_multiply(p, p));  // sqrt not yet tested
      // pe should be approximately 1
      uint32_t res = pmfl_to_u32(pmfl_shl(pe, 16));
      int32_t diff = (int32_t)res - 0x10000;
      if (abs(diff) > 384) {
        xprintf("a=%d pmfl(x)=%x  pmfl(rsqrt(x))=%x pmfl(rsqrt(x)^2*x)=%x ",
                a_32, p1, p, pe);
        xprintf("shift=%d rsqrt(%d)^2*%d*0x10000=%x, diff=%d\n", sa, a_32, a_32,
                res, diff);
      }
      test(abs(diff) <= 384, "rsqrt error");
    }
  }

  trace("Check square");
  for (int16_t sa = -20; sa <= 20; sa++) {
    for (uint32_t a_32 = 1; a_32 <= 0x1ff; a_32++) {
      p1 = pmfl_from(a_32);
      if (sa > 0) {
        p1 = pmfl_shl(p1, sa);
      } else if (sa < 0) {
        p1 = pmfl_shr(p1, -sa);
      }
      pmf_logarithmic p = pmfl_square(p1);
      pmf_logarithmic pe = pmfl_multiply(p1, p1);
      int32_t diff = (int32_t)p - (int32_t)pe;
      if (diff > 1) {  // square has better precision than multiply
        xprintf("a=%d pmfl(x)=%x  pmfl(square(x))=%x pmfl(x*x)=%x ", a_32, p1,
                p, pe);
        xprintf("shift=%d, diff=%d\n", sa, 0);
      }
      test(diff <= 1, "square error");
    }
  }

  trace("Check reciprocal square");
  for (int16_t sa = -20; sa <= 20; sa++) {
    for (uint32_t a_32 = 1; a_32 <= 0x1ff; a_32++) {
      p1 = pmfl_from(a_32);
      if (sa > 0) {
        p1 = pmfl_shl(p1, sa);
      } else if (sa < 0) {
        p1 = pmfl_shr(p1, -sa);
      }
      pmf_logarithmic p = pmfl_rsquare(p1);

      pmf_logarithmic pe = pmfl_multiply(p, pmfl_square(p1));
      // pe should be approximately 1
      uint32_t res = pmfl_to_u32(pmfl_shl(pe, 16));
      int32_t diff = (int32_t)res - 0x10000;
      if (abs(diff) > 384) {
        xprintf("a=%d pmfl(x)=%x  pmfl(rsquare(x))=%x pmfl(rsquare(x)*x^2)=%x ",
                a_32, p1, p, pe);
        xprintf("shift=%d rsquare(%d)*%d^2*0x10000=%x, diff=%d\n", sa, a_32,
                a_32, res, diff);
      }
      test(abs(diff) <= 384, "rsquare error");
    }
  }

  trace("Check reciprocal");
  for (int16_t sa = -20; sa <= 20; sa++) {
    for (uint32_t a_32 = 1; a_32 <= 0x1ff; a_32++) {
      p1 = pmfl_from(a_32);
      if (sa > 0) {
        p1 = pmfl_shl(p1, sa);
      } else if (sa < 0) {
        p1 = pmfl_shr(p1, -sa);
      }
      pmf_logarithmic p = pmfl_reciprocal(p1);

      pmf_logarithmic pe = pmfl_multiply(p, p1);
      // xe should be approximately 1
      uint32_t res = pmfl_to_u32(pmfl_shl(pe, 16));
      int32_t diff = (int32_t)res - 0x10000;
      if (abs(diff) > 384) {
        xprintf(
            "a=%d pmfl(x)=%x  pmfl(reciprocal(x))=%x pmfl(reciprocal(x)*x)=%x ",
            a_32, p1, p, pe);
        xprintf("shift=%d reciprocal(%d)*%d*0x10000=%x, diff=%d\n", sa, a_32,
                a_32, res, diff);
      }
      test(abs(diff) <= 384, "reciprocal error");
    }
  }

  trace("Check specific use cases");
  pmf_logarithmic x, x1, x2;
  x1 = pmfl_from((uint32_t)0x0ffff);
  x2 = pmfl_from((uint32_t)0x10100);
  x = pmfl_multiply(x1, x2);
  unsigned long back = pmfl_to_u32(x);
  test(back == 0xffffffff, "overflow not catched");

  x1 = pmfl_from((uint32_t)0x5555);
  x2 = pmfl_from((uint32_t)0x0055);
  x = pmfl_divide(x1, x2);
  back = pmfl_to_u32(x);
  xprintf("%x/%x=%x (back=%ld)\n", x1, x2, x, back);
  test(back == 0x0101, "wrong division");

  x1 = pmfl_from((uint32_t)0xf455);
  x2 = pmfl_from((uint32_t)0x0030);
  x = pmfl_divide(x1, x2);
  back = pmfl_to_u32(x);
  back--;  // result is too high by one
  xprintf("%x/%x=%x (%ld) f455/0030=%d\n", x1, x2, x, back, 0xf455 / 0x30);
  test((back * 0x0030) <= 0xf455, "wrong division 1");
  test((back * 0x0031) > 0xf455, "wrong division 2");

  x1 = pmfl_from((uint32_t)0xf4555);
  x = pmfl_shl(x1, 4);
  back = pmfl_to_u32(x);
  xprintf("%x => %x (%lx)\n", x1, x, back);
  test(back == 0xf44000, "wrong pmfl_shl");
  x1 = pmfl_from((uint32_t)0xf4555);
  x = pmfl_shr(x1, 4);
  back = pmfl_to_u32(x);
  xprintf("%x => %x (%lx)\n", x1, x, back);
  test(back == 0xf440, "wrong pmfl_shr");

  x1 = pmfl_from((uint32_t)250);
  x2 = pmfl_from((uint32_t)10000);
  x = pmfl_divide(x1, x2);
  back = pmfl_to_u32(x);
  xprintf("%x/%x=%x (%ld)\n", x1, x2, x, back);
  test(back == 0, "pmfl_divide 1");
  x = pmfl_multiply(x, x2);
  back = pmfl_to_u32(x);
  xprintf("%x/%x*%x=%x (%ld)\n", x1, x2, x2, x, back);
  back--;  // value is one too high
  test(back == 249, "pmfl_divide 2");

  x1 = pmfl_from((uint32_t)250);
  x2 = pmfl_from((uint32_t)10000);
  x = pmfl_divide(x1, x2);
  back = pmfl_to_u32(x);
  xprintf("%x/%x=%x (%ld)\n", x1, x2, x, back);
  test(back == 0, "pmfl_divide");
  x = pmfl_shl(x, 10);
  back = pmfl_to_u32(x);
  xprintf("pmfl_shl(%x/%x,10)=%x (%ld)\n", x1, x2, x, back);
  test(back == 25, "pmfl_divide/pmfl_shl");

  x1 = pmfl_from((uint32_t)1600);
  x2 = pmfl_from((uint32_t)1000000);
  x = pmfl_divide(x1, x2);
  back = pmfl_to_u32(x);
  xprintf("%x/%x=%x (%ld)\n", x1, x2, x, back);
  test(back == 0, "pmfl_divide");
  x = pmfl_shl(x, 20);
  back = pmfl_to_u32(x);
  xprintf("pmfl_shl(%x/%x,20)=%x (%ld)\n", x1, x2, x, back);
  test(back + 2 == 1678, "pmfl_divide/pmfl_shl");
  x = pmfl_shr(x, 20);
  back = pmfl_to_u32(x);
  xprintf("pmfl_shr(pmfl_shl(%x/%x,20),20)=%x (%ld)\n", x1, x2, x, back);
  test(back == 0, "pmfl_divide/pmfl_shl");

  x1 = pmf_logarithmic((uint32_t)1500);
  x = pmfl_pow_div_3(x1);
  xprintf("%d/3=%d\n", x1, x);
  // +1 is deviation
  test(x + 1 == 500, "pmfl_pow_div_3");

  return (error_cnt == 0);
}
