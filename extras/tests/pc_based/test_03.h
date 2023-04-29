#include <math.h>

struct const_tab {
	  uint32_t val_nom;
	  uint32_t val_denom;
	  bool squared;
	  pmf_logarithmic c;
};

bool perform_test() {
  static const struct const_tab constants[13] = {
	  { 1, 1, false, PMF_CONST_1 },
      { 16000000, 1, false, PMF_CONST_16E6 },
      { 500, 1, false, PMF_CONST_500 },
      { 1000, 1, false, PMF_CONST_1000 },
      { 2000, 1, false, PMF_CONST_2000 },
      { 32000, 1, false, PMF_CONST_32000 },
      { 11313708, 1, false, PMF_CONST_16E6_DIV_SQRT_OF_2 },
      { 21000000, 1, false, PMF_CONST_21E6 },
      { 42000, 1, false, PMF_CONST_42000 },
      { 14849242, 1, false, PMF_CONST_21E6_DIV_SQRT_OF_2 },
      { 1, 500, false, PMF_CONST_1_DIV_500 },
      { 16000000, 2, true, PMF_CONST_128E12 }, // (16e6)^2 / 2
      { 22100000, 2, true, PMF_CONST_2205E11 }  // (21e6)^2 / 2
  };
  upm_float x, x1, x2, x3, y1, y2, y3;
  uint16_t l1,l2,l3,l12;
  pmf_logarithmic p1; 

  trace("Check conversion u8 <=> pmfl");
  p1 = pmfl_from((uint8_t)1);
  l1 = pmfl_to_u16(p1);
  xprintf("%x %d\n", p1,l1);
  test(p1 == 0x0000,"value 1");
  test(l1 == 1,"value 1");

  p1 = pmfl_from((uint8_t)2);
  l1 = pmfl_to_u16(p1);
  xprintf("%x %d\n", p1,l1);
  test(p1 == 0x0200,"value 2");
  test(l1 == 2,"value 2");

  for (uint8_t x_8 = 255; x_8 > 0; x_8--) {
	p1 = pmfl_from((uint8_t)x_8);
    uint16_t res_16 = pmfl_to_u16(p1);
    if (res_16 != x_8) {
      xprintf("%u => %x => %u\n", x_8, p1, res_16);
    }
    test(res_16 == x_8, "conversion error from uint8_t and back to uint16_t");
  }

  for (uint8_t n = 1;n <= 8;n++) {
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
		limit <<= n-1;
		if (delta > limit) {
		  xprintf("%u: %u => %x => %u, shifted: %d\n", x_8, x_16, p1, res_16, n);
		}
		test(delta <= limit, "conversion error from uint8_t and back to uint16_t with shift");
	  }
  }

  trace("Check conversion u8");
  for (uint8_t x_8 = 255; x_8 > 0; x_8--) {
    x = upm_from((uint8_t)x_8);
    uint16_t res_16 = upm_to_u16(x);
    if (res_16 != x_8) {
      xprintf("%u => %x => %u\n", x_8, x, res_16);
    }
    test(res_16 == x_8, "conversion error from uint8_t and back to uint16_t");
  }

  trace("Check conversion u16 <=> pmfl");
  uint16_t limit = 0x100;
  uint16_t trigger_16 = 0x8000;
  for (uint16_t x_16 = 0xffff; x_16 > 0; x_16--) {
    if ((x_16 & trigger_16) == 0) {
      limit >>= 1;
      trigger_16 >>= 1;
    }
    x = pmfl_from((uint16_t)x_16);
    uint16_t res_16 = pmfl_to_u16(x);
	uint16_t delta = x_16 - res_16;
	if (res_16 > x_16) {
		delta = res_16 - x_16;
	}
    if (delta > limit) {
      xprintf("%x => %x => %x  (limit=%x)\n", x_16, x, res_16,
              limit);
    }
    test(delta <= limit,
         "conversion error from uint16_t and back to uint16_t");
  }

  trace("Check conversion u16");
  uint16_t significant_16 = 0xff80;
  trigger_16 = 0x8000;
  for (uint16_t x_16 = 0xffff; x_16 > 0; x_16--) {
    if ((x_16 & trigger_16) == 0) {
      significant_16 >>= 1;
      trigger_16 >>= 1;
    }
    x = upm_from((uint16_t)x_16);
    uint16_t res_16 = upm_to_u16(x);
    if (res_16 != (x_16 & significant_16)) {
      xprintf("%x => %x => %x  (significant=%x)\n", x_16, x, res_16,
              significant_16);
    }
    test(res_16 == (x_16 & significant_16),
         "conversion error from uint16_t and back to uint16_t");
  }

#ifndef SIMULATOR
  uint32_t significant_32 = 0xff800000;
  uint32_t trigger_32 = 0x80000000;
  uint32_t delta_32 = 0x00400000;
  for (uint32_t x32 = 0xffffffff; x32 > 0; x32 -= delta_32) {
    if ((x32 & trigger_32) == 0) {
      significant_32 >>= 1;
      trigger_32 >>= 1;
      delta_32 >>= 1;
      if (delta_32 == 0) {
        delta_32 = 1;
      }
    }
    x = upm_from((uint32_t)x32);
    uint32_t res_32 = upm_to_u32(x);
    if (res_32 != (x32 & significant_32)) {
      xprintf("%x => %x => %x  (significant=%x)\n", x32, x, res_32,
              significant_32);
    }
    test(res_32 == (x32 & significant_32),
         "conversion error from uint32_t and back to uint32_t");
  }

  x = upm_from((uint32_t)0x10000);
  test(upm_to_u16(x) == 0xffff, "wrong overflow 16bit");
  x = upm_from((uint32_t)0x80000000);
  x = upm_shl(x, 1);
  test(upm_to_u32(x) == 0xffffffff, "wrong overflow 32bit");

  trace("Check multiply");
  for (int16_t sa = -40; sa <= 40; sa++) {
    for (uint32_t a_32 = 1; a_32 <= 0x1ff; a_32++) {
      for (uint32_t b_32 = 1; b_32 <= 0x1ff; b_32++) {
        x1 = upm_from(a_32);
        x2 = upm_from(b_32);
        if (sa > 0) {
          x1 = upm_shl(x1, sa);
        } else if (sa < 0) {
          x1 = upm_shr(x1, -sa);
        }
        x = upm_multiply(x1, x2);
        if (sa > 0) {
          x = upm_shr(x, sa);
        } else if (sa < 0) {
          x = upm_shl(x, -sa);
        }
        uint32_t res = upm_to_u32(x);
        uint32_t real_res = a_32 * b_32;
        uint32_t repr_real = upm_to_u32(upm_from(real_res));
        if (res != repr_real) {
          xprintf("%d*%d=%d ~ %d =?= %d, diff=%d\n", a_32, b_32, a_32 * b_32,
                  repr_real, res, (int32_t)res - (int32_t)repr_real);
        }
        test(res == repr_real, "upm_multiply error");
      }
    }
  }
#endif

  trace("Check pmf constants");
  bool error = false;
  for (uint8_t i = 0;i < 13;i++) {
	  const struct const_tab *dut = &constants[i];
	  pmf_logarithmic val = pmfl_from(dut->val_nom);
	  if (dut->val_denom > 1) {
		pmf_logarithmic val_denom = pmfl_from(dut->val_denom);
		val -= val_denom;
	  }
	  if (dut->squared) {
		  val += val;
	  }
	  pmf_logarithmic c = dut->c;
	  if (c != val) {
		  xprintf("(%d/%d)^%d => %x != %x\n", dut->val_nom, dut->val_denom, dut->squared ? 2:1, val, c);
		  error = true;
	  }
  }
  test(!error, "constants");

  trace("Check rsqrt");
  for (int16_t sa = -20; sa <= 20; sa++) {
    for (uint32_t a_32 = 1; a_32 <= 0x1ff; a_32++) {
      x1 = upm_from(a_32);
      if (sa > 0) {
        x1 = upm_shl(x1, sa);
      } else if (sa < 0) {
        x1 = upm_shr(x1, -sa);
      }
      x = upm_rsqrt(x1);
      upm_float xe =
          upm_multiply(x1, upm_multiply(x, x));  // sqrt not yet tested
      // xe should be approximately 1
      uint32_t res = upm_to_u32(upm_shl(xe, 16));
      int32_t diff = (int32_t)res - 0x10000;
      if (abs(diff) > 384) {
        xprintf("a=%d upm(x)=%x  upm(rsqrt(x))=%x upm(rsqrt(x)^2*x)=%x ", a_32,
                x1, x, xe);
        xprintf("shift=%d rsqrt(%d)^2*%d*0x10000=%x, diff=%d\n", sa, a_32, a_32,
                res, diff);
      }
      test(abs(diff) <= 384, "rsqrt error");
    }
  }

  trace("Check square");
  for (int16_t sa = -20; sa <= 20; sa++) {
    for (uint32_t a_32 = 1; a_32 <= 0x1ff; a_32++) {
      x1 = upm_from(a_32);
      if (sa > 0) {
        x1 = upm_shl(x1, sa);
      } else if (sa < 0) {
        x1 = upm_shr(x1, -sa);
      }
      x = upm_square(x1);
      upm_float xe = upm_multiply(x1, x1);
      // uint32_t res = upm_to_u32(upm_shl(xe,16));
      int32_t diff = (int32_t)x - (int32_t)xe;
      if (diff > 1) {  // square has better precision than multiply
        xprintf("a=%d upm(x)=%x  upm(square(x))=%x upm(x*x)=%x ", a_32, x1, x,
                xe);
        xprintf("shift=%d, diff=%d\n", sa, 0);
      }
      test(diff <= 1, "square error");
    }
  }

  trace("Check reciprocal square");
  for (int16_t sa = -20; sa <= 20; sa++) {
    for (uint32_t a_32 = 1; a_32 <= 0x1ff; a_32++) {
      x1 = upm_from(a_32);
      if (sa > 0) {
        x1 = upm_shl(x1, sa);
      } else if (sa < 0) {
        x1 = upm_shr(x1, -sa);
      }
      x = upm_rsquare(x1);

      upm_float xe = upm_multiply(x, upm_square(x1));
      // xe should be approximately 1
      uint32_t res = upm_to_u32(upm_shl(xe, 16));
      int32_t diff = (int32_t)res - 0x10000;
      if (abs(diff) > 384) {
        xprintf("a=%d upm(x)=%x  upm(rsquare(x))=%x upm(rsquare(x)*x^2)=%x ",
                a_32, x1, x, xe);
        xprintf("shift=%d rsquare(%d)*%d^2*0x10000=%x, diff=%d\n", sa, a_32,
                a_32, res, diff);
      }
      test(abs(diff) <= 384, "rsquare error");
    }
  }

  trace("Check reciprocal");
  for (int16_t sa = -20; sa <= 20; sa++) {
    for (uint32_t a_32 = 1; a_32 <= 0x1ff; a_32++) {
      x1 = upm_from(a_32);
      if (sa > 0) {
        x1 = upm_shl(x1, sa);
      } else if (sa < 0) {
        x1 = upm_shr(x1, -sa);
      }
      x = upm_reciprocal(x1);

      upm_float xe = upm_multiply(x, x1);
      // xe should be approximately 1
      uint32_t res = upm_to_u32(upm_shl(xe, 16));
      int32_t diff = (int32_t)res - 0x10000;
      if (abs(diff) > 384) {
        xprintf(
            "a=%d upm(x)=%x  upm(reciprocal(x))=%x upm(reciprocal(x)*x)=%x ",
            a_32, x1, x, xe);
        xprintf("shift=%d reciprocal(%d)*%d*0x10000=%x, diff=%d\n", sa, a_32,
                a_32, res, diff);
      }
      test(abs(diff) <= 384, "reciprocal error");
    }
  }

  trace("Check specific use cases");
  x1 = upm_from((uint32_t)0x0ffff);
  x2 = upm_from((uint32_t)0x10100);
  x = upm_multiply(x1, x2);
  unsigned long back = upm_to_u32(x);
  test(back == 0xffffffff, "overflow not catched");

  x1 = upm_from((uint32_t)0x5555);
  x2 = upm_from((uint32_t)0x0055);
  x = upm_divide(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x=%x (back=%ld)\n", x1, x2, x, back);
  test(back == 0x0101, "wrong division");

  x1 = upm_from((uint32_t)0xf455);
  x2 = upm_from((uint32_t)0x0030);
  x = upm_divide(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x=%x (%ld)\n", x1, x2, x, back);
  test((back * 0x0030) <= 0xf455, "wrong division");
  test((back * 0x0031) > 0xf455, "wrong division");

  x1 = upm_from((uint32_t)0xf4555);
  x = upm_shl(x1, 4);
  back = upm_to_u32(x);
  test(back == 0xf40000, "wrong upm_shl");
  x1 = upm_from((uint32_t)0xf4555);
  x = upm_shr(x1, 4);
  back = upm_to_u32(x);
  test(back == 0xf400, "wrong upm_shr");

  x1 = upm_from((uint32_t)250);
  x2 = upm_from((uint32_t)10000);
  x = upm_divide(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x=%x (%ld)\n", x1, x2, x, back);
  test(back == 0, "upm_divide");
  x = upm_multiply(x, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x*%x=%x (%ld)\n", x1, x2, x2, x, back);
  test(back == 249, "upm_divide");

  x1 = upm_from((uint32_t)250);
  x2 = upm_from((uint32_t)10000);
  x = upm_divide(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x=%x (%ld)\n", x1, x2, x, back);
  test(back == 0, "upm_divide");
  x = upm_shl(x, 10);
  back = upm_to_u32(x);
  xprintf("upm_shl(%x/%x,10)=%x (%ld)\n", x1, x2, x, back);
  test(back == 25, "upm_divide/upm_shl");

  x1 = upm_from((uint32_t)1600);
  x2 = upm_from((uint32_t)1000000);
  x = upm_divide(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x=%x (%ld)\n", x1, x2, x, back);
  test(back == 0, "upm_divide");
  x = upm_shl(x, 20);
  back = upm_to_u32(x);
  xprintf("upm_shl(%x/%x,20)=%x (%ld)\n", x1, x2, x, back);
  test(back == 1680, "upm_divide/upm_shl");
  x = upm_shr(x, 20);
  back = upm_to_u32(x);
  xprintf("upm_shr(upm_shl(%x/%x,20),20)=%x (%ld)\n", x1, x2, x, back);
  test(back == 0, "upm_divide/upm_shl");

  x1 = upm_from((uint32_t)1);
  xprintf("const 1=0x%x\n", x1);
  test(x1 == UPM_CONST_1, "const 1");
  x1 = upm_from((uint32_t)500);
  test(x1 == UPM_CONST_500, "const 500");
  x1 = upm_divide(UPM_CONST_1, UPM_CONST_500);
  xprintf("const 1/500=0x%x\n", x1);
  test(x1 == UPM_CONST_1_DIV_500, "const 0.002");
  x1 = upm_from((uint32_t)1000);
  test(x1 == UPM_CONST_1000, "const 1000");
  x1 = upm_from((uint32_t)2000);
  test(x1 == UPM_CONST_2000, "const 2000");
  x1 = upm_from((uint32_t)32000);
  test(x1 == UPM_CONST_32000, "const 32000");
  x1 = upm_from((uint32_t)42000);
  test(x1 == UPM_CONST_42000, "const 42000");
  x1 = upm_from((uint32_t)16000000);
  test(x1 == UPM_CONST_16E6, "const 16E6");
  x1 = upm_from((uint32_t)21000000);
  test(x1 == UPM_CONST_21E6, "const 21E6");
  x1 = upm_from((uint32_t)128000000);
  x1 = upm_multiply(x1, UPM_CONST_1000);
  x1 = upm_multiply(x1, UPM_CONST_1000);
  xprintf("const 128e12=0x%x\n", x1);
  test(x1 == UPM_CONST_128E12, "const 128E12");
  x1 = upm_from((uint32_t)221000000);
  x1 = upm_multiply(x1, UPM_CONST_1000);
  x1 = upm_multiply(x1, UPM_CONST_1000);
  xprintf("const 2205e11=0x%x\n", x1);
  xprintf("const 2205e11=0x%x\n", UPM_CONST_2205E11);
  test(x1 == UPM_CONST_2205E11, "const 2205E11");
  x1 = UPM_CONST_16E6;
  x1 = upm_multiply(x1, upm_rsqrt(upm_from((uint16_t)2)));
  xprintf("const 16e6/sqrt(2)=0x%x\n", x1);
  test(x1 == UPM_CONST_16E6_DIV_SQRT_OF_2, "const 16E6/sqrt(2)");
  x1 = UPM_CONST_21E6;
  x1 = upm_multiply(x1, upm_rsqrt(upm_from((uint16_t)2)));
  xprintf("const 21e6/sqrt(2)=0x%x\n", x1);
  xprintf("%d\n", upm_to_u32(UPM_CONST_21E6_DIV_SQRT_OF_2));
  test(x1 == UPM_CONST_21E6_DIV_SQRT_OF_2, "const 21E6/sqrt(2)");

  return (error_cnt == 0);
}
