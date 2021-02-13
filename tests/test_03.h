bool perform_test() {
  upm_float x;

  for (uint8_t x8 = 255;x8 > 0;x8--) {
	x = upm_from((uint8_t)x8);
	uint16_t res_16 = upm_to_u16(x);
	if (res_16 != x8) {
		xprintf("%u => %x => %u\n", x8, x, res_16);
	}
	test(res_16 == x8, "conversion error from uint8_t and back to uint16_t");
  }

  uint16_t significant_16 =0xff80;
  uint16_t trigger_16 = 0x8000;
  for (uint16_t x16 = 0xffff;x16 > 0 ;x16--) {
	if ((x16 & trigger_16) == 0) {
		significant_16 >>= 1;
		trigger_16 >>= 1;
	}
	x = upm_from((uint16_t)x16);
	uint16_t res_16 = upm_to_u16(x);
	if (res_16 != (x16 & significant_16)) {
		xprintf("%x => %x => %x  (significant=%x)\n", x16, x, res_16, significant_16);
	}
	test(res_16 == (x16 & significant_16), "conversion error from uint16_t and back to uint16_t");
  }

  uint32_t significant_32 =0xff800000;
  uint32_t trigger_32 = 0x80000000;
  uint32_t delta_32 = 0x00400000;
  for (uint32_t x32 = 0xffffffff;x32 > 0 ;x32 -= delta_32) {
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
		xprintf("%x => %x => %x  (significant=%x)\n", x32, x, res_32, significant_32);
	}
	test(res_32 == (x32 & significant_32), "conversion error from uint32_t and back to uint32_t");
  }

  x = upm_from((uint32_t)0x10000);
  test(upm_to_u16(x) == 0xffff, "wrong overflow 16bit");
  x = upm_from((uint32_t)0x80000000);
  x = upm_shl(x , 1);
  test(upm_to_u32(x) == 0xffffffff, "wrong overflow 32bit");

  // Check multiply
  upm_float x1, x2, x3;
  for (int16_t sa = -40;sa <= 40;sa++) {
	  for (uint32_t a_32 = 1;a_32 <= 0x1ff;a_32++) {
		  for (uint32_t b_32 = 1;b_32 <= 0x1ff;b_32++) {
				x1 = upm_from(a_32);
				x2 = upm_from(b_32);
				if (sa > 0) {
					x1 = upm_shl(x1, sa);
				}
				else if (sa < 0) {
					x1 = upm_shr(x1, -sa);
				}
				x = upm_multiply(x1, x2);
				if (sa > 0) {
					x = upm_shr(x, sa);
				}
				else if (sa < 0) {
					x = upm_shl(x, -sa);
				}
				uint32_t res = upm_to_u32(x);
				uint32_t real_res = a_32*b_32;
				uint32_t repr_real = upm_to_u32(upm_from(real_res));
				int32_t diff = (int32_t)res - (int32_t)repr_real;
				if (res != repr_real) {
					xprintf("%d*%d=%d ~ %d =?= %d, diff=%d\n", a_32, b_32, a_32*b_32, repr_real, res,diff);
				}
				test(res == repr_real, "upm_multiply error");
		  }
	  }
  }

  x1 = upm_from((uint32_t)0x0ffff);
  x2 = upm_from((uint32_t)0x1fffe);
  x2 = upm_from((uint32_t)0x10100);
  x = upm_multiply(x1, x2);
  unsigned long back = upm_to_u32(x);
  test(back == 0xffffffff, "overflow not catched");

  x1 = upm_from((uint32_t)0x5555);
  x2 = upm_from((uint32_t)0x0055);
  x = upm_divide(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x=%x (back=%ld)\n", x1, x2, x, back);
  test(back == 0x0100, "wrong division");

  x1 = upm_from((uint32_t)0xf455);
  x2 = upm_from((uint32_t)0x0030);
  x = upm_divide(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x=%x (%ld)\n", x1, x2, x, back);
  test((back * 0x0030) <= 0xf455, "wrong division");
  test((back * 0x0031) > 0xf455, "wrong division");

  x = upm_from((uint32_t)16000000);
  xprintf("%x\n", x);
  test(x == 0x97f4, "conversion error from uint32_t 16000000");
  x = upm_multiply(x, x);
  xprintf("%x\n", x);
  test(x == 0xafe8, "upm_multiply error from uint32_t 16000000²");

  x1 = upm_from((uint32_t)0xf455);
  x2 = upm_from((uint32_t)0xf455);
  x = upm_abs_diff(x1, x2);
  back = upm_to_u32(x);
  xprintf("|%x-%x|=%x (%ld)\n", x1, x2, x, back);
  test(back == 0, "wrong upm_abs_diff");

  x1 = upm_from((uint32_t)0xf455);
  x2 = upm_from((uint32_t)0xf400);
  x = upm_abs_diff(x1, x2);
  back = upm_to_u32(x);
  xprintf("|%x-%x|=%x (%ld)\n", x1, x2, x, back);
  test(back == 0, "wrong upm_abs_diff");
  x = upm_abs_diff(x2, x1);
  back = upm_to_u32(x);
  xprintf("|%x-%x|=%x (%ld)\n", x1, x2, x, back);
  test(back == 0, "wrong upm_abs_diff");
  x = upm_sum(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x+%x=%x (%ld)\n", x1, x2, x, back);
  test(back == 0x1e800, "wrong upm_sum");
  x = upm_sum(x2, x1);
  back = upm_to_u32(x);
  xprintf("%x+%x=%x (%ld)\n", x1, x2, x, back);
  test(back == 0x1e800, "wrong upm_sum");

  x1 = upm_from((uint32_t)0xf455);
  x2 = upm_from((uint32_t)0xf3ff);
  x = upm_abs_diff(x1, x2);
  back = upm_to_u32(x);
  xprintf("|%x-%x|=%x (%ld)\n", x1, x2, x, back);
  test(back == 0x0100, "wrong upm_abs_diff");
  x = upm_abs_diff(x2, x1);
  back = upm_to_u32(x);
  xprintf("|%x-%x|=%x (%ld)\n", x1, x2, x, back);
  test(back == 0x0100, "wrong upm_abs_diff");
  x = upm_sum(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x+%x=%x (%ld)\n", x1, x2, x, back);
  test(back == 0x1e600, "wrong upm_sum");
  x = upm_sum(x2, x1);
  back = upm_to_u32(x);
  xprintf("%x+%x=%x (%ld)\n", x1, x2, x, back);
  test(back == 0x1e600, "wrong upm_sum");

  x1 = upm_from((uint32_t)0xf4555);
  x2 = upm_from((uint32_t)0x0f3ff);
  x = upm_abs_diff(x1, x2);
  back = upm_to_u32(x);
  xprintf("|%x-%x|=%x (%ld)\n", x1, x2, x, back);
  test(back == 0xe5000, "wrong upm_abs_diff");
  x = upm_abs_diff(x2, x1);
  back = upm_to_u32(x);
  xprintf("|%x-%x|=%x (%ld)\n", x1, x2, x, back);
  test(back == 0xe5000, "wrong upm_abs_diff");
  x = upm_sum(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x+%x=%x (%ld)\n", x1, x2, x, back);
  test(back == 0x102000, "wrong upm_sum");
  x = upm_sum(x2, x1);
  back = upm_to_u32(x);
  xprintf("%x+%x=%x (%ld)\n", x1, x2, x, back);
  test(back == 0x102000, "wrong upm_sum");

  x1 = upm_from((uint32_t)0xf4555);
  x = upm_shl(x1, 4);
  back = upm_to_u32(x);
  test(back == 0xf40000, "wrong upm_shl");
  x1 = upm_from((uint32_t)0xf4555);
  x = upm_shr(x1, 4);
  back = upm_to_u32(x);
  test(back == 0xf400, "wrong upm_shr");

  x1 = upm_from((uint32_t)0x20000);
  x = upm_sqrt(x1);
  back = upm_to_u32(x);
  xprintf("upm_sqrt(%x)=0x%x (%ld)  = 362 ?\n", x1, x, back);
  test(back == 362, "upm_sqrt");

  x1 = upm_from((uint32_t)0x10000);
  x = upm_sqrt(x1);
  back = upm_to_u32(x);
  xprintf("upm_sqrt(%x)=0x%x (%ld)\n", x1, x, back);
  test(back == 0x100, "upm_sqrt");

  x1 = upm_from((uint16_t)59536);
  test(x1 == 0x8fe8, "upm_from");
  x1 = upm_from((uint32_t)59536);
  test(x1 == 0x8fe8, "upm_from");
  x1 = upm_from((uint32_t)(244L * 244L));
  test(x1 == 0x8fe8, "upm_from");
  x = upm_sqrt(x1);
  back = upm_to_u32(x);
  xprintf("upm_sqrt(%x)=0x%x (%ld)\n", x1, x, back);
  test(back == 244, "upm_sqrt");

  x1 = upm_from((uint32_t)(122 * 122));
  x = upm_sqrt(x1);
  back = upm_to_u32(x);
  xprintf("upm_sqrt(%x)=0x%x (%ld)\n", x1, x, back);
  test(back == 122, "upm_sqrt");

  x1 = upm_from((uint32_t)(174L * 174));
  x = upm_sqrt(x1);
  back = upm_to_u32(x);
  xprintf("upm_sqrt(%x)=0x%x (%ld)\n", x1, x, back);
  test(back == 174, "upm_sqrt");

  x1 = upm_from((uint32_t)4);
  x = upm_sqrt(x1);
  back = upm_to_u32(x);
  xprintf("upm_sqrt(%x)=0x%x (%ld)\n", x1, x, back);
  test(back == 2, "upm_sqrt");
  x = upm_shl(upm_sqrt(upm_shr(x1, 2)), 1);
  back = upm_to_u32(x);
  xprintf("upm_shl(upm_sqrt(upm_shr(%x,2)),1)=0x%x (%ld)\n", x1, x, back);
  test(back == 2, "upm_sqrt");
  x = upm_shl(upm_sqrt(upm_shr(x1, 4)), 2);
  back = upm_to_u32(x);
  xprintf("upm_shl(upm_sqrt(upm_shr(%x,4)),2)=0x%x (%ld)\n", x1, x, back);
  x = upm_shl(upm_sqrt(upm_shr(x1, 42)), 21);
  back = upm_to_u32(x);
  xprintf("upm_shl(upm_sqrt(upm_shr(%x,42)),21)=0x%x (%ld)\n", x1, x, back);
  test(back == 2, "upm_sqrt");

  x1 = upm_from((uint32_t)250);
  x2 = upm_from((uint32_t)10000);
  x = upm_divide(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x=%x (%ld)\n", x1, x2, x, back);
  test(back == 0, "upm_divide");
  x = upm_multiply(x, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x*%x=%x (%ld)\n", x1, x2, x2, x, back);
  test(back == 251, "upm_divide");

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
  x = upm_sqrt(x);
  back = upm_to_u32(x);
  xprintf("upm_sqrt(%x/%x)=%x (%ld)\n", x1, x2, x, back);
  test(back == 0, "upm_divide/upm_sqrt");
  x = upm_multiply(x, x2);
  back = upm_to_u32(x);
  xprintf("upm_sqrt(%x/%x)*%x=%x (%ld)\n", x1, x2, x2, x, back);
  test(back == 39936, "upm_divide/upm_sqrt/upm_multiply");

  x1 = upm_from((uint32_t)(174));
  x = upm_multiply(x1, x1);
  back = upm_to_u32(x);
  xprintf("upm_multiply(%x,%x)=0x%x (%ld)\n", x1, x1, x, back);
  test(back == 30208, "upm_multiply x*x");
  x = upm_square(x1);
  back = upm_to_u32(x);
  xprintf("upm_square(%x)=0x%x (%ld)\n", x1, x, back);
  test(back == 30208, "upm_square");

  x1 = upm_from((uint32_t)(6400));
  x = upm_multiply(x1, x1);
  back = upm_to_u32(x);
  xprintf("upm_multiply(%x,%x)=0x%x (%ld)\n", x1, x1, x, back);
  test(back == 40894464, "upm_multiply x*x");
  x = upm_square(x1);
  back = upm_to_u32(x);
  xprintf("upm_square(%x)=0x%x (%ld)\n", x1, x, back);
  test(back == 40894464, "upm_square");

  x1 = upm_from((uint32_t)500);
  test(x1 == UPM_CONST_500, "const 500");
  x1 = upm_from((uint32_t)1000);
  test(x1 == UPM_CONST_1000, "const 1000");
  x1 = upm_from((uint32_t)2000);
  test(x1 == UPM_CONST_2000, "const 2000");
  x1 = upm_from((uint32_t)32000);
  test(x1 == UPM_CONST_32000, "const 32000");
  x1 = upm_from((uint32_t)16000000);
  test(x1 == UPM_CONST_16E6, "const 16E6");
  x1 = upm_from((uint32_t)128000000);
  x1 = upm_multiply(x1, UPM_CONST_1000);
  x1 = upm_multiply(x1, UPM_CONST_1000);
  xprintf("const 128e12=0x%x\n", x1);
  test(x1 == UPM_CONST_128E12, "const 128E12");

  return (error_cnt == 0);
}
