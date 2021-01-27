bool perform_test() {
  upm_float x;

  x = upm_from((uint8_t)1);
  test(x == 0x8080, "conversion error from uint8_t 1");
  x = upm_from((uint8_t)2);
  test(x == 0x8180, "conversion error from uint8_t 2");
  x = upm_from((uint8_t)3);
  test(x == 0x81c0, "conversion error from uint8_t 3");
  x = upm_from((uint8_t)4);
  xprintf("%x\n", x);
  test(x == 0x8280, "conversion error from uint8_t 4");
  x = upm_from((uint8_t)8);
  test(x == 0x8380, "conversion error from uint8_t 8");
  x = upm_from((uint8_t)16);
  test(x == 0x8480, "conversion error from uint8_t 16");
  x = upm_from((uint8_t)32);
  test(x == 0x8580, "conversion error from uint8_t 32");
  x = upm_from((uint8_t)64);
  test(x == 0x8680, "conversion error from uint8_t 64");
  x = upm_from((uint8_t)128);
  test(x == 0x8780, "conversion error from uint8_t 128");

  x = upm_from((uint16_t)1);
  test(x == 0x8080, "conversion error from uint16_t 1");
  x = upm_from((uint16_t)256);
  test(x == 0x8880, "conversion error from uint16_t 256");
  x = upm_from((uint16_t)512);
  test(x == 0x8980, "conversion error from uint16_t 512");
  x = upm_from((uint16_t)1024);
  test(x == 0x8a80, "conversion error from uint16_t 1024");
  x = upm_from((uint16_t)2048);
  test(x == 0x8b80, "conversion error from uint16_t 2048");
  x = upm_from((uint16_t)4096);
  test(x == 0x8c80, "conversion error from uint16_t 4096");
  x = upm_from((uint16_t)8192);
  test(x == 0x8d80, "conversion error from uint16_t 8192");
  x = upm_from((uint16_t)16384);
  test(x == 0x8e80, "conversion error from uint16_t 16384");
  x = upm_from((uint16_t)32768);
  test(x == 0x8f80, "conversion error from uint16_t 32768");

  x = upm_from((uint32_t)1);
  test(x == 0x8080, "conversion error from uint32_t 1");
  x = upm_from((uint32_t)3);
  xprintf("%x\n", x);
  test(x == 0x81c0, "conversion error from uint32_t 3");
  x = upm_from((uint32_t)5);
  test(x == 0x82a0, "conversion error from uint32_t 5");
  x = upm_from((uint32_t)65536);
  test(x == 0x9080, "conversion error from uint32_t 65536");
  x = upm_from((uint32_t)131072);
  test(x == 0x9180, "conversion error from uint32_t 131072");
  x = upm_from((uint32_t)262144);
  test(x == 0x9280, "conversion error from uint32_t 262144");
  x = upm_from((uint32_t)524288);
  test(x == 0x9380, "conversion error from uint32_t 524288");
  x = upm_from((uint32_t)1048576);
  test(x == 0x9480, "conversion error from uint32_t 1048576");
  x = upm_from((uint32_t)2097152);
  test(x == 0x9580, "conversion error from uint32_t 2097152");
  x = upm_from((uint32_t)4194304);
  test(x == 0x9680, "conversion error from uint32_t 4194304");
  x = upm_from((uint32_t)8388608);
  test(x == 0x9780, "conversion error from uint32_t 8388608");
  x = upm_from((uint32_t)16777216);
  test(x == 0x9880, "conversion error from uint32_t 16777216");
  x = upm_from((uint32_t)33554432);
  test(x == 0x9980, "conversion error from uint32_t 33554432");
  x = upm_from((uint32_t)67108864);
  test(x == 0x9a80, "conversion error from uint32_t 67108864");
  x = upm_from((uint32_t)134217728);
  test(x == 0x9b80, "conversion error from uint32_t 134217728");
  x = upm_from((uint32_t)268435456);
  test(x == 0x9c80, "conversion error from uint32_t 268435456");
  x = upm_from((uint32_t)536870912);
  test(x == 0x9d80, "conversion error from uint32_t 536870912");
  x = upm_from((uint32_t)1073741824);
  test(x == 0x9e80, "conversion error from uint32_t 1073741824");
  x = upm_from((uint32_t)2147483648);
  test(x == 0x9f80, "conversion error from uint32_t 2147483648");

  upm_float x1, x2, x3;
  x1 = upm_from((uint32_t)3);
  x2 = upm_from((uint32_t)5);
  x3 = upm_from((uint32_t)15);
  x = upm_multiply(x1, x2);
  xprintf("%x*%x=%x\n", x1, x2, x);
  test(x == x3, "upm_multiply error 3*5");
  test(x == 0x83f0, "upm_multiply error 3*5");

  for (uint64_t i = 1; i < (1LL << 16); i *= 5) {
    x = upm_from((uint16_t)i);
    uint16_t back = upm_to_u16(x);

    test(i >= back, "conversion error to/back");
    test((i & back) == back, "conversion error to/back");
  }

  for (uint64_t i = 1; i < (1LL << 32); i *= 3) {
    x = upm_from((uint32_t)i);
    uint32_t back = upm_to_u32(x);

    test(i >= back, "conversion error to/back");
    test((i & back) == back, "conversion error to/back");
  }

  // Check overflows
  x1 = upm_from((uint32_t)0x0ffff);
  x2 = upm_from((uint32_t)0x1fffe);
  x = upm_multiply(x1, x2);
  unsigned long back = upm_to_u32(x);
  xprintf("%x*%x=%x (back=%ld)\n", x1, x2, x, back);
  test(x == 0xa0fe, "upm_multiply error");
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
  x1 = upm_from((uint32_t)16000000);
  test(x1 == UPM_CONST_16E6, "const 16E6");
  x1 = upm_from((uint32_t)128000000);
  x1 = upm_multiply(x1, UPM_CONST_1000);
  x1 = upm_multiply(x1, UPM_CONST_1000);
  xprintf("const 128e12=0x%x\n", x1);
  test(x1 == UPM_CONST_128E12, "const 128E12");

  return (error_cnt == 0);
}
