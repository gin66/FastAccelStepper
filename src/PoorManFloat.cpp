#include <stdint.h>
#include "PoorManFloat.h"

// representation is:
//
//     76543210:76543210
//     XXXXXXXX:1XXXXXXX
//     exponent mantissa
//
//  0x0080 => is 1


upm_float upm_from(uint8_t x) {
	uint16_t res;
	if ((x & 0xf0) == 0) {
	   if (x & 0x03) {
		   if (x & 0x01) {
			   x <<= 7;
			   res = x;
		   }
		   else {
			   x <<= 6;
			   res = x | 0x0100;
		   }
	   }
	   else {
		   if (x & 0x04) {
			   x <<= 5;
			   res = x | 0x0200;
		   }
		   else {
			   x <<= 4;
			   res = x | 0x0300;
		   }
	   }
	}
	else {
	   if (x & 0x30) {
		   if (x & 0x10) {
			   x <<= 3;
			   res = x | 0x0400;
		   }
		   else {
			   x <<= 2;
			   res = x | 0x0500;
		   }
	   }
	   else {
		   if (x & 0x40) {
			   x <<= 1;
			   res = x | 0x0600;
		   }
		   else {
			   res = x | 0x0700;
		   }
	   }
	}
	return res;
}
upm_float upm_from(uint16_t x) {
	uint8_t exponent;
	if ((x & 0xff00) == 0) {
		uint8_t b = x & 0xff;
		return upm_from(b);
	}
	if (x & 0xf000) {
		exponent = 8+4;
	}
	else {
		x <<= 4;
		exponent = 8+0;
	}
	if ((x & 0xc000) == 0) {
		x <<= 2;
	}
	else {
		exponent += 2;
	}
	if ((x & 0x8000) == 0) {
		x <<= 1;
	}
	else {
		exponent += 1;
	}
	x >>= 8;
	return x | (exponent << 8);
}
upm_float upm_from(uint32_t x) {
	if ((x & 0xffff0000) == 0) {
		uint16_t w = x & 0xffff;
		return upm_from(w);
	}
}
upm_float multiply(upm_float x,upm_float y) {
	uint8_t a = x & 255;
	uint8_t b = y & 255;
	uint16_t ab = a*b;
	if (ab >= 32768) {
		ab >>= 8;
		ab += (a & 0xff00) + (b & 0xff00) + 0x0800;
	}
	else {
		ab >>= 7;
		ab += (a & 0xff00) + (b & 0xff00) + 0x0700;
	}
	return ab;
}
