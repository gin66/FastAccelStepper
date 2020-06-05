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
	   if ((x & 0x0c) == 0) {
		   if ((x & 0x02) == 0) {
			   x <<= 7;
			   res = x;
		   }
		   else {
			   x <<= 6;
			   res = x | 0x0100;
		   }
	   }
	   else {
		   if ((x & 0x08) == 0) {
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
	   if ((x & 0xc0) == 0) {
		   if ((x & 0x20) == 0) {
			   x <<= 3;
			   res = x | 0x0400;
		   }
		   else {
			   x <<= 2;
			   res = x | 0x0500;
		   }
	   }
	   else {
		   if ((x & 0x80) == 0) {
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
	if ((x & 0xff000000) == 0) {
		uint16_t w = x >> 8;
		return upm_from(w) + 0x0800;
	}
	else {
		uint16_t w = x >> 16;
		return upm_from(w) + 0x1000;
	}
}
upm_float multiply(upm_float x,upm_float y) {
	uint8_t a = x & 255;
	uint8_t b = y & 255;
	uint16_t ab = a*b;
	if (ab & 0x8000)  {
		ab >>= 8;
		ab += (x & 0xff00) + (y & 0xff00);
		ab += 0x0100;
	}
	else {
		ab >>= 7;
		ab += (x & 0xff00) + (y & 0xff00);
	}
	return ab;
}
upm_float divide(upm_float x,upm_float y) {
	if (x < y) {
		return 0;
	}
	uint8_t a = x & 255;
	uint8_t b = y & 255;

	uint8_t res = (x & 0xff00) - (y & 0xff00);
	uint8_t mask = 0x80;
	while(mask) {
		if (a == b) {
			break;
		}
		if (a > b) {
			a -= b;
			res |= mask;
		}
		a <<= 1;
		mask >>= 1;
	}
	if ((mask & 0x80) == 0) {
		res -= 0x0100;
	}
	res |= mask;
	return res;
}
