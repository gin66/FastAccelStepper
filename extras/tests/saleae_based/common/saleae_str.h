/*
 * saleae_str.h — String and const-table placement for the Saleae test apps.
 *
 * Why this exists
 * ---------------
 * On AVR, avr-gcc puts every string literal and every `const` object in
 * `.rodata`, and the linker script copies `.rodata` into SRAM to initialise it
 * at reset. So a literal that reads like a compile-time detail is a permanent
 * SRAM allocation:
 *
 *   .data = 1136 B, of which 96 B are real variables
 *            -> 1040 B (51 % of the 328P's 2048 B) was the string pool
 *
 * The library itself is already PROGMEM-clean (`FAS_PSTR` in
 * src/fas_arch/result_codes.h); this harness was not, because it is a serial
 * protocol and a serial protocol is mostly text. Two earlier revisions of the
 * firmware were already pushed over 90 % of SRAM by adding a few words to an
 * error message (see extras/todo/120_saleae_based_test_harness.md).
 *
 * So every literal in `common/` goes through this header. The rule that makes
 * it work: a flash string is never handed to anything that reads RAM. `%s` in
 * snprintf and strcmp are the two ways to get that wrong, so the helpers for
 * both cases are here too -- `sal_to_ram()` to copy a flash string into a RAM
 * buffer for a `%s`, and `sal_strcmp()` to compare a RAM token against a flash
 * name without copying at all.
 *
 * Everything collapses to the ordinary libc call on every other target, so the
 * ESP32/Pico/IDF builds read exactly as they did before.
 *
 * What is NOT here
 * ----------------
 * `snprintf_P` cannot print a flash string with `%s`; avr-libc spells that
 * `%S`. Nothing needs it: a `%s` argument in this harness is always a number's
 * neighbour, a token copied out of the line buffer, or a name deliberately
 * copied by `sal_to_ram()`. Using `%S` would make the AVR reply path differ
 * from the ESP32 one for no gain, so it is not used.
 */

#ifndef SALEAE_STR_H
#define SALEAE_STR_H

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#if defined(__AVR__)

#include <avr/pgmspace.h>

#define SAL_PROGMEM PROGMEM
#define SAL_PSTR(s) (reinterpret_cast<const char*>(PSTR(s)))

#define sal_strcmp strcmp_P
#define sal_strncmp strncmp_P
#define sal_strcpy strcpy_P
#define sal_strlen strlen_P
#define sal_snprintf snprintf_P
#define sal_vsnprintf vsnprintf_P
#define sal_sscanf sscanf_P

#define sal_pgm_read_byte(a) pgm_read_byte(a)
#define sal_pgm_read_word(a) pgm_read_word(a)

#else

#define SAL_PROGMEM
#define SAL_PSTR(s) (s)

#define sal_strcmp strcmp
#define sal_strncmp strncmp
#define sal_strcpy strcpy
#define sal_strlen strlen
#define sal_snprintf snprintf
#define sal_vsnprintf vsnprintf
#define sal_sscanf sscanf

#define sal_pgm_read_byte(a) (*(a))
#define sal_pgm_read_word(a) (*(a))

#endif

#ifdef __cplusplus
extern "C" {
#endif

// Copy a flash string into a RAM buffer, NUL-terminated, never writing more
// than `cap` bytes. The destination is what a `%s` needs, so this is the
// bridge between the two halves of the rule above.
//
// Truncating rather than overrunning is deliberate: `cap` is the size of the
// reply buffer the copy is usually destined for, and a driver name that does
// not fit must be visibly wrong rather than a corrupted reply.
static inline void sal_to_ram(char* dst, const char* src, size_t cap) {
  size_t i = 0;
  for (; i + 1 < cap; i++) {
    const char c = (char)sal_pgm_read_byte(src + i);
    if (c == '\0') {
      break;
    }
    dst[i] = c;
  }
  dst[i] = '\0';
}

#ifdef __cplusplus
}
#endif

#endif /* SALEAE_STR_H */