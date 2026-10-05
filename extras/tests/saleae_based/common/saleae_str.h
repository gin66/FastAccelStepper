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
 * ESP32/Pico/IDF builds read exactly as they did before -- with one exception
 * that turned out to matter: off AVR the *formatter and the line splitter are
 * this header's own*, because libc's are far too expensive for a task stack.
 * See the block comment above `sal_snprintf`.
 *
 * What is NOT here
 * ----------------
 * The small formatter cannot print a flash string with `%s`; avr-libc spells
 * that `%S`. Nothing needs it: a `%s` argument in this harness is always a
 * number's neighbour, a token copied out of the line buffer, or a name
 * deliberately copied by `sal_to_ram()`. Using `%S` would make the AVR reply path
 * differ from the ESP32 one for no gain, so it is not used.
 */

#ifndef SALEAE_STR_H
#define SALEAE_STR_H

#include <stdarg.h>
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

#if !defined(__AVR__)

// ---------------------------------------------------------------------------
// A printf for a protocol that only prints five things
// ---------------------------------------------------------------------------
//
// Why this exists, and why it is not optional
// -----------------------------------------
// On ESP32 `sal_snprintf` used to be libc `snprintf` and `sal_sscanf` libc
// `sscanf`. Measured on the connected board with
// `uxTaskGetStackHighWaterMark()`, one call site each:
//
//     handle_line's frame + one libc sscanf   1496 bytes of stack
//     one libc snprintf                        384 bytes of stack
//
// The protocol itself only ever formats `%s`, `%u`, `%d`, `%lu`, `%ld` and
// splits one line into five whitespace-separated tokens -- so that was the
// price of newlib's `__svfscanf`/`__svfprintf` machinery, which includes float
// support this harness cannot print anyway. `CONFIG_ESP_MAIN_TASK_STACK_SIZE`
// is 3584 bytes, so most of the task's entire budget went to two library calls,
// and the overflow runs off the *top* of the stack into the DRAM tlsf pool,
// where it surfaces much later as a corrupted free list inside
// `tlsf_malloc`. See extras/doc/implemented/idf55_main_task_stack_overflow.md.
//
// AVR keeps `snprintf_P`: avr-libc's vfprintf is already small, and the 328P is
// constrained by SRAM for strings rather than by stack, so there is nothing to
// win there and a second code path to maintain.
//
// What it supports: `%s`, `%u`, `%d`, `%lu`, `%ld`. Anything else returns -1 and
// writes nothing, so a conversion cannot be added by accident and silently
// printed as garbage. `TestSaleaeFmt` in scripts/tests asserts that the format
// strings in common/ stay inside this set.
//
// Two deliberate differences from libc snprintf:
//
//   - The return value is the number of characters *stored*, not the number
//     that would have been written. The reply paths accumulate it as
//     `len += sal_snprintf(buf + len, sizeof(buf) - len, ...)`, which with
//     libc's semantics walks `len` past the end of the buffer on any reply that
//     truncates. Returning what was stored makes that pattern safe by
//     construction, on every platform.
//   - A `%s` argument is read from RAM, like every other target's `%s`. There
//     is no flash-string case and none is needed; see the header comment.

static inline size_t sal_utoa(char* out, size_t cap, unsigned long v) {
  char tmp[20];
  size_t n = 0;
  do {
    tmp[n++] = (char)('0' + (char)(v % 10UL));
    v /= 10UL;
  } while (v != 0UL && n < sizeof(tmp));
  size_t len = 0;
  while (n > 0 && len + 1 < cap) {
    out[len++] = tmp[--n];
  }
  return len;
}

static inline size_t sal_itoa(char* out, size_t cap, long v) {
  if (v < 0) {
    if (cap < 2) {
      return 0;
    }
    out[0] = '-';
    // Negate in unsigned arithmetic so LONG_MIN does not overflow.
    return 1 + sal_utoa(out + 1, cap - 1, 0UL - (unsigned long)v);
  }
  return sal_utoa(out, cap, (unsigned long)v);
}

static inline int sal_vfmt(char* dst, size_t cap, const char* fmt, va_list ap) {
  size_t n = 0;
  for (const char* p = fmt; *p != '\0'; p++) {
    if (*p != '%') {
      if (n + 1 < cap) {
        dst[n] = *p;
      }
      n++;
      continue;
    }
    p++;
    int is_long = 0;
    if (*p == 'l') {
      is_long = 1;
      p++;
    }
    char tmp[24];
    const char* str = tmp;
    size_t slen;
    switch (*p) {
      case 'u':
        slen = sal_utoa(
            tmp, sizeof(tmp),
            is_long ? va_arg(ap, unsigned long)
                    : (unsigned long)va_arg(ap, unsigned int));
        break;
      case 'd':
        slen = sal_itoa(tmp, sizeof(tmp),
                        is_long ? va_arg(ap, long) : (long)va_arg(ap, int));
        break;
      case 's': {
        const char* v = va_arg(ap, const char*);
        slen = strlen(v);
        str = v;
        break;
      }
      default:
        return -1;
    }
    // Room for the characters plus the NUL.
    if (n + slen < cap) {
      memcpy(dst + n, str, slen);
    }
    n += slen;
  }
  const size_t stored = (n < cap) ? n : cap - 1;
  if (cap > 0) {
    dst[stored] = '\0';
  }
  return (int)stored;
}

// printf for the five conversions above. See the block comment for why this
// exists and how it differs from libc snprintf.
static inline int sal_snprintf(char* dst, size_t cap, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  const int len = sal_vfmt(dst, cap, fmt, ap);
  va_end(ap);
  return len;
}

#endif  // !__AVR__

// One destination and its %Ns width. The buffer must be width + 1 bytes.
struct sal_field {
  char* dst;
  size_t width;
};

// Replaces the single `sscanf` this harness used. The `%Ns` semantics that
// matter for a line-oriented protocol:
//
//   - leading whitespace is skipped;
//   - at most `width` characters are stored, plus a NUL;
//   - the rest of the token is *discarded*, not pushed back -- sscanf does not
//     push it back either, which is what makes the driver-list field safe to
//     size exactly;
//   - scanning stops when `count` fields are filled or the line runs out.
//
// Returns the number of fields assigned, so `n <= 0` means "no command on this
// line", the same test the sscanf call used.
//
// Every platform uses this, AVR included: the widths are runtime values
// (`SALEAE_ARG2_MAX` grows with the stepper count, 24 to 384), so there is
// nothing to gain from sscanf's field-width syntax here, and `line` and the
// destinations are all RAM -- no flash string is involved.
static inline int sal_tokenize(char* line, const struct sal_field* fields,
                               size_t count) {
  int assigned = 0;
  for (size_t i = 0; i < count; i++) {
    while (*line == ' ' || *line == '\t') {
      line++;
    }
    if (*line == '\0') {
      break;
    }
    size_t w = 0;
    while (*line != '\0' && *line != ' ' && *line != '\t') {
      if (w < fields[i].width) {
        fields[i].dst[w++] = *line;
      }
      line++;
    }
    fields[i].dst[w] = '\0';
    assigned++;
  }
  return assigned;
}

#ifdef __cplusplus
}
#endif

#endif /* SALEAE_STR_H */