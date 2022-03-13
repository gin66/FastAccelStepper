#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_AVR_NANO_EVERY)
#define get_char(x) pgm_read_byte(x)
#define MSG_TYPE PGM_P
#elif defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_SAM)
#define get_char(x) *x
#define MSG_TYPE const char *
#elif defined(ARDUINO_AVR_NANO_EVERY)
#else
#error "Unsupported derivate"
#endif
