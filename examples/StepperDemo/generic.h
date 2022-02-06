#if defined(ARDUINO_ARCH_AVR)
#define get_char(x) pgm_read_byte(x)
#define MSG_TYPE PGM_P
#elif defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_SAM)
#define get_char(x) *x
#define MSG_TYPE const char *
#else
#error "Unsupported derivate"
#endif
