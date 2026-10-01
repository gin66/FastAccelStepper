/*
 * saleae_test.h — Platform-independent Saleae test logic.
 *
 * The test apps are tiny platform entry points around this shared module:
 *   Arduino : setup()/loop()
 *   ESP-IDF : app_main()
 */

#ifndef SALEAE_TEST_H
#define SALEAE_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

void saleae_test_setup(void);
void saleae_test_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* SALEAE_TEST_H */
