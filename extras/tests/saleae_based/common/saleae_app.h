/*
 * saleae_app.h — Saleae test app entry points (platform independent).
 *
 * The Arduino (.ino) and ESP-IDF (app_main) entry points only call these.
 */

#ifndef SALEAE_APP_H
#define SALEAE_APP_H

#ifdef __cplusplus
extern "C" {
#endif

void saleae_app_setup(void);
void saleae_app_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* SALEAE_APP_H */
