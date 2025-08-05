#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* WiFi Credentials */
/* To update the NVS with your WiFi credentials, provide them in the appropriate macros in the platformio.ini file
 * and use the param_update build environment.
 * Then build and flash the firmware. Later, you can change the macro values
 * back to placeholders and go back to the normal env so you don't accidentally commit your credentials. */
/* You can host a hotspot with these credentials if you don't want to provide your own */
#define WIFI_SSID_DEFAULT "kekwifi"
#define WIFI_PASSWORD_DEFAULT "kekpassword"

bool web_updater_wifi_start(const char *ssid, const char *password);

void start_mdns_service(void);

#ifdef __cplusplus
}
#endif
