#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool web_updater_wifi_start(const char *ssid, const char *password);

void start_mdns_service(void);

#ifdef __cplusplus
}
#endif
