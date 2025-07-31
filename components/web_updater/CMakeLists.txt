idf_component_register(
    SRCS "src/web_updater.c" "src/web_updater_wifi.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server esp_https_ota app_update esp_system esp_wifi nvs_flash esp_netif mdns
)
