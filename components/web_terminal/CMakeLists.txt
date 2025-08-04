idf_component_register(
    SRCS "src/web_terminal.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server esp_wifi nvs_flash
)
