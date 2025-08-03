idf_component_register(
    SRCS "src/logger.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_client json mbedtls
)
