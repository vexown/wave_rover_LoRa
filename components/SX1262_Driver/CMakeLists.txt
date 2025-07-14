# Component based on the SX126x driver recommended by Semtech:
# https://github.com/Lora-net/sx126x_driver

idf_component_register(
    SRCS "sx126x_driver/src/lr_fhss_mac.c" "sx126x_driver/src/sx126x_driver_version.c" "sx126x_driver/src/sx126x_lr_fhss.c" "sx126x_driver/src/sx126x.c"
    INCLUDE_DIRS "sx126x_driver/src"
    REQUIRES "driver" "esp_timer"
)
