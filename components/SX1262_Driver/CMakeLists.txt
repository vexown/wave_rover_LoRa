# Component based on the SX126x driver recommended by Semtech:
# https://github.com/Lora-net/sx126x_driver

# Tip for understanding the driver code. If you see a function such as sx126x_set_tx_params and others 
# look for it in the SX1262 datasheet - https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/RQ000008nKCH/hp2iKwMDKWl34g1D3LBf_zC7TGBRIo2ff5LMnS8r19s
# but type it like this - instead of sx126x_set_tx_params do "SetTxParams" - these are Commands that are defined in the datasheet
# and implemented as functions in this driver. You will find much more info in the datasheet than in driver code.

idf_component_register(
    SRCS "sx126x_driver/src/lr_fhss_mac.c" "sx126x_driver/src/sx126x_driver_version.c" "sx126x_driver/src/sx126x_lr_fhss.c" "sx126x_driver/src/sx126x.c"
    INCLUDE_DIRS "sx126x_driver/src"
    REQUIRES "driver" "esp_timer"
)
