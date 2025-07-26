#ifndef SX1262_INTERFACE_H
#define SX1262_INTERFACE_H

/* SX1262 is a "LoRa Connect™ Long Range Low Power LoRa® Transceiver" by Semtech: https://www.semtech.com/products/wireless-rf/lora-connect/sx1262
 * This header file provides the interface for initializing and interacting with the SX1262 module.
 *
 * Semtech is the sole owner and developer of the core LoRa modulation technology (physical layer) and 
 * thus is the only company that manufactures the fundamental LoRa transceiver chips, like the SX1262. 
 * Other companies do not independently create LoRa chips; instead, they either integrate Semtech's chips 
 * into larger modules (like Heltec does) or license Semtech's LoRa IP to include it directly into their 
 * own System-on-Chips (SoCs), as seen with STMicroelectronics' STM32WL series. The LoRaWAN protocol, built 
 * on top of Semtech's LoRa technology, is, however, an open standard managed by the LoRa Alliance.
 * 
 * Datasheet: https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/RQ000008nKCH/hp2iKwMDKWl34g1D3LBf_zC7TGBRIo2ff5LMnS8r19s
 * SX126x Driver: https://github.com/Lora-net/sx126x_driver 
 * 
 */

#include "sx126x.h"

/**
 * @brief Initialize the SX1262 LoRa transceiver with default settings.
 *
 * This function configures the SX1262 chip for LoRa operation, including
 * setting up the radio parameters and hardware interface. It must be called
 * before any other SX1262 LoRa operations.
 *
 * @return SX126X_STATUS_OK on success, or an appropriate error code from sx126x_status_t on failure.
 */
sx126x_status_t sx1262_init_lora(void);

sx126x_status_t sx1262_send_packet(uint8_t* payload, uint16_t payload_length);

sx126x_status_t sx1262_receive_packet(uint8_t* payload, uint16_t payload_length, uint32_t rx_timeout_ms);

esp_err_t control_external_LED(bool state);

#endif // SX1262_INTERFACE_H
