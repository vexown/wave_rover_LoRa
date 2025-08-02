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

#define MAX_LORA_PAYLOAD_LENGTH 255 // Maximum payload length for LoRa packets (uint8_t)

#include <stdint.h>

typedef struct 
{
    /**
     * Signal RSSI after despreading (in dBm).
     * This represents the estimated power of the LoRa signal itself,
     * measured after the chirp spread-spectrum demodulation has removed most of the noise.
     * It's generally more accurate for evaluating actual link quality, since it excludes background noise.
     * Use this to assess how strong the useful LoRa signal was.
     */
    int8_t signal_rssi_dbm;

    /**
     * Average RSSI over the entire received packet (in dBm).
     * This includes both signal and background noise.
     * Use this if you want to know the total RF energy level during reception,
     * or if you're comparing to systems where RSSI includes noise.
     */
    int8_t rssi_dbm;

    /**
     * Signal-to-noise ratio of the received LoRa packet (in dB).
     * Calculated during demodulation; useful for understanding how well the signal
     * stands out from noise. Positive values indicate good quality.
     */
    int8_t snr_db;
} lora_packet_metrics_t;


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

sx126x_status_t sx1262_send_packet(uint8_t* payload, uint8_t payload_length);

sx126x_status_t sx1262_receive_packet(uint8_t* payload, uint8_t payload_length, lora_packet_metrics_t* pkt_metrics, uint32_t rx_timeout_ms);

esp_err_t control_external_LED(bool state);

#endif // SX1262_INTERFACE_H
