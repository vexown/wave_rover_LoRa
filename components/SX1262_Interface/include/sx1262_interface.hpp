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
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdint.h>

#define MAX_LORA_PAYLOAD_LENGTH 255 // Maximum payload length for LoRa packets (uint8_t)

/* Mutex for protecting LoRaMessageGlobal access */
extern SemaphoreHandle_t lora_message_mutex;

/* Device modes */
/* To change the device mode, select appropriate mode in platformio.ini and use the param_update build environment.
 * The device mode is stored in NVS, so you can change it later without reflashing the firmware. */
#define RECEIVER_MODE 0
#define TRANSMITTER_MODE 1
#define TRANSCEIVER_MODE 2
#define DEVICE_MODE_DEFAULT RECEIVER_MODE

/* ETSI EN 300 220-2 V3.3.1 (2025-01) defines the frequency bands and power limits for "short-range" (more like low-power) (SRD) 
 * sub 1GHz (25MHz - 1000MHz) up to 500mW e.r.p RF devices in Europe (see Annex B for details):
 * https://www.etsi.org/deliver/etsi_en/300200_300299/30022002/03.03.01_30/en_30022002v030301va.pdf 
 * 
 * As for WHERE exactly within the Band to place your carrier frequency:
 * Per ETSI EN 300 220-2, Clause 4.2.3, for equipment where the
 * Permitted Frequency Band (PFB) is not channelized by regulation (band D or band J)
 * the Nominal Operating Frequency (carrier frequency) must always be
 * the mid-point of your specific Operating Channel (OC).
 * 
 * Your "Operating Channel" is defined by the bandwidth of your LoRa signal.
 * 
 * This rule applies regardless of how wide the overall permitted band is.
 * A wider band simply offers flexibility in *where* you place your LoRa
 * operating channel within that band. However, once placed, the carrier
 * for that particular operating channel must remain centered within it.
 * 
 * For example:
 * - If using the 869.400 MHz to 869.650 MHz band (Band O) with a 250 kHz OCW,
 *   the ideal carrier is 869.525 MHz to center the 250 kHz signal within that band.
 * - If using a wider band (e.g., 863 MHz - 870 MHz) and you choose to transmit
 *   a 125 kHz signal centered at 868.1 MHz, your Operating Channel would be
 *   868.0375 MHz to 868.1625 MHz, and 868.1 MHz is its midpoint.
 * 
 * This ensures your signal remains entirely within your chosen operating channel
 * and does not intrude on other frequencies.
 * 
 * Below are the definitions for ETSI bands relevant to what this specific Heltec V3 board is tuned to (says 863-928MHz on the packaging) 
 */
#define FREQUENCY_ETSI_EN_300_220_BAND_K 864000000UL    // (864MHz)      863 MHz to 865 MHz 25 mW e.r.p. ≤ 0,1 % duty cycle
#define FREQUENCY_ETSI_EN_300_220_BAND_L 866500000UL    // (866.5MHz)    865 MHz to 868 MHz 25 mW e.r.p. ≤ 1 % duty cycle
#define FREQUENCY_ETSI_EN_300_220_BAND_M 868300000UL    // (868.3MHz)    868,000 MHz to 868,600 MHz 25 mW e.r.p. ≤ 1 % duty cycle
#define FREQUENCY_ETSI_EN_300_220_BAND_N 868950000UL    // (868.950MHz)  868,700 MHz to 869,200 MHz 25 mW e.r.p. ≤ 0,1 % duty cycle
#define FREQUENCY_ETSI_EN_300_220_BAND_O 869525000UL    // (869.525MHz)  869,400 MHz to 869,650 MHz 500 mW e.r.p. ≤ 10 % duty cycle

/* ETSI EN 300 220-2 duty cycle limits */
#define DUTY_CYCLE_LIMIT_ETSI_EN_300_220_BAND_K 0.001f  // 0.1%
#define DUTY_CYCLE_LIMIT_ETSI_EN_300_220_BAND_L 0.01f   // 1%
#define DUTY_CYCLE_LIMIT_ETSI_EN_300_220_BAND_M 0.01f   // 1%
#define DUTY_CYCLE_LIMIT_ETSI_EN_300_220_BAND_N 0.001f  // 0.1%
#define DUTY_CYCLE_LIMIT_ETSI_EN_300_220_BAND_O 0.1f    // 10%

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
 * @brief Configuration structure for SX1262 initialization
 */
typedef struct
{
    uint32_t frequency_hz;              //!< RF frequency in Hz
    sx126x_standby_cfg_t standby_mode;  //!< Standby mode configuration
    sx126x_reg_mod_t regulator_mode;    //!< Regulator mode (LDO or DC-DC)
    sx126x_tcxo_ctrl_voltages_t tcxo_voltage; //!< TCXO voltage
    uint32_t tcxo_timeout_steps;        //!< TCXO startup timeout in RTC steps
    bool dio2_rf_switch_ctrl;           //!< Enable DIO2 as RF switch control
    sx126x_pkt_type_t packet_type;      //!< Packet type (LoRa, GFSK, etc.)
    sx126x_pa_cfg_params_t pa_config;   //!< PA configuration parameters
    int8_t tx_power_dbm;                //!< TX output power in dBm
    sx126x_ramp_time_t ramp_time;       //!< PA ramp time
    sx126x_mod_params_lora_t lora_mod_params; //!< LoRa modulation parameters
    sx126x_irq_mask_t irq_mask;         //!< IRQ mask for DIO1
    float duty_cycle_ratio;             //!< Duty cycle ratio (0.0 to 1.0)
} sx1262_init_config_t;

/**
 * @brief Configuration structure for packet transmission
 */
typedef struct
{
    uint16_t preamble_length_symbols;   //!< Preamble length in symbols
    sx126x_lora_pkt_len_modes_t header_type; //!< Header type (explicit/implicit)
    bool crc_enabled;                   //!< Enable CRC
    bool invert_iq;                     //!< Invert I/Q for downlink
    uint8_t sync_word;                  //!< LoRa sync word
} sx1262_tx_config_t;

/**
 * @brief Configuration structure for packet reception
 */
typedef struct
{
    uint16_t preamble_length_symbols;   //!< Expected preamble length in symbols
    sx126x_lora_pkt_len_modes_t header_type; //!< Expected header type
    bool crc_enabled;                   //!< Expect CRC
    bool invert_iq;                     //!< Invert I/Q for downlink reception
    uint32_t rx_timeout_ms;             //!< Reception timeout in milliseconds
} sx1262_rx_config_t;

/**
 * @brief Get default initialization configuration
 * 
 * @param freq_hz RF frequency in Hz
 * @param duty_cycle Duty cycle ratio (e.g., 0.1 for 10%)
 * @return sx1262_init_config_t Default configuration structure
 */
sx1262_init_config_t sx1262_get_default_init_config(uint32_t freq_hz, float duty_cycle);

/**
 * @brief Get default TX configuration
 * 
 * @return sx1262_tx_config_t Default TX configuration structure
 */
sx1262_tx_config_t sx1262_get_default_tx_config(void);

/**
 * @brief Get default RX configuration
 * 
 * @param timeout_ms Reception timeout in milliseconds
 * @return sx1262_rx_config_t Default RX configuration structure
 */
sx1262_rx_config_t sx1262_get_default_rx_config(uint32_t timeout_ms);

/**
 * @brief Initialize the SX1262 LoRa transceiver with custom configuration.
 *
 * This function configures the SX1262 chip for LoRa operation, including
 * setting up the radio parameters and hardware interface. It must be called
 * before any other SX1262 LoRa operations.
 *
 * @param[in] config Pointer to initialization configuration structure
 * @return SX126X_STATUS_OK on success, or an appropriate error code from sx126x_status_t on failure.
 */
sx126x_status_t sx1262_init_lora(const sx1262_init_config_t* config);

/**
 * @brief Send a LoRa packet with custom configuration
 * 
 * @param[in] payload Pointer to payload data
 * @param[in] payload_length Length of payload in bytes
 * @param[in] config Pointer to TX configuration structure
 * @return sx126x_status_t Status of the operation
 */
sx126x_status_t sx1262_send_packet(uint8_t* payload, uint8_t payload_length, const sx1262_tx_config_t* config);

/**
 * @brief Receive a LoRa packet with custom configuration
 * 
 * @param[out] payload Buffer to store received payload
 * @param[in] payload_length Maximum length of payload buffer
 * @param[out] pkt_metrics Pointer to structure to store packet metrics
 * @param[in] config Pointer to RX configuration structure
 * @return sx126x_status_t Status of the operation
 */
sx126x_status_t sx1262_receive_packet(uint8_t* payload, uint8_t payload_length, lora_packet_metrics_t* pkt_metrics, const sx1262_rx_config_t* config);

esp_err_t control_external_LED(bool state);

/**
 * @brief Get the instantaneous RSSI value
 * 
 * @param[out] rssi_dbm Pointer to store the RSSI value in dBm
 * 
 * @note This function must be called while the radio is in RX mode
 * 
 * @returns SX126X_STATUS_OK on success, error code otherwise
 */
sx126x_status_t sx1262_get_rssi_instant(int16_t* rssi_dbm);

/**
 * @brief Set the radio into continuous RX mode
 * 
 * @details In continuous RX mode, the radio remains in receive mode indefinitely
 * until explicitly commanded to change modes. Useful for noise measurements
 * or continuous monitoring.
 * 
 * @returns SX126X_STATUS_OK on success, error code otherwise
 */
sx126x_status_t sx1262_set_continuous_rx(void);

/* Thread-safe LoRa message access functions */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Thread-safe function to get the current LoRa message
 * 
 * @param buffer Buffer to copy the message into
 * @param buffer_size Size of the buffer
 * @param timeout_ms Timeout in milliseconds to wait for mutex
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t lora_message_get_safe(char* buffer, size_t buffer_size, uint32_t timeout_ms);

/**
 * @brief Thread-safe function to set the LoRa message
 * 
 * @param message New message to set
 * @param timeout_ms Timeout in milliseconds to wait for mutex
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t lora_message_set_safe(const char* message, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // SX1262_INTERFACE_H
