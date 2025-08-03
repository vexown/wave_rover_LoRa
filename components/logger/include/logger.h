#ifndef LOGGER_H
#define LOGGER_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send LoRa data to the Firebase Realtime Database
 * 
 * @param rssi Signal strength in dBm
 * @param snr Signal-to-noise ratio
 * @param packet_data The packet data as a string
 * @param frequency LoRa frequency in Hz
 * @param spreading_factor LoRa spreading factor
 */
void send_lora_to_firebase(int rssi, float snr, const char* packet_data, uint32_t frequency, int spreading_factor);

#ifdef __cplusplus
}
#endif

#endif // LOGGER_H
