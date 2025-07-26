#ifndef HELTECV3_HAL_H
#define HELTECV3_HAL_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>

/* Heltec V3 is developlment board with SX1262 LoRa module and a ESP32S3 MCU: https://heltec.org/project/wifi-lora-32-v3/
 * This HAL provides an interface to interact with the board's hardware without worrying about the underlying implementation details.
 *
 * Datasheet:        https://resource.heltec.cn/download/WiFi_LoRa_32_V3/HTIT-WB32LA_V3.2.pdf
 * Schematic (v3.1): https://resource.heltec.cn/download/WiFi_LoRa_32_V3/HTIT-WB32LA(F)_V3.1_Schematic_Diagram.pdf
 * User manual:      https://docs.heltec.org/en/node/esp32/wifi_lora_32/index.html
 * More resources:   https://resource.heltec.cn/download/WiFi_LoRa_32_V3/
 */

/* Heltec WiFi LoRa 32 V3.1 pin definitions based on the schematic (see link above) */
#define SX126X_MOSI_PIN     10
#define SX126X_MISO_PIN     11
#define SX126X_SCK_PIN      9
#define SX126X_NSS_PIN      8
#define SX126X_RST_PIN      12
#define SX126X_DIO1_PIN     14
#define SX126X_BUSY_PIN     13
#define HELTEC_V3_PIN_37    (gpio_num_t)37

/* Semaphore for signaling DIO1 interrupts */
extern SemaphoreHandle_t dio1_sem;


#endif // HELTECV3_HAL_H
