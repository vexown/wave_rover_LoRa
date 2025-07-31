#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "sx126x.h"
#include <string.h>
#include "sx1262_interface.hpp"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "i2c_manager.h"
#include "oled_display.h"
#include "web_updater_wifi.h"
#include "web_updater.h"

#define RX_TIMEOUT_MS 5000

/* Device modes */
#define RECEIVER_MODE 0
#define TRANSMITTER_MODE 1
#define TRANSCEIVER_MODE 2

/* Select the mode of operation (defined in platformio.ini) */
static uint8_t device_mode = DEVICE_MODE;

esp_err_t oled_err = ESP_FAIL; 

static const char* TAG = "SX1262_TEST";

extern "C"
{
    void app_main(void);
}

static void transmitterMode(void);
static void receiverMode(void);
static void transceiverMode(void);

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Wave Rover LoRa application...");

    const char *wifi_ssid = "StatekMatka_V2";
    const char *wifi_pass = "TODO"; //TODO, add real password but load it from NVS 
    web_updater_wifi_start(wifi_ssid, wifi_pass);

    ESP_LOGI(TAG, "Initializing I2C Manager...");
    esp_err_t i2c_err = i2c_manager_init(I2C_MANAGER_DEFAULT_PORT, I2C_MANAGER_DEFAULT_SDA, I2C_MANAGER_DEFAULT_SCL);
    if (i2c_err != ESP_OK)
    {
        ESP_LOGI(TAG, "I2C Manager initialization failed: %s", esp_err_to_name(i2c_err)); // Log the error but continue execution (TODO: Decide how to handle failure)
    }
    else
    {
        ESP_LOGI(TAG, "I2C Manager Initialized.");
    }

    ESP_LOGI(TAG, "Initializing OLED Display...");
    oled_err = oled_init(); // oled_err is global, so it's updated directly
    if (oled_err != ESP_OK)
    {
        ESP_LOGI(TAG, "OLED initialization failed: %s", esp_err_to_name(oled_err)); // Log the error but continue execution (TODO: Decide how to handle failure)
    }
    else
    {
        ESP_LOGI(TAG, "OLED Display Initialized.");
    }

    if (oled_err == ESP_OK)
    {
        /* Display welcome message on the OLED display */
        oled_clear_buffer();
        oled_write_string(0, "Wave Rover LoRa");
        oled_write_string(1, "                ");
        oled_write_string(2, "    (o-o)    ");
        oled_write_string(3, "   /  |  \\   ");
        oled_write_string(4, "  /_______\\  ");
        oled_write_string(5, "  o       o  ");
        oled_write_string(6, "  ~~~~~~~~~  ");
        oled_write_string(7, " ~         ~ ");
        oled_refresh();
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a bit for system to stabilize
    
    if(device_mode == TRANSMITTER_MODE)
    {
        ESP_LOGI(TAG, "Running in Transmitter Mode...");
        transmitterMode();
    }
    else if(device_mode == RECEIVER_MODE)
    {
        ESP_LOGI(TAG, "Running in Receiver Mode...");
        receiverMode();
    }
    else if(device_mode == TRANSCEIVER_MODE)
    {
        ESP_LOGI(TAG, "Running in Transceiver Mode...");
        transceiverMode();
    }
    else
    {
        ESP_LOGE(TAG, "Invalid device mode selected!");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000)); // Forever block execution if invalid mode (TODO - handle it more gracefully)
    }

    ESP_LOGI(TAG, "Application finished."); // This part will not be reached
}

static void transceiverMode()
{
    sx126x_status_t status;

    ESP_LOGI(TAG, "Initializing SX1262...");
    status = sx1262_init_lora();
    if (status != SX126X_STATUS_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SX1262: %d", status);
        esp_restart();
    }

    char message[] = "Yo from SX1262#1";
    uint16_t payload_len = strlen(message);
    uint8_t rx_payload[256] = {}; // Buffer for received data
    uint16_t rx_payload_len = sizeof(rx_payload);

    while (1) 
    {
        status = sx1262_send_packet((uint8_t*)message, payload_len);
        if (status != SX126X_STATUS_OK) 
        {
            ESP_LOGE(TAG, "Failed to send packet: %d", status);
        } 
        else
        {
            oled_clear_buffer();
            oled_write_string(0, "Sent:");
            oled_write_string_multiline(1, message);
            oled_refresh();
        }

        status = sx1262_receive_packet(rx_payload, rx_payload_len, 5000);
        if (status != SX126X_STATUS_OK) 
        {
            (void)control_external_LED(false);
            oled_clear_buffer();
            oled_write_string_multiline(0, "Failed to receive a packet or nothing to receive :( ");
            oled_refresh();
        } 
        else
        {
            (void)control_external_LED(true);
            oled_clear_buffer();
            oled_write_string(0, "Received:");
            oled_write_string_multiline(1, (char*)rx_payload);
            oled_refresh();
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds before next iteration
    }
}

static void transmitterMode()
{
    sx126x_status_t status;

    ESP_LOGI(TAG, "Initializing SX1262...");
    status = sx1262_init_lora();
    if (status != SX126X_STATUS_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SX1262: %d", status);
        esp_restart();
    }

    char message[] = "Yo from SX1262#1";
    uint16_t payload_len = strlen(message);

    while (1) 
    {
        status = sx1262_send_packet((uint8_t*)message, payload_len);
        if (status != SX126X_STATUS_OK) 
        {
            (void)control_external_LED(false);
            oled_clear_buffer();
            oled_write_string_multiline(0, "Failed to send packet :( ");
            oled_refresh();
        } 
        else
        {
            (void)control_external_LED(true);
            oled_clear_buffer();
            oled_write_string(0, "Sent:");
            oled_write_string_multiline(1, message);
            oled_refresh();
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds between transmissions
    }
}

static void receiverMode()
{
    sx126x_status_t status;

    ESP_LOGI(TAG, "Initializing SX1262...");
    status = sx1262_init_lora();
    if (status != SX126X_STATUS_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SX1262: %d", status);
        esp_restart();
    }

    uint8_t rx_payload[256] = {};
    uint16_t rx_payload_len = sizeof(rx_payload);

    while (1) 
    {
        status = sx1262_receive_packet(rx_payload, rx_payload_len, RX_TIMEOUT_MS);
        if (status != SX126X_STATUS_OK) 
        {
            (void)control_external_LED(false);
            oled_clear_buffer();
            oled_write_string_multiline(0, "Failed to receive a packet or nothing to receive :( ");
            oled_refresh();
        } 
        else
        {
            (void)control_external_LED(true);
            oled_clear_buffer();
            oled_write_string(0, "Received:");
            oled_write_string_multiline(1, (char*)rx_payload);
            oled_refresh();
        }
    }
}