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

esp_err_t oled_err = ESP_FAIL; 

static const char* TAG = "SX1262_TEST";

extern "C"
{
    void app_main(void);
}

void sx1262_basic_test()
{
    sx126x_status_t status;

    ESP_LOGI(TAG, "Initializing SX1262...");
    status = sx1262_init_lora();

    while (1) 
    {
        char message[] = "Hello from SX1262!";
        uint16_t payload_len = strlen(message);
        status = sx1262_send_packet((uint8_t*)message, payload_len);
        if (status != SX126X_STATUS_OK) 
        {
            ESP_LOGE(TAG, "Failed to send packet: %d", status);
        } 

        uint8_t rx_payload[256] = {}; // Buffer for received data
        uint16_t rx_payload_len = sizeof(rx_payload);
        status = sx1262_receive_packet(rx_payload, rx_payload_len, 5000);
        if (status != SX126X_STATUS_OK) 
        {
            ESP_LOGE(TAG, "Failed to receive packet: %d", status);
        } 
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Wave Rover LoRa application...");

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
    
    sx1262_basic_test(); // Run SX1262 test which now loops forever sending packets

    ESP_LOGI(TAG, "Application finished."); // This part will not be reached
}
