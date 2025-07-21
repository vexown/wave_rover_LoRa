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


extern SemaphoreHandle_t dio1_sem;

esp_err_t oled_err = ESP_FAIL; 

static const char* TAG = "SX1262_TEST";

static const void* context = NULL; // Context is not used in the HAL, so it can remain NULL.

extern "C"
{
    void app_main(void);
}

// Simple test function to demonstrate SX1262 basic functionality
void sx1262_basic_test()
{
    sx126x_status_t status;

    ESP_LOGI(TAG, "Initializing SX1262...");
    status = sx1262_init_lora();

    ESP_LOGI(TAG, "Configuring LoRa packet parameters...");
    const char* test_message = "Hello from SX1262!";
    uint8_t payload_len = strlen(test_message);

    sx126x_pkt_params_lora_t lora_pkt_params = {
        .preamble_len_in_symb = 8,
        .header_type = SX126X_LORA_PKT_EXPLICIT,
        .pld_len_in_bytes = payload_len,
        .crc_is_on = true,
        .invert_iq_is_on = false
    };
    status = sx126x_set_lora_pkt_params(context, &lora_pkt_params);
    if (status != SX126X_STATUS_OK) {
        ESP_LOGE(TAG, "Set LoRa packet params failed: %d", status);
        return;
    }



    // Main loop for sending packets
    int packet_count = 0;
    while (1) {
        char message[50];
        sprintf(message, "Hello from SX1262! Count: %d", ++packet_count);
        payload_len = strlen(message);

        ESP_LOGI(TAG, "Sending packet #%d: \"%s\"", packet_count, message);

        // Update payload length in packet params
        lora_pkt_params.pld_len_in_bytes = payload_len;
        status = sx126x_set_lora_pkt_params(context, &lora_pkt_params);
         if (status != SX126X_STATUS_OK) {
            ESP_LOGE(TAG, "Set LoRa packet params failed: %d", status);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // Write payload to buffer
        status = sx126x_write_buffer(context, 0x00, (const uint8_t*)message, payload_len);
        if (status != SX126X_STATUS_OK) {
            ESP_LOGE(TAG, "Write buffer failed: %d", status);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // Clear any old IRQs
        status = sx126x_clear_irq_status(context, SX126X_IRQ_ALL);
        if (status != SX126X_STATUS_OK) {
            ESP_LOGE(TAG, "Clear IRQ failed: %d", status);
        }

        // Start transmission
        ESP_LOGI(TAG, "Starting transmission...");
        status = sx126x_set_tx(context, 0); // Timeout 0 = no timeout
        if (status != SX126X_STATUS_OK) {
            ESP_LOGE(TAG, "Set TX failed: %d", status);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // Wait for TX Done or Timeout
        bool tx_completed = false;
        if (xSemaphoreTake(dio1_sem, pdMS_TO_TICKS(5000)) == pdTRUE) {
            sx126x_irq_mask_t irq_status;
            status = sx126x_get_and_clear_irq_status(context, &irq_status);
            if (status == SX126X_STATUS_OK) {
                if (irq_status & SX126X_IRQ_TX_DONE) {
                    ESP_LOGI(TAG, "✅ Transmission completed successfully!");
                    tx_completed = true;
                }
                if (irq_status & SX126X_IRQ_TIMEOUT) {
                    ESP_LOGW(TAG, "⚠️ Transmission timeout IRQ!");
                    tx_completed = true; // Treat as completed to exit loop
                }
            }
        }

        if (!tx_completed) {
            ESP_LOGW(TAG, "No TX completion IRQ detected after 5 seconds. Checking for radio errors...");
            
            sx126x_errors_mask_t errors;
            status = sx126x_get_device_errors(context, &errors);
            if (status == SX126X_STATUS_OK) {
                if (errors != 0) {
                    ESP_LOGE(TAG, "Radio reported errors: 0x%04X", errors);
                    if (errors & SX126X_ERRORS_PA_RAMP) ESP_LOGE(TAG, " - PA Ramping failed");
                    if (errors & SX126X_ERRORS_PLL_LOCK) ESP_LOGE(TAG, " - PLL lock failed");
                    if (errors & SX126X_ERRORS_XOSC_START) ESP_LOGE(TAG, " - XOSC start failed");
                    if (errors & SX126X_ERRORS_IMG_CALIBRATION) ESP_LOGE(TAG, " - Image calibration failed");
                    // Clear the errors to not see them again on the next loop
                    sx126x_clear_device_errors(context);
                } else {
                    ESP_LOGI(TAG, "No errors reported by the radio.");
                }
            } else {
                ESP_LOGE(TAG, "Failed to get device errors.");
            }

            ESP_LOGW(TAG, "Resetting radio.");
            sx126x_reset(context); // Attempt to recover
        }

        // Return to standby after transmission
        sx126x_set_standby(context, SX126X_STANDBY_CFG_RC);

        // --- Reception ---
        ESP_LOGI(TAG, "Switching to receive mode...");
        status = sx126x_set_rx(context, 5000); // 5 second RX timeout
        if (status != SX126X_STATUS_OK) {
            ESP_LOGE(TAG, "Set RX failed: %d", status);
        } else {
            // Wait for RX Done or Timeout
            bool rx_completed = false;
            if (xSemaphoreTake(dio1_sem, pdMS_TO_TICKS(5000)) == pdTRUE) {
                sx126x_irq_mask_t irq_status;
                status = sx126x_get_and_clear_irq_status(context, &irq_status);
                if (status == SX126X_STATUS_OK) {
                    if (irq_status & SX126X_IRQ_RX_DONE) {
                        ESP_LOGI(TAG, "✅ Reception completed successfully!");
                        rx_completed = true;
                        // Read received buffer status
                        sx126x_rx_buffer_status_t rx_status;
                        status = sx126x_get_rx_buffer_status(context, &rx_status);
                        if (status == SX126X_STATUS_OK && rx_status.pld_len_in_bytes > 0) {
                            uint8_t rx_buf[256] = {0};
                            status = sx126x_read_buffer(context, rx_status.buffer_start_pointer, rx_buf, rx_status.pld_len_in_bytes);
                            if (status == SX126X_STATUS_OK) {
                                ESP_LOGI(TAG, "Received: %.*s", rx_status.pld_len_in_bytes, rx_buf);
                            }
                        }
                    }
                    if (irq_status & SX126X_IRQ_TIMEOUT) {
                        ESP_LOGW(TAG, "⚠️ Reception timeout IRQ!");
                        rx_completed = true;
                    }
                }
            }
            if (!rx_completed) {
                ESP_LOGW(TAG, "No RX completion IRQ detected after 5 seconds.");
            }
        }

        // Return to standby after reception
        sx126x_set_standby(context, SX126X_STANDBY_CFG_RC);

        ESP_LOGI(TAG, "Waiting 5 seconds before next transmission...");
        vTaskDelay(pdMS_TO_TICKS(5000));
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

    // Wait a bit for system to stabilize
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Run SX1262 test which now loops forever sending packets
    sx1262_basic_test();

    // This part will not be reached
    ESP_LOGI(TAG, "Application finished.");
}
