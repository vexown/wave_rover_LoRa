#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "sx126x.h"
#include <string.h>
#include "sx1262_interface.hpp"

extern SemaphoreHandle_t dio1_sem;

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

        ESP_LOGI(TAG, "Waiting 5 seconds before next transmission...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Wave Rover LoRa application...");

    // Wait a bit for system to stabilize
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Run SX1262 test which now loops forever sending packets
    sx1262_basic_test();

    // This part will not be reached
    ESP_LOGI(TAG, "Application finished.");
}
