#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "sx126x.h"
#include <string.h>

extern SemaphoreHandle_t dio1_sem;

static const char* TAG = "SX1262_TEST";

extern "C"
{
    void app_main(void);
}

// Simple test function to demonstrate SX1262 basic functionality
void sx1262_basic_test()
{
    ESP_LOGI(TAG, "Starting SX1262 basic test...");

    // The context pointer is not used in your HAL, so it can remain NULL.
    const void* context = NULL;
    sx126x_status_t status;

    // The HAL will be initialized automatically by the first call to a HAL function (e.g., sx126x_reset).
    // The following initialization sequence is based on the Semtech driver recommendations.

    // Step 1: Reset and wake up the radio
    ESP_LOGI(TAG, "Resetting SX1262...");
    status = sx126x_reset(context);
    if (status != SX126X_STATUS_OK) {
        ESP_LOGE(TAG, "Reset failed: %d", status);
        return;
    }
    // A small delay after reset is good practice.
    vTaskDelay(pdMS_TO_TICKS(20));

    // The wakeup command is implicitly handled by the HAL during the first SPI transaction after reset.
    // An explicit wakeup call is good for clarity and ensures the chip is ready.
    status = sx126x_wakeup(context);
     if (status != SX126X_STATUS_OK) {
        ESP_LOGE(TAG, "Wakeup failed: %d", status);
        return;
    }

    // Step 2: Set to standby mode
    ESP_LOGI(TAG, "Setting standby mode...");
    status = sx126x_set_standby(context, SX126X_STANDBY_CFG_RC);
    if (status != SX126X_STATUS_OK) {
        ESP_LOGE(TAG, "Set standby failed: %d", status);
        return;
    }

    // Step 3: Configure regulator and RF switch
    ESP_LOGI(TAG, "Configuring regulator...");
    status = sx126x_set_reg_mode(context, SX126X_REG_MODE_DCDC);
    if (status != SX126X_STATUS_OK) {
        ESP_LOGE(TAG, "Set regulator failed: %d", status);
        return;
    }

    // Step 3.1: Configure and enable the TCXO
    ESP_LOGI(TAG, "Configuring TCXO...");
    // The timeout is given in RTC steps. 1 step = 15.625 µs.
    // 10ms timeout = 10000 / 15.625 = 640 steps.
    status = sx126x_set_dio3_as_tcxo_ctrl(context, SX126X_TCXO_CTRL_1_8V, 640);
    if (status != SX126X_STATUS_OK) {
        ESP_LOGE(TAG, "Set TCXO control failed: %d", status);
        return;
    }

    ESP_LOGI(TAG, "Configuring DIO2 as RF switch...");
    status = sx126x_set_dio2_as_rf_sw_ctrl(context, true);
    if (status != SX126X_STATUS_OK) {
        ESP_LOGE(TAG, "Set DIO2 RF switch failed: %d", status);
        return;
    }

    // Step 4: Set packet type to LoRa
    ESP_LOGI(TAG, "Setting LoRa packet type...");
    status = sx126x_set_pkt_type(context, SX126X_PKT_TYPE_LORA);
    if (status != SX126X_STATUS_OK) {
        ESP_LOGE(TAG, "Set packet type failed: %d", status);
        return;
    }

    // Step 5: Set RF frequency
    ESP_LOGI(TAG, "Setting RF frequency to 868 MHz...");
    uint32_t freq_in_hz = 868000000;
    status = sx126x_set_rf_freq(context, freq_in_hz);
    if (status != SX126X_STATUS_OK) {
        ESP_LOGE(TAG, "Set RF frequency failed: %d", status);
        return;
    }

    // Step 6: Configure PA and TX power for +14 dBm (a robust choice)
    ESP_LOGI(TAG, "Configuring PA for +14 dBm...");
    sx126x_pa_cfg_params_t pa_config = {
        .pa_duty_cycle = 0x02, // Recommended value for +14 dBm
        .hp_max = 0x02,        // Recommended value for +14 dBm
        .device_sel = 0x00,    // 0x00 for SX1262
        .pa_lut = 0x01
    };
    status = sx126x_set_pa_cfg(context, &pa_config);
    if (status != SX126X_STATUS_OK) {
        ESP_LOGE(TAG, "Set PA config failed: %d", status);
        return;
    }

    ESP_LOGI(TAG, "Setting TX power to +14 dBm...");
    status = sx126x_set_tx_params(context, 14, SX126X_RAMP_200_US);
    if (status != SX126X_STATUS_OK) {
        ESP_LOGE(TAG, "Set TX params failed: %d", status);
        return;
    }

    // Step 7: Configure LoRa modulation parameters
    ESP_LOGI(TAG, "Configuring LoRa modulation...");
    sx126x_mod_params_lora_t lora_mod_params = {
        .sf = SX126X_LORA_SF7,
        .bw = SX126X_LORA_BW_125,
        .cr = SX126X_LORA_CR_4_5,
        .ldro = 0
    };
    status = sx126x_set_lora_mod_params(context, &lora_mod_params);
    if (status != SX126X_STATUS_OK) {
        ESP_LOGE(TAG, "Set LoRa modulation failed: %d", status);
        return;
    }

    // Step 8: Configure LoRa packet parameters
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

    // Step 9: Configure DIO1 for TX done interrupt
    ESP_LOGI(TAG, "Configuring IRQ for TX Done...");
    status = sx126x_set_dio_irq_params(context, SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT, SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT, 0, 0);
    if (status != SX126X_STATUS_OK) {
        ESP_LOGE(TAG, "Set DIO IRQ params failed: %d", status);
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
