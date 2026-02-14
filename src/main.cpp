#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_timer.h"
#include <string.h>
#include "sx1262_interface.hpp"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "i2c_manager.h"
#include "oled_display.h"
#include "nvs_flash.h"
#include "web_updater_wifi.h"
#include "web_updater.h"
#include "persistent_params.h"
#include "logger.h"
#include "web_terminal.h"

/* Misc defines */
#define RX_TIMEOUT_MS 5000

typedef enum 
{
    INIT_SUCCESS = 0,
    NVS_INIT_FAILED,
    PARAM_FAILED,
    WIFI_INIT_FAILED,
    I2C_INIT_FAILED,
    OLED_INIT_FAILED,
    SX1262_INIT_FAILED,
    WEB_TERMINAL_INIT_FAILED
} init_status_t;

char LoRaMessageGlobal[MAX_LORA_PAYLOAD_LENGTH] = "HELLOLORA"; // Default message to send

/* Mutex for protecting LoRaMessageGlobal access */
SemaphoreHandle_t lora_message_mutex = NULL;

/* Select the mode of operation (stored in NVS, use param_update env in platformio.ini to change it) */
static int32_t device_mode;

esp_err_t oled_err = ESP_FAIL; 

static const char* TAG = "SX1262_TEST";

extern "C"
{
    void app_main(void);
}

static init_status_t init_components(void);
static void transmitterMode(void);
static void receiverMode(void);
static void transceiverMode(void);

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Wave Rover LoRa application...");

    init_status_t init_status = init_components();
    if (init_status != INIT_SUCCESS)
    {
        ESP_LOGE(TAG, "Initialization failed with status: %d", init_status);
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait a bit before restarting
        esp_restart(); // Restart the system on initialization failure
    }
    else
    {
        ESP_LOGI(TAG, "All components initialized successfully.");
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
    { // if invalid mode selected, go to receiver mode
        ESP_LOGE(TAG, "Invalid device mode selected! Changing to Receiver Mode...");
        device_mode = RECEIVER_MODE;
        receiverMode();
    }
}

static void transceiverMode()
{
    sx126x_status_t status;
    char message_buffer[MAX_LORA_PAYLOAD_LENGTH];
    uint8_t payload_len;
    uint8_t rx_payload[MAX_LORA_PAYLOAD_LENGTH] = {}; // Buffer for received data
    uint8_t rx_payload_len = sizeof(rx_payload);
    lora_packet_metrics_t pkt_metrics = {};
    char metrics_str[64] = {};
    
    /* Get default TX and RX configurations */
    sx1262_tx_config_t tx_config = sx1262_get_default_tx_config();
    sx1262_rx_config_t rx_config = sx1262_get_default_rx_config(RX_TIMEOUT_MS);

    while (1) 
    {
        // Get message and send packet (thread-safe)
        if (lora_message_get_safe(message_buffer, sizeof(message_buffer), 1000) == ESP_OK)
        {
            payload_len = strlen(message_buffer);
            status = sx1262_send_packet((uint8_t*)message_buffer, payload_len, &tx_config);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to get LoRa message in transceiverMode");
            status = SX126X_STATUS_ERROR;
        }
        
        if (status != SX126X_STATUS_OK) 
        {
            ESP_LOGE(TAG, "Failed to send packet: %d", status);
        } 
        else
        {
            oled_clear_buffer();
            oled_write_string(0, "Sent:");
            oled_write_string_multiline(1, message_buffer);
            oled_refresh();
        }

        status = sx1262_receive_packet(rx_payload, rx_payload_len, &pkt_metrics, &rx_config);
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
            oled_write_string(0, "Packet Received!");
            snprintf(metrics_str, sizeof(metrics_str), "SNR: %d dB  RSSI: %d dBm RSSI[sig]: %d dBm", 
                                                        pkt_metrics.snr_db, pkt_metrics.rssi_dbm, pkt_metrics.signal_rssi_dbm);
            oled_write_string_multiline(1, metrics_str);
            oled_refresh();
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds before next iteration
    }
}

static void transmitterMode()
{
    sx126x_status_t status;
    char message_buffer[MAX_LORA_PAYLOAD_LENGTH];
    uint8_t payload_len;
    
    sx1262_tx_config_t tx_config = sx1262_get_default_tx_config();
    tx_config.sync_word = 0x7F;
    tx_config.crc_enabled = true;

    while (1) 
    {
        // Get message and send packet (thread-safe)
        if (lora_message_get_safe(message_buffer, sizeof(message_buffer), 1000) == ESP_OK)
        {
            payload_len = strlen(message_buffer);
            status = sx1262_send_packet((uint8_t*)message_buffer, payload_len, &tx_config);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to get LoRa message in transmitterMode");
            status = SX126X_STATUS_ERROR;
        }
        
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
            oled_write_string_multiline(1, message_buffer);
            oled_refresh();
        }

        vTaskDelay(pdMS_TO_TICKS(4000)); // Wait 4 seconds between transmissions
    }
}

static void receiverMode()
{
    sx126x_status_t status;

#ifdef NOISE_LEVEL_MEASUREMENT
    int16_t rssi_instant_dbm = 0;
    ESP_LOGI(TAG, "NOISE LEVEL MEASUREMENT MODE");
    status = sx1262_set_continuous_rx();
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "CRITICAL: Failed to set radio to continuous RX mode: %d", status);
        vTaskDelete(NULL);
        return;
    }
    while (1) 
    {
        /* Get instantaneous RSSI (noise floor when no signal present) */
        status = sx1262_get_rssi_instant(&rssi_instant_dbm);
        if (status != SX126X_STATUS_OK) ESP_LOGE(TAG, "Failed to read RSSI: %d", status);
        vTaskDelay(pdMS_TO_TICKS(1000)); // measure noise level every second
    }
#else // normal receiver mode with packet reception

    uint8_t rx_payload[MAX_LORA_PAYLOAD_LENGTH] = {};
    uint8_t rx_payload_len = sizeof(rx_payload);
    lora_packet_metrics_t pkt_metrics = {};
    char metrics_str[64] = {};
    
    sx1262_rx_config_t rx_config = sx1262_get_default_rx_config(RX_TIMEOUT_MS);

    while (1) 
    {
        status = sx1262_receive_packet(rx_payload, rx_payload_len, &pkt_metrics, &rx_config);
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
            oled_write_string(0, "Packet Received!");
            snprintf(metrics_str, sizeof(metrics_str), "SNR: %d dB", pkt_metrics.snr_db);
            oled_write_string(1, metrics_str);
            snprintf(metrics_str, sizeof(metrics_str), "RSSI: %d dBm", pkt_metrics.rssi_dbm);
            oled_write_string(2, metrics_str);
            snprintf(metrics_str, sizeof(metrics_str), "RSSI[sig]: %d dBm", pkt_metrics.signal_rssi_dbm);
            oled_write_string(3, metrics_str);
            oled_refresh();

            send_lora_to_firebase(pkt_metrics.rssi_dbm, pkt_metrics.snr_db, (const char*)rx_payload, 0, 0);
        }
    }
#endif
}

static init_status_t init_components(void)
{
    /* ----------------- #00 - Mutex Initialization ----------------- */
    lora_message_mutex = xSemaphoreCreateMutex();
    if (lora_message_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create LoRa message mutex");
        return INIT_SUCCESS; // We'll treat this as non-fatal for now
    }
    ESP_LOGI(TAG, "LoRa message mutex created successfully");

    /* ----------------- #01 - NVS Initialization ----------------- */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_LOGW(TAG, "NVS Flash initialization failed, erasing and reinitializing...");
        /* Erase NVS partition and try to initialize again */
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
        if(ret != ESP_OK)
        {
            ESP_LOGE(TAG, "NVS Flash initialization failed: %s", esp_err_to_name(ret));
            return NVS_INIT_FAILED;
        }
    }

    /* ----------------- #02 - Params Initialization ----------------- */
    esp_err_t param_status;

    /* Param #01 - Device Mode */
    param_status = persistent_params_init_integer("device_mode", DEVICE_MODE_DEFAULT);
    if (param_status != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to add Device Mode parameter: %s", esp_err_to_name(param_status));
        return PARAM_FAILED;
    }
#if defined(UPDATE_DEVICE_MODE_PARAM) || defined(UPDATE_ALL_PARAMS)
    param_status = persistent_params_set_integer("device_mode", DEVICE_MODE);
    if (param_status != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to set Device Mode parameter: %s", esp_err_to_name(param_status));
        return PARAM_FAILED;
    }
#endif
    param_status = persistent_params_get_integer("device_mode", &device_mode);
    if (param_status != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to get Device Mode parameter: %s", esp_err_to_name(param_status));
        return PARAM_FAILED;
    }

    /* Param #2 - WiFi Credentials */
    param_status = persistent_params_init_string("wifi_ssid", WIFI_SSID_DEFAULT);
    if (param_status != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to add WiFi SSID parameter: %s", esp_err_to_name(param_status));
        return PARAM_FAILED;
    }
    param_status = persistent_params_init_string("wifi_password", WIFI_PASSWORD_DEFAULT);
    if (param_status != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to add WiFi Password parameter: %s", esp_err_to_name(param_status));
        return PARAM_FAILED;
    }
#if defined(UPDATE_WIFI_CREDENTIALS_PARAM) || defined(UPDATE_ALL_PARAMS)
    param_status = persistent_params_set_string("wifi_ssid", WIFI_SSID);
    if (param_status != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to set WiFi SSID parameter: %s", esp_err_to_name(param_status));
        return PARAM_FAILED;
    }
    param_status = persistent_params_set_string("wifi_password", WIFI_PASSWORD);
    if (param_status != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to set WiFi Password parameter: %s", esp_err_to_name(param_status));
        return PARAM_FAILED;
    }
#endif
    char wifi_ssid[64] = {};
    char wifi_password[64] = {};
    param_status = persistent_params_get_string("wifi_ssid", wifi_ssid, sizeof(wifi_ssid));
    if (param_status != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to get WiFi SSID parameter: %s", esp_err_to_name(param_status));
        return PARAM_FAILED;
    }
    param_status = persistent_params_get_string("wifi_password", wifi_password, sizeof(wifi_password));
    if (param_status != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to get WiFi Password parameter: %s", esp_err_to_name(param_status));
        return PARAM_FAILED;
    }

    /* ----------------- #03 - WiFi Initialization ----------------- */
    bool wifi_status = web_updater_wifi_start(wifi_ssid, wifi_password);
    if (!wifi_status) 
    {
        ESP_LOGE(TAG, "WiFi initialization failed");
        return WIFI_INIT_FAILED;
    }
    else
    {
        ESP_LOGI(TAG, "WiFi initialized successfully.");
    }

    /* ----------------- #04 - I2C Manager Initialization ----------------- */
    ESP_LOGI(TAG, "Initializing I2C Manager...");
    esp_err_t i2c_err = i2c_manager_init(I2C_MANAGER_DEFAULT_PORT, I2C_MANAGER_DEFAULT_SDA, I2C_MANAGER_DEFAULT_SCL);
    if (i2c_err != ESP_OK)
    {
        ESP_LOGI(TAG, "I2C Manager initialization failed: %s", esp_err_to_name(i2c_err)); 
        return I2C_INIT_FAILED;
    }
    else
    {
        ESP_LOGI(TAG, "I2C Manager Initialized.");
    }

    /* ----------------- #05 - OLED Display Initialization ----------------- */
    ESP_LOGI(TAG, "Initializing OLED Display...");
    oled_err = oled_init(); // oled_err is global, so it's updated directly
    if (oled_err != ESP_OK)
    {
        ESP_LOGI(TAG, "OLED initialization failed: %s", esp_err_to_name(oled_err)); 
        return OLED_INIT_FAILED;
    }
    else
    {
        ESP_LOGI(TAG, "OLED Display Initialized.");

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

    /* ----------------- #06 - SX1262 Initialization ----------------- */
    ESP_LOGI(TAG, "Initializing SX1262...");
    
    /* Get default initialization configuration for ETSI Band O */
    sx1262_init_config_t sx1262_config = sx1262_get_default_init_config(
        FREQUENCY_ETSI_EN_300_220_BAND_O, 
        DUTY_CYCLE_LIMIT_ETSI_EN_300_220_BAND_O
    );
    sx1262_config.lora_mod_params.sf = SX126X_LORA_SF8;
    sx1262_config.lora_mod_params.bw = SX126X_LORA_BW_062;
    
    sx126x_status_t status = sx1262_init_lora(&sx1262_config);
    if (status != SX126X_STATUS_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SX1262: %d", status);
        return SX1262_INIT_FAILED;
    }
    else
    {
        ESP_LOGI(TAG, "SX1262 Initialized successfully.");
    }

    /* ----------------- #07 - Web Terminal Initialization ----------------- */
    ESP_LOGI(TAG, "Initializing Web Terminal...");
    esp_err_t web_terminal_status = web_terminal_start();
    if (web_terminal_status != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize Web Terminal: %s", esp_err_to_name(web_terminal_status));
        return WEB_TERMINAL_INIT_FAILED;
    }
    else
    {
        ESP_LOGI(TAG, "Web Terminal initialized successfully.");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a bit for web terminal to start
        web_terminal_send_message("Wave Rover LoRa Web Terminal started!");
    }

    return INIT_SUCCESS; /* All components initialized successfully */
}

/* Thread-safe LoRa message access functions */
extern "C" {

esp_err_t lora_message_get_safe(char* buffer, size_t buffer_size, uint32_t timeout_ms)
{
    if (buffer == NULL || buffer_size == 0 || lora_message_mutex == NULL)
    {
        return ESP_FAIL;
    }
    
    if (xSemaphoreTake(lora_message_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
    {
        strncpy(buffer, LoRaMessageGlobal, buffer_size - 1);
        buffer[buffer_size - 1] = '\0';
        xSemaphoreGive(lora_message_mutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

esp_err_t lora_message_set_safe(const char* message, uint32_t timeout_ms)
{
    if (message == NULL || lora_message_mutex == NULL)
    {
        return ESP_FAIL;
    }
    
    if (strlen(message) >= MAX_LORA_PAYLOAD_LENGTH)
    {
        return ESP_FAIL;
    }
    
    if (xSemaphoreTake(lora_message_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
    {
        strncpy(LoRaMessageGlobal, message, MAX_LORA_PAYLOAD_LENGTH);
        LoRaMessageGlobal[MAX_LORA_PAYLOAD_LENGTH - 1] = '\0';
        xSemaphoreGive(lora_message_mutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

}

