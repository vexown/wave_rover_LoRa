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
#include "nvs_flash.h"
#include "web_updater_wifi.h"
#include "web_updater.h"
#include "persistent_params.h"

/* You can host a hotspot with these credentials if you don't want to provide your own */
#define WIFI_SSID_DEFAULT "kekwifi"
#define WIFI_PASSWORD_DEFAULT "kekpassword"
/* To update the NVS with your WiFi credentials, provide them in these macros
 * and use the param_update build environment in platformio.ini 
 * Then build and flash the firmware. Later, you can change the macro values
 * back to placeholders and go back to the normal env so you don't accidentally commit your credentials. */
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

#define RX_TIMEOUT_MS 5000

/* Device modes */
#define RECEIVER_MODE 0
#define TRANSMITTER_MODE 1
#define TRANSCEIVER_MODE 2

typedef enum 
{
    INIT_SUCCESS = 0,
    NVS_INIT_FAILED,
    PARAM_FAILED,
    WIFI_INIT_FAILED,
    I2C_INIT_FAILED,
    OLED_INIT_FAILED,
    SX1262_INIT_FAILED
} init_status_t;

/* Select the mode of operation (defined in platformio.ini) */
static uint8_t device_mode = DEVICE_MODE;

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
    {
        ESP_LOGE(TAG, "Invalid device mode selected!");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000)); // Forever block execution if invalid mode (TODO - handle it more gracefully)
    }
}

static void transceiverMode()
{
    sx126x_status_t status;

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

        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds before next iteration
    }
}

static void transmitterMode()
{
    sx126x_status_t status;

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

static init_status_t init_components(void)
{
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

    /* ----------------- #02 - WiFi Initialization ----------------- */
    esp_err_t param_status = persistent_params_init_string("wifi_ssid", WIFI_SSID_DEFAULT);
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
#ifdef UPDATE_PARAMS_IN_NVS
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

    /* ----------------- #03 - I2C Manager Initialization ----------------- */
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

    /* ----------------- #04 - OLED Display Initialization ----------------- */
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

    /* ----------------- #05 - SX1262 Initialization ----------------- */
    ESP_LOGI(TAG, "Initializing SX1262...");
    sx126x_status_t status = sx1262_init_lora();
    if (status != SX126X_STATUS_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SX1262: %d", status);
        return SX1262_INIT_FAILED;
    }
    else
    {
        ESP_LOGI(TAG, "SX1262 Initialized successfully.");
    }

    return INIT_SUCCESS; // All components initialized successfully
}