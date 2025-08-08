#include "esp_http_client.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <time.h>
#include "esp_crt_bundle.h"
#include "esp_sntp.h"

static const char *TAG = "FIREBASE_CLIENT";

#define FIREBASE_URL "https://waveroverlogger-default-rtdb.europe-west1.firebasedatabase.app/lora_packets.json"

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) 
    {
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "Firebase response: %.*s", evt->data_len, (char*)evt->data);
            break;
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "HTTP_EVENT_ERROR");
            break;
        default:
            break;
    }
    return ESP_OK;
}

void send_lora_to_firebase(int rssi, float snr, const char* packet_data, uint32_t frequency, int spreading_factor)
{
    /* #01 - Create JSON payload */
    cJSON *json = cJSON_CreateObject();

    /* #02 - Add timestamp (make sure you initialized SNTP first in your WiFi component after connecting) */
    time_t now;
    time(&now);
    cJSON_AddNumberToObject(json, "timestamp", now);

    /* #03 - Add LoRa data */
    cJSON_AddNumberToObject(json, "rssi", rssi);
    cJSON_AddNumberToObject(json, "snr", snr);
    cJSON_AddStringToObject(json, "packet", packet_data);
    cJSON_AddNumberToObject(json, "frequency", frequency);
    cJSON_AddNumberToObject(json, "spreading_factor", spreading_factor);

    /* #04 - Add device info (optional) */
    cJSON_AddStringToObject(json, "device_id", "WaveRoverLoRa");

    /* #05 - Convert JSON object to string */
    char *json_string = cJSON_Print(json);

    /* #06 - Define HTTP client configuration for Firebase */
    esp_http_client_config_t config = 
    {
        .url = FIREBASE_URL,
        .event_handler = _http_event_handler,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    /* #07 - Apply the config and initialize HTTP client */
    esp_http_client_handle_t client = esp_http_client_init(&config);

    /* #08 - Set headers and POST data */
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_string, strlen(json_string));

    /* #09 - Send HTTP POST request to Firebase */
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) 
    {
        int status_code = esp_http_client_get_status_code(client);
        if (status_code == 200) // 200 OK indicates success
        {
            ESP_LOGI(TAG, "✓ Data sent to Firebase successfully");
        } 
        else 
        {
            ESP_LOGW(TAG, "Firebase returned status: %d", status_code);
        }
    } 
    else 
    {
        ESP_LOGE(TAG, "Firebase request failed: %s", esp_err_to_name(err));
    }

    /* #10 - Cleanup resources */
    esp_http_client_cleanup(client);
    free(json_string);
    cJSON_Delete(json);
}
