#include "web_updater_wifi.h"
#include <string.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_netif.h>
#include <mdns.h>
#include <esp_log.h>
#include "web_updater.h"

static const char *TAG = "web_updater_wifi";

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

void web_updater_wifi_start(const char *ssid, const char *password)
{
    // Initialize NVS (required for Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize networking stack and event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Initialize Wi-Fi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // --- Register the event handlers ---
    // This is the key change. We now listen for events.
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // Configure and start Wi-Fi
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start()); // This call is non-blocking

    ESP_LOGI(TAG, "WiFi STA configured. Waiting for connection and IP address...");
}

void start_mdns_service(void) 
{
    // Initialize mDNS
    ESP_ERROR_CHECK(mdns_init());
    
    // Set hostname
    ESP_ERROR_CHECK(mdns_hostname_set("esp32-ota"));
    
    // Set default instance
    ESP_ERROR_CHECK(mdns_instance_name_set("ESP32 OTA Web Updater"));
    
    // Announce the web server service
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);

    ESP_LOGI("mDNS", "mDNS service started for http://esp32-ota.local");
}

// Event handler for Wi-Fi and IP events
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) 
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI("wifi_event", "Station mode started, connecting to AP...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("wifi_event", "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        
        // ---- THIS IS THE CRITICAL PART ----
        // Now that we have an IP, we can start services that need it
        start_mdns_service();
        web_updater_start();
    }
}