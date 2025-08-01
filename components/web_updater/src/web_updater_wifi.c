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

bool web_updater_wifi_start(const char *ssid, const char *password)
{
    /* Initialize networking stack and event loop */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Create default Wi-Fi station interface */
    esp_netif_t *esp_netif = esp_netif_create_default_wifi_sta();
    if (esp_netif == NULL) 
    {
        ESP_LOGE(TAG, "Failed to create default WiFi STA interface");
        return false;
    }

    /* Initialize Wi-Fi driver with default configuration */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Register event handlers with a common handler */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    /* Configure the credentials and start Wi-Fi */
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    
    /* Set Wi-Fi mode to station, apply the configuration, and start Wi-Fi */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start()); // This call is non-blocking

    ESP_LOGI(TAG, "WiFi STA configured. Waiting for connection and IP address...");
    /* Now the event handler will handle connection and IP events */

    return true; // Indicate that Wi-Fi setup was successful
}

void start_mdns_service(void) 
{
    /* Initialize mDNS */
    ESP_ERROR_CHECK(mdns_init());
    
    /* Set hostname */
    char hostname[] = "esp32-ota"; // TODO: Change to a unique name saved in NVS
    ESP_ERROR_CHECK(mdns_hostname_set(hostname));

    /* Set default instance */
    ESP_ERROR_CHECK(mdns_instance_name_set("ESP32 OTA Web Updater"));
    
    /* Announce the web server service */
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);

    ESP_LOGI("mDNS", "mDNS service started for http://%s.local", hostname);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) 
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
    {
        esp_wifi_connect();
        ESP_LOGI("wifi_event", "Station mode started, connecting to AP...");
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("wifi_event", "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));

        /* Now that we have an IP, we can start services that need it */
        start_mdns_service();
        web_updater_start();
    }
    else
    {
        ESP_LOGW("wifi_event", "Unhandled event: %ld", event_id); // add more event handling as needed
    }
}