#include "web_updater.h"
#include <string.h>
#include <esp_log.h>
#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include <esp_flash_partitions.h>
#include <esp_partition.h>
#include <esp_system.h>

#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif

static const char *TAG = "web_updater";

/*
 * OTA update handler
 *
 * This function handles the HTTP POST request to upload a new firmware binary.
 * It expects the entire request body to be the firmware binary data.
 */
static esp_err_t update_post_handler(httpd_req_t *req)
{
    char buf[1024];
    esp_ota_handle_t ota_handle = 0;
    int remaining = req->content_len;
    int binary_file_len = 0;

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "Passive OTA partition not found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA partition not found");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%" PRIx32,
             update_partition->subtype, update_partition->address);

    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "esp_ota_begin succeeded");

    // Loop to receive and write firmware data
    while (remaining > 0) {
        int received = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                // Retry if timeout occurred
                continue;
            }
            ESP_LOGE(TAG, "File reception failed!");
            esp_ota_abort(ota_handle); // Abort OTA on error
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File reception failed");
            return ESP_FAIL;
        }

        err = esp_ota_write(ota_handle, (const void *)buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed (%s)!", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            return ESP_FAIL;
        }
        binary_file_len += received;
        remaining -= received;
        ESP_LOGI(TAG, "Written %d of %d bytes", binary_file_len, req->content_len);
    }

    ESP_LOGI(TAG, "Total Write binary data length: %d", binary_file_len);

    // Finalize the OTA update
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return ESP_FAIL;
    }

    // Set the new boot partition
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set boot partition");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA Update successful!");
    const char resp_str[] = "Firmware uploaded successfully! Rebooting...";
    httpd_resp_send(req, resp_str, strlen(resp_str));
    
    // Give the response time to be sent before restarting
    vTaskDelay(500 / portTICK_PERIOD_MS); 
    esp_restart();

    return ESP_OK;
}

/*
 * Simple GET handler to show a basic information page.
 * The HTML form is removed as updates are now done via curl or a similar tool.
 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char resp[] = "<html><body><h1>ESP32 Web Updater</h1>"
                        "<p>To update firmware, use the following curl command:</p>"
                        "<pre>curl -X POST --data-binary \"@/path/to/your/firmware.bin\" http://your_esp32_ip/</pre>"
                        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/*
 * Function to start the web server
 */
void web_updater_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.stack_size = 8192;
    
    // It's better to have a dedicated endpoint for the update
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Starting HTTP Server");
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start http server");
        return;
    }

    // URI handler for the root page (GET)
    httpd_uri_t root_uri = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = root_get_handler,
    };
    httpd_register_uri_handler(server, &root_uri);

    // URI handler for the firmware update (POST)
    httpd_uri_t update_uri = {
        .uri      = "/*", // Or be more specific, e.g., "/update"
        .method   = HTTP_POST,
        .handler  = update_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &update_uri);

    ESP_LOGI(TAG, "Web Updater started. Use 'curl' to upload firmware.");
}