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

static esp_err_t upload_post_handler(httpd_req_t *req)
{
    char buf[1024];
    int received;
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        ESP_LOGE(TAG, "No OTA partition found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition found");
        return ESP_FAIL;
    }
    ESP_ERROR_CHECK(esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle));
    int remaining = req->content_len;
    bool started = false;
    int header_bytes = 0;
    char header_buf[2048]; // Large enough for typical multipart headers

    while (remaining > 0) {
        received = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));
        if (received <= 0) {
            ESP_LOGE(TAG, "File reception failed!");
            esp_ota_end(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File reception failed");
            return ESP_FAIL;
        }
        if (!started) {
            // Accumulate header data until we find the end of headers
            if (header_bytes + received >= sizeof(header_buf)) {
                ESP_LOGE(TAG, "Multipart header too large!");
                esp_ota_end(ota_handle);
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Multipart header too large");
                return ESP_FAIL;
            }
            memcpy(header_buf + header_bytes, buf, received);
            header_bytes += received;
            header_buf[header_bytes] = '\0';
            char *body = strstr(header_buf, "\r\n\r\n");
            if (body) {
                body += 4; // Skip past the header
                int body_offset = body - header_buf;
                int body_len = header_bytes - body_offset;
                ESP_ERROR_CHECK(esp_ota_write(ota_handle, body, body_len));
                started = true;
            }
            // If not found, keep accumulating
        } else {
            ESP_ERROR_CHECK(esp_ota_write(ota_handle, buf, received));
        }
        remaining -= received;
    }
    if (esp_ota_end(ota_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed!");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return ESP_FAIL;
    }
    ESP_ERROR_CHECK(esp_ota_set_boot_partition(update_partition));
    httpd_resp_sendstr(req, "Firmware uploaded! Rebooting...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    esp_restart();
    return ESP_OK;
}

static esp_err_t upload_get_handler(httpd_req_t *req)
{
    const char resp[] = "<html><body><h2>ESP32 OTA Web Updater</h2>"
        "<form method='POST' enctype='multipart/form-data'>"
        "<input type='file' name='firmware'>"
        "<input type='submit' value='Update Firmware'>"
        "</form></body></html>";
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void web_updater_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t upload_get_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = upload_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &upload_get_uri);

    httpd_uri_t upload_post_uri = {
        .uri = "/",
        .method = HTTP_POST,
        .handler = upload_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &upload_post_uri);

    ESP_LOGI(TAG, "Web Updater started. Connect to http://<device_ip>/ to update firmware.");
}
