#include "web_terminal.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <sys/param.h>
#include "persistent_params.h"
#include "sx1262_interface.hpp"
#include "esp_chip_info.h"
#include "esp_timer.h"
#include "esp_flash.h"

extern char LoRaMessageGlobal[MAX_LORA_PAYLOAD_LENGTH];

static const char *TAG = "WEB_TERMINAL";

/* HTTP Server handle */
static httpd_handle_t server = NULL;

/* Queue for storing messages to send to clients */
static QueueHandle_t message_queue = NULL;

/* HTML page for the web terminal */
static const char web_terminal_html[] = 
"<!DOCTYPE html>"
"<html>"
"<head>"
"    <title>ESP32 Web Terminal</title>"
"    <style>"
"        body { font-family: monospace; background-color: #000; color: #0f0; margin: 0; padding: 20px; }"
"        #terminal { width: 100%; height: 400px; background-color: #000; color: #0f0; border: 1px solid #0f0; padding: 10px; overflow-y: scroll; white-space: pre-wrap; }"
"        #input { width: 80%; padding: 5px; background-color: #000; color: #0f0; border: 1px solid #0f0; font-family: monospace; }"
"        #send { padding: 5px 15px; background-color: #0f0; color: #000; border: none; cursor: pointer; font-family: monospace; }"
"        #send:hover { background-color: #0a0; }"
"        .input-container { margin-top: 10px; }"
"        .prompt { color: #0f0; }"
"    </style>"
"</head>"
"<body>"
"    <h1>ESP32 Web Terminal</h1>"
"    <div id=\"terminal\"></div>"
"    <div class=\"input-container\">"
"        <input type=\"text\" id=\"input\" placeholder=\"Enter command...\" autofocus>"
"        <button id=\"send\">Send</button>"
"    </div>"
"    <script>"
"        const terminal = document.getElementById('terminal');"
"        const input = document.getElementById('input');"
"        const sendBtn = document.getElementById('send');"
"        "
"        function addToTerminal(text, isCommand = false) {"
"            const timestamp = new Date().toLocaleTimeString();"
"            if (isCommand) {"
"                terminal.innerHTML += `<span class=\"prompt\">[${timestamp}] > ${text}</span>\\n`;"
"            } else {"
"                terminal.innerHTML += `<span>[${timestamp}] ${text}</span>\\n`;"
"            }"
"            terminal.scrollTop = terminal.scrollHeight;"
"        }"
"        "
"        function sendCommand() {"
"            const command = input.value.trim();"
"            if (command) {"
"                addToTerminal(command, true);"
"                fetch('/api/command', {"
"                    method: 'POST',"
"                    headers: { 'Content-Type': 'application/json' },"
"                    body: JSON.stringify({ command: command })"
"                })"
"                .then(response => response.json())"
"                .then(data => {"
"                    if (data.response) {"
"                        addToTerminal(data.response);"
"                    }"
"                })"
"                .catch(error => {"
"                    addToTerminal('Error: ' + error.message);"
"                });"
"                input.value = '';"
"            }"
"        }"
"        "
"        function pollMessages() {"
"            fetch('/api/messages')"
"            .then(response => response.json())"
"            .then(data => {"
"                if (data.message) {"
"                    addToTerminal(data.message);"
"                }"
"            })"
"            .catch(error => {"
"                /* Silent fail for polling */"
"            });"
"        }"
"        "
"        sendBtn.addEventListener('click', sendCommand);"
"        input.addEventListener('keypress', function(e) {"
"            if (e.key === 'Enter') {"
"                sendCommand();"
"            }"
"        });"
"        "
"        addToTerminal('Web Terminal Connected');"
"        addToTerminal('Type commands to interact with the ESP32');"
"        "
"        /* Poll for new messages every 1 second */"
"        setInterval(pollMessages, 1000);"
"    </script>"
"</body>"
"</html>";

static void web_terminal_command_handler(const char *command);

/* HTTP GET handler for the main terminal page */
static esp_err_t terminal_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, web_terminal_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* HTTP POST handler for receiving commands */
static esp_err_t command_post_handler(httpd_req_t *req)
{
    char content[256];
    size_t recv_size = MIN(req->content_len, sizeof(content) - 1);
    
    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) 
    {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) 
        {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    
    content[recv_size] = '\0';
    
    /* Parse JSON to extract command */
    char *command_start = strstr(content, "\"command\":\"");
    if (command_start) 
    {
        command_start += 11; /* Skip past "command":" */
        char *command_end = strchr(command_start, '"');
        if (command_end) 
        {
            *command_end = '\0';
            
            ESP_LOGI(TAG, "Received command: %s", command_start);
            
            /* Call the command callback to handle the command */
            web_terminal_command_handler(command_start);

            /* Send response */
            httpd_resp_set_type(req, "application/json");
            char response[128];
            snprintf(response, sizeof(response), "{\"status\":\"ok\",\"response\":\"Command '%s' received\"}", command_start);
            httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
    }
    
    /* Invalid JSON format */
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"Invalid JSON format\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
}

/* HTTP GET handler for polling messages */
static esp_err_t messages_get_handler(httpd_req_t *req)
{
    char message[256];
    
    /* Try to receive a message from the queue (non-blocking) */
    if (xQueueReceive(message_queue, message, 0) == pdTRUE) 
    {
        httpd_resp_set_type(req, "application/json");
        char response[320];
        snprintf(response, sizeof(response), "{\"message\":\"%s\"}", message);
        httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    } 
    else 
    {
        /* No messages available */
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{}", HTTPD_RESP_USE_STRLEN);
    }
    
    return ESP_OK;
}

/* URI handlers */
static const httpd_uri_t terminal_uri = 
{
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = terminal_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t command_uri = 
{
    .uri       = "/api/command",
    .method    = HTTP_POST,
    .handler   = command_post_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t messages_uri = 
{
    .uri       = "/api/messages",
    .method    = HTTP_GET,
    .handler   = messages_get_handler,
    .user_ctx  = NULL
};

esp_err_t web_terminal_start(void)
{
    /* Create message queue */
    message_queue = xQueueCreate(10, 256);
    if (message_queue == NULL) 
    {
        ESP_LOGE(TAG, "Failed to create message queue");
        return ESP_FAIL;
    }
    
    /* Start the HTTP server */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 8080;
    config.max_uri_handlers = 8;
    
    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
    
    if (httpd_start(&server, &config) == ESP_OK) 
    {
        /* Register URI handlers */
        httpd_register_uri_handler(server, &terminal_uri);
        httpd_register_uri_handler(server, &command_uri);
        httpd_register_uri_handler(server, &messages_uri);
        
        ESP_LOGI(TAG, "Web terminal started successfully");
        ESP_LOGI(TAG, "Access terminal at http://[ESP32_IP]:8080/");
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to start HTTP server");
    return ESP_FAIL;
}

esp_err_t web_terminal_stop(void)
{
    if (server) 
    {
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "Web terminal server stopped");
    }
    
    if (message_queue) 
    {
        vQueueDelete(message_queue);
        message_queue = NULL;
    }
    
    return ESP_OK;
}

esp_err_t web_terminal_send_message(const char *message)
{
    if (message_queue && message) 
    {
        char msg_copy[256];
        strncpy(msg_copy, message, sizeof(msg_copy) - 1);
        msg_copy[sizeof(msg_copy) - 1] = '\0';
        
        if (xQueueSend(message_queue, msg_copy, 0) != pdTRUE) 
        {
            ESP_LOGW(TAG, "Message queue full, message dropped");
            return ESP_FAIL;
        }
        return ESP_OK;
    }
    return ESP_FAIL;
}

/* Web Terminal Command Handler */
static void web_terminal_command_handler(const char *command)
{
    ESP_LOGI(TAG, "Processing web terminal command: %s", command);
    
    if (strcmp(command, "help") == 0) 
    {
        web_terminal_send_message("Available commands:");
        web_terminal_send_message("  help - Show this help message");
        web_terminal_send_message("  status - Show system status");
        web_terminal_send_message("  getMsg - Show current LoRa message");
        web_terminal_send_message("  setMsg <message> - Change LoRa payload");
        web_terminal_send_message("  restart - Restart the ESP32");
        web_terminal_send_message("  info - Show device information");
    }
    else if (strcmp(command, "status") == 0) 
    {
        char status_msg[128];
        int32_t device_mode = 0;
        (void)persistent_params_get_integer("device_mode", &device_mode);

        snprintf(status_msg, sizeof(status_msg), "Device Mode: %s", 
                device_mode == TRANSMITTER_MODE ? "Transmitter" :
                device_mode == RECEIVER_MODE ? "Receiver" : 
                device_mode == TRANSCEIVER_MODE ? "Transceiver" : "Unknown");
        web_terminal_send_message(status_msg);
        
        snprintf(status_msg, sizeof(status_msg), "Free heap: %d bytes", (int)esp_get_free_heap_size());
        web_terminal_send_message(status_msg);
        
        snprintf(status_msg, sizeof(status_msg), "Uptime: %d seconds", (int)(esp_timer_get_time() / 1000000));
        web_terminal_send_message(status_msg);
    }
    else if (strcmp(command, "getMsg") == 0) 
    {
        char current_msg[MAX_LORA_PAYLOAD_LENGTH];
        if (lora_message_get_safe(current_msg, sizeof(current_msg), 1000) == ESP_OK)
        {
            char response_msg[MAX_LORA_PAYLOAD_LENGTH + 20];
            snprintf(response_msg, sizeof(response_msg), "Current LoRa message: %.233s", current_msg);
            web_terminal_send_message(response_msg);
        }
        else
        {
            web_terminal_send_message("Error: Failed to get current LoRa message");
        }
    }
    else if (strncmp(command, "setMsg ", strlen("setMsg ")) == 0) 
    {
        const char *message = command + strlen("setMsg "); /* Skip "setMsg " */
        if ((strlen(message) > 0) && (strlen(message) < MAX_LORA_PAYLOAD_LENGTH))
        {
            // Use thread-safe function to set LoRa message
            if (lora_message_set_safe(message, 1000) == ESP_OK)
            {
                web_terminal_send_message("LoRa message updated successfully");
            }
            else
            {
                web_terminal_send_message("Error: Failed to update LoRa message");
            }
        } 
        else 
        {
            web_terminal_send_message("Error: Invalid message length");
        }
    }
    else if (strcmp(command, "restart") == 0) 
    {
        web_terminal_send_message("Restarting ESP32 in 3 seconds...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }
    else if (strcmp(command, "info") == 0) 
    {
        esp_chip_info_t chip_info;
        esp_chip_info(&chip_info);
        
        char info_msg[128];
        snprintf(info_msg, sizeof(info_msg), "ESP32 Model: %s", 
                chip_info.model == CHIP_ESP32 ? "ESP32" :
                chip_info.model == CHIP_ESP32S2 ? "ESP32-S2" :
                chip_info.model == CHIP_ESP32S3 ? "ESP32-S3" :
                chip_info.model == CHIP_ESP32C3 ? "ESP32-C3" : "Unknown");
        web_terminal_send_message(info_msg);
        
        snprintf(info_msg, sizeof(info_msg), "CPU Cores: %d", chip_info.cores);
        web_terminal_send_message(info_msg);

        uint32_t flash_size = 0;
        esp_flash_get_size(NULL, &flash_size);
        snprintf(info_msg, sizeof(info_msg), "Flash Size: %d MB", (int)(flash_size / (1024 * 1024)));
        web_terminal_send_message(info_msg);
    }
    else 
    {
        char error_msg[128];
        snprintf(error_msg, sizeof(error_msg), "Unknown command: %s. Type 'help' for available commands.", command);
        web_terminal_send_message(error_msg);
    }
}