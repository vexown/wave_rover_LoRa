#pragma once

#include "esp_http_server.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and start the web terminal server
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t web_terminal_start(void);

/**
 * @brief Stop the web terminal server
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t web_terminal_stop(void);

/**
 * @brief Send a message to all connected web terminal clients
 * @param message The message to send
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t web_terminal_send_message(const char *message);

/**
 * @brief Set callback function for handling received commands
 * @param callback Function to call when a command is received
 */
void web_terminal_set_command_callback(void (*callback)(const char *command));

#ifdef __cplusplus
}
#endif
