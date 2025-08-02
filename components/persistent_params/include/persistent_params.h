/**
 * @file persistent_params.h
 * @brief Persistent parameter management for string and integer types using NVS.
 *
 * This component provides functions to initialize, get, and set persistent parameters
 * (strings and integers) stored in Non-Volatile Storage (NVS).
 *
 * @warning Use this component only after NVS has been initialized.
 *          Failure to do so may result in undefined behavior or errors.
 */

#ifndef PERSISTENT_PARAMS_H
#define PERSISTENT_PARAMS_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * @brief Initializes a string parameter (creates if missing)
 * 
 * @param name The name of the parameter.
 * @param default_value The default value of the parameter.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t persistent_params_init_string(const char* name, const char* default_value);

/*
 * @brief Gets the value of a string parameter
 * 
 * @param name The name of the parameter.
 * @param value Buffer to store the value.
 * @param len Length of the buffer.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t persistent_params_get_string(const char* name, char* value, size_t len);

/*
 * @brief Sets the value of a string parameter
 * 
 * @param name The name of the parameter.
 * @param value The new value of the parameter.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t persistent_params_set_string(const char* name, const char* value);

/*
 * @brief Initializes an integer parameter (creates if missing)
 * 
 * @param name The name of the parameter.
 * @param default_value The default value of the parameter.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t persistent_params_init_integer(const char* name, int32_t default_value);

/*
 * @brief Gets the value of an integer parameter
 * 
 * @param name The name of the parameter.
 * @param value Pointer to store the value.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t persistent_params_get_integer(const char* name, int32_t* value);

/*
 * @brief Sets the value of an integer parameter
 * 
 * @param name The name of the parameter.
 * @param value The new value of the parameter.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t persistent_params_set_integer(const char* name, int32_t value);

#ifdef __cplusplus
}
#endif

#endif /* PERSISTENT_PARAMS_H */
